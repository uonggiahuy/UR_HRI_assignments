"""Reference-guided semantic sections; occupancy and approach poses stay separate."""

import argparse
import hashlib
import math
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.ndimage import distance_transform_edt, label
import yaml


from turtlebot4.semantic_regions import DEFINITIONS, COLORS


class WarehouseMap:
    def __init__(self, path):
        self.path = Path(path)
        self.meta = yaml.safe_load(self.path.read_text())
        self.image_path = self.path.parent / self.meta["image"]
        self.image = np.asarray(Image.open(self.image_path).convert("L"))
        self.height, self.width = self.image.shape
        self.resolution = float(self.meta["resolution"])
        self.origin = tuple(float(x) for x in self.meta["origin"])
        if (
            self.meta.get("mode", "trinary") != "trinary"
            or self.resolution <= 0
            or len(self.origin) != 3
            or not all(math.isfinite(v) for v in (*self.origin, self.resolution))
            or self.meta["negate"] not in (0, 1)
            or not 0 <= self.meta["free_thresh"] < self.meta["occupied_thresh"] <= 1
        ):
            raise ValueError("Invalid or unsupported map metadata")
        probability = self.image.astype(float) / 255.0
        if not self.meta["negate"]:
            probability = 1.0 - probability
        self.free = probability < self.meta["free_thresh"]
        self.occupied = probability > self.meta["occupied_thresh"]
        self.unknown = ~(self.free | self.occupied)
        # Pad as blocked: a footprint must not extend outside the map.
        self.clearance = (
            distance_transform_edt(np.pad(self.free, 1))[1:-1, 1:-1] * self.resolution
        )
        self.safe = self.free & (self.clearance >= 0.55)
        self.components, count = label(self.safe)
        sizes = np.bincount(self.components.ravel(), minlength=count + 1)
        sizes[0] = 0
        self.main_component = int(np.argmax(sizes)) if count else -1
        self.hashes = {
            key: hashlib.sha256(p.read_bytes()).hexdigest()
            for key, p in [("yaml", self.path), ("pgm", self.image_path)]
        }

    def pixel_to_world(self, col, row):
        x = (col + 0.5) * self.resolution
        y = (self.height - row - 0.5) * self.resolution
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        return self.origin[0] + c * x - s * y, self.origin[1] + s * x + c * y

    def world_to_pixel(self, x, y):
        dx, dy = x - self.origin[0], y - self.origin[1]
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        return (
            (c * dx + s * dy) / self.resolution - 0.5,
            self.height - 0.5 - (-s * dx + c * dy) / self.resolution,
        )

    def validate_goal(self, goal, clearance=0.55):
        if not all(math.isfinite(goal[k]) for k in ("x", "y", "yaw")):
            raise ValueError("Non-finite goal")
        col, row = self.world_to_pixel(goal["x"], goal["y"])
        col, row = math.floor(col + 0.5), math.floor(row + 0.5)
        if not (0 <= col < self.width and 0 <= row < self.height):
            raise ValueError("Goal outside map")
        if not self.free[row, col] or self.clearance[row, col] < clearance:
            raise ValueError(
                "Goal blocked, unknown or insufficient footprint clearance"
            )
        if self.components[row, col] != self.main_component:
            raise ValueError("Goal disconnected from main clearance-safe component")
        return float(self.clearance[row, col])


def polygon_mask(shape, polygon):
    yy, xx = np.indices(shape, dtype=np.float32)
    # Pixel centers; strict interior allows shared polygon edges without overlap.
    contour = np.asarray(polygon, np.float32)
    if (
        len(contour) != 4
        or not np.isfinite(contour).all()
        or not cv2.isContourConvex(contour)
        or cv2.contourArea(contour) < 1
    ):
        raise ValueError("Invalid polygon")
    result = np.ones(shape, bool)
    area_sign = np.sign(cv2.contourArea(contour, oriented=True))
    for a, b in zip(contour, np.roll(contour, -1, axis=0)):
        cross = (b[0] - a[0]) * (yy - a[1]) - (b[1] - a[1]) * (xx - a[0])
        result &= area_sign * cross > 1e-4
    return result


def validate_polygons(grid, regions):
    seen = np.zeros(grid.image.shape, bool)
    for region in regions:
        polygon = np.asarray(region["polygon_pixels"])
        if (
            polygon[:, 0].min() < 0
            or polygon[:, 0].max() >= grid.width
            or polygon[:, 1].min() < 0
            or polygon[:, 1].max() >= grid.height
        ):
            raise ValueError("Polygon outside map")
        mask = polygon_mask(grid.image.shape, polygon)
        if np.any(mask & seen):
            raise ValueError("Unintended region overlap")
        seen |= mask


def generate(grid):
    regions = []
    for ident, name, description, aliases, bounds in DEFINITIONS:
        u0, v0, u1, v1 = bounds
        polygon = [
            [v * (grid.width - 1), (1 - u) * (grid.height - 1)]
            for u, v in [(u0, v0), (u1, v0), (u1, v1), (u0, v1)]
        ]
        center = np.asarray(polygon).mean(axis=0)
        mask = polygon_mask(grid.image.shape, polygon)
        candidates = np.argwhere(
            mask & grid.safe & (grid.components == grid.main_component)
        )
        region = dict(
            id=ident,
            name=name,
            aliases=aliases,
            description=description,
            polygon_pixels=polygon,
            polygon_map=[list(grid.pixel_to_world(*p)) for p in polygon],
            representative_point=dict(
                zip(("x", "y"), map(float, grid.pixel_to_world(*center)))
            ),
            approach_goal=None,
            validation="INVALID",
            reachability="UNAVAILABLE",
            reason="No footprint-safe aisle in main free-space component",
            nav2_path="NOT TESTED",
            live_navigation="NOT TESTED",
        )
        if len(candidates):
            # Choose a clear aisle near the semantic center, never a shelf center.
            scores = np.linalg.norm(candidates[:, ::-1] - center, axis=1)
            row, col = candidates[int(np.argmin(scores))]
            x, y = grid.pixel_to_world(int(col), int(row))
            yaw = math.atan2(
                region["representative_point"]["y"] - y,
                region["representative_point"]["x"] - x,
            )
            goal = dict(x=float(x), y=float(y), yaw=float(yaw))
            region.update(
                approach_goal=goal,
                clearance_m=grid.validate_goal(goal),
                validation="VALID",
                reachability="GEOMETRIC_ONLY",
                reason="Inside semantic section; known-free aisle in largest 0.55 m clearance component; Nav2 plan required",
            )
        regions.append(region)
    validate_polygons(grid, regions)
    return dict(
        version=1,
        frame_id="map",
        map_sha256=grid.hashes,
        map_metadata=grid.meta,
        image_size=[grid.width, grid.height],
        minimum_clearance=0.55,
        alignment="Reference clockwise view (u,v) -> PGM (v,1-u), exterior walls and three racks",
        regions=regions,
    )


def load_regions(path, map_path):
    grid = WarehouseMap(map_path)
    config = yaml.safe_load(Path(path).read_text())
    if (
        config.get("map_sha256") != grid.hashes
        or config.get("map_metadata") != grid.meta
    ):
        raise ValueError("Stale semantic map: occupancy map hash/metadata mismatch")
    if (
        config.get("frame_id") != "map"
        or config.get("version") != 1
        or [r["id"] for r in config["regions"]] != list("ABCDEFGHI")
        or config.get("minimum_clearance", 0) < 0.55
    ):
        raise ValueError("Invalid region schema")
    validate_polygons(grid, config["regions"])
    for region, definition in zip(config["regions"], DEFINITIONS):
        if region["name"] != definition[1] or region["aliases"] != definition[3]:
            raise ValueError("Semantic definitions differ")
        if not np.allclose(
            region["polygon_map"],
            [grid.pixel_to_world(*p) for p in region["polygon_pixels"]],
        ):
            raise ValueError("Polygon map/pixel geometry differs")
        if region["validation"] == "VALID":
            grid.validate_goal(region["approach_goal"], config["minimum_clearance"])
            col, row = grid.world_to_pixel(
                region["approach_goal"]["x"], region["approach_goal"]["y"]
            )
            if (
                cv2.pointPolygonTest(
                    np.asarray(region["polygon_pixels"], np.float32), (col, row), False
                )
                < 0
            ):
                raise ValueError(
                    "Approach goal association is not inside its documented region"
                )
    return config, grid


def visualize(grid, config, output):
    base = Image.fromarray(grid.image).convert("RGBA")
    overlay = Image.new("RGBA", base.size)
    draw = ImageDraw.Draw(overlay)
    font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    large = ImageFont.truetype(font_path, 29)
    small = ImageFont.truetype(font_path, 15)
    for region, color in zip(config["regions"], COLORS):
        rgb = tuple(int(color[i : i + 2], 16) for i in (1, 3, 5))
        points = [tuple(p) for p in region["polygon_pixels"]]
        draw.polygon(points, fill=rgb + (42,), outline=rgb + (255,), width=3)
        x, y = np.asarray(points).mean(axis=0)
        draw.text(
            (x - 10, y - 18),
            region["id"],
            font=large,
            fill=(10, 50, 70, 255),
            stroke_width=1,
        )
        draw.ellipse((x - 3, y - 3, x + 3, y + 3), outline=(0, 0, 0, 255))
        if region["approach_goal"]:
            p = region["approach_goal"]
            gx, gy = grid.world_to_pixel(p["x"], p["y"])
            draw.line((gx - 6, gy, gx + 6, gy), fill=(190, 0, 0, 255), width=3)
            draw.line((gx, gy - 6, gx, gy + 6), fill=(190, 0, 0, 255), width=3)
    canvas = Image.new("RGB", (grid.width + 430, grid.height), "white")
    canvas.paste(Image.alpha_composite(base, overlay).convert("RGB"))
    legend = ImageDraw.Draw(canvas)
    legend.text((grid.width + 15, 20), "Assignment 7 — A–I", font=large, fill="black")
    for i, (region, color) in enumerate(zip(config["regions"], COLORS)):
        y = 90 + i * 65
        legend.rectangle((grid.width + 15, y, grid.width + 28, y + 18), fill=color)
        legend.text(
            (grid.width + 40, y),
            region["id"] + " — " + region["name"],
            font=small,
            fill="black",
        )
    legend.text(
        (grid.width + 15, 735),
        "Đỏ (+): đích tiếp cận; tròn: tâm vùng",
        font=small,
        fill="black",
    )
    legend.text(
        (grid.width + 15, 765),
        "Giữ hướng PGM; ảnh tham chiếu xoay 90°",
        font=small,
        fill="black",
    )
    legend.text(
        (grid.width + 15, 795),
        "Đích hình học ≠ đường Nav2 đã kiểm chứng",
        font=small,
        fill="black",
    )
    canvas.save(output)


def write_rviz_legend(config, output):
    """A textured plane preserves Vietnamese glyphs in Ogre's ASCII text viewer."""
    canvas = Image.new("RGB", (650, 850), "white")
    draw = ImageDraw.Draw(canvas)
    font = ImageFont.truetype("DejaVuSans.ttf", 34)
    draw.text((25, 25), "Assignment 7 — A–I", font=font, fill="black")
    for index, (region, color) in enumerate(zip(config["regions"], COLORS)):
        y = 110 + index * 78
        draw.rectangle((20, y, 34, y + 24), fill=color)
        draw.text(
            (45, y), region["id"] + " — " + region["name"], font=font, fill="black"
        )
    canvas.save(output / "semantic_legend.png")
    (output / "semantic_legend.dae").write_text(LEGEND_DAE)


LEGEND_DAE = """<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
<asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
<library_images><image id="legend-image"><init_from>semantic_legend.png</init_from></image></library_images>
<library_effects><effect id="legend-effect"><profile_COMMON>
<newparam sid="surface"><surface type="2D"><init_from>legend-image</init_from></surface></newparam>
<newparam sid="sampler"><sampler2D><source>surface</source></sampler2D></newparam>
<technique sid="common"><lambert><diffuse><texture texture="sampler" texcoord="UVMap"/></diffuse></lambert></technique>
</profile_COMMON></effect></library_effects>
<library_materials><material id="legend-material"><instance_effect url="#legend-effect"/></material></library_materials>
<library_geometries><geometry id="legend"><mesh>
<source id="positions"><float_array id="positions-array" count="12">0 0 0 20 0 0 20 44 0 0 44 0</float_array>
<technique_common><accessor source="#positions-array" count="4" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
<source id="uv"><float_array id="uv-array" count="8">0 0 1 0 1 1 0 1</float_array>
<technique_common><accessor source="#uv-array" count="4" stride="2"><param name="S" type="float"/><param name="T" type="float"/></accessor></technique_common></source>
<vertices id="vertices"><input semantic="POSITION" source="#positions"/></vertices>
<triangles count="2" material="material"><input semantic="VERTEX" source="#vertices" offset="0"/><input semantic="TEXCOORD" source="#uv" offset="1" set="0"/><p>0 0 1 1 2 2 0 0 2 2 3 3</p></triangles>
</mesh></geometry></library_geometries>
<library_visual_scenes><visual_scene id="scene"><node id="legend-node"><instance_geometry url="#legend"><bind_material><technique_common><instance_material symbol="material" target="#legend-material"><bind_vertex_input semantic="UVMap" input_semantic="TEXCOORD" input_set="0"/></instance_material></technique_common></bind_material></instance_geometry></node></visual_scene></library_visual_scenes>
<scene><instance_visual_scene url="#scene"/></scene></COLLADA>
"""


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args(argv)
    grid = WarehouseMap(args.map)
    config = generate(grid)
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    (output / "regions.yaml").write_text(
        yaml.safe_dump(config, allow_unicode=True, sort_keys=False)
    )
    visualize(grid, config, output / "segmented_map.png")
    write_rviz_legend(config, output)
    for region in config["regions"]:
        print(
            region["id"],
            region["approach_goal"],
            region["validation"],
            region["reachability"],
        )


if __name__ == "__main__":
    main()
