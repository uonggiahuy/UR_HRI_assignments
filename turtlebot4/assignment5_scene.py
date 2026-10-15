"""Spawn packaged textured targets through the installed Fortress create interface."""
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory, get_package_prefix
import signal
import yaml


def scene_models(share):
    share = Path(share)
    scene = yaml.safe_load((share / 'config/assignment5_scene.yaml').read_text())
    names = [item['name'] for item in scene['objects']]
    if len(names) != len(set(names)):
        raise ValueError('Scene entity names must be unique')
    for item in scene['objects']:
        directory = share / 'models' / item['model']
        tree = ET.fromstring((directory / 'model.sdf').read_text())
        tree.find('model').set('name', item['name'])
        for uri in tree.findall('.//uri'):
            prefix = f"model://{item['model']}/"
            if not uri.text.startswith(prefix):
                raise ValueError('Only packaged local mesh resources are supported')
            relative = Path(uri.text[len(prefix):])
            if relative.is_absolute() or '..' in relative.parts:
                raise ValueError('Mesh paths must stay within the packaged model')
            # colcon --symlink-install links individual assets into the source
            # tree; lexical containment is checked before resolving those links.
            path = (directory / relative).resolve()
            if not path.is_file():
                raise ValueError(f'Missing or unsafe model resource: {path}')
            uri.text = str(path)
        yield scene['world'], item, ET.tostring(tree, encoding='unicode')


def main():
    def interrupt(_signum, _frame):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    try:
        executable = str(Path(get_package_prefix('ros_gz_sim')) / 'lib/ros_gz_sim/create')
        for world, item, sdf in scene_models(get_package_share_directory('turtlebot4')):
            x, y, z, heading = item['pose']
            subprocess.run([
                executable, '-world', world,
                '-name', item['name'], '-allow_renaming', 'false', '-string', sdf,
                '-x', str(x), '-y', str(y), '-z', str(z), '-Y', str(heading),
            ], check=True, timeout=45)
            print(f"SCENE requested {item['name']} world pose={item['pose']}", flush=True)
        return 0
    except KeyboardInterrupt:
        return 130
    except (OSError, ValueError, subprocess.SubprocessError) as exc:
        print(f'SCENE FAIL: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
