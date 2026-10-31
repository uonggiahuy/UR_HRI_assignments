"""Single UTF-8 vocabulary and reference geometry shared by map and speech."""

# Bounds in the reference's clockwise warehouse view, normalized to exterior walls.
# The PGM is portrait: reference (u,v) maps to PGM (v,1-u).
DEFINITIONS = [
    (
        "A",
        "Khu vực cửa ra vào",
        "Main warehouse entrance.",
        ["khu A", "khu vực A", "cửa ra vào", "khu cửa ra vào", "lối vào kho"],
        (0, 0.25, 0.063, 0.76),
    ),
    (
        "B",
        "Kệ linh kiện cảm biến",
        "Storage for LiDAR, cameras, IMU and encoders.",
        ["khu B", "khu cảm biến", "kệ cảm biến", "khu linh kiện cảm biến"],
        (0.063, 0.02, 0.435, 0.28),
    ),
    (
        "C",
        "Kệ linh kiện truyền động",
        "Storage for DC motors, servos, gearboxes and actuators.",
        ["khu C", "khu truyền động", "kệ truyền động", "khu linh kiện truyền động"],
        (0.063, 0.28, 0.435, 0.54),
    ),
    (
        "D",
        "Kệ linh kiện điều khiển",
        "Storage for microcontrollers, motor drivers and control electronics.",
        ["khu D", "khu điều khiển", "kệ điều khiển", "khu linh kiện điều khiển"],
        (0.063, 0.54, 0.435, 0.80),
    ),
    (
        "E",
        "Khu vực đóng gói và xuất kho",
        "Component packing and dispatch area.",
        ["khu E", "khu đóng gói", "khu xuất kho", "khu đóng gói và xuất kho"],
        (0.063, 0.80, 0.435, 1),
    ),
    (
        "F",
        "Khu vực trung tâm",
        "Central circulation and transfer area.",
        ["khu F", "khu trung tâm", "khu vực trung tâm", "khu trung chuyển", "khu giữa"],
        (0.435, 0.19, 0.644, 1),
    ),
    (
        "G",
        "Kệ linh kiện nguồn và pin",
        "Storage for batteries, power supplies, BMS and chargers.",
        ["khu G", "khu nguồn", "khu pin", "khu linh kiện nguồn", "khu pin và nguồn"],
        (0.644, 0.17, 0.88, 1),
    ),
    (
        "H",
        "Sảnh vào phòng họp",
        "Hallway and waiting area near the meeting room.",
        ["khu H", "sảnh phòng họp", "sảnh vào phòng họp", "hành lang phòng họp"],
        (0.88, 0, 1, 1),
    ),
    (
        "I",
        "Phòng họp kỹ thuật",
        "Technical discussion and meeting room.",
        ["khu I", "phòng họp", "phòng họp kỹ thuật"],
        (0.58, 0, 0.88, 0.17),
    ),
]
COLORS = [
    "#00bcd4",
    "#3979cc",
    "#af895b",
    "#ed6370",
    "#f3ab49",
    "#e95337",
    "#8351de",
    "#ce4781",
    "#2aab84",
]
