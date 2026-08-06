from pathlib import Path
import heapq
import json
import math
import os

import cv2
import numpy as np
from PIL import Image, ImageDraw


ROOT = Path(__file__).resolve().parents[1]
TEXTURE_DIR = ROOT / "media" / "materials" / "textures"
TEXTURE = TEXTURE_DIR / "kookmin_track.png"
ROAD_TEXTURE = TEXTURE_DIR / "kookmin_track_road.png"
WHITE_LINE_TEXTURE = TEXTURE_DIR / "kookmin_white_lines.png"
YELLOW_CENTERLINE_TEXTURE = TEXTURE_DIR / "kookmin_yellow_centerline.png"
WHITE_LINE_PARTS_METADATA = TEXTURE_DIR / "kookmin_white_line_parts.json"
YELLOW_CENTERLINE_PARTS_METADATA = TEXTURE_DIR / "kookmin_yellow_centerline_parts.json"
WHITE_LINE_BASE_PARTS = [
    "straight",
    "curve_left_top",
    "curve_left_bottom",
    "curve_right_s_01",
    "curve_right_s_02",
    "curve_right_s_03",
    "curve_right_s_04",
    "curve_right_s_05",
]
WHITE_LINE_SIDE_NAMES = ("left", "center", "right")
GZ_WORLD = ROOT / "worlds" / "kookmin_xycar_track_gz.sdf"
CLASSIC_WORLD = ROOT / "worlds" / "kookmin_xycar_track.world"
PREVIEW = ROOT / "preview_topdown.png"
PREVIEW_CLEAN = ROOT / "preview_clean_topdown.png"
BLUEPRINT_IMAGE = Path("/home/as/.codex/attachments/77f77e6b-220f-4b5f-921f-f417ed4c918f/image-1.png")
CAD_WHITE_LINE_DXF = Path(
    os.environ.get(
        "KOOKMIN_TRACK_DXF",
        str(Path.home() / "Downloads" / "\ucd5c\uc885\ud2b8\ub799.dxf"),
    )
)
CAD_WHITE_LINE_ENABLED = True
CAD_USE_REAL_UNITS = True
CAD_UNIT_TO_M = 0.01
CAD_PRESERVE_ASPECT = False
CAD_ADD_CENTER_DASHES = False
CAD_MAIN_COMPONENT_GAP_RATIO = 0.025
CAD_REMOVE_STOP_LINES = True
CAD_EXPAND_TO_INNER_WIDTH = False
CAD_TARGET_INNER_ROAD_W = 0.800
BLUEPRINT_ROI = (198, 234, 813, 581)
BLUEPRINT_RIGHT_S_Y_RANGE = (54, 318)
BLUEPRINT_RIGHT_S_SCAN_X_RANGE = (470, 552)
BLUEPRINT_RIGHT_S_CENTER_X_RANGE = (480, 545)
RIGHT_S_SPLINE_SMOOTHING = 0.02
RIGHT_S_SPLINE_SAMPLES = 240
CENTER_DASH_LEN = 0.420
CENTER_DASH_GAP = 0.460
CENTER_DASH_START = 0.180
YELLOW_CENTER_DASH_LEN = 0.300
YELLOW_CENTER_DASH_GAP = 0.300
YELLOW_CENTER_LINE_W = 0.024
YELLOW_CENTER_CONNECT_MAX_GAP = 0.900
YELLOW_CENTER_ROAD_AREA_MAX_RATIO = 0.70
YELLOW_CENTER_CURVE_SAMPLE_STEP = 0.010
YELLOW_CENTER_CURVE_SEARCH_RADIUS = 0.950
YELLOW_CENTER_CURVE_TANGENT_TOL = 0.220
BLUEPRINT_RIGHT_S_REPLACE_X_RANGE = (478, 570)
BLUEPRINT_RIGHT_S_REPLACE_Y_RANGE = (58, 306)
BLUEPRINT_RIGHT_S_CENTER_DASH_TOL_PX = 12
RIGHT_S_SIDE_LINE_OFFSET_M = 0.500
RIGHT_S_CENTER_COMPONENT_BAND_M = 0.250
RIGHT_S_INNER_STRAIGHT_CONNECT_SEARCH_Y_PX = 8
WHITE_LINE_CENTER_SPLIT_BAND_M = 0.220

# Blueprint-driven dimensions, converted from mm to meters where possible.
BLUEPRINT_OUTER_W = 20.150
BLUEPRINT_OUTER_H = 11.350
BLUEPRINT_CENTERLINE_LEN = 49.200
ROAD_W = 1.632
LANE_LINE_W = 0.015
ROAD_BORDER_LINE_W = 0.024
TRACK_TOP_CENTER_Y = 3.909
TRACK_BOTTOM_CENTER_Y = -4.726
LEFT_TURN_R = 1.850
RIGHT_S_POS_AMP = 0.600
RIGHT_S_NEG_AMP = 0.600
RIGHT_S_FREQ = 4.0
LEFT_TURN_CENTER_X = -6.816

# Real Xycar geometry and the steering range needed to reproduce the
# 2026-07-12 measured response and the temporary +/-42 curvature extrapolation.
# The 0.55 m overall envelope consists of a 0.50 m body plus the LiDAR
# protruding 0.05 m beyond the body's front face.
XYCAR_BODY_LENGTH = 0.50
# Measured real-vehicle widths: 0.20 m central body and 0.28 m at the
# outer tyre faces.  With 0.035 m-wide tyres, their centre spacing is 0.245 m.
XYCAR_BODY_WIDTH = 0.20
XYCAR_BODY_HEIGHT = 0.12
XYCAR_TOTAL_HEIGHT = 0.25
XYCAR_WHEEL_Y = 0.1225
XYCAR_STEERING_LINK_Y = 0.1100
XYCAR_WHEEL_SEPARATION = 0.245
XYCAR_KINGPIN_WIDTH = 0.220
XYCAR_STEERING_LIMIT = 0.560
XYCAR_STEERING_JOINT_LIMIT = 0.700
XYCAR_SPEED_MIN = -4.0
XYCAR_SPEED_MAX = 8.0
XYCAR_CHASSIS_Z = 0.135
XYCAR_FRONT_WHEEL_CENTER_X = 0.16
XYCAR_FRONT_WHEEL_CENTER_Z = 0.06
XYCAR_CAMERA_FROM_FRONT_WHEEL_X = -0.04
XYCAR_CAMERA_FROM_FRONT_WHEEL_Z = 0.17
# 2026-07-12 laser overlay: camera position in laser frame
# (-0.105, 0.000, +0.090) m. Keep the measured camera origin fixed.
XYCAR_LIDAR_FROM_FRONT_WHEEL_X = 0.065
XYCAR_LIDAR_FROM_FRONT_WHEEL_Z = 0.080
XYCAR_CAMERA_X = XYCAR_FRONT_WHEEL_CENTER_X + XYCAR_CAMERA_FROM_FRONT_WHEEL_X
XYCAR_CAMERA_Z = (
    XYCAR_FRONT_WHEEL_CENTER_Z
    + XYCAR_CAMERA_FROM_FRONT_WHEEL_Z
    - XYCAR_CHASSIS_Z
)
XYCAR_LIDAR_X = XYCAR_FRONT_WHEEL_CENTER_X + XYCAR_LIDAR_FROM_FRONT_WHEEL_X
XYCAR_LIDAR_Z = (
    XYCAR_FRONT_WHEEL_CENTER_Z
    + XYCAR_LIDAR_FROM_FRONT_WHEEL_Z
    - XYCAR_CHASSIS_Z
)
XYCAR_LIDAR_RADIUS = 0.040
XYCAR_LIDAR_FRONT_OVERHANG = 0.050
XYCAR_BODY_X = (
    XYCAR_LIDAR_X
    + XYCAR_LIDAR_RADIUS
    - XYCAR_LIDAR_FRONT_OVERHANG
    - 0.5 * XYCAR_BODY_LENGTH
)
XYCAR_CAMERA_VISIBILITY_MASK = 4294967293
XYCAR_SELF_VISIBILITY_FLAGS = 2
# Effective raw-image FOV recovered from the measured OpenCV fisheye K/D.
# The lens is marketed as 170 deg, but the calibrated 1280 px image spans 102.95 deg.
XYCAR_CAMERA_WIDTH = 1280
XYCAR_CAMERA_HEIGHT = 1024
XYCAR_SIM_CAMERA_WIDTH = 320
XYCAR_SIM_CAMERA_HEIGHT = 256
XYCAR_CAMERA_RECTIFIED_FX = 560.3981298315102
XYCAR_CAMERA_HFOV = 2.0 * math.atan(
    XYCAR_CAMERA_WIDTH / (2.0 * XYCAR_CAMERA_RECTIFIED_FX)
)
XYCAR_CAMERA_PITCH = 0.187
XYCAR_LIDAR_SAMPLES = 505
XYCAR_LIDAR_RANGE_MAX = 12.0
XYCAR_SPAWN_X = -2.70
XYCAR_SPAWN_Y = 2.665
XYCAR_SPAWN_Z = 0.05
XYCAR_SPAWN_YAW = math.pi

WORLD_W = 22.50
WORLD_H = 13.50
IMG_W = 2250
IMG_H = 1350
AA = 4

COLORS = {
    "floor": (156, 157, 151, 255),
    "road": (92, 94, 91, 255),
    "lane": (242, 242, 236, 255),
    "yellow": (255, 196, 12, 255),
    "black": (7, 7, 8, 255),
    "wood": (130, 101, 66, 255),
    "wood_dark": (100, 78, 50, 255),
    "wall": (218, 218, 210, 255),
}


def line_points(p0, p1, n):
    return [
        (
            p0[0] + (p1[0] - p0[0]) * i / (n - 1),
            p0[1] + (p1[1] - p0[1]) * i / (n - 1),
        )
        for i in range(n)
    ]


def cubic_bezier(p0, p1, p2, p3, n):
    pts = []
    for i in range(n):
        t = i / (n - 1)
        u = 1.0 - t
        pts.append(
            (
                u**3 * p0[0] + 3 * u * u * t * p1[0] + 3 * u * t * t * p2[0] + t**3 * p3[0],
                u**3 * p0[1] + 3 * u * u * t * p1[1] + 3 * u * t * t * p2[1] + t**3 * p3[1],
            )
        )
    return pts


def arc_points(center, radius, a0, a1, n):
    return [
        (
            center[0] + radius * math.cos(a0 + (a1 - a0) * i / (n - 1)),
            center[1] + radius * math.sin(a0 + (a1 - a0) * i / (n - 1)),
        )
        for i in range(n)
    ]


def dxf_group_pairs(path):
    raw = path.read_text(encoding="latin1", errors="ignore").splitlines()
    pairs = []
    for idx in range(0, len(raw) - 1, 2):
        try:
            code = int(raw[idx].strip())
        except ValueError:
            continue
        pairs.append((code, raw[idx + 1].strip()))
    return pairs


def iter_dxf_entities(path):
    pairs = dxf_group_pairs(path)
    in_entities = False
    idx = 0
    while idx < len(pairs):
        code, value = pairs[idx]
        if code == 0 and value == "SECTION" and idx + 1 < len(pairs) and pairs[idx + 1] == (2, "ENTITIES"):
            in_entities = True
            idx += 2
            continue
        if in_entities and code == 0 and value == "ENDSEC":
            break
        if in_entities and code == 0 and value in {"LINE", "LWPOLYLINE", "ARC"}:
            entity_type = value
            data = []
            idx += 1
            while idx < len(pairs) and pairs[idx][0] != 0:
                data.append(pairs[idx])
                idx += 1
            yield entity_type, data
            continue
        idx += 1


def first_dxf_float(data, group_code, default=None):
    for code, value in data:
        if code == group_code:
            try:
                return float(value)
            except ValueError:
                return default
    return default


def dxf_arc_path(data):
    cx = first_dxf_float(data, 10)
    cy = first_dxf_float(data, 20)
    radius = first_dxf_float(data, 40)
    a0 = first_dxf_float(data, 50)
    a1 = first_dxf_float(data, 51)
    if None in (cx, cy, radius, a0, a1):
        return []
    if a1 < a0:
        a1 += 360.0
    samples = max(10, int(math.ceil(abs(a1 - a0) / 2.0)) + 1)
    return [
        (
            cx + radius * math.cos(math.radians(a0 + (a1 - a0) * i / (samples - 1))),
            cy + radius * math.sin(math.radians(a0 + (a1 - a0) * i / (samples - 1))),
        )
        for i in range(samples)
    ]


def dxf_bulge_segment(p0, p1, bulge):
    if abs(bulge) < 1e-9:
        return [p0, p1]

    x0, y0 = p0
    x1, y1 = p1
    dx = x1 - x0
    dy = y1 - y0
    chord = math.hypot(dx, dy)
    if chord < 1e-9:
        return [p0, p1]

    theta = 4.0 * math.atan(bulge)
    radius = chord * (1.0 + bulge * bulge) / (4.0 * abs(bulge))
    ux = dx / chord
    uy = dy / chord
    nx = -uy
    ny = ux
    midpoint = ((x0 + x1) / 2.0, (y0 + y1) / 2.0)
    center_distance = chord * (1.0 - bulge * bulge) / (4.0 * bulge)
    center = (midpoint[0] + nx * center_distance, midpoint[1] + ny * center_distance)

    a0 = math.atan2(y0 - center[1], x0 - center[0])
    a1 = math.atan2(y1 - center[1], x1 - center[0])
    if bulge > 0:
        while a1 <= a0:
            a1 += 2.0 * math.pi
    else:
        while a1 >= a0:
            a1 -= 2.0 * math.pi

    arc_len = abs(radius * theta)
    samples = max(4, int(math.ceil(arc_len / 8.0)) + 1)
    return [
        (
            center[0] + radius * math.cos(a0 + (a1 - a0) * i / (samples - 1)),
            center[1] + radius * math.sin(a0 + (a1 - a0) * i / (samples - 1)),
        )
        for i in range(samples)
    ]


def dxf_lwpolyline_path(data):
    vertices = []
    current = None
    closed = False
    for code, value in data:
        if code == 70:
            try:
                closed = bool(int(value) & 1)
            except ValueError:
                closed = False
        elif code == 10:
            if current is not None and current.get("x") is not None and current.get("y") is not None:
                vertices.append((current["x"], current["y"], current.get("bulge", 0.0)))
            try:
                current = {"x": float(value), "y": None, "bulge": 0.0}
            except ValueError:
                current = None
        elif code == 20 and current is not None:
            try:
                current["y"] = float(value)
            except ValueError:
                current["y"] = None
        elif code == 42 and current is not None:
            try:
                current["bulge"] = float(value)
            except ValueError:
                current["bulge"] = 0.0

    if current is not None and current.get("x") is not None and current.get("y") is not None:
        vertices.append((current["x"], current["y"], current.get("bulge", 0.0)))
    if len(vertices) < 2:
        return []

    segment_count = len(vertices) if closed else len(vertices) - 1
    path = []
    for idx in range(segment_count):
        x0, y0, bulge = vertices[idx]
        x1, y1, _ = vertices[(idx + 1) % len(vertices)]
        segment = dxf_bulge_segment((x0, y0), (x1, y1), bulge)
        if path:
            path.extend(segment[1:])
        else:
            path.extend(segment)
    return path


def dxf_entity_path(entity_type, data):
    if entity_type == "LINE":
        x0 = first_dxf_float(data, 10)
        y0 = first_dxf_float(data, 20)
        x1 = first_dxf_float(data, 11)
        y1 = first_dxf_float(data, 21)
        if None in (x0, y0, x1, y1):
            return []
        return [(x0, y0), (x1, y1)]
    if entity_type == "LWPOLYLINE":
        return dxf_lwpolyline_path(data)
    if entity_type == "ARC":
        return dxf_arc_path(data)
    return []


def dxf_paths(path):
    return [points for entity_type, data in iter_dxf_entities(path) if len(points := dxf_entity_path(entity_type, data)) >= 2]


def polyline_length(points):
    return sum(
        math.hypot(points[idx][0] - points[idx - 1][0], points[idx][1] - points[idx - 1][1])
        for idx in range(1, len(points))
    )


def path_bbox(points):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return min(xs), min(ys), max(xs), max(ys)


def merge_bboxes(bboxes):
    return (
        min(b[0] for b in bboxes),
        min(b[1] for b in bboxes),
        max(b[2] for b in bboxes),
        max(b[3] for b in bboxes),
    )


def bbox_gap(a, b):
    dx = max(a[0] - b[2], b[0] - a[2], 0.0)
    dy = max(a[1] - b[3], b[1] - a[3], 0.0)
    return math.hypot(dx, dy)


def select_main_dxf_paths(paths):
    if not paths:
        return []

    bboxes = [path_bbox(points) for points in paths]
    whole = merge_bboxes(bboxes)
    span = max(whole[2] - whole[0], whole[3] - whole[1], 1.0)
    gap_tol = max(5.0, span * CAD_MAIN_COMPONENT_GAP_RATIO)
    parent = list(range(len(paths)))

    def find(idx):
        while parent[idx] != idx:
            parent[idx] = parent[parent[idx]]
            idx = parent[idx]
        return idx

    def union(a, b):
        ra = find(a)
        rb = find(b)
        if ra != rb:
            parent[rb] = ra

    for i in range(len(paths)):
        for j in range(i + 1, len(paths)):
            if bbox_gap(bboxes[i], bboxes[j]) <= gap_tol:
                union(i, j)

    groups = {}
    for idx in range(len(paths)):
        groups.setdefault(find(idx), []).append(idx)

    def group_length(indices):
        return sum(polyline_length(paths[idx]) for idx in indices)

    main_indices = max(groups.values(), key=group_length)
    main_length = max(group_length(main_indices), 1e-9)
    main_bbox = merge_bboxes([bboxes[idx] for idx in main_indices])
    pad = span * 0.03
    selected = set(main_indices)
    for indices in groups.values():
        if indices is main_indices:
            continue
        component_length = group_length(indices)
        component_bbox = merge_bboxes([bboxes[idx] for idx in indices])
        inside_main_area = (
            component_bbox[0] >= main_bbox[0] - pad
            and component_bbox[1] >= main_bbox[1] - pad
            and component_bbox[2] <= main_bbox[2] + pad
            and component_bbox[3] <= main_bbox[3] + pad
        )
        if inside_main_area and component_length >= main_length * 0.02:
            selected.update(indices)

    return [paths[idx] for idx in sorted(selected)]


def metric_bbox_from_blueprint_mask(mask, width, height):
    y_idx, x_idx = np.nonzero(mask)
    if len(x_idx) == 0:
        return (-BLUEPRINT_OUTER_W / 2.0, -BLUEPRINT_OUTER_H / 2.0, BLUEPRINT_OUTER_W / 2.0, BLUEPRINT_OUTER_H / 2.0)
    sx = BLUEPRINT_OUTER_W / width
    sy = BLUEPRINT_OUTER_H / height
    x0 = x_idx.min() * sx - BLUEPRINT_OUTER_W / 2.0
    x1 = (x_idx.max() + 1) * sx - BLUEPRINT_OUTER_W / 2.0
    y0 = BLUEPRINT_OUTER_H / 2.0 - (y_idx.max() + 1) * sy
    y1 = BLUEPRINT_OUTER_H / 2.0 - y_idx.min() * sy
    return x0, y0, x1, y1


def cad_target_metric_bbox(width, height):
    reference = continuous_blueprint_road_border_mask(width, height)
    return metric_bbox_from_blueprint_mask(reference, width, height)


def remove_cad_stop_line_segments(paths):
    cleaned = []
    for points in paths:
        if len(points) < 2:
            continue

        x0, y0, x1, y1 = path_bbox(points)
        width = x1 - x0
        height = y1 - y0
        closed = math.hypot(points[0][0] - points[-1][0], points[0][1] - points[-1][1]) < 1e-6

        # The straight top / bottom CAD lanes are closed rectangular
        # polylines. Their short vertical closing edges look like stop lines
        # in Gazebo, so render only the two long horizontal lane borders.
        if closed and width > 300.0 and 50.0 < height < 120.0:
            for start, end in zip(points, points[1:]):
                dx = end[0] - start[0]
                dy = end[1] - start[1]
                if abs(dy) < 1e-6 and abs(dx) > 100.0:
                    cleaned.append([start, end])
        else:
            cleaned.append(points)
    return cleaned


def selected_cad_white_line_paths(include_stop_lines=False):
    if not CAD_WHITE_LINE_ENABLED or not CAD_WHITE_LINE_DXF.exists():
        return None

    paths = select_main_dxf_paths(dxf_paths(CAD_WHITE_LINE_DXF))
    if not paths:
        return None
    if CAD_REMOVE_STOP_LINES and not include_stop_lines:
        paths = remove_cad_stop_line_segments(paths)
    return paths


def cad_metric_paths_base(include_stop_lines=False):
    paths = selected_cad_white_line_paths(include_stop_lines=include_stop_lines)
    if not paths:
        return None

    source_bbox = merge_bboxes([path_bbox(points) for points in paths])
    src_x0, src_y0, src_x1, src_y1 = source_bbox
    target_x0, target_y0, target_x1, target_y1 = cad_target_metric_bbox(BLUEPRINT_ROI[2] - BLUEPRINT_ROI[0], BLUEPRINT_ROI[3] - BLUEPRINT_ROI[1])
    src_w = max(src_x1 - src_x0, 1e-9)
    src_h = max(src_y1 - src_y0, 1e-9)
    target_w = target_x1 - target_x0
    target_h = target_y1 - target_y0

    if CAD_USE_REAL_UNITS:
        src_cx = (src_x0 + src_x1) / 2.0
        src_cy = (src_y0 + src_y1) / 2.0
        target_cx = (target_x0 + target_x1) / 2.0
        target_cy = (target_y0 + target_y1) / 2.0

        def cad_to_metric(cad_x, cad_y):
            return (
                target_cx + (cad_x - src_cx) * CAD_UNIT_TO_M,
                target_cy + (cad_y - src_cy) * CAD_UNIT_TO_M,
            )
    elif CAD_PRESERVE_ASPECT:
        scale = min(target_w / src_w, target_h / src_h)
        fit_w = src_w * scale
        fit_h = src_h * scale
        target_x0 += (target_w - fit_w) / 2.0
        target_x1 = target_x0 + fit_w
        target_y0 += (target_h - fit_h) / 2.0
        target_y1 = target_y0 + fit_h
        sx = sy = scale

        def cad_to_metric(cad_x, cad_y):
            return target_x0 + (cad_x - src_x0) * sx, target_y0 + (cad_y - src_y0) * sy
    else:
        sx = target_w / src_w
        sy = target_h / src_h

        def cad_to_metric(cad_x, cad_y):
            return target_x0 + (cad_x - src_x0) * sx, target_y0 + (cad_y - src_y0) * sy

    return [[cad_to_metric(cad_x, cad_y) for cad_x, cad_y in points] for points in paths]


def sample_open_metric_path(points, step=0.025):
    if len(points) < 2:
        return points
    lengths = open_cumulative_lengths(points)
    total = lengths[-1]
    if total <= 1e-9:
        return points
    count = max(2, int(math.ceil(total / step)) + 1)
    return [point_at_open_polyline(points, lengths, total * idx / (count - 1)) for idx in range(count)]


def nearest_center_indices(points, centers):
    try:
        from scipy.spatial import cKDTree

        _, indices = cKDTree(np.array(centers, dtype=np.float64)).query(np.array(points, dtype=np.float64), k=1)
        return indices
    except ImportError:
        point_arr = np.array(points, dtype=np.float64)
        center_arr = np.array(centers, dtype=np.float64)
        indices = np.zeros(len(points), dtype=np.int64)
        chunk = 4096
        for start in range(0, len(points), chunk):
            end = min(len(points), start + chunk)
            delta = point_arr[start:end, None, :] - center_arr[None, :, :]
            indices[start:end] = np.argmin(np.sum(delta * delta, axis=2), axis=1)
        return indices


def expand_cad_paths_to_inner_width(metric_paths):
    if not CAD_EXPAND_TO_INNER_WIDTH or not metric_paths:
        return metric_paths

    reference_paths = cad_metric_paths_base(include_stop_lines=True)
    center_paths = cad_centerline_metric_paths(reference_paths)
    center_samples = []
    for path in center_paths:
        center_samples.extend(sample_open_metric_path(path, step=0.025))
    if not center_samples:
        return metric_paths

    target_center_offset = (CAD_TARGET_INNER_ROAD_W + ROAD_BORDER_LINE_W) / 2.0
    expanded_paths = []
    for points in metric_paths:
        if not points:
            expanded_paths.append(points)
            continue

        if len(points) == 2:
            x0, y0 = points[0]
            x1, y1 = points[1]
            horizontal = abs(y1 - y0) < 1e-6 and abs(x1 - x0) > 1e-6
            vertical = abs(x1 - x0) < 1e-6 and abs(y1 - y0) > 1e-6
            if horizontal or vertical:
                midpoint = [((x0 + x1) / 2.0, (y0 + y1) / 2.0)]
                center = center_samples[int(nearest_center_indices(midpoint, center_samples)[0])]
                vx = midpoint[0][0] - center[0]
                vy = midpoint[0][1] - center[1]
                distance = math.hypot(vx, vy)
                if distance >= 0.12:
                    corrected_distance = distance + max(-0.08, min(0.08, target_center_offset - distance))
                    if horizontal:
                        sign = 1.0 if vy >= 0.0 else -1.0
                        new_y = center[1] + sign * corrected_distance
                        expanded_paths.append([(x0, new_y), (x1, new_y)])
                    else:
                        sign = 1.0 if vx >= 0.0 else -1.0
                        new_x = center[0] + sign * corrected_distance
                        expanded_paths.append([(new_x, y0), (new_x, y1)])
                    continue

        nearest_idx = nearest_center_indices(points, center_samples)
        expanded = []
        for point, idx in zip(points, nearest_idx):
            cx, cy = center_samples[int(idx)]
            vx = point[0] - cx
            vy = point[1] - cy
            distance = math.hypot(vx, vy)
            if distance < 0.12:
                expanded.append(point)
                continue
            # Keep the correction gentle so local CAD shape details are not
            # flattened by the width normalization.
            corrected_distance = distance + max(-0.08, min(0.08, target_center_offset - distance))
            scale = corrected_distance / distance
            expanded.append((cx + vx * scale, cy + vy * scale))
        expanded_paths.append(expanded)
    return expanded_paths


def cad_metric_paths():
    paths = cad_metric_paths_base(include_stop_lines=False)
    if not paths:
        return None
    return expand_cad_paths_to_inner_width(paths)


def draw_metric_paths_on_blueprint_mask(metric_paths, width, height):
    line_px = max(1, int(round(ROAD_BORDER_LINE_W / (BLUEPRINT_OUTER_W / width))))
    mask = np.zeros((height, width), dtype=np.uint8)

    for points in metric_paths:
        coords = []
        for metric_x, metric_y in points:
            px, py = blueprint_pixel_from_metric((metric_x, metric_y), width, height)
            coords.append((int(round(px)), int(round(py))))
        if len(coords) >= 2:
            cv2.polylines(mask, [np.array(coords, dtype=np.int32)], False, 255, thickness=line_px, lineType=cv2.LINE_AA)
    return mask


def draw_metric_paths_on_texture_mask(metric_paths):
    line_px = max(1, int(round(ROAD_BORDER_LINE_W / (WORLD_W / IMG_W))))
    mask = np.zeros((IMG_H, IMG_W), dtype=np.uint8)

    for points in metric_paths:
        coords = np.array([texture_pixel_from_metric(point) for point in points], dtype=np.int32)
        if len(coords) >= 2:
            cv2.polylines(mask, [coords], False, 255, thickness=line_px, lineType=cv2.LINE_AA)
    return mask


def build_cad_white_line_mask(width=615, height=347):
    metric_paths = cad_metric_paths()
    if not metric_paths:
        return None

    mask = draw_metric_paths_on_blueprint_mask(metric_paths, width, height)

    if CAD_ADD_CENTER_DASHES:
        mask[continuous_blueprint_center_dash_mask(width, height)] = 255
    return mask > 0


def build_cad_white_line_alpha():
    metric_paths = cad_metric_paths()
    if not metric_paths:
        return None
    mask = draw_metric_paths_on_texture_mask(metric_paths)
    return Image.fromarray(mask, "L")


def metric_from_blueprint_pixel(px, py):
    bx0, by0, bx1, by1 = BLUEPRINT_ROI
    sx = BLUEPRINT_OUTER_W / (bx1 - bx0)
    sy = BLUEPRINT_OUTER_H / (by1 - by0)
    return px * sx - BLUEPRINT_OUTER_W / 2.0, BLUEPRINT_OUTER_H / 2.0 - py * sy


def blueprint_pixel_from_metric(point, width, height):
    x, y = point
    px = (x + BLUEPRINT_OUTER_W / 2.0) / BLUEPRINT_OUTER_W * width
    py = (BLUEPRINT_OUTER_H / 2.0 - y) / BLUEPRINT_OUTER_H * height
    return px, py


def right_s_points(base_x, n=180):
    pts = []
    y0 = TRACK_BOTTOM_CENTER_Y + ROAD_W / 2.0
    y1 = TRACK_TOP_CENTER_Y - ROAD_W / 2.0
    for i in range(n):
        t = i / (n - 1)
        u = t * t * (3.0 - 2.0 * t)
        wave = math.sin(RIGHT_S_FREQ * math.pi * u)
        x = base_x + (RIGHT_S_POS_AMP * wave if wave >= 0.0 else RIGHT_S_NEG_AMP * wave)
        y = y0 + (y1 - y0) * t
        pts.append((x, y))
    return pts


def legacy_raw_centerline(right_base_x):
    top_y = TRACK_TOP_CENTER_Y
    bottom_y = TRACK_BOTTOM_CENTER_Y
    left_center_x = LEFT_TURN_CENTER_X
    s_bottom_y = bottom_y + ROAD_W / 2.0
    s_top_y = top_y - ROAD_W / 2.0
    pts = []

    def extend(segment):
        pts.extend(segment[1:] if pts else segment)

    extend(line_points((left_center_x, bottom_y), (right_base_x - 0.350, bottom_y), 100))
    extend(
        cubic_bezier(
            (right_base_x - 0.350, bottom_y),
            (right_base_x + 0.350, bottom_y),
            (right_base_x + 0.500, s_bottom_y - 0.500),
            (right_base_x, s_bottom_y),
            40,
        )
    )
    extend(right_s_points(right_base_x, 180))
    extend(
        cubic_bezier(
            (right_base_x, s_top_y),
            (right_base_x + 0.500, s_top_y + 0.500),
            (right_base_x + 0.350, top_y),
            (right_base_x - 0.350, top_y),
            40,
        )
    )
    extend(line_points((right_base_x - 0.350, top_y), (left_center_x, top_y), 100))
    top_left_arc_y = top_y - LEFT_TURN_R
    bottom_left_arc_y = bottom_y + LEFT_TURN_R
    extend(arc_points((left_center_x, top_left_arc_y), LEFT_TURN_R, math.pi / 2.0, math.pi, 60))
    extend(line_points((left_center_x - LEFT_TURN_R, top_left_arc_y), (left_center_x - LEFT_TURN_R, bottom_left_arc_y), 70))
    extend(arc_points((left_center_x, bottom_left_arc_y), LEFT_TURN_R, math.pi, 3.0 * math.pi / 2.0, 60))
    return pts


def extract_blueprint_right_s_centerline():
    if not BLUEPRINT_IMAGE.exists():
        return []

    bp = Image.open(BLUEPRINT_IMAGE).convert("RGB").crop(BLUEPRINT_ROI)
    arr = np.array(bp)
    mx = arr.max(axis=2)
    mn = arr.min(axis=2)
    line_mask = (mx > 135) & ((mx - mn) < 120)

    x_scan0, x_scan1 = BLUEPRINT_RIGHT_S_SCAN_X_RANGE
    x_keep0, x_keep1 = BLUEPRINT_RIGHT_S_CENTER_X_RANGE
    y0, y1 = BLUEPRINT_RIGHT_S_Y_RANGE
    rows = []
    centers = []
    for y in range(y0, y1 + 1):
        xs = np.flatnonzero(line_mask[y, x_scan0:x_scan1]) + x_scan0
        xs = np.array([x for x in xs if x_keep0 <= x <= x_keep1], dtype=np.float64)
        if len(xs) < 2:
            continue

        # The blueprint contains text near the S-curve. Trimmed percentiles
        # keep the two road edge traces while ignoring small label fragments.
        left = np.percentile(xs, 10)
        right = np.percentile(xs, 90)
        if right - left < 10:
            continue
        rows.append(y)
        centers.append((left + right) / 2.0)

    if len(rows) < 16:
        return []

    rows = np.array(rows, dtype=np.float64)
    centers = np.array(centers, dtype=np.float64)
    smoothed = []
    for y in rows:
        near = np.abs(rows - y) <= 4
        smoothed.append(float(np.median(centers[near])))

    # Return the driving order: bottom straight into right S, then top straight.
    pts = [metric_from_blueprint_pixel(px, py) for py, px in zip(rows, smoothed)]
    return list(reversed(pts))


def smooth_right_s_centerline(points):
    if len(points) < 8:
        return points
    try:
        from scipy.interpolate import UnivariateSpline
    except ImportError:
        return points

    ys = np.array([p[1] for p in points], dtype=np.float64)
    xs = np.array([p[0] for p in points], dtype=np.float64)
    spline = UnivariateSpline(ys, xs, k=3, s=RIGHT_S_SPLINE_SMOOTHING)
    y_smooth = np.linspace(ys[0], ys[-1], RIGHT_S_SPLINE_SAMPLES)
    return [(float(spline(y)), float(y)) for y in y_smooth]


def raw_centerline_from_right_s(right_s):
    if not right_s:
        return legacy_raw_centerline(solve_right_base_x())

    top_y = TRACK_TOP_CENTER_Y
    bottom_y = TRACK_BOTTOM_CENTER_Y
    left_center_x = LEFT_TURN_CENTER_X
    pts = []

    def extend(segment):
        pts.extend(segment[1:] if pts else segment)

    extend(line_points((left_center_x, bottom_y), right_s[0], 120))
    extend(right_s)
    extend(line_points(right_s[-1], (left_center_x, top_y), 120))
    top_left_arc_y = top_y - LEFT_TURN_R
    bottom_left_arc_y = bottom_y + LEFT_TURN_R
    extend(arc_points((left_center_x, top_left_arc_y), LEFT_TURN_R, math.pi / 2.0, math.pi, 72))
    extend(line_points((left_center_x - LEFT_TURN_R, top_left_arc_y), (left_center_x - LEFT_TURN_R, bottom_left_arc_y), 90))
    extend(arc_points((left_center_x, bottom_left_arc_y), LEFT_TURN_R, math.pi, 3.0 * math.pi / 2.0, 72))
    return pts


def track_length(points):
    return sum(
        math.hypot(points[(i + 1) % len(points)][0] - points[i][0], points[(i + 1) % len(points)][1] - points[i][1])
        for i in range(len(points))
    )


def solve_right_base_x():
    lo = 6.0
    hi = 9.0
    for _ in range(60):
        mid = (lo + hi) / 2.0
        if track_length(legacy_raw_centerline(mid)) < BLUEPRINT_CENTERLINE_LEN:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2.0


def solve_right_s_scale(base_points):
    if not base_points:
        return 1.0
    anchor_x = sum(p[0] for p in base_points) / len(base_points)

    def scaled(scale):
        return [(anchor_x + (x - anchor_x) * scale, y) for x, y in base_points]

    lo = 0.50
    hi = 1.80
    for _ in range(50):
        mid = (lo + hi) / 2.0
        if track_length(raw_centerline_from_right_s(scaled(mid))) < BLUEPRINT_CENTERLINE_LEN:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2.0


def build_right_s_centerline(scaled=True):
    base_points = extract_blueprint_right_s_centerline()
    if not base_points:
        return right_s_points(solve_right_base_x())
    if not scaled:
        return base_points

    smooth_points = smooth_right_s_centerline(base_points)
    anchor_x = sum(p[0] for p in smooth_points) / len(smooth_points)
    scale = solve_right_s_scale(smooth_points)
    return [(anchor_x + (x - anchor_x) * scale, y) for x, y in smooth_points]


def build_centerline():
    return raw_centerline_from_right_s(build_right_s_centerline(scaled=True))


def cumulative_lengths(points):
    lengths = [0.0]
    for i in range(1, len(points)):
        x0, y0 = points[i - 1]
        x1, y1 = points[i]
        lengths.append(lengths[-1] + math.hypot(x1 - x0, y1 - y0))
    x0, y0 = points[-1]
    x1, y1 = points[0]
    lengths.append(lengths[-1] + math.hypot(x1 - x0, y1 - y0))
    return lengths


def point_at(points, lengths, distance):
    total = lengths[-1]
    distance %= total
    for i in range(1, len(lengths)):
        if distance <= lengths[i]:
            seg_start = lengths[i - 1]
            seg_len = max(lengths[i] - seg_start, 1e-9)
            t = (distance - seg_start) / seg_len
            p0 = points[(i - 1) % len(points)]
            p1 = points[i % len(points)]
            x = p0[0] + (p1[0] - p0[0]) * t
            y = p0[1] + (p1[1] - p0[1]) * t
            yaw = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
            return x, y, yaw
    x0, y0 = points[-1]
    x1, y1 = points[0]
    return x0, y0, math.atan2(y1 - y0, x1 - x0)


def sample_centerline(points, step=0.20):
    lengths = cumulative_lengths(points)
    total = lengths[-1]
    count = max(8, int(math.ceil(total / step)))
    return [point_at(points, lengths, total * i / count)[:2] for i in range(count)]


def pixel(point):
    x, y = point
    px = (x + WORLD_W / 2.0) / WORLD_W * IMG_W * AA
    py = (WORLD_H / 2.0 - y) / WORLD_H * IMG_H * AA
    return px, py


def draw_line(draw, points, width_m, fill):
    coords = [pixel(p) for p in points]
    width_px = max(1, int(width_m * IMG_W / WORLD_W * AA))
    draw.line(coords, fill=fill, width=width_px, joint="curve")


def draw_cv_polyline(image, points, width_m, fill, closed=True):
    arr = np.array(image)
    coords = np.array([[int(round(x)), int(round(y))] for x, y in [pixel(p) for p in points]], dtype=np.int32)
    width_px = max(1, int(width_m * IMG_W / WORLD_W * AA))
    cv2.polylines(arr, [coords], closed, fill, thickness=width_px, lineType=cv2.LINE_AA)
    return Image.fromarray(arr, "RGBA")


def draw_rotated_rect(draw, center, size, yaw, fill):
    x, y = center
    length, width = size
    c = math.cos(yaw)
    s = math.sin(yaw)
    corners = []
    for dx, dy in [(-length / 2, -width / 2), (length / 2, -width / 2), (length / 2, width / 2), (-length / 2, width / 2)]:
        corners.append(pixel((x + c * dx - s * dy, y + s * dx + c * dy)))
    draw.polygon(corners, fill=fill)


def draw_start_finish(draw):
    start_x = -3.250
    center_y = TRACK_BOTTOM_CENTER_Y
    square = 0.205
    rows = int(math.ceil(ROAD_W / square))
    for col in range(4):
        for row in range(rows):
            fill = COLORS["black"] if (col + row) % 2 == 0 else COLORS["lane"]
            x = start_x + (col - 1.5) * square
            y = center_y - ROAD_W / 2.0 + square / 2.0 + row * square
            draw_rotated_rect(draw, (x, y), (square, square), 0.0, fill)
    draw_rotated_rect(draw, (start_x + 0.560, center_y), (0.055, ROAD_W + 0.130), 0.0, COLORS["yellow"])


def add_subtle_asphalt_noise(img):
    arr = np.array(img, dtype=np.int16)
    road = (
        (np.abs(arr[..., 0] - COLORS["road"][0]) < 4)
        & (np.abs(arr[..., 1] - COLORS["road"][1]) < 4)
        & (np.abs(arr[..., 2] - COLORS["road"][2]) < 4)
    )
    rng = np.random.default_rng(42)
    noise = rng.normal(0, 4, arr.shape[:2]).astype(np.int16)
    for channel in range(3):
        arr[..., channel][road] += noise[road]
    return Image.fromarray(np.clip(arr, 0, 255).astype("uint8"), "RGBA")


def extract_blueprint_road_border_mask():
    if not BLUEPRINT_IMAGE.exists():
        return None

    bp = Image.open(BLUEPRINT_IMAGE).convert("RGB").crop(BLUEPRINT_ROI)
    arr = np.array(bp)
    mx = arr.max(axis=2)
    mn = arr.min(axis=2)
    mask = (mx > 135) & ((mx - mn) < 120)

    # Keep only the road boundary area from the blueprint crop. This removes
    # wall/furniture strips, dimension text, and the table hatch pattern.
    mask[:22, :] = False
    mask[340:, :] = False
    mask[:, :18] = False
    mask[:, 572:] = False
    mask[70:304, 80:476] = False

    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype("uint8"), 8)
    border = np.zeros_like(mask)
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        elongated = w >= 24 or h >= 24
        sizeable = area >= 45
        top_fixture = y < 28 and h < 10
        tiny_label = area < 65 and w < 18 and h < 18
        if elongated and sizeable and not top_fixture and not tiny_label:
            border[labels == i] = True

    return border


def continuous_blueprint_road_border_mask(width=615, height=347):
    center = build_centerline()
    coords = np.array(
        [[int(round(x)), int(round(y))] for x, y in [blueprint_pixel_from_metric(p, width, height) for p in center]],
        dtype=np.int32,
    )
    sx = BLUEPRINT_OUTER_W / width
    road_px = max(1, int(round(ROAD_W / sx)))
    line_px = max(1, int(round(ROAD_BORDER_LINE_W / sx)))

    wide = np.zeros((height, width), dtype=np.uint8)
    road = np.zeros((height, width), dtype=np.uint8)
    cv2.polylines(wide, [coords], True, 255, thickness=road_px + 2 * line_px, lineType=cv2.LINE_AA)
    cv2.polylines(road, [coords], True, 255, thickness=road_px, lineType=cv2.LINE_AA)
    return (wide > 64) & (road < 192)


def continuous_blueprint_center_dash_mask(width=615, height=347):
    center = build_centerline()
    lengths = cumulative_lengths(center)
    sx = BLUEPRINT_OUTER_W / width
    line_px = max(1, int(round(LANE_LINE_W / sx)))
    mask = np.zeros((height, width), dtype=np.uint8)

    s = CENTER_DASH_START
    while s < lengths[-1]:
        p0 = point_at(center, lengths, s)[:2]
        p1 = point_at(center, lengths, min(s + CENTER_DASH_LEN, lengths[-1]))[:2]
        x0, y0 = blueprint_pixel_from_metric(p0, width, height)
        x1, y1 = blueprint_pixel_from_metric(p1, width, height)
        cv2.line(
            mask,
            (int(round(x0)), int(round(y0))),
            (int(round(x1)), int(round(y1))),
            255,
            thickness=line_px,
            lineType=cv2.LINE_AA,
        )
        s += CENTER_DASH_LEN + CENTER_DASH_GAP

    return mask > 0


def scaled_blueprint_region(width, height, x_range, y_range):
    base_w = BLUEPRINT_ROI[2] - BLUEPRINT_ROI[0]
    base_h = BLUEPRINT_ROI[3] - BLUEPRINT_ROI[1]
    x0 = int(round(x_range[0] / base_w * width))
    x1 = int(round(x_range[1] / base_w * width))
    y0 = int(round(y_range[0] / base_h * height))
    y1 = int(round(y_range[1] / base_h * height))
    region = np.zeros((height, width), dtype=bool)
    region[max(0, y0):min(height, y1), max(0, x0):min(width, x1)] = True
    return region


def right_s_blueprint_region(width=615, height=347):
    return scaled_blueprint_region(
        width,
        height,
        BLUEPRINT_RIGHT_S_REPLACE_X_RANGE,
        BLUEPRINT_RIGHT_S_REPLACE_Y_RANGE,
    )


def right_s_center_x_by_row(width=615, height=347):
    center = build_right_s_centerline(scaled=True)
    coords = np.array([blueprint_pixel_from_metric(p, width, height) for p in center], dtype=np.float64)
    order = np.argsort(coords[:, 1])
    ys = coords[order, 1]
    xs = coords[order, 0]
    return np.interp(np.arange(height, dtype=np.float64), ys, xs, left=xs[0], right=xs[-1])


def shift_mask_x(mask, offset_px):
    shifted = np.zeros_like(mask)
    if offset_px > 0:
        shifted[:, offset_px:] = mask[:, :-offset_px]
    elif offset_px < 0:
        shifted[:, :offset_px] = mask[:, -offset_px:]
    else:
        shifted[:] = mask
    return shifted


def extract_right_s_blueprint_white_lines(raw_white_mask):
    height, width = raw_white_mask.shape
    region = right_s_blueprint_region(width, height)
    candidate = raw_white_mask & region

    n, labels, stats, _ = cv2.connectedComponentsWithStats(candidate.astype("uint8"), 8)
    curves = np.zeros_like(candidate)
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        long_fragment = max(w, h) >= 10 and area >= 8
        compact_text = w <= 18 and h <= 12 and area <= 45
        if long_fragment and not compact_text:
            curves[labels == i] = True

    center_dash = continuous_blueprint_center_dash_mask(width, height)
    dash_dist = cv2.distanceTransform((~center_dash).astype("uint8"), cv2.DIST_L2, 3)
    tol = max(2, int(round(BLUEPRINT_RIGHT_S_CENTER_DASH_TOL_PX * width / (BLUEPRINT_ROI[2] - BLUEPRINT_ROI[0]))))
    dashes = candidate & (dash_dist <= tol)

    combined = (curves | dashes).astype("uint8") * 255
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel, iterations=1)
    return (combined > 0) & region


def extract_right_s_shifted_side_white_lines(raw_white_mask):
    original = extract_right_s_blueprint_white_lines(raw_white_mask)
    height, width = original.shape
    sx = BLUEPRINT_OUTER_W / width
    offset_px = max(1, int(round(RIGHT_S_SIDE_LINE_OFFSET_M / sx)))
    center_band_px = max(1, int(round(RIGHT_S_CENTER_COMPONENT_BAND_M / sx)))
    center_x = right_s_center_x_by_row(width, height)

    n, labels, stats, centroids = cv2.connectedComponentsWithStats(original.astype("uint8"), 8)
    left_side = np.zeros_like(original)
    center_line = np.zeros_like(original)
    right_side = np.zeros_like(original)
    for i in range(1, n):
        area = stats[i, cv2.CC_STAT_AREA]
        if area <= 0:
            continue
        x = stats[i, cv2.CC_STAT_LEFT]
        y = stats[i, cv2.CC_STAT_TOP]
        w = stats[i, cv2.CC_STAT_WIDTH]
        h = stats[i, cv2.CC_STAT_HEIGHT]
        cx, cy = centroids[i]
        row = int(np.clip(round(cy), 0, height - 1))
        dx = cx - center_x[row]
        component = labels == i
        side_shaped = max(w, h) >= 24 and area >= 30
        if dx < -center_band_px or (side_shaped and dx < 0.0):
            left_side[component] = True
        elif dx > center_band_px or (side_shaped and dx >= 0.0):
            right_side[component] = True
        else:
            center_line[component] = True

    shifted_sides = shift_mask_x(left_side, -offset_px) | shift_mask_x(right_side, offset_px)
    return center_line | shifted_sides


def extend_inner_straights_to_right_s(clean_mask, right_s_lines):
    height, width = clean_mask.shape
    sx = BLUEPRINT_OUTER_W / width
    line_px = max(1, int(round(ROAD_BORDER_LINE_W / sx)))
    base_w = BLUEPRINT_ROI[2] - BLUEPRINT_ROI[0]
    x_start = int(round(BLUEPRINT_RIGHT_S_REPLACE_X_RANGE[0] / base_w * width))
    x_start = max(0, min(width - 1, x_start - 2 * line_px))
    search_y = max(2, int(round(RIGHT_S_INNER_STRAIGHT_CONNECT_SEARCH_Y_PX * width / base_w)))

    extended = clean_mask.astype("uint8") * 255
    inner_edges = [
        TRACK_TOP_CENTER_Y - ROAD_W / 2.0,
        TRACK_BOTTOM_CENTER_Y + ROAD_W / 2.0,
    ]
    for y_m in inner_edges:
        _, y_px = blueprint_pixel_from_metric((0.0, y_m), width, height)
        y = int(round(y_px))
        y0 = max(0, y - search_y)
        y1 = min(height, y + search_y + 1)
        band = right_s_lines[y0:y1, :]
        _, xs = np.nonzero(band)
        xs = xs[xs >= x_start]
        if len(xs) == 0:
            continue
        x_end = int(xs.min()) + 2 * line_px
        cv2.line(
            extended,
            (x_start, y),
            (min(width - 1, x_end), y),
            255,
            thickness=line_px,
            lineType=cv2.LINE_AA,
        )
    return extended > 0


def blueprint_road_corridor_mask(width=615, height=347, padding_m=0.25):
    center = build_centerline()
    coords = np.array(
        [[int(round(x)), int(round(y))] for x, y in [blueprint_pixel_from_metric(p, width, height) for p in center]],
        dtype=np.int32,
    )
    sx = BLUEPRINT_OUTER_W / width
    corridor_px = max(1, int(round((ROAD_W + 2.0 * padding_m) / sx)))
    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.polylines(mask, [coords], True, 255, thickness=corridor_px, lineType=cv2.LINE_AA)
    return mask > 0


def extract_blueprint_white_line_mask(clean=True):
    if not BLUEPRINT_IMAGE.exists():
        return build_cad_white_line_mask() if clean else None

    bp = Image.open(BLUEPRINT_IMAGE).convert("RGB").crop(BLUEPRINT_ROI)

    if clean:
        cad_mask = build_cad_white_line_mask(bp.width, bp.height)
        if cad_mask is not None:
            return cad_mask

    arr = np.array(bp)
    mx = arr.max(axis=2)
    mn = arr.min(axis=2)
    mask = (mx > 135) & ((mx - mn) < 120)

    if not clean:
        return mask

    # The raw blueprint uses white for dimensions, walls, and furniture as
    # well as lane paint. Rebuild the paint from the blueprint-fitted
    # centerline so the edge lines stay continuous and the center line keeps
    # a regular dash rhythm instead of inheriting broken scan fragments.
    border = continuous_blueprint_road_border_mask(bp.width, bp.height)
    center_dash = continuous_blueprint_center_dash_mask(bp.width, bp.height)
    clean_mask = border | center_dash

    # In the right S section, keep the blueprint-extracted shape, but push only
    # the side boundary lines outward. The center dash stays fixed.
    right_s_region = right_s_blueprint_region(bp.width, bp.height)
    right_s_lines = extract_right_s_shifted_side_white_lines(mask)
    clean_mask[right_s_region] = False
    clean_mask |= right_s_lines
    clean_mask = extend_inner_straights_to_right_s(clean_mask, right_s_lines)
    return clean_mask


def metric_frame_bounds_px():
    frame_x0 = int(round(((-BLUEPRINT_OUTER_W / 2.0) + WORLD_W / 2.0) / WORLD_W * IMG_W))
    frame_x1 = int(round(((BLUEPRINT_OUTER_W / 2.0) + WORLD_W / 2.0) / WORLD_W * IMG_W))
    frame_y0 = int(round((WORLD_H / 2.0 - BLUEPRINT_OUTER_H / 2.0) / WORLD_H * IMG_H))
    frame_y1 = int(round((WORLD_H / 2.0 + BLUEPRINT_OUTER_H / 2.0) / WORLD_H * IMG_H))
    return frame_x0, frame_y0, frame_x1, frame_y1


def build_white_line_alpha():
    cad_alpha = build_cad_white_line_alpha()
    if cad_alpha is not None:
        return cad_alpha

    mask = extract_blueprint_white_line_mask(clean=True)
    alpha = Image.new("L", (IMG_W, IMG_H), 0)
    if mask is None:
        return alpha

    frame_x0, frame_y0, frame_x1, frame_y1 = metric_frame_bounds_px()
    resample = getattr(Image, "Resampling", Image).LANCZOS
    mask_img = Image.fromarray((mask * 255).astype("uint8"))
    mask_img = mask_img.resize((frame_x1 - frame_x0, frame_y1 - frame_y0), resample)
    alpha.paste(mask_img, (frame_x0, frame_y0))
    return alpha


def build_plain_road_texture():
    return Image.new("RGB", (IMG_W, IMG_H), COLORS["road"][:3])


def build_white_line_overlay_texture(alpha=None):
    if alpha is None:
        alpha = build_white_line_alpha()
    if isinstance(alpha, np.ndarray):
        alpha_arr = alpha.astype(np.uint8)
    else:
        alpha_arr = np.array(alpha, dtype=np.uint8)
    height, width = alpha_arr.shape
    arr = np.zeros((height, width, 4), dtype=np.uint8)
    arr[..., 0] = COLORS["lane"][0]
    arr[..., 1] = COLORS["lane"][1]
    arr[..., 2] = COLORS["lane"][2]
    arr[..., 3] = alpha_arr
    return Image.fromarray(arr, "RGBA")


def texture_pixel_from_metric(point):
    x, y = point
    px = int(round((x + WORLD_W / 2.0) / WORLD_W * IMG_W))
    py = int(round((WORLD_H / 2.0 - y) / WORLD_H * IMG_H))
    return px, py


def texture_corridor_mask(points, width_m):
    mask = np.zeros((IMG_H, IMG_W), dtype=np.uint8)
    coords = np.array([texture_pixel_from_metric(p) for p in points], dtype=np.int32)
    width_px = max(1, int(round(width_m * IMG_W / WORLD_W)))
    cv2.polylines(mask, [coords], False, 255, thickness=width_px, lineType=cv2.LINE_AA)
    return mask > 0


def texture_metric_from_pixel(px, py):
    x = ((px + 0.5) / IMG_W) * WORLD_W - WORLD_W / 2.0
    y = WORLD_H / 2.0 - ((py + 0.5) / IMG_H) * WORLD_H
    return x, y


def shifted_neighbors(binary):
    p2 = np.zeros_like(binary)
    p3 = np.zeros_like(binary)
    p4 = np.zeros_like(binary)
    p5 = np.zeros_like(binary)
    p6 = np.zeros_like(binary)
    p7 = np.zeros_like(binary)
    p8 = np.zeros_like(binary)
    p9 = np.zeros_like(binary)
    p2[1:, :] = binary[:-1, :]
    p3[1:, :-1] = binary[:-1, 1:]
    p4[:, :-1] = binary[:, 1:]
    p5[:-1, :-1] = binary[1:, 1:]
    p6[:-1, :] = binary[1:, :]
    p7[:-1, 1:] = binary[1:, :-1]
    p8[:, 1:] = binary[:, :-1]
    p9[1:, 1:] = binary[:-1, :-1]
    return p2, p3, p4, p5, p6, p7, p8, p9


def zhang_suen_thinning(binary):
    img = (binary > 0).astype(np.uint8)
    if img.shape[0] < 3 or img.shape[1] < 3:
        return img

    changed = True
    iterations = 0
    while changed and iterations < 200:
        changed = False
        iterations += 1
        for step in (0, 1):
            p2, p3, p4, p5, p6, p7, p8, p9 = shifted_neighbors(img)
            neighbors = [p2, p3, p4, p5, p6, p7, p8, p9]
            b = sum(neighbors)
            a = np.zeros_like(img)
            for idx in range(8):
                a += ((neighbors[idx] == 0) & (neighbors[(idx + 1) % 8] == 1)).astype(np.uint8)

            if step == 0:
                preserve = (p2 * p4 * p6 == 0) & (p4 * p6 * p8 == 0)
            else:
                preserve = (p2 * p4 * p8 == 0) & (p2 * p6 * p8 == 0)
            remove = (img == 1) & (b >= 2) & (b <= 6) & (a == 1) & preserve
            remove[[0, -1], :] = False
            remove[:, [0, -1]] = False
            if np.any(remove):
                img[remove] = 0
                changed = True
    return img


def largest_skeleton_component(skeleton):
    count, labels, stats, _ = cv2.connectedComponentsWithStats(skeleton.astype("uint8"), 8)
    if count <= 1:
        return skeleton.astype(bool)
    best = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return labels == best


def ordered_skeleton_path(skeleton):
    rows, cols = np.nonzero(skeleton)
    if len(rows) < 2:
        return []

    points = list(zip(rows.tolist(), cols.tolist()))
    index = {point: idx for idx, point in enumerate(points)}
    neighbor_offsets = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
    adjacency = [[] for _ in points]
    for idx, (row, col) in enumerate(points):
        for dy, dx in neighbor_offsets:
            neighbor = (row + dy, col + dx)
            if neighbor in index:
                weight = math.sqrt(2.0) if dy != 0 and dx != 0 else 1.0
                adjacency[idx].append((index[neighbor], weight))

    endpoints = [idx for idx, edges in enumerate(adjacency) if len(edges) <= 1]
    start = endpoints[0] if endpoints else 0

    def dijkstra(source):
        dist = [math.inf] * len(points)
        prev = [-1] * len(points)
        dist[source] = 0.0
        heap = [(0.0, source)]
        while heap:
            distance, node = heapq.heappop(heap)
            if distance > dist[node]:
                continue
            for neighbor, weight in adjacency[node]:
                candidate = distance + weight
                if candidate < dist[neighbor]:
                    dist[neighbor] = candidate
                    prev[neighbor] = node
                    heapq.heappush(heap, (candidate, neighbor))
        return dist, prev

    dist, _ = dijkstra(start)
    candidates = endpoints if endpoints else range(len(points))
    source = max(candidates, key=lambda idx: dist[idx] if math.isfinite(dist[idx]) else -1.0)
    dist, prev = dijkstra(source)
    candidates = endpoints if endpoints else range(len(points))
    target = max(candidates, key=lambda idx: dist[idx] if math.isfinite(dist[idx]) else -1.0)

    ordered = []
    node = target
    while node != -1:
        ordered.append(points[node])
        if node == source:
            break
        node = prev[node]
    ordered.reverse()
    return ordered


def smooth_metric_path(points, window=9):
    if len(points) < window:
        return points
    arr = np.array(points, dtype=np.float64)
    half = window // 2
    padded = np.pad(arr, ((half, half), (0, 0)), mode="edge")
    kernel = np.ones(window, dtype=np.float64) / window
    smoothed = np.column_stack(
        [
            np.convolve(padded[:, 0], kernel, mode="valid"),
            np.convolve(padded[:, 1], kernel, mode="valid"),
        ]
    )
    smoothed[0] = arr[0]
    smoothed[-1] = arr[-1]
    return [tuple(point) for point in smoothed]


def open_cumulative_lengths(points):
    lengths = [0.0]
    for idx in range(1, len(points)):
        x0, y0 = points[idx - 1]
        x1, y1 = points[idx]
        lengths.append(lengths[-1] + math.hypot(x1 - x0, y1 - y0))
    return lengths


def point_at_open_polyline(points, lengths, distance):
    if not points:
        return (0.0, 0.0)
    if distance <= 0.0:
        return points[0]
    if distance >= lengths[-1]:
        return points[-1]
    for idx in range(1, len(lengths)):
        if distance <= lengths[idx]:
            seg_start = lengths[idx - 1]
            seg_len = max(lengths[idx] - seg_start, 1e-9)
            t = (distance - seg_start) / seg_len
            x0, y0 = points[idx - 1]
            x1, y1 = points[idx]
            return x0 + (x1 - x0) * t, y0 + (y1 - y0) * t
    return points[-1]


def cad_road_area_component_masks(metric_paths=None):
    if metric_paths is None:
        metric_paths = cad_metric_paths()
    if not metric_paths:
        return []

    line = draw_metric_paths_on_texture_mask(metric_paths)
    close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    boundary = cv2.morphologyEx(line, cv2.MORPH_CLOSE, close_kernel, iterations=1)

    outside = ((boundary == 0).astype(np.uint8)) * 255
    flood_mask = np.zeros((IMG_H + 2, IMG_W + 2), np.uint8)
    cv2.floodFill(outside, flood_mask, (0, 0), 128)
    enclosed = outside == 255
    road = enclosed & (boundary == 0)

    count, labels, stats, _ = cv2.connectedComponentsWithStats(road.astype("uint8"), 8)
    if count <= 1:
        return []

    areas = stats[1:, cv2.CC_STAT_AREA]
    largest_area = int(areas.max())
    min_area = max(300, int(0.015 * largest_area))
    component_masks = []
    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area < min_area:
            continue
        if largest_area > 0 and area > largest_area * YELLOW_CENTER_ROAD_AREA_MAX_RATIO:
            continue
        component_masks.append(labels == label)
    return component_masks


def cad_centerline_metric_paths(metric_paths=None):
    paths = []
    for road_mask in cad_road_area_component_masks(metric_paths):
        x, y, w, h = cv2.boundingRect(road_mask.astype(np.uint8))
        pad = 4
        x0 = max(0, x - pad)
        y0 = max(0, y - pad)
        x1 = min(IMG_W, x + w + pad)
        y1 = min(IMG_H, y + h + pad)
        crop = road_mask[y0:y1, x0:x1]
        skeleton = zhang_suen_thinning(crop)
        skeleton = largest_skeleton_component(skeleton)
        ordered = ordered_skeleton_path(skeleton)
        if len(ordered) < 2:
            continue
        metric_path = [texture_metric_from_pixel(col + x0, row + y0) for row, col in ordered]
        metric_path = smooth_metric_path(metric_path)
        if open_cumulative_lengths(metric_path)[-1] >= YELLOW_CENTER_DASH_LEN * 0.75:
            paths.append(metric_path)
    return paths


def draw_dashed_metric_path(mask, points, dash_len, gap, width_m):
    if len(points) < 2:
        return
    lengths = open_cumulative_lengths(points)
    total = lengths[-1]
    if total <= 1e-6:
        return

    width_px = max(1, int(round(width_m / (WORLD_W / IMG_W))))
    sample_step = max(0.025, width_m * 0.75)
    distance = 0.0
    while distance < total:
        end = min(distance + dash_len, total)
        sample_count = max(2, int(math.ceil((end - distance) / sample_step)) + 1)
        dash_points = [
            point_at_open_polyline(points, lengths, distance + (end - distance) * idx / (sample_count - 1))
            for idx in range(sample_count)
        ]
        coords = np.array([texture_pixel_from_metric(point) for point in dash_points], dtype=np.int32)
        cv2.polylines(mask, [coords], False, 255, thickness=width_px, lineType=cv2.LINE_AA)
        distance += dash_len + gap


def center_path_endpoint_connectors(center_paths):
    endpoints = []
    for path_idx, path in enumerate(center_paths):
        if len(path) >= 2:
            endpoints.append((path_idx, 0, path[0]))
            endpoints.append((path_idx, 1, path[-1]))

    candidates = []
    for idx, first in enumerate(endpoints):
        for second in endpoints[idx + 1:]:
            if first[0] == second[0]:
                continue
            dx = second[2][0] - first[2][0]
            dy = second[2][1] - first[2][1]
            distance = math.hypot(dx, dy)
            if 0.05 < distance <= YELLOW_CENTER_CONNECT_MAX_GAP:
                candidates.append((distance, first, second))

    connectors = []
    used = set()
    for _, first, second in sorted(candidates, key=lambda item: item[0]):
        first_key = (first[0], first[1])
        second_key = (second[0], second[1])
        if first_key in used or second_key in used:
            continue
        connectors.append((first[2], second[2]))
        used.add(first_key)
        used.add(second_key)
    return connectors


def horizontal_centerline_y(center_paths, upper=True):
    candidates = []
    for path in center_paths:
        if len(path) < 2:
            continue
        xs = [point[0] for point in path]
        ys = [point[1] for point in path]
        if max(xs) - min(xs) < 1.0:
            continue
        if max(ys) - min(ys) > 0.08:
            continue
        mean_y = sum(ys) / len(ys)
        if (upper and mean_y > 0.0) or ((not upper) and mean_y < 0.0):
            candidates.append(mean_y)
    if not candidates:
        return None
    return max(candidates) if upper else min(candidates)


def interpolate_to_y(p0, p1, target_y):
    dy = p1[1] - p0[1]
    if abs(dy) < 1e-9:
        return p0
    t = (target_y - p0[1]) / dy
    t = max(0.0, min(1.0, t))
    return p0[0] + (p1[0] - p0[0]) * t, target_y


def trim_curve_centerline_endpoints(center_paths):
    top_y = horizontal_centerline_y(center_paths, upper=True)
    bottom_y = horizontal_centerline_y(center_paths, upper=False)
    adjusted = []
    for path in center_paths:
        if len(path) < 2:
            adjusted.append(path)
            continue

        xs = [point[0] for point in path]
        ys = [point[1] for point in path]
        horizontal_straight = (max(xs) - min(xs) > 1.0) and (max(ys) - min(ys) <= 0.08)
        if horizontal_straight:
            adjusted.append(path)
            continue

        trimmed = list(path)
        if top_y is not None and trimmed[-1][1] > top_y + 0.01:
            for idx in range(len(trimmed) - 2, -1, -1):
                if trimmed[idx][1] <= top_y:
                    crossing = interpolate_to_y(trimmed[idx], trimmed[idx + 1], top_y)
                    trimmed = trimmed[: idx + 1] + [crossing]
                    break

        if bottom_y is not None and trimmed[0][1] < bottom_y - 0.01:
            for idx in range(1, len(trimmed)):
                if trimmed[idx][1] >= bottom_y:
                    crossing = interpolate_to_y(trimmed[idx - 1], trimmed[idx], bottom_y)
                    trimmed = [crossing] + trimmed[idx:]
                    break

        adjusted.append(trimmed)
    return adjusted


def is_horizontal_centerline_path(points):
    if len(points) < 2:
        return False
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return (max(xs) - min(xs) > 1.0) and (max(ys) - min(ys) <= 0.08)


def curve_centerline_tangent(points, idx):
    prev = np.array(points[max(0, idx - 2)], dtype=np.float64)
    nxt = np.array(points[min(len(points) - 1, idx + 2)], dtype=np.float64)
    tangent = nxt - prev
    norm = np.linalg.norm(tangent)
    if norm <= 1e-9:
        return None
    return tangent / norm


def sampled_white_line_points(metric_paths):
    samples = []
    for path in metric_paths or []:
        samples.extend(sample_open_metric_path(path, step=YELLOW_CENTER_CURVE_SAMPLE_STEP))
    if not samples:
        return np.empty((0, 2), dtype=np.float64)
    return np.array(samples, dtype=np.float64)


def candidate_boundary_indices(boundary_points, tree, point):
    if tree is not None:
        indices = tree.query_ball_point(point, r=YELLOW_CENTER_CURVE_SEARCH_RADIUS)
        if len(indices) >= 2:
            return np.array(indices, dtype=np.int64)
    return np.arange(len(boundary_points), dtype=np.int64)


def corrected_curve_center_point(point, tangent, boundary_points, tree):
    candidate_idx = candidate_boundary_indices(boundary_points, tree, point)
    if len(candidate_idx) < 2:
        return tuple(point)

    normal = np.array([-tangent[1], tangent[0]], dtype=np.float64)
    candidates = boundary_points[candidate_idx]
    rel = candidates - point
    tangent_distance = rel @ tangent
    normal_distance = rel @ normal

    expected_offset = (CAD_TARGET_INNER_ROAD_W + ROAD_BORDER_LINE_W) / 2.0
    valid = (
        (np.abs(tangent_distance) <= YELLOW_CENTER_CURVE_TANGENT_TOL)
        & (np.abs(normal_distance) >= 0.12)
        & (np.abs(normal_distance) <= 0.90)
    )
    if not np.any(valid):
        return tuple(point)

    valid_idx = np.nonzero(valid)[0]
    plus = valid_idx[normal_distance[valid_idx] > 0.0]
    minus = valid_idx[normal_distance[valid_idx] < 0.0]
    if len(plus) == 0 or len(minus) == 0:
        return tuple(point)

    def choose(side_indices):
        score = np.abs(tangent_distance[side_indices]) * 5.0
        score += np.abs(np.abs(normal_distance[side_indices]) - expected_offset) * 0.8
        return side_indices[int(np.argmin(score))]

    plus_idx = choose(plus)
    minus_idx = choose(minus)
    tangent_shift = (tangent_distance[plus_idx] + tangent_distance[minus_idx]) / 2.0
    normal_shift = (normal_distance[plus_idx] + normal_distance[minus_idx]) / 2.0
    corrected = point + tangent * tangent_shift + normal * normal_shift

    delta = corrected - point
    delta_norm = np.linalg.norm(delta)
    max_shift = 0.25
    if delta_norm > max_shift:
        corrected = point + delta / delta_norm * max_shift
    return tuple(corrected)


def local_curve_centerline_correction_weight(point, top_y, bottom_y):
    x, y = point
    weight = 0.0
    if top_y is not None and x < -3.0:
        weight = max(weight, max(0.0, min(1.0, (y - (top_y - 0.55)) / 0.35)))
    if bottom_y is not None and x > 2.0:
        weight = max(weight, max(0.0, min(1.0, ((bottom_y + 0.55) - y) / 0.35)))
    return weight


def correct_curve_centerlines_to_white_midline(center_paths):
    boundary_paths = cad_metric_paths()
    boundary_points = sampled_white_line_points(boundary_paths)
    if len(boundary_points) < 2:
        return center_paths

    try:
        from scipy.spatial import cKDTree

        tree = cKDTree(boundary_points)
    except ImportError:
        tree = None

    top_y = horizontal_centerline_y(center_paths, upper=True)
    bottom_y = horizontal_centerline_y(center_paths, upper=False)
    corrected_paths = []
    for path in center_paths:
        if len(path) < 3 or is_horizontal_centerline_path(path):
            corrected_paths.append(path)
            continue

        corrected = []
        for idx, raw_point in enumerate(path):
            point = np.array(raw_point, dtype=np.float64)
            weight = local_curve_centerline_correction_weight(point, top_y, bottom_y)
            if weight <= 0.0:
                corrected.append(raw_point)
                continue
            tangent = curve_centerline_tangent(path, idx)
            if tangent is None:
                corrected.append(raw_point)
                continue
            adjusted = np.array(corrected_curve_center_point(point, tangent, boundary_points, tree), dtype=np.float64)
            blended = point + (adjusted - point) * weight
            corrected.append(tuple(blended))
        corrected_paths.append(corrected)
    return corrected_paths


def adjusted_connector_dash_points(p0, p1, center_paths):
    mid_x = (p0[0] + p1[0]) / 2.0
    mid_y = (p0[1] + p1[1]) / 2.0

    # At the removed stop-line gaps, the endpoint-to-endpoint connector can
    # lean toward a curved section. Keep these short dashes centered between
    # the straight white lane borders instead.
    if mid_x < -2.8 and mid_y > 2.2:
        y = horizontal_centerline_y(center_paths, upper=True)
        if y is not None:
            return (mid_x - YELLOW_CENTER_DASH_LEN / 2.0, y), (mid_x + YELLOW_CENTER_DASH_LEN / 2.0, y)
    if mid_x > 2.0 and mid_y < -3.0:
        y = horizontal_centerline_y(center_paths, upper=False)
        if y is not None:
            return (mid_x - YELLOW_CENTER_DASH_LEN / 2.0, y), (mid_x + YELLOW_CENTER_DASH_LEN / 2.0, y)
    return p0, p1


def draw_centered_connector_dash(mask, p0, p1):
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    length = math.hypot(dx, dy)
    if length <= 1e-6:
        return

    ux = dx / length
    uy = dy / length
    dash_len = min(YELLOW_CENTER_DASH_LEN, length)
    mid = ((p0[0] + p1[0]) / 2.0, (p0[1] + p1[1]) / 2.0)
    dash = [
        (mid[0] - ux * dash_len / 2.0, mid[1] - uy * dash_len / 2.0),
        (mid[0] + ux * dash_len / 2.0, mid[1] + uy * dash_len / 2.0),
    ]
    width_px = max(1, int(round(YELLOW_CENTER_LINE_W / (WORLD_W / IMG_W))))
    coords = np.array([texture_pixel_from_metric(point) for point in dash], dtype=np.int32)
    cv2.polylines(mask, [coords], False, 255, thickness=width_px, lineType=cv2.LINE_AA)


def build_yellow_centerline_alpha():
    reference_paths = cad_metric_paths_base(include_stop_lines=True)
    center_paths = cad_centerline_metric_paths(reference_paths)
    center_paths = trim_curve_centerline_endpoints(center_paths)
    center_paths = correct_curve_centerlines_to_white_midline(center_paths)
    alpha = np.zeros((IMG_H, IMG_W), dtype=np.uint8)
    for path in center_paths:
        draw_dashed_metric_path(
            alpha,
            path,
            YELLOW_CENTER_DASH_LEN,
            YELLOW_CENTER_DASH_GAP,
            YELLOW_CENTER_LINE_W,
        )
    for p0, p1 in center_path_endpoint_connectors(center_paths):
        p0, p1 = adjusted_connector_dash_points(p0, p1, center_paths)
        draw_centered_connector_dash(alpha, p0, p1)
    return Image.fromarray(alpha, "L")


def build_yellow_centerline_overlay_texture(alpha=None):
    if alpha is None:
        alpha = build_yellow_centerline_alpha()
    alpha_arr = np.array(alpha, dtype=np.uint8)
    height, width = alpha_arr.shape
    arr = np.zeros((height, width, 4), dtype=np.uint8)
    arr[..., 0] = COLORS["yellow"][0]
    arr[..., 1] = COLORS["yellow"][1]
    arr[..., 2] = COLORS["yellow"][2]
    arr[..., 3] = alpha_arr
    return Image.fromarray(arr, "RGBA")


def curve_piece_selection_masks():
    curve_width = ROAD_W + 2.0 * RIGHT_S_SIDE_LINE_OFFSET_M + 0.450
    masks = [
        (
            "curve_left_top",
            texture_corridor_mask(
                arc_points(
                    (LEFT_TURN_CENTER_X, TRACK_TOP_CENTER_Y - LEFT_TURN_R),
                    LEFT_TURN_R,
                    math.pi / 2.0,
                    math.pi,
                    120,
                ),
                curve_width,
            ),
        ),
        (
            "curve_left_bottom",
            texture_corridor_mask(
                arc_points(
                    (LEFT_TURN_CENTER_X, TRACK_BOTTOM_CENTER_Y + LEFT_TURN_R),
                    LEFT_TURN_R,
                    math.pi,
                    3.0 * math.pi / 2.0,
                    120,
                ),
                curve_width,
            ),
        ),
    ]

    right_s = build_right_s_centerline(scaled=True)
    for idx in range(5):
        start = max(0, int(round(len(right_s) * idx / 5.0)) - 2)
        end = min(len(right_s), int(round(len(right_s) * (idx + 1) / 5.0)) + 2)
        masks.append((f"curve_right_s_{idx + 1:02d}", texture_corridor_mask(right_s[start:end], curve_width)))
    return masks


def metric_points_from_texture_pixels(x_idx, y_idx):
    x = ((x_idx.astype(np.float64) + 0.5) / IMG_W) * WORLD_W - WORLD_W / 2.0
    y = WORLD_H / 2.0 - ((y_idx.astype(np.float64) + 0.5) / IMG_H) * WORLD_H
    return np.column_stack([x, y])


def classify_white_line_sides(line_pixels):
    side_masks = {name: np.zeros((IMG_H, IMG_W), dtype=bool) for name in WHITE_LINE_SIDE_NAMES}
    y_idx, x_idx = np.nonzero(line_pixels)
    if len(x_idx) == 0:
        return side_masks

    samples = np.array(sample_centerline(build_centerline(), step=0.035), dtype=np.float64)
    prev_samples = np.roll(samples, 1, axis=0)
    next_samples = np.roll(samples, -1, axis=0)
    tangents = next_samples - prev_samples
    tangent_norm = np.linalg.norm(tangents, axis=1)
    tangents /= np.maximum(tangent_norm[:, None], 1e-9)

    metric_points = metric_points_from_texture_pixels(x_idx, y_idx)
    try:
        from scipy.spatial import cKDTree

        _, nearest_idx = cKDTree(samples).query(metric_points, k=1)
    except ImportError:
        nearest_idx = np.zeros(len(metric_points), dtype=np.int64)
        chunk = 4096
        for start in range(0, len(metric_points), chunk):
            end = min(len(metric_points), start + chunk)
            delta = metric_points[start:end, None, :] - samples[None, :, :]
            nearest_idx[start:end] = np.argmin(np.sum(delta * delta, axis=2), axis=1)

    nearest = samples[nearest_idx]
    tangent = tangents[nearest_idx]
    offset = metric_points - nearest
    signed = tangent[:, 0] * offset[:, 1] - tangent[:, 1] * offset[:, 0]

    left = signed > WHITE_LINE_CENTER_SPLIT_BAND_M
    right = signed < -WHITE_LINE_CENTER_SPLIT_BAND_M
    center = ~(left | right)
    side_masks["left"][y_idx[left], x_idx[left]] = True
    side_masks["right"][y_idx[right], x_idx[right]] = True
    side_masks["center"][y_idx[center], x_idx[center]] = True
    return side_masks


def split_cad_white_line_alpha():
    metric_paths = cad_metric_paths()
    if not metric_paths:
        return None

    line_px = max(1, int(round(ROAD_BORDER_LINE_W / (WORLD_W / IMG_W))))
    parts = []
    for idx, points in enumerate(metric_paths, start=1):
        mask = np.zeros((IMG_H, IMG_W), dtype=np.uint8)
        coords = np.array([texture_pixel_from_metric(point) for point in points], dtype=np.int32)
        if len(coords) >= 2:
            cv2.polylines(mask, [coords], False, 255, thickness=line_px, lineType=cv2.LINE_AA)
        if np.any(mask):
            parts.append((f"white_line_cad_{idx:02d}", mask.astype(np.uint8)))
    return parts


def split_white_line_alpha(alpha):
    cad_parts = split_cad_white_line_alpha()
    if cad_parts is not None:
        return cad_parts

    alpha_arr = np.array(alpha, dtype=np.uint8)
    line_pixels = alpha_arr > 0
    claimed = np.zeros((IMG_H, IMG_W), dtype=bool)
    side_masks = classify_white_line_sides(line_pixels)
    base_part_masks = []
    mask_by_name = dict(curve_piece_selection_masks())
    for name in WHITE_LINE_BASE_PARTS:
        if name == "straight":
            continue
        piece_mask = line_pixels & mask_by_name[name] & ~claimed
        claimed |= piece_mask
        base_part_masks.append((name, piece_mask))

    straight_mask = line_pixels & ~claimed
    base_part_masks.insert(0, ("straight", straight_mask))

    parts = []
    for base_name, base_mask in base_part_masks:
        for side_name in WHITE_LINE_SIDE_NAMES:
            piece_mask = base_mask & side_masks[side_name]
            if piece_mask.any():
                name = f"white_line_{base_name}_{side_name}"
                parts.append((name, np.where(piece_mask, alpha_arr, 0).astype(np.uint8)))
    return parts


def white_line_texture_path(name):
    return TEXTURE_DIR / f"kookmin_{name}.png"


def yellow_centerline_texture_path(name):
    return TEXTURE_DIR / f"kookmin_{name}.png"


def repo_relative_path(path):
    path = Path(path)
    if path.is_absolute():
        return path.resolve().relative_to(ROOT).as_posix()
    return path.as_posix()


def repo_path(path):
    path = Path(path)
    return path if path.is_absolute() else ROOT / path


def gz_resource_uri(path):
    return f"file://{repo_relative_path(path)}"


def texture_bounds_to_world_pose(x0, y0, x1, y1):
    center_x_px = (x0 + x1) / 2.0
    center_y_px = (y0 + y1) / 2.0
    x = center_x_px / IMG_W * WORLD_W - WORLD_W / 2.0
    y = WORLD_H / 2.0 - center_y_px / IMG_H * WORLD_H
    size_x = (x1 - x0) / IMG_W * WORLD_W
    size_y = (y1 - y0) / IMG_H * WORLD_H
    return (x, y), (size_x, size_y)


def save_cropped_white_line_part(name, alpha_arr, padding_px=4):
    alpha_img = Image.fromarray(alpha_arr, "L")
    bbox = alpha_img.getbbox()
    if bbox is None:
        return None

    x0, y0, x1, y1 = bbox
    x0 = max(0, x0 - padding_px)
    y0 = max(0, y0 - padding_px)
    x1 = min(IMG_W, x1 + padding_px)
    y1 = min(IMG_H, y1 + padding_px)
    cropped_alpha = alpha_arr[y0:y1, x0:x1]
    path = white_line_texture_path(name)
    build_white_line_overlay_texture(cropped_alpha).save(path)
    pose_xy, size_xy = texture_bounds_to_world_pose(x0, y0, x1, y1)
    return {
        "name": name,
        "texture": repo_relative_path(path),
        "pose_xy": [pose_xy[0], pose_xy[1]],
        "size_xy": [size_xy[0], size_xy[1]],
        "bbox_px": [x0, y0, x1, y1],
    }


def split_yellow_centerline_alpha(alpha):
    alpha_arr = np.array(alpha, dtype=np.uint8)
    binary = alpha_arr > 0
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(binary.astype("uint8"), 8)
    parts = []
    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area < 4:
            continue
        left = int(stats[label, cv2.CC_STAT_LEFT])
        top = int(stats[label, cv2.CC_STAT_TOP])
        width = int(stats[label, cv2.CC_STAT_WIDTH])
        height = int(stats[label, cv2.CC_STAT_HEIGHT])
        cx, cy = centroids[label]
        component_alpha = np.where(labels == label, alpha_arr, 0).astype(np.uint8)
        parts.append((top, left, cy, cx, area, width, height, component_alpha))

    parts.sort(key=lambda item: (item[0], item[1], item[2], item[3]))
    return [(f"yellow_centerline_dash_{idx:03d}", part[-1]) for idx, part in enumerate(parts, start=1)]


def save_cropped_yellow_centerline_part(name, alpha_arr, padding_px=4):
    alpha_img = Image.fromarray(alpha_arr, "L")
    bbox = alpha_img.getbbox()
    if bbox is None:
        return None

    x0, y0, x1, y1 = bbox
    x0 = max(0, x0 - padding_px)
    y0 = max(0, y0 - padding_px)
    x1 = min(IMG_W, x1 + padding_px)
    y1 = min(IMG_H, y1 + padding_px)
    cropped_alpha = alpha_arr[y0:y1, x0:x1]
    path = yellow_centerline_texture_path(name)
    build_yellow_centerline_overlay_texture(cropped_alpha).save(path)
    pose_xy, size_xy = texture_bounds_to_world_pose(x0, y0, x1, y1)
    return {
        "name": name,
        "texture": repo_relative_path(path),
        "pose_xy": [pose_xy[0], pose_xy[1]],
        "size_xy": [size_xy[0], size_xy[1]],
        "bbox_px": [x0, y0, x1, y1],
    }


def composite_road_and_lines(road, lines):
    composed = road.convert("RGBA")
    composed.alpha_composite(lines)
    return composed.convert("RGB")


def build_blueprint_white_texture():
    road = build_plain_road_texture()
    lines = build_white_line_overlay_texture()
    return composite_road_and_lines(road, lines)


def build_track_texture():
    road = build_plain_road_texture()
    alpha = build_white_line_alpha()
    lines = build_white_line_overlay_texture(alpha)
    yellow_alpha = build_yellow_centerline_alpha()
    yellow_lines = build_yellow_centerline_overlay_texture(yellow_alpha)
    line_parts = split_white_line_alpha(alpha)
    yellow_parts = split_yellow_centerline_alpha(yellow_alpha)
    img = composite_road_and_lines(road, lines)
    img = composite_road_and_lines(img, yellow_lines)

    TEXTURE.parent.mkdir(parents=True, exist_ok=True)
    road.save(ROAD_TEXTURE)
    lines.save(WHITE_LINE_TEXTURE)
    yellow_lines.save(YELLOW_CENTERLINE_TEXTURE)
    metadata = []
    for name, part_alpha in line_parts:
        spec = save_cropped_white_line_part(name, part_alpha)
        if spec is not None:
            metadata.append(spec)
    WHITE_LINE_PARTS_METADATA.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    yellow_metadata = []
    for name, part_alpha in yellow_parts:
        spec = save_cropped_yellow_centerline_part(name, part_alpha)
        if spec is not None:
            yellow_metadata.append(spec)
    YELLOW_CENTERLINE_PARTS_METADATA.write_text(json.dumps(yellow_metadata, indent=2), encoding="utf-8")
    img.save(TEXTURE)
    img.save(PREVIEW)
    img.save(PREVIEW_CLEAN)


def rgb(color):
    return f"{color[0] / 255.0:.4f} {color[1] / 255.0:.4f} {color[2] / 255.0:.4f} 1"


def material_xml(color):
    c = rgb(color)
    return f"<material><ambient>{c}</ambient><diffuse>{c}</diffuse></material>"


def box_link(name, pose, size, color, collision=False):
    x, y, z, roll, pitch, yaw = pose
    sx, sy, sz = size
    collision_xml = ""
    if collision:
        collision_xml = f"""
        <collision name="collision">
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
          <surface>
            <friction>
              <ode><mu>50</mu><mu2>50</mu2></ode>
              <bullet><friction>1</friction><rolling_friction>0.1</rolling_friction></bullet>
            </friction>
          </surface>
        </collision>"""
    return f"""
      <link name="{name}">
        <pose>{x:.4f} {y:.4f} {z:.4f} {roll:.6f} {pitch:.6f} {yaw:.6f}</pose>{collision_xml}
        <visual name="visual">
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
          {material_xml(color)}
        </visual>
      </link>"""


def segment_link(name, p0, p1, width, height, z, color, offset=0.0, collision=False, overlap=0.035):
    x0, y0 = p0
    x1, y1 = p1
    dx = x1 - x0
    dy = y1 - y0
    length = math.hypot(dx, dy)
    if length < 1e-4:
        return ""
    nx = -dy / length
    ny = dx / length
    cx = (x0 + x1) / 2.0 + nx * offset
    cy = (y0 + y1) / 2.0 + ny * offset
    yaw = math.atan2(dy, dx)
    return box_link(name, (cx, cy, z, 0, 0, yaw), (length + overlap, width, height), color, collision=collision)


def polyline_links(points, prefix, width, height, z, color, offset=0.0, collision=False, overlap=0.035):
    links = []
    for i in range(len(points)):
        p0 = points[i]
        p1 = points[(i + 1) % len(points)]
        link = segment_link(f"{prefix}_{i:03d}", p0, p1, width, height, z, color, offset=offset, collision=collision, overlap=overlap)
        if link:
            links.append(link)
    return "".join(links)


def dashed_centerline_links(points):
    lengths = cumulative_lengths(points)
    links = []
    dash_len = 0.420
    gap = 0.460
    s = 0.180
    idx = 0
    while s < lengths[-1]:
        p0 = point_at(points, lengths, s)
        p1 = point_at(points, lengths, min(s + dash_len, lengths[-1]))
        links.append(
            segment_link(
                f"center_dash_{idx:03d}",
                (p0[0], p0[1]),
                (p1[0], p1[1]),
                LANE_LINE_W,
                0.008,
                0.024,
                COLORS["lane"],
                overlap=0.0,
            )
        )
        s += dash_len + gap
        idx += 1
    return "".join(links)


def start_finish_links():
    links = []
    start_x = -3.250
    center_y = TRACK_BOTTOM_CENTER_Y
    square = 0.205
    rows = int(math.ceil(ROAD_W / square))
    idx = 0
    for col in range(4):
        for row in range(rows):
            color = COLORS["black"] if (col + row) % 2 == 0 else COLORS["lane"]
            x = start_x + (col - 1.5) * square
            y = center_y - ROAD_W / 2.0 + square / 2.0 + row * square
            links.append(box_link(f"checker_{idx:02d}", (x, y, 0.030, 0, 0, 0), (square, square, 0.010), color))
            idx += 1
    links.append(box_link("yellow_start_reference", (start_x + 0.560, center_y, 0.031, 0, 0, 0), (0.055, ROAD_W + 0.130, 0.011), COLORS["yellow"]))
    return "".join(links)


def room_reference_links():
    links = [
        box_link("floor_base", (0, 0, -0.015, 0, 0, 0), (WORLD_W, WORLD_H, 0.030), COLORS["floor"], collision=True),
        box_link("muted_wood_infield", (-0.450, 0, 0.003, 0, 0, 0), (12.250, 6.300, 0.006), COLORS["wood"]),
    ]
    return "".join(links)


def room_context_object_model(name, pose, size, color):
    x, y, z, roll, pitch, yaw = pose
    sx, sy, sz = size
    return f"""
    <model name="room_{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} {roll:.6f} {pitch:.6f} {yaw:.6f}</pose>
      <link name="{name}">
        <visual name="visual">
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
          {material_xml(color)}
        </visual>
      </link>
    </model>"""


def room_context_models():
    wall = (220, 222, 214, 255)
    locker = (48, 50, 50, 255)
    table = (126, 98, 63, 255)
    chair = (24, 25, 26, 255)
    light = (245, 245, 235, 255)
    cone = (220, 80, 24, 255)

    objects = [
        ("north_wall", (0, 6.70, 0.90, 0, 0, 0), (22.3, 0.06, 1.70), wall),
        ("south_wall", (0, -6.70, 0.90, 0, 0, 0), (22.3, 0.06, 1.70), wall),
        ("east_wall", (11.10, 0, 0.90, 0, 0, 0), (0.06, 13.2, 1.70), wall),
        ("west_wall", (-11.10, 0, 0.90, 0, 0, 0), (0.06, 13.2, 1.70), wall),
        ("north_wood_band", (0, 6.64, 0.20, 0, 0, 0), (22.1, 0.08, 0.40), COLORS["wood_dark"]),
        ("south_wood_band", (0, -6.64, 0.20, 0, 0, 0), (22.1, 0.08, 0.40), COLORS["wood_dark"]),
        ("locker_bank", (7.4, 5.95, 0.70, 0, 0, 0), (4.0, 0.18, 1.15), locker),
        ("black_display_stand", (-7.0, 4.90, 0.75, 0, 0, 0), (1.4, 0.18, 1.10), locker),
        ("orange_cone_reference", (6.4, -3.7, 0.18, 0, 0, 0), (0.18, 0.18, 0.36), cone),
    ]

    for idx, x in enumerate([-5.4, -3.6, -1.8, 0.0, 1.8]):
        objects.append((f"table_top_{idx}", (x, 1.0, 0.43, 0, 0, 0), (1.30, 0.62, 0.06), table))
        objects.append((f"chair_back_{idx}", (x, 1.43, 0.42, 0, 0, 0), (0.55, 0.08, 0.52), chair))
        objects.append((f"chair_seat_{idx}", (x, 1.34, 0.26, 0, 0, 0), (0.48, 0.40, 0.08), chair))

    for idx, x in enumerate([-8.0, -5.7, -3.4, -1.1, 1.2, 3.5, 5.8, 8.1]):
        yaw = 0.2 if idx % 2 == 0 else -0.15
        objects.append((f"ceiling_light_{idx}", (x, 0.5 + (idx % 3) * 1.7, 2.15, 0, 0, yaw), (1.10, 0.055, 0.035), light))

    return "".join(room_context_object_model(name, pose, size, color) for name, pose, size, color in objects)


def track_model():
    texture_uri = gz_resource_uri(ROAD_TEXTURE)
    return f"""
    <model name="kookmin_track">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      {box_link("floor_base", (0, 0, -0.015, 0, 0, 0), (WORLD_W, WORLD_H, 0.030), COLORS["floor"], collision=True)}
      <link name="track_texture_surface">
        <pose>0 0 0.006 0 0 0</pose>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>{WORLD_W:.4f} {WORLD_H:.4f}</size></plane></geometry>
          <material>
            <ambient>1 1 1 1</ambient>
            <diffuse>1 1 1 1</diffuse>
            <pbr><metal><albedo_map>{texture_uri}</albedo_map><roughness>1</roughness><metalness>0</metalness></metal></pbr>
          </material>
        </visual>
      </link>
    </model>"""


def white_line_overlay_model(spec, z):
    name = spec["name"]
    texture_path = repo_path(spec["texture"])
    texture_uri = gz_resource_uri(texture_path)
    x, y = spec["pose_xy"]
    size_x, size_y = spec["size_xy"]
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
      <link name="{name}_surface">
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>{size_x:.4f} {size_y:.4f}</size></plane></geometry>
          <material>
            <ambient>1 1 1 1</ambient>
            <diffuse>1 1 1 1</diffuse>
            <emissive>0.9490 0.9490 0.9255 1</emissive>
            <pbr><metal><albedo_map>{texture_uri}</albedo_map><roughness>1</roughness><metalness>0</metalness></metal></pbr>
          </material>
        </visual>
      </link>
    </model>"""


def white_line_overlay_models():
    if not WHITE_LINE_PARTS_METADATA.exists():
        return ""
    specs = json.loads(WHITE_LINE_PARTS_METADATA.read_text(encoding="utf-8"))
    models = []
    for idx, spec in enumerate(specs):
        models.append(white_line_overlay_model(spec, 0.0140 + idx * 0.0002))
    return "".join(models)


def yellow_centerline_overlay_model(spec, z):
    name = spec["name"]
    texture_path = repo_path(spec["texture"])
    texture_uri = gz_resource_uri(texture_path)
    x, y = spec["pose_xy"]
    size_x, size_y = spec["size_xy"]
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
      <link name="{name}_surface">
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>{size_x:.4f} {size_y:.4f}</size></plane></geometry>
          <material>
            <ambient>1 1 1 1</ambient>
            <diffuse>1 1 1 1</diffuse>
            <emissive>1.0000 0.7686 0.0471 1</emissive>
            <pbr><metal><albedo_map>{texture_uri}</albedo_map><roughness>1</roughness><metalness>0</metalness></metal></pbr>
          </material>
        </visual>
      </link>
    </model>"""


def yellow_centerline_overlay_models():
    if not YELLOW_CENTERLINE_PARTS_METADATA.exists():
        return ""
    specs = json.loads(YELLOW_CENTERLINE_PARTS_METADATA.read_text(encoding="utf-8"))
    models = []
    for idx, spec in enumerate(specs):
        models.append(yellow_centerline_overlay_model(spec, 0.0265 + idx * 0.0002))
    return "".join(models)


def short_centerline_white_model(name, x, y, yaw=0.0):
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} 0.0300 0 0 {yaw:.6f}</pose>
      <link name="{name}_surface">
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry><box><size>{CENTER_DASH_LEN:.4f} {LANE_LINE_W:.4f} 0.0100</size></box></geometry>
          {material_xml(COLORS["lane"])}
        </visual>
      </link>
    </model>"""


def short_centerline_white_models():
    if cad_metric_paths():
        return ""
    return "".join(
        [
            short_centerline_white_model("short_centerline_white_01", -0.300, 0.300),
            short_centerline_white_model("short_centerline_white_02", 0.300, 0.300),
        ]
    )


def vehicle_box_visual(name, pose, size, color):
    """Return a non-colliding box visual attached to the chassis link."""
    x, y, z, roll, pitch, yaw = pose
    sx, sy, sz = size
    return f"""
        <visual name="{name}">
          <visibility_flags>{XYCAR_SELF_VISIBILITY_FLAGS}</visibility_flags>
          <pose>{x:.4f} {y:.4f} {z:.4f} {roll:.5f} {pitch:.5f} {yaw:.5f}</pose>
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
          {material_xml(color)}
        </visual>"""


def vehicle_model():
    cyan = (48, 169, 211, 255)
    pink = (221, 42, 112, 255)
    red = (196, 30, 54, 255)
    yellow = (244, 218, 38, 255)
    shell_black = (22, 25, 34, 255)
    aluminium = (172, 178, 182, 255)
    dark_metal = (54, 60, 65, 255)
    shell_visuals = "".join(
        [
            # Narrow exposed chassis visible in the first reference photo.
            vehicle_box_visual(
                "chassis_plate",
                (XYCAR_BODY_X, 0.0, -0.045, 0.0, 0.0, 0.0),
                (XYCAR_BODY_LENGTH, XYCAR_BODY_WIDTH, 0.025),
                dark_metal,
            ),
            vehicle_box_visual(
                "electronics_lower_deck",
                (-0.035, 0.0, 0.025, 0.0, 0.0, 0.0),
                (0.25, 0.17, 0.018),
                aluminium,
            ),
            # The installed shell: wide front / rear decks and an open centre.
            vehicle_box_visual(
                "shell_front_hood",
                (0.135, 0.0, 0.058, 0.0, -0.055, 0.0),
                (0.160, 0.270, 0.065),
                cyan,
            ),
            vehicle_box_visual(
                "shell_front_nose",
                (0.207, 0.0, 0.035, 0.0, -0.10, 0.0),
                (0.016, 0.280, 0.060),
                pink,
            ),
            vehicle_box_visual(
                "shell_rear_deck",
                (-0.205, 0.0, 0.055, 0.0, 0.050, 0.0),
                (0.160, 0.270, 0.065),
                cyan,
            ),
            vehicle_box_visual(
                "shell_rear_bumper",
                (-0.277, 0.0, 0.020, 0.0, 0.08, 0.0),
                (0.016, 0.250, 0.050),
                shell_black,
            ),
            vehicle_box_visual(
                "shell_left_centre_rail",
                (-0.035, 0.120, 0.030, 0.0, 0.0, 0.0),
                (0.180, 0.040, 0.070),
                cyan,
            ),
            vehicle_box_visual(
                "shell_right_centre_rail",
                (-0.035, -0.120, 0.030, 0.0, 0.0, 0.0),
                (0.180, 0.040, 0.070),
                cyan,
            ),
            # Pink fender caps reproduce the shell surrounding all four tyres.
            *[
                vehicle_box_visual(
                    f"shell_fender_{axle}_{side}",
                    (x, y, 0.077, 0.0, 0.0, 0.0),
                    (0.085, 0.055, 0.018),
                    pink,
                )
                for axle, x in (("front", 0.160), ("rear", -0.205))
                for side, y in (("left", 0.1125), ("right", -0.1125))
            ],
            # Black window / vent panels around the open electronics bay.
            vehicle_box_visual(
                "shell_front_window",
                (0.070, 0.0, 0.093, 0.0, -0.055, 0.0),
                (0.055, 0.155, 0.008),
                shell_black,
            ),
            vehicle_box_visual(
                "shell_rear_window",
                (-0.135, 0.0, 0.091, 0.0, 0.050, 0.0),
                (0.045, 0.150, 0.008),
                shell_black,
            ),
            # Approximate the red, black and yellow wrap graphics in the photo.
            vehicle_box_visual(
                "shell_front_red_stripe",
                (0.145, 0.0, 0.095, 0.0, -0.055, 0.55),
                (0.170, 0.014, 0.006),
                red,
            ),
            vehicle_box_visual(
                "shell_front_black_stripe",
                (0.145, 0.0, 0.097, 0.0, -0.055, -0.48),
                (0.165, 0.012, 0.006),
                shell_black,
            ),
            vehicle_box_visual(
                "shell_rear_red_stripe",
                (-0.210, 0.0, 0.094, 0.0, 0.050, -0.52),
                (0.165, 0.014, 0.006),
                red,
            ),
            vehicle_box_visual(
                "shell_rear_yellow_stripe",
                (-0.210, 0.0, 0.096, 0.0, 0.050, 0.48),
                (0.150, 0.010, 0.006),
                yellow,
            ),
            # Exposed computer and camera hardware remain visible through the cut-out.
            vehicle_box_visual(
                "mini_pc_body",
                (-0.020, 0.0, 0.115, 0.0, 0.0, 0.0),
                (0.170, 0.130, 0.065),
                shell_black,
            ),
            vehicle_box_visual(
                "mini_pc_top_plate",
                (-0.020, 0.0, 0.151, 0.0, 0.0, 0.0),
                (0.180, 0.140, 0.008),
                aluminium,
            ),
            vehicle_box_visual(
                "front_camera_body",
                (XYCAR_CAMERA_X, 0.0, XYCAR_CAMERA_Z, 0.0, XYCAR_CAMERA_PITCH, 0.0),
                (0.045, 0.055, 0.040),
                shell_black,
            ),
        ]
    )
    return f"""
    <model name="xycar_ackermann">
      <pose>{XYCAR_SPAWN_X:.4f} {XYCAR_SPAWN_Y:.4f} {XYCAR_SPAWN_Z:.4f} 0 0 {XYCAR_SPAWN_YAW:.4f}</pose>
      <link name="chassis">
        <pose>0 0 {XYCAR_CHASSIS_Z:.3f} 0 0 0</pose>
        <inertial>
          <pose>{XYCAR_BODY_X:.3f} 0 0 0 0 0</pose>
          <mass>2.2</mass>
          <inertia><ixx>0.055</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.13</iyy><iyz>0</iyz><izz>0.16</izz></inertia>
        </inertial>
        <collision name="collision"><pose>{XYCAR_BODY_X:.3f} 0 0 0 0 0</pose><geometry><box><size>{XYCAR_BODY_LENGTH:.3f} {XYCAR_BODY_WIDTH:.3f} {XYCAR_BODY_HEIGHT:.3f}</size></box></geometry></collision>
        {shell_visuals}
        <visual name="lidar_body">
          <pose>{XYCAR_LIDAR_X:.3f} 0 {XYCAR_LIDAR_Z - 0.0175:.4f} 0 0 0</pose>
          <geometry><cylinder><length>0.035</length><radius>{XYCAR_LIDAR_RADIUS:.3f}</radius></cylinder></geometry>
          <material><ambient>0.01 0.01 0.01 1</ambient><diffuse>0.01 0.01 0.01 1</diffuse></material>
        </visual>
        <collision name="lidar_collision">
          <pose>{XYCAR_LIDAR_X:.3f} 0 {XYCAR_LIDAR_Z - 0.0175:.4f} 0 0 0</pose>
          <geometry><cylinder><length>0.035</length><radius>{XYCAR_LIDAR_RADIUS:.3f}</radius></cylinder></geometry>
        </collision>
        <sensor name="front_camera" type="camera">
          <pose>{XYCAR_CAMERA_X:.3f} 0 {XYCAR_CAMERA_Z:.3f} 0 {XYCAR_CAMERA_PITCH:.4f} 0</pose>
          <topic>/image_raw</topic>
          <update_rate>14</update_rate>
          <camera>
            <camera_info_topic>/camera_info</camera_info_topic>
            <horizontal_fov>{XYCAR_CAMERA_HFOV:.4f}</horizontal_fov>
            <image><width>{XYCAR_SIM_CAMERA_WIDTH}</width><height>{XYCAR_SIM_CAMERA_HEIGHT}</height><format>R8G8B8</format></image>
            <clip><near>0.03</near><far>20</far></clip>
            <visibility_mask>{XYCAR_CAMERA_VISIBILITY_MASK}</visibility_mask>
            <lens>
              <type>gnomonical</type>
              <scale_to_hfov>true</scale_to_hfov>
              <cutoff_angle>1.5707963267948966</cutoff_angle>
              <env_texture_size>512</env_texture_size>
            </lens>
          </camera>
          <always_on>true</always_on>
          <visualize>false</visualize>
        </sensor>
        <sensor name="lidar" type="gpu_lidar">
          <pose>{XYCAR_LIDAR_X:.3f} 0 {XYCAR_LIDAR_Z:.3f} 0 0 0</pose>
          <topic>/scan</topic>
          <update_rate>10</update_rate>
          <ray>
            <scan>
              <horizontal>
                <samples>{XYCAR_LIDAR_SAMPLES}</samples>
                <resolution>1</resolution>
                <min_angle>-3.14159</min_angle>
                <max_angle>3.14159</max_angle>
              </horizontal>
            </scan>
            <range><min>0.10</min><max>{XYCAR_LIDAR_RANGE_MAX:.1f}</max><resolution>0.01</resolution></range>
          </ray>
          <always_on>true</always_on>
          <visualize>false</visualize>
        </sensor>
      </link>
      <link name="front_left_wheel_steering_link">
        <pose>0.16 {XYCAR_STEERING_LINK_Y:.4f} 0.06 0 0 0</pose>
        <inertial><mass>0.08</mass><inertia><ixx>0.0002</ixx><iyy>0.0002</iyy><izz>0.0002</izz></inertia></inertial>
      </link>
      <link name="front_right_wheel_steering_link">
        <pose>0.16 -{XYCAR_STEERING_LINK_Y:.4f} 0.06 0 0 0</pose>
        <inertial><mass>0.08</mass><inertia><ixx>0.0002</ixx><iyy>0.0002</iyy><izz>0.0002</izz></inertia></inertial>
      </link>
      <link name="front_left_wheel">
        <pose>0.16 {XYCAR_WHEEL_Y:.4f} 0.06 -1.5707 0 0</pose>
        <inertial><mass>0.12</mass><inertia><ixx>0.0004</ixx><iyy>0.0004</iyy><izz>0.00025</izz></inertia></inertial>
        <collision name="collision"><geometry><cylinder><length>0.035</length><radius>0.06</radius></cylinder></geometry></collision>
        <visual name="visual"><visibility_flags>{XYCAR_SELF_VISIBILITY_FLAGS}</visibility_flags><geometry><cylinder><length>0.035</length><radius>0.06</radius></cylinder></geometry><material><ambient>0.03 0.03 0.03 1</ambient><diffuse>0.03 0.03 0.03 1</diffuse></material></visual>
      </link>
      <link name="front_right_wheel">
        <pose>0.16 -{XYCAR_WHEEL_Y:.4f} 0.06 -1.5707 0 0</pose>
        <inertial><mass>0.12</mass><inertia><ixx>0.0004</ixx><iyy>0.0004</iyy><izz>0.00025</izz></inertia></inertial>
        <collision name="collision"><geometry><cylinder><length>0.035</length><radius>0.06</radius></cylinder></geometry></collision>
        <visual name="visual"><visibility_flags>{XYCAR_SELF_VISIBILITY_FLAGS}</visibility_flags><geometry><cylinder><length>0.035</length><radius>0.06</radius></cylinder></geometry><material><ambient>0.03 0.03 0.03 1</ambient><diffuse>0.03 0.03 0.03 1</diffuse></material></visual>
      </link>
      <link name="rear_left_wheel">
        <pose>-0.16 {XYCAR_WHEEL_Y:.4f} 0.06 -1.5707 0 0</pose>
        <inertial><mass>0.12</mass><inertia><ixx>0.0004</ixx><iyy>0.0004</iyy><izz>0.00025</izz></inertia></inertial>
        <collision name="collision"><geometry><cylinder><length>0.035</length><radius>0.06</radius></cylinder></geometry></collision>
        <visual name="visual"><visibility_flags>{XYCAR_SELF_VISIBILITY_FLAGS}</visibility_flags><geometry><cylinder><length>0.035</length><radius>0.06</radius></cylinder></geometry><material><ambient>0.03 0.03 0.03 1</ambient><diffuse>0.03 0.03 0.03 1</diffuse></material></visual>
      </link>
      <link name="rear_right_wheel">
        <pose>-0.16 -{XYCAR_WHEEL_Y:.4f} 0.06 -1.5707 0 0</pose>
        <inertial><mass>0.12</mass><inertia><ixx>0.0004</ixx><iyy>0.0004</iyy><izz>0.00025</izz></inertia></inertial>
        <collision name="collision"><geometry><cylinder><length>0.035</length><radius>0.06</radius></cylinder></geometry></collision>
        <visual name="visual"><visibility_flags>{XYCAR_SELF_VISIBILITY_FLAGS}</visibility_flags><geometry><cylinder><length>0.035</length><radius>0.06</radius></cylinder></geometry><material><ambient>0.03 0.03 0.03 1</ambient><diffuse>0.03 0.03 0.03 1</diffuse></material></visual>
      </link>
      <joint name="front_left_wheel_steering_joint" type="revolute">
        <parent>chassis</parent><child>front_left_wheel_steering_link</child>
        <axis><xyz>0 0 1</xyz><limit><lower>-{XYCAR_STEERING_JOINT_LIMIT:.4f}</lower><upper>{XYCAR_STEERING_JOINT_LIMIT:.4f}</upper><velocity>2.0</velocity><effort>8</effort></limit></axis>
      </joint>
      <joint name="front_right_wheel_steering_joint" type="revolute">
        <parent>chassis</parent><child>front_right_wheel_steering_link</child>
        <axis><xyz>0 0 1</xyz><limit><lower>-{XYCAR_STEERING_JOINT_LIMIT:.4f}</lower><upper>{XYCAR_STEERING_JOINT_LIMIT:.4f}</upper><velocity>2.0</velocity><effort>8</effort></limit></axis>
      </joint>
      <joint name="front_left_wheel_joint" type="revolute">
        <parent>front_left_wheel_steering_link</parent><child>front_left_wheel</child>
        <axis><xyz>0 0 1</xyz><limit><lower>-1.79769e+308</lower><upper>1.79769e+308</upper></limit></axis>
      </joint>
      <joint name="front_right_wheel_joint" type="revolute">
        <parent>front_right_wheel_steering_link</parent><child>front_right_wheel</child>
        <axis><xyz>0 0 1</xyz><limit><lower>-1.79769e+308</lower><upper>1.79769e+308</upper></limit></axis>
      </joint>
      <joint name="rear_left_wheel_joint" type="revolute">
        <parent>chassis</parent><child>rear_left_wheel</child>
        <axis><xyz>0 0 1</xyz><limit><lower>-1.79769e+308</lower><upper>1.79769e+308</upper></limit></axis>
      </joint>
      <joint name="rear_right_wheel_joint" type="revolute">
        <parent>chassis</parent><child>rear_right_wheel</child>
        <axis><xyz>0 0 1</xyz><limit><lower>-1.79769e+308</lower><upper>1.79769e+308</upper></limit></axis>
      </joint>
      <plugin filename="gz-sim-ackermann-steering-system" name="gz::sim::systems::AckermannSteering">
        <left_joint>front_left_wheel_joint</left_joint>
        <left_joint>rear_left_wheel_joint</left_joint>
        <right_joint>front_right_wheel_joint</right_joint>
        <right_joint>rear_right_wheel_joint</right_joint>
        <left_steering_joint>front_left_wheel_steering_joint</left_steering_joint>
        <right_steering_joint>front_right_wheel_steering_joint</right_steering_joint>
        <kingpin_width>{XYCAR_KINGPIN_WIDTH:.3f}</kingpin_width>
        <steering_limit>{XYCAR_STEERING_LIMIT:.4f}</steering_limit>
        <wheel_base>0.32</wheel_base>
        <wheel_separation>{XYCAR_WHEEL_SEPARATION:.3f}</wheel_separation>
        <wheel_radius>0.06</wheel_radius>
        <min_velocity>{XYCAR_SPEED_MIN:.4f}</min_velocity>
        <max_velocity>{XYCAR_SPEED_MAX:.4f}</max_velocity>
        <min_acceleration>-2</min_acceleration>
        <max_acceleration>2</max_acceleration>
      </plugin>
    </model>"""


def gz_gui_xml():
    return """
    <gui fullscreen="0">
      <window>
        <width>1000</width>
        <height>845</height>
        <style
          material_theme="Light"
          material_primary="DeepOrange"
          material_accent="LightBlue"
          toolbar_color_light="#f3f3f3"
          toolbar_text_color_light="#111111"
          toolbar_color_dark="#414141"
          toolbar_text_color_dark="#f3f3f3"
          plugin_toolbar_color_light="#bbdefb"
          plugin_toolbar_text_color_light="#111111"
          plugin_toolbar_color_dark="#607d8b"
          plugin_toolbar_text_color_dark="#eeeeee"
        />
        <menus>
          <drawer default="false"/>
        </menus>
        <dialog_on_exit>true</dialog_on_exit>
      </window>
      <plugin filename="MinimalScene" name="3D View">
        <gz-gui>
          <title>3D View</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property>
        </gz-gui>
        <engine>ogre2</engine>
        <scene>scene</scene>
        <ambient_light>0.75 0.75 0.75</ambient_light>
        <background_color>0.82 0.84 0.86</background_color>
        <camera_pose>0 -14 12 0 0.760 1.570796</camera_pose>
        <camera_clip><near>0.1</near><far>1000</far></camera_clip>
      </plugin>
      <plugin filename="EntityContextMenuPlugin" name="Entity context menu">
        <gz-gui>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
      </plugin>
      <plugin filename="GzSceneManager" name="Scene Manager">
        <gz-gui>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
      </plugin>
      <plugin filename="InteractiveViewControl" name="Interactive view control">
        <gz-gui>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
      </plugin>
      <plugin filename="CameraTracking" name="Camera Tracking">
        <gz-gui>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
      </plugin>
      <plugin filename="MarkerManager" name="Marker manager">
        <gz-gui>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
      </plugin>
      <plugin filename="SelectEntities" name="Select Entities">
        <gz-gui>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
      </plugin>
      <plugin filename="Spawn" name="Spawn Entities">
        <gz-gui>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
      </plugin>
      <plugin filename="VisualizationCapabilities" name="Visualization Capabilities">
        <gz-gui>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
      </plugin>
      <plugin filename="WorldControl" name="World control">
        <gz-gui>
          <title>World control</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property>
          <property type="double" key="height">72</property>
          <property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View"><line own="left" target="left"/><line own="bottom" target="bottom"/></anchors>
        </gz-gui>
        <play_pause>true</play_pause><step>true</step><start_paused>true</start_paused><use_event>true</use_event>
      </plugin>
      <plugin filename="WorldStats" name="World stats">
        <gz-gui>
          <title>World stats</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property>
          <property type="double" key="height">110</property>
          <property type="double" key="width">290</property>
          <property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View"><line own="right" target="right"/><line own="bottom" target="bottom"/></anchors>
        </gz-gui>
        <sim_time>true</sim_time><real_time>true</real_time><real_time_factor>true</real_time_factor><iterations>true</iterations>
      </plugin>
      <plugin filename="Shapes" name="Shapes">
        <gz-gui>
          <property key="resizable" type="bool">false</property>
          <property key="x" type="double">0</property>
          <property key="y" type="double">0</property>
          <property key="width" type="double">300</property>
          <property key="height" type="double">50</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
          <property key="cardBackground" type="string">#666666</property>
        </gz-gui>
      </plugin>
      <plugin filename="Lights" name="Lights">
        <gz-gui>
          <property key="resizable" type="bool">false</property>
          <property key="x" type="double">300</property>
          <property key="y" type="double">0</property>
          <property key="width" type="double">150</property>
          <property key="height" type="double">50</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
          <property key="cardBackground" type="string">#666666</property>
        </gz-gui>
      </plugin>
      <plugin filename="TransformControl" name="Transform control">
        <gz-gui>
          <property key="resizable" type="bool">false</property>
          <property key="x" type="double">0</property>
          <property key="y" type="double">50</property>
          <property key="width" type="double">250</property>
          <property key="height" type="double">50</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
          <property key="cardBackground" type="string">#777777</property>
        </gz-gui>
      </plugin>
      <plugin filename="Screenshot" name="Screenshot">
        <gz-gui>
          <property key="resizable" type="bool">false</property>
          <property key="x" type="double">250</property>
          <property key="y" type="double">50</property>
          <property key="width" type="double">50</property>
          <property key="height" type="double">50</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
          <property key="cardBackground" type="string">#777777</property>
        </gz-gui>
      </plugin>
      <plugin filename="CopyPaste" name="CopyPaste">
        <gz-gui>
          <property key="resizable" type="bool">false</property>
          <property key="x" type="double">300</property>
          <property key="y" type="double">50</property>
          <property key="width" type="double">100</property>
          <property key="height" type="double">50</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
          <property key="cardBackground" type="string">#777777</property>
        </gz-gui>
      </plugin>
      <plugin filename="ComponentInspector" name="Component inspector">
        <gz-gui>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property>
        </gz-gui>
      </plugin>
      <plugin filename="EntityTree" name="Entity tree">
        <gz-gui>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property>
        </gz-gui>
      </plugin>
    </gui>"""


def sdf_world(include_gui=True):
    gui = gz_gui_xml() if include_gui else ""
    return f"""<?xml version="1.0"?>
<sdf version="1.7">
  <world name="kookmin_xycar_track">
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-6 2.3e-5 -4.2e-5</magnetic_field>
    <atmosphere type="adiabatic"/>
    <physics name="default_physics" default="1" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    {gui}
    <scene>
      <ambient>0.75 0.75 0.75 1</ambient>
      <background>0.82 0.84 0.86 1</background>
      <shadows>false</shadows>
    </scene>
    <light name="sun" type="directional">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.82 0.82 0.82 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.45 0.20 -0.88</direction>
    </light>
    {track_model()}
    {room_context_models()}
    {white_line_overlay_models()}
    {yellow_centerline_overlay_models()}
    {short_centerline_white_models()}
    {vehicle_model()}
  </world>
</sdf>
"""


def main():
    center = build_centerline()
    length = track_length(center)
    xs = [p[0] for p in center]
    ys = [p[1] for p in center]

    build_track_texture()
    GZ_WORLD.parent.mkdir(parents=True, exist_ok=True)
    # Let Gazebo Sim load its default GUI config. Embedding a custom <gui>
    # block hides the usual World / Entity Tree panels on some gz-sim builds.
    GZ_WORLD.write_text(sdf_world(include_gui=False), encoding="utf-8")
    CLASSIC_WORLD.write_text(sdf_world(include_gui=False), encoding="utf-8")

    print(f"centerline length: {length:.3f} m (target {BLUEPRINT_CENTERLINE_LEN:.3f} m)")
    print(f"outer blueprint frame: {BLUEPRINT_OUTER_W:.3f} m x {BLUEPRINT_OUTER_H:.3f} m")
    print(f"road width: {ROAD_W:.3f} m")
    print(f"white border width: {ROAD_BORDER_LINE_W:.3f} m, yellow centerline width: {YELLOW_CENTER_LINE_W:.3f} m")
    print(f"top/bottom center y: {TRACK_TOP_CENTER_Y:.3f} m / {TRACK_BOTTOM_CENTER_Y:.3f} m")
    print(f"track bbox centerline x=[{min(xs):.3f}, {max(xs):.3f}], y=[{min(ys):.3f}, {max(ys):.3f}]")
    cad_paths = cad_metric_paths()
    if cad_paths:
        cad_points = [point for path in cad_paths for point in path]
        cad_min_x = min(point[0] for point in cad_points)
        cad_max_x = max(point[0] for point in cad_points)
        cad_min_y = min(point[1] for point in cad_points)
        cad_max_y = max(point[1] for point in cad_points)
        print(f"cad white-line size: {cad_max_x - cad_min_x:.3f} m x {cad_max_y - cad_min_y:.3f} m (source unit: cm)")
    print(f"wrote {TEXTURE}")
    print(f"wrote {ROAD_TEXTURE}")
    print(f"wrote {WHITE_LINE_TEXTURE}")
    print(f"wrote {YELLOW_CENTERLINE_TEXTURE}")
    print(f"wrote {WHITE_LINE_PARTS_METADATA}")
    for spec in json.loads(WHITE_LINE_PARTS_METADATA.read_text(encoding="utf-8")):
        print(f"wrote {spec['texture']}")
    print(f"wrote {YELLOW_CENTERLINE_PARTS_METADATA}")
    for spec in json.loads(YELLOW_CENTERLINE_PARTS_METADATA.read_text(encoding="utf-8")):
        print(f"wrote {spec['texture']}")
    print(f"wrote {GZ_WORLD}")
    print(f"wrote {CLASSIC_WORLD}")
    print(f"wrote {PREVIEW}")
    print(f"wrote {PREVIEW_CLEAN}")


if __name__ == "__main__":
    main()
