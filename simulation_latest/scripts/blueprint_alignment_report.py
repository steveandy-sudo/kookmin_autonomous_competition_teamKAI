from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw

from generate_kookmin_track import (
    BLUEPRINT_CENTERLINE_LEN,
    BLUEPRINT_OUTER_H,
    BLUEPRINT_OUTER_W,
    IMG_H,
    IMG_W,
    LANE_LINE_W,
    LEFT_TURN_CENTER_X,
    PREVIEW,
    ROAD_BORDER_LINE_W,
    ROAD_W,
    ROOT,
    TRACK_BOTTOM_CENTER_Y,
    TRACK_TOP_CENTER_Y,
    WORLD_H,
    WORLD_W,
    build_centerline,
    build_right_s_centerline,
    continuous_blueprint_road_border_mask,
    extract_blueprint_road_border_mask,
    extract_blueprint_white_line_mask,
    track_length,
)


BLUEPRINT_IMAGE = Path("/home/as/.codex/attachments/77f77e6b-220f-4b5f-921f-f417ed4c918f/image-1.png")
OUT_DIR = ROOT / "alignment"
BLUEPRINT_ROI = (198, 234, 813, 581)  # 20.150 m x 11.350 m frame in the attached blueprint.


def preview_frame_crop():
    img = Image.open(PREVIEW).convert("RGB")
    x0 = int(round(((-BLUEPRINT_OUTER_W / 2.0) + WORLD_W / 2.0) / WORLD_W * IMG_W))
    x1 = int(round(((BLUEPRINT_OUTER_W / 2.0) + WORLD_W / 2.0) / WORLD_W * IMG_W))
    y0 = int(round((WORLD_H / 2.0 - BLUEPRINT_OUTER_H / 2.0) / WORLD_H * IMG_H))
    y1 = int(round((WORLD_H / 2.0 + BLUEPRINT_OUTER_H / 2.0) / WORLD_H * IMG_H))
    return img.crop((x0, y0, x1, y1))


def road_mask_from_generated(img):
    arr = np.array(img.convert("RGB"))
    # Dark asphalt / checker black, excluding blue reference when used elsewhere.
    return (arr[..., 0] < 95) & (arr[..., 1] < 95) & (arr[..., 2] < 95)


def line_mask_from_blueprint(img):
    arr = np.array(img.convert("RGB"))
    mx = arr.max(axis=2)
    mn = arr.min(axis=2)
    mask = (mx > 135) & ((mx - mn) < 120)

    # Remove most table hatch lines; keep the road corridor, outer frame, and lane lines.
    h, w = mask.shape
    infield = np.zeros_like(mask)
    infield[68:292, 80:480] = True
    mask &= ~infield

    # Remove the top wall furniture strip and bottom annotation strip from the metric score.
    peripheral = np.zeros_like(mask)
    peripheral[:18, :] = True
    peripheral[340:, :] = True
    mask &= ~peripheral

    # Drop tiny text fragments that otherwise dominate the score.
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype("uint8"), 8)
    clean = np.zeros_like(mask)
    for i in range(1, n):
        x, y, cw, ch, area = stats[i]
        if area >= 18 and (cw >= 3 or ch >= 3):
            clean[labels == i] = True
    return clean


def lane_mask_from_generated(img):
    arr = np.array(img.convert("RGB"))
    # Lane / border lines are light gray-white.
    return (arr[..., 0] > 180) & (arr[..., 1] > 180) & (arr[..., 2] > 170)


def road_border_mask_from_generated(img, reference_border):
    lane = lane_mask_from_generated(img)
    ref_dist = cv2.distanceTransform((~reference_border).astype("uint8"), cv2.DIST_L2, 3)
    return lane & (ref_dist <= 3)


def distance_score(a_mask, b_mask, tolerance_px=5):
    if not a_mask.any() or not b_mask.any():
        return 0.0
    b_dist = cv2.distanceTransform((~b_mask).astype("uint8"), cv2.DIST_L2, 3)
    a_dist = cv2.distanceTransform((~a_mask).astype("uint8"), cv2.DIST_L2, 3)
    a_to_b = (b_dist[a_mask] <= tolerance_px).mean()
    b_to_a = (a_dist[b_mask] <= tolerance_px).mean()
    return 100.0 * 2.0 * a_to_b * b_to_a / max(a_to_b + b_to_a, 1e-9)


def directed_coverage(source_mask, target_mask, tolerance_px=2):
    if not source_mask.any() or not target_mask.any():
        return 0.0
    target_dist = cv2.distanceTransform((~target_mask).astype("uint8"), cv2.DIST_L2, 3)
    return 100.0 * float((target_dist[source_mask] <= tolerance_px).mean())


def curve_errors_by_y(reference_points, generated_points):
    if not reference_points or not generated_points:
        return np.array([], dtype=np.float64)
    ref_y = np.array([p[1] for p in reference_points], dtype=np.float64)
    ref_x = np.array([p[0] for p in reference_points], dtype=np.float64)
    gen_y = np.array([p[1] for p in generated_points], dtype=np.float64)
    gen_x = np.array([p[0] for p in generated_points], dtype=np.float64)
    order = np.argsort(gen_y)
    gen_y = gen_y[order]
    gen_x = gen_x[order]
    generated_at_ref = np.interp(ref_y, gen_y, gen_x)
    return np.abs(generated_at_ref - ref_x)


def main():
    OUT_DIR.mkdir(exist_ok=True)

    blueprint = Image.open(BLUEPRINT_IMAGE).convert("RGB")
    bx0, by0, bx1, by1 = BLUEPRINT_ROI
    bp_crop = blueprint.crop(BLUEPRINT_ROI)
    resample = getattr(Image, "Resampling", Image).LANCZOS
    gen_crop = preview_frame_crop().resize(bp_crop.size, resample)

    ref_lines = line_mask_from_blueprint(bp_crop)
    gen_lines = lane_mask_from_generated(gen_crop)
    raw_white_lines = extract_blueprint_white_line_mask(clean=False)
    ref_white_lines = extract_blueprint_white_line_mask(clean=True)
    gen_white_lines = gen_lines
    raw_ref_border = extract_blueprint_road_border_mask()
    ref_border = continuous_blueprint_road_border_mask(bp_crop.width, bp_crop.height)
    gen_border = road_border_mask_from_generated(gen_crop, ref_border)

    overlay = np.array(bp_crop).astype(np.float32)
    yellow = np.zeros_like(overlay)
    yellow[..., 0] = 255
    yellow[..., 1] = 230
    overlay[gen_lines & ~ref_lines] = overlay[gen_lines & ~ref_lines] * 0.25 + yellow[gen_lines & ~ref_lines] * 0.75
    green = np.zeros_like(overlay)
    green[..., 1] = 255
    overlay[gen_lines & ref_lines] = overlay[gen_lines & ref_lines] * 0.25 + green[gen_lines & ref_lines] * 0.75
    overlay = np.clip(overlay, 0, 255).astype("uint8")

    border_overlay = np.array(bp_crop).astype(np.float32)
    matched = ref_border & gen_border
    missing = ref_border & ~gen_border
    extra = gen_border & ~ref_border
    green = np.zeros_like(border_overlay)
    green[..., 1] = 255
    red = np.zeros_like(border_overlay)
    red[..., 0] = 255
    yellow = np.zeros_like(border_overlay)
    yellow[..., 0] = 255
    yellow[..., 1] = 230
    border_overlay[matched] = border_overlay[matched] * 0.20 + green[matched] * 0.80
    border_overlay[missing] = border_overlay[missing] * 0.20 + red[missing] * 0.80
    border_overlay[extra] = border_overlay[extra] * 0.20 + yellow[extra] * 0.80
    border_overlay = np.clip(border_overlay, 0, 255).astype("uint8")

    white_overlay = np.array(bp_crop).astype(np.float32)
    white_matched = ref_white_lines & gen_white_lines
    white_missing = ref_white_lines & ~gen_white_lines
    white_extra = gen_white_lines & ~ref_white_lines
    green = np.zeros_like(white_overlay)
    green[..., 1] = 255
    red = np.zeros_like(white_overlay)
    red[..., 0] = 255
    yellow = np.zeros_like(white_overlay)
    yellow[..., 0] = 255
    yellow[..., 1] = 230
    white_overlay[white_matched] = white_overlay[white_matched] * 0.20 + green[white_matched] * 0.80
    white_overlay[white_missing] = white_overlay[white_missing] * 0.20 + red[white_missing] * 0.80
    white_overlay[white_extra] = white_overlay[white_extra] * 0.20 + yellow[white_extra] * 0.80
    white_overlay = np.clip(white_overlay, 0, 255).astype("uint8")

    # Metric control points measured from the blueprint crop in pixel coordinates.
    sx = BLUEPRINT_OUTER_W / (bx1 - bx0)
    sy = BLUEPRINT_OUTER_H / (by1 - by0)
    measured = {
        "left_center_x": 43 * sx - BLUEPRINT_OUTER_W / 2.0,
        "top_center_y": (bp_crop.height - 54) * sy - BLUEPRINT_OUTER_H / 2.0,
        "bottom_center_y": (bp_crop.height - 318) * sy - BLUEPRINT_OUTER_H / 2.0,
    }
    actual = {
        "left_center_x": LEFT_TURN_CENTER_X - 1.850,
        "top_center_y": TRACK_TOP_CENTER_Y,
        "bottom_center_y": TRACK_BOTTOM_CENTER_Y,
    }

    control_errors = {k: abs(actual[k] - measured[k]) for k in measured}
    control_score = max(0.0, 100.0 * (1.0 - sum(control_errors.values()) / (len(control_errors) * 0.25)))

    line_score = distance_score(gen_lines, ref_lines, tolerance_px=7)
    white_score = distance_score(ref_white_lines, gen_white_lines, tolerance_px=2)
    white_ref_coverage = directed_coverage(ref_white_lines, gen_white_lines, tolerance_px=2)
    white_gen_coverage = directed_coverage(gen_white_lines, ref_white_lines, tolerance_px=2)
    border_score = distance_score(ref_border, gen_border, tolerance_px=2)
    border_ref_coverage = directed_coverage(ref_border, gen_border, tolerance_px=2)
    border_gen_coverage = directed_coverage(gen_border, ref_border, tolerance_px=2)
    length_score = max(0.0, 100.0 * (1.0 - abs(track_length(build_centerline()) - BLUEPRINT_CENTERLINE_LEN) / 0.25))
    road_width_score = max(0.0, 100.0 * (1.0 - abs(ROAD_W - 1.632) / 0.05))
    line_width_score = max(0.0, 100.0 * (1.0 - abs(LANE_LINE_W - 0.015) / 0.005))

    right_s_reference = build_right_s_centerline(scaled=False)
    right_s_generated = build_right_s_centerline(scaled=True)
    right_s_errors = curve_errors_by_y(right_s_reference, right_s_generated)
    if len(right_s_errors):
        right_s_mean_error = float(right_s_errors.mean())
        right_s_max_error = float(right_s_errors.max())
        right_s_curve_score = 100.0 * float((right_s_errors <= 0.20).mean())
    else:
        right_s_mean_error = 999.0
        right_s_max_error = 999.0
        right_s_curve_score = 0.0

    geometry_score = (
        0.20 * control_score
        + 0.25 * length_score
        + 0.15 * road_width_score
        + 0.10 * line_width_score
        + 0.30 * right_s_curve_score
    )

    Image.fromarray(overlay).save(OUT_DIR / "blueprint_generated_overlay.png")
    Image.fromarray(white_overlay).save(OUT_DIR / "blueprint_white_line_overlay.png")
    Image.fromarray(border_overlay).save(OUT_DIR / "blueprint_border_overlay.png")
    bp_crop.save(OUT_DIR / "blueprint_metric_crop.png")
    gen_crop.save(OUT_DIR / "generated_metric_crop.png")
    Image.fromarray((ref_lines * 255).astype("uint8")).save(OUT_DIR / "blueprint_line_mask.png")
    Image.fromarray((gen_lines * 255).astype("uint8")).save(OUT_DIR / "generated_line_mask.png")
    Image.fromarray((raw_white_lines * 255).astype("uint8")).save(OUT_DIR / "blueprint_white_line_raw_mask.png")
    Image.fromarray((ref_white_lines * 255).astype("uint8")).save(OUT_DIR / "blueprint_white_line_mask.png")
    Image.fromarray((gen_white_lines * 255).astype("uint8")).save(OUT_DIR / "generated_white_line_mask.png")
    Image.fromarray((raw_ref_border * 255).astype("uint8")).save(OUT_DIR / "blueprint_road_border_raw_mask.png")
    Image.fromarray((ref_border * 255).astype("uint8")).save(OUT_DIR / "blueprint_road_border_mask.png")
    Image.fromarray((gen_border * 255).astype("uint8")).save(OUT_DIR / "generated_road_border_mask.png")

    report = [
        f"blueprint_roi_px={BLUEPRINT_ROI}",
        "texture_mode=blueprint_white_lines_on_gray",
        f"metric_frame_m={BLUEPRINT_OUTER_W:.3f} x {BLUEPRINT_OUTER_H:.3f}",
        f"scale_m_per_px={sx:.6f}, {sy:.6f}",
        f"centerline_length_m={track_length(build_centerline()):.3f} target={BLUEPRINT_CENTERLINE_LEN:.3f}",
            f"road_width_m={ROAD_W:.3f} line_width_m={LANE_LINE_W:.3f}",
            f"road_border_line_width_m={ROAD_BORDER_LINE_W:.3f}",
            "appearance_score=ignored",
            "scoring_note=visual color is intentionally not compared; score uses metric geometry only",
            "control_points_m:",
    ]
    for k in measured:
        report.append(f"  {k}: measured={measured[k]:.3f} actual={actual[k]:.3f} error={control_errors[k]:.3f}")
    report.extend(
        [
            f"control_score={control_score:.2f}",
            f"line_mask_diagnostic_px7={line_score:.2f}",
            f"blueprint_white_line_alignment_score={white_score:.2f}",
            f"blueprint_white_line_ref_coverage_px2={white_ref_coverage:.2f}",
            f"blueprint_white_line_generated_coverage_px2={white_gen_coverage:.2f}",
            f"road_border_alignment_score={border_score:.2f}",
            f"road_border_ref_coverage_px2={border_ref_coverage:.2f}",
            f"road_border_generated_coverage_px2={border_gen_coverage:.2f}",
            f"right_s_samples={len(right_s_reference)}",
            f"right_s_mean_error_m={right_s_mean_error:.4f}",
            f"right_s_max_error_m={right_s_max_error:.4f}",
            f"right_s_curve_score_20cm={right_s_curve_score:.2f}",
            f"length_score={length_score:.2f}",
            f"road_width_score={road_width_score:.2f}",
            f"line_width_score={line_width_score:.2f}",
            f"geometry_alignment_score={geometry_score:.2f}",
        ]
    )
    (OUT_DIR / "fit_report.txt").write_text("\n".join(report) + "\n", encoding="utf-8")
    print("\n".join(report))


if __name__ == "__main__":
    main()
