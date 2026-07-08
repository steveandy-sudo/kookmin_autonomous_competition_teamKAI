import csv
import json
import os
import re
import tempfile
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple


CSV_FIELDS = [
    "timestamp_ns",
    "front_image_path",
    "left_image_path",
    "right_image_path",
    "rear_image_path",
    "scan_npz_path",
    "motor_angle",
    "motor_speed",
    "mission_label",
    "dataset_profile",
    "source_mode",
    "session_id",
    "lap_index",
    "notes",
]


PROFILE_ALLOWED_LABELS = {
    "drive": ["general_drive", "lane_drive", "hill_drive", "shortcut", "recovery"],
    "cone": ["cone_drive", "recovery"],
    "overtake": ["vehicle_overtake", "overtake_start", "overtake_end", "recovery"],
}


def expand_path(path: str) -> Path:
    return Path(os.path.expandvars(os.path.expanduser(path))).resolve()


def sanitize_name(name: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "_", name.strip())
    return cleaned.strip("._-") or "session"


def make_session_id(session_name: str, now: Optional[datetime] = None) -> str:
    stamp = (now or datetime.now()).strftime("%Y%m%d_%H%M%S")
    return f"{stamp}_{sanitize_name(session_name)}"


def make_session_dir(
    output_root: str,
    dataset_profile: str,
    session_name: str,
    now: Optional[datetime] = None,
) -> Tuple[str, Path]:
    profile = sanitize_name(dataset_profile)
    session_id = make_session_id(session_name, now=now)
    session_dir = expand_path(output_root) / profile / session_id
    for relative in [
        "images/front",
        "images/left",
        "images/right",
        "images/rear",
        "scan",
        "debug",
    ]:
        (session_dir / relative).mkdir(parents=True, exist_ok=True)
    return session_id, session_dir


def parse_allowed_labels(value: Any, dataset_profile: str) -> List[str]:
    if value is None:
        labels: List[str] = []
    elif isinstance(value, str):
        labels = [part.strip() for part in value.split(",") if part.strip()]
    elif isinstance(value, Iterable):
        labels = [str(part).strip() for part in value if str(part).strip()]
    else:
        labels = []
    if labels:
        return labels
    return list(PROFILE_ALLOWED_LABELS.get(str(dataset_profile), []))


def open_samples_csv(path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    handle = path.open("w", newline="", encoding="utf-8")
    writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
    writer.writeheader()
    return handle, writer


def atomic_write_json(path: Path, data: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        "w", encoding="utf-8", dir=str(path.parent), delete=False
    ) as tmp:
        json.dump(data, tmp, indent=2, ensure_ascii=False, sort_keys=True)
        tmp.write("\n")
        tmp_name = tmp.name
    os.replace(tmp_name, path)


def relative_to_session(session_dir: Path, path: Optional[Path]) -> str:
    if path is None:
        return ""
    try:
        return str(path.relative_to(session_dir)).replace("\\", "/")
    except ValueError:
        return str(path)


def stamp_to_ns(msg: Any, fallback_ns: Optional[int] = None) -> int:
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if sec is not None and nanosec is not None and (sec != 0 or nanosec != 0):
        return int(sec) * 1_000_000_000 + int(nanosec)
    if fallback_ns is not None:
        return int(fallback_ns)
    return int(datetime.now().timestamp() * 1_000_000_000)


def ros_time_to_ns(clock_now: Any) -> int:
    return int(clock_now.nanoseconds)


def motor_from_msg(msg: Any):
    if hasattr(msg, "angle") and hasattr(msg, "speed"):
        return float(msg.angle), float(msg.speed)
    data = getattr(msg, "data", None)
    if data is not None and len(data) >= 2:
        return float(data[0]), float(data[1])
    raise ValueError(f"unsupported motor message type: {type(msg)!r}")


def string_from_msg(msg: Any, default: str = "idle") -> str:
    value = getattr(msg, "data", None)
    if value is None:
        return default
    return str(value) or default


def image_suffix(image_format: str) -> str:
    value = str(image_format).lower().strip(".")
    if value not in {"jpg", "jpeg", "png"}:
        raise ValueError("image_format must be jpg or png")
    return "jpg" if value == "jpeg" else value


def write_session_readme(
    session_dir: Path,
    session_id: str,
    dataset_profile: str,
    allowed_labels: List[str],
    topics: Dict[str, str],
) -> None:
    lines = [
        f"# IL data session: {session_id}",
        "",
        "이 폴더는 `il_data_tools`의 `il_common_recorder`가 생성한 데이터 세션입니다.",
        "",
        "## Profile",
        "",
        f"- dataset_profile: `{dataset_profile}`",
        f"- allowed_labels: `{', '.join(allowed_labels) if allowed_labels else '(none)'}`",
        "",
        "## Topics",
    ]
    for key, value in sorted(topics.items()):
        lines.append(f"- {key}: `{value}`")
    lines.extend(
        [
            "",
            "## Safety",
            "",
            "이 recorder는 `/xycar_motor`를 publish하지 않습니다.",
            "최종 차량 제어와 `/xycar_motor` publish는 rule-based 주행 코드가 담당해야 합니다.",
        ]
    )
    (session_dir / "README_session.md").write_text(
        "\n".join(lines) + "\n", encoding="utf-8"
    )
