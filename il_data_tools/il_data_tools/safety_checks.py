import shutil
from pathlib import Path
from typing import Dict


def disk_usage_gb(path: Path) -> Dict[str, float]:
    usage = shutil.disk_usage(str(path))
    gb = 1024.0 ** 3
    return {
        "total_gb": usage.total / gb,
        "used_gb": usage.used / gb,
        "free_gb": usage.free / gb,
    }


def directory_size_gb(path: Path) -> float:
    if not path.exists():
        return 0.0
    total = 0
    for child in path.rglob("*"):
        if child.is_file():
            try:
                total += child.stat().st_size
            except OSError:
                continue
    return total / (1024.0 ** 3)


def disk_space_warning(path: Path, min_free_gb: float = 2.0) -> str:
    usage = disk_usage_gb(path)
    if usage["free_gb"] < min_free_gb:
        return f"low disk space: {usage['free_gb']:.2f} GB free at {path}"
    return ""


def session_size_warning(session_dir: Path, max_session_gb: float) -> str:
    if max_session_gb <= 0:
        return ""
    size_gb = directory_size_gb(session_dir)
    if size_gb >= max_session_gb:
        return f"session size limit reached: {size_gb:.2f} GB >= {max_session_gb:.2f} GB"
    return ""
