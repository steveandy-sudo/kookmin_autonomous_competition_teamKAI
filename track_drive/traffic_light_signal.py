"""Traffic-light class IDs를 출발 신호 의미로 변환하는 순수 후처리."""

from typing import Optional, Set, Tuple


SignalFlags = Tuple[bool, bool, bool, bool]


def detection_signal_flags(
    *,
    class_id: int,
    score: float,
    valid: bool,
    red_color: bool,
    red_ids: Optional[Set[int]],
    yellow_ids: Optional[Set[int]],
    green_ids: Optional[Set[int]],
    left_ids: Optional[Set[int]],
    red_threshold: float,
    yellow_threshold: float,
    left_threshold: float,
) -> SignalFlags:
    """한 detection을 red, green, left, yellow 플래그로 변환한다."""

    if not valid:
        return False, False, False, False

    normalized_id = int(class_id)
    normalized_score = float(score)
    red_allowed = _class_id_allowed(normalized_id, red_ids)

    red = red_allowed and (
        normalized_score >= float(red_threshold) or bool(red_color)
    )
    green = _class_id_allowed(normalized_id, green_ids)
    left = (
        normalized_score >= float(left_threshold)
        and _class_id_allowed(normalized_id, left_ids)
    )
    yellow = (
        normalized_score >= float(yellow_threshold)
        and _class_id_allowed(normalized_id, yellow_ids)
    )
    return bool(red), bool(green), bool(left), bool(yellow)


def resolve_signal_state(
    *,
    red: bool,
    green: bool,
    left: bool,
    yellow: bool,
    best_label: str,
) -> str:
    """프레임 플래그를 단일 상태로 만들며 충돌 신호는 unknown으로 둔다."""

    if sum((bool(red), bool(green), bool(yellow))) > 1:
        return "unknown"
    if green:
        return "green"
    if left:
        return "left"
    if red:
        return "red"
    if yellow:
        return "yellow"
    if best_label != "unknown":
        return best_label
    return "none"


def _class_id_allowed(
    class_id: int,
    allowed_class_ids: Optional[Set[int]],
) -> bool:
    return (
        allowed_class_ids is None
        or int(class_id) in allowed_class_ids
    )
