"""Pure keyboard intent and parking status formatting helpers."""

from __future__ import annotations

from dataclasses import dataclass


STATE_KO = {
    "LOCALIZING": "위치추정 안정화 중",
    "READY": "시작 준비 완료",
    "RUNNING": "주차 미션 주행 중",
    "HOLDING": "목표 지점 정지 확인 중",
    "PAUSED_LOCALIZATION": "위치추정 이상으로 일시정지",
    "COMPLETED": "주차 미션 완료",
    "ABORTED": "주차 미션 중단",
}

LOCALIZATION_REASON_KO = {
    "ok": "AMCL 정상 표본 수가 아직 부족함",
    "no_pose": "AMCL 위치정보가 아직 없음",
    "bad_covariance": "AMCL 오차정보 형식 이상",
    "non_finite": "AMCL 위치정보에 유효하지 않은 값이 있음",
    "bad_stamp": "AMCL 시간정보 이상",
    "stale_pose": "AMCL 위치정보가 오래되어 갱신 필요",
    "xy_uncertain": "AMCL 위치 오차가 허용범위보다 큼",
    "yaw_uncertain": "AMCL 방향 오차가 허용범위보다 큼",
    "non_monotonic_stamp": "AMCL 시간정보 순서가 뒤바뀜",
    "position_jump": "AMCL 위치가 갑자기 크게 변함",
    "yaw_jump": "AMCL 방향이 갑자기 크게 변함",
}

MISSION_REASON_KO = {
    "operator_abort": "운전자가 SPACE로 정지함",
    "operator_reset": "SPACE 시작을 위해 미션과 위치추정을 초기화함",
    "waiting_for_nav2_action_server": "Nav2 경로주행 서버 시작 대기 중",
    "waiting_for_nav2_activation": "Nav2 전체 활성화 대기 중",
    "nav2_activation_timeout": "Nav2 활성화 시간 초과",
    "goal_rejected": "Nav2가 목표를 거부함",
    "goal_cancelled_for_localization": "위치추정 이상으로 목표를 취소함",
    "returned_to_start": "출발지 복귀 완료",
    "mission_time_limit": "3분 제한시간 초과",
    "amcl_stable": "AMCL 위치추정이 안정됨",
    "localization_recovered": "AMCL 위치추정이 다시 정상화됨",
}

MOTOR_REASON_KO = {
    "startup": "모터 안전게이트 초기화 중",
    "ok": "정상 주행 허용",
    "not_authorized": "미션 주행 권한이 아직 없음",
    "no_cmd_vel": "Nav2 주행 명령이 아직 없음",
    "stale_cmd_vel": "Nav2 주행 명령이 끊김",
    "no_scan": "주차용 LiDAR 데이터가 아직 없음",
    "stale_scan": "주차용 LiDAR 데이터가 끊김",
    "insufficient_scan": "유효한 LiDAR 점이 부족함",
    "non_finite_twist": "유효하지 않은 주행 명령",
    "stopped": "Nav2 정지 명령",
    "rotate_in_place_rejected": "차량이 수행할 수 없는 제자리 회전 명령",
    "lidar_swept_collision": "예상 주행 궤적에서 장애물을 감지함",
    "direction_change_dwell": "전진·후진 전환 전 안전 정지 중",
    "steering_settle": "목표 조향각 정렬 중",
    "shutdown": "노드 종료로 정지",
}


@dataclass
class ParkingKeyboardIntent:
    desired_running: bool = False
    quit_requested: bool = False

    def handle_key(self, key: str) -> str | None:
        if key == " ":
            self.desired_running = not self.desired_running
            return "START" if self.desired_running else "STOP"
        if key in {"q", "Q", "\x1b", "\x03"}:
            self.desired_running = False
            self.quit_requested = True
            return "QUIT"
        return None


def parse_status_line(data: str) -> dict[str, str]:
    fields: dict[str, str] = {}
    for item in str(data).split():
        key, separator, value = item.partition("=")
        if separator and key:
            fields[key] = value
    return fields


def explain_status(
    fields: dict[str, str],
    motor_reason: str,
    *,
    desired_running: bool,
) -> tuple[str, str]:
    if not fields:
        return "상태 수신 대기", "주차 미션 상태 토픽을 기다리는 중"

    state = fields.get("state", "UNKNOWN")
    state_text = STATE_KO.get(state, state)
    localization = fields.get("localization", "")
    mission_reason = fields.get("reason", "")

    if not desired_running:
        return state_text, "운전자가 STOP 상태로 설정함"
    if state in {"LOCALIZING", "PAUSED_LOCALIZATION"}:
        return state_text, LOCALIZATION_REASON_KO.get(localization, localization)
    if state == "ABORTED":
        return state_text, MISSION_REASON_KO.get(mission_reason, mission_reason or "미션 중단")
    if state == "COMPLETED":
        return state_text, "모든 주차 단계와 출발지 복귀 완료"
    if state == "READY":
        return state_text, "SPACE 시작 요청 또는 Nav2 첫 목표 전송 대기"
    if state == "HOLDING":
        return state_text, "목표 도착 후 정지시간 확인 중"
    if motor_reason and motor_reason != "ok":
        return state_text, MOTOR_REASON_KO.get(motor_reason, motor_reason)
    if mission_reason:
        return state_text, MISSION_REASON_KO.get(mission_reason, mission_reason)
    return state_text, MOTOR_REASON_KO.get(motor_reason, "정상 주행 중")
