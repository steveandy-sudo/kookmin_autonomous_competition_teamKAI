from xycar_parking_nav.parking_keyboard_core import (
    ParkingKeyboardIntent,
    explain_status,
    parse_status_line,
)


def test_space_toggles_start_and_stop():
    intent = ParkingKeyboardIntent()

    assert intent.handle_key(" ") == "START"
    assert intent.desired_running is True
    assert intent.handle_key(" ") == "STOP"
    assert intent.desired_running is False


def test_quit_always_requests_stop():
    intent = ParkingKeyboardIntent(desired_running=True)

    assert intent.handle_key("q") == "QUIT"
    assert intent.desired_running is False
    assert intent.quit_requested is True


def test_status_parser_and_stale_pose_reason():
    fields = parse_status_line(
        "state=PAUSED_LOCALIZATION step=A_ROUTE_ARC_1 index=0/31 "
        "localization=stale_pose elapsed=1.0"
    )

    state, reason = explain_status(fields, "not_authorized", desired_running=True)
    assert state == "위치추정 이상으로 일시정지"
    assert reason == "AMCL 위치정보가 오래되어 갱신 필요"


def test_motor_gate_reason_is_shown_while_running():
    fields = parse_status_line(
        "state=RUNNING step=A_PARK index=11/31 localization=ok"
    )

    state, reason = explain_status(
        fields, "lidar_swept_collision", desired_running=True
    )
    assert state == "주차 미션 주행 중"
    assert reason == "예상 주행 궤적에서 장애물을 감지함"


def test_route_localization_gate_reason_is_explained():
    fields = parse_status_line(
        "state=LOCALIZING step=WP_01 index=0/4 "
        "localization=route_localization_not_ready"
    )

    state, reason = explain_status(fields, "not_authorized", desired_running=True)
    assert state == "위치추정 안정화 중"
    assert "경로 기반 LiDAR" in reason


def test_automatic_obstacle_recovery_is_explained():
    fields = parse_status_line(
        "state=RECOVERING step=A_ROUTE_ARC_1 index=0/31 "
        "localization=ok reason=nav2_status_6_automatic_recovery"
    )

    state, reason = explain_status(fields, "not_authorized", desired_running=True)
    assert state == "장애물 우회 경로 재탐색 중"
    assert "짧은 간격" in reason
    assert "우회 경로" in reason


def test_stalled_forward_switch_to_reverse_is_explained():
    fields = parse_status_line(
        "state=RUNNING step=WP_04 index=3/13 localization=ok "
        "reason=forward_progress_stalled_reverse_fallback"
    )

    state, reason = explain_status(
        fields, "not_authorized", desired_running=True
    )
    assert state == "주차 미션 주행 중"
    assert "전진 진행이 멈춰" in reason
    assert "후진 허용" in reason


def test_vesc_low_voltage_stop_reason_is_explained():
    fields = parse_status_line(
        "state=RUNNING step=A_ROUTE_ALIGN index=2/31 localization=ok"
    )

    state, reason = explain_status(
        fields, "vesc_low_voltage_stop", desired_running=True
    )
    assert state == "주차 미션 주행 중"
    assert "VESC 입력전압" in reason
