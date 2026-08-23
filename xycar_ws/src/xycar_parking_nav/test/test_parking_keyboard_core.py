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
