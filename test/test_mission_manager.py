from dataclasses import FrozenInstanceError, fields
import unittest

from track_drive.mission import (
    ControlMode,
    MissionContext,
    MissionDecision,
    MissionManager,
    MissionManagerConfig,
    MissionObservation,
    MissionState,
    StartSignal,
)


def observation(now_sec, **changes):
    return MissionObservation(now_sec=now_sec, **changes)


def manual_start(manager, now_sec=0.0, drive=True, lane=False):
    assert manager.set_manual_override("START")
    return manager.update(
        observation(
            now_sec,
            drive_policy_valid=drive,
            lane_fallback_valid=lane,
        )
    )


def cone_entry_inputs(
    *,
    camera_count=4,
    camera_valid=True,
    lidar_source_valid=True,
    lidar_path_ready=True,
    lidar_present=True,
):
    return {
        "camera_cone_valid": camera_valid,
        "camera_cone_count": camera_count,
        "lidar_cone_source_valid": lidar_source_valid,
        "lidar_cone_path_ready": lidar_path_ready,
        "lidar_cone_present": lidar_present,
    }


def cone_exit_inputs(
    *,
    camera_count=1,
    camera_valid=True,
    lidar_source_valid=True,
    lidar_path_ready=False,
    lidar_present=False,
):
    return {
        "camera_cone_valid": camera_valid,
        "camera_cone_count": camera_count,
        "lidar_cone_source_valid": lidar_source_valid,
        "lidar_cone_path_ready": lidar_path_ready,
        "lidar_cone_present": lidar_present,
    }


def enter_cone_mode(manager):
    manual_start(manager, 0.0, drive=True)
    manager.update(
        observation(
            1.0,
            drive_policy_valid=True,
            **cone_entry_inputs(),
        )
    )
    return manager.update(
        observation(
            1.25,
            drive_policy_valid=True,
            **cone_entry_inputs(),
        )
    )


class DataModelTest(unittest.TestCase):
    def test_enum_members_are_exact(self):
        self.assertEqual(
            list(MissionState.__members__),
            [
                "WAIT_START_SIGNAL",
                "LANE_DRIVING",
                "CONE_SECTION",
                "FIXED_OBSTACLE_SECTION",
                "OVERTAKE_SECTION",
                "ROUTE_SELECTION",
                "SHORTCUT_SECTION",
            ],
        )
        self.assertEqual(
            list(ControlMode.__members__),
            [
                "STOP",
                "NORMAL_IL",
                "CONE_DRIVE_RULE",
                "LANE_FALLBACK",
                "FIXED_OBSTACLE_RULE",
                "VEHICLE_FOLLOW",
                "VEHICLE_OVERTAKE",
                "SHORTCUT_RULE",
            ],
        )
        self.assertEqual(
            list(StartSignal.__members__),
            ["UNKNOWN", "RED", "YELLOW", "GREEN"],
        )
        self.assertNotIn("BOOT", MissionState.__members__)
        self.assertNotIn("RACING", MissionState.__members__)
        self.assertNotIn("FINISHED", MissionState.__members__)
        self.assertNotIn("MANUAL_RECOVERY", MissionState.__members__)
        self.assertNotIn("RACE_COMPLETE", MissionState.__members__)
        self.assertNotIn("RACE_ABORTED", MissionState.__members__)
        self.assertNotIn("EMERGENCY_STOP", MissionState.__members__)
        self.assertNotIn("ROUTE_SELECT", ControlMode.__members__)

    def test_observation_fields_are_exact_and_frozen(self):
        self.assertEqual(
            [item.name for item in fields(MissionObservation)],
            [
                "now_sec",
                "safety_stop_required",
                "start_signal",
                "start_signal_valid",
                "safety_ready",
                "drive_policy_valid",
                "lane_fallback_valid",
                "camera_cone_valid",
                "camera_cone_count",
                "lidar_cone_source_valid",
                "lidar_cone_path_ready",
                "lidar_cone_present",
                "fixed_obstacle_detected",
                "vehicle_detected",
                "shortcut_signal_detected",
                "lap_crossing_detected",
            ],
        )
        item = MissionObservation(now_sec=0.0)
        with self.assertRaises(FrozenInstanceError):
            item.now_sec = 1.0

    def test_context_fields_and_defaults_are_exact(self):
        self.assertEqual(
            [item.name for item in fields(MissionContext)],
            [
                "mission_state",
                "control_mode",
                "state_enter_sec",
                "mode_enter_sec",
                "red_signal_seen_since",
                "start_signal_seen_since",
                "start_signal_armed",
                "cone_seen_since",
                "cone_missing_since",
                "cone_last_seen_sec",
                "drive_valid_since",
                "lane_fallback_ready_since",
                "manual_override",
                "lap_count",
                "shortcut_used",
            ],
        )
        context = MissionContext()
        self.assertIs(
            context.mission_state, MissionState.WAIT_START_SIGNAL
        )
        self.assertIs(context.control_mode, ControlMode.STOP)
        self.assertEqual(context.manual_override, "AUTO")
        self.assertEqual(context.lap_count, 0)
        self.assertFalse(context.shortcut_used)

    def test_decision_has_only_the_five_required_fields(self):
        names = [item.name for item in fields(MissionDecision)]
        self.assertEqual(
            names,
            [
                "mission_state",
                "control_mode",
                "selected_source",
                "speed_profile",
                "stop_required",
            ],
        )
        self.assertNotIn("transition_reason", names)

    def test_config_defaults_are_exact(self):
        self.assertEqual(
            MissionManagerConfig(),
            MissionManagerConfig(
                start_signal_red_hold_sec=0.3,
                start_signal_green_hold_sec=0.3,
                cone_enter_hold_sec=0.25,
                cone_exit_hold_sec=0.7,
                cone_min_dwell_sec=1.0,
                cone_reenter_cooldown_sec=1.0,
                drive_recover_hold_sec=0.4,
                lane_fallback_ready_hold_sec=0.2,
                minimum_camera_cone_count=4,
                maximum_camera_cone_count_for_exit=1,
                status_log_period_sec=1.0,
            ),
        )


class MissionControlContractTest(unittest.TestCase):
    def test_each_mission_state_has_exact_allowed_control_modes(self):
        manager = MissionManager(MissionManagerConfig())
        expected = {
            MissionState.WAIT_START_SIGNAL: {ControlMode.STOP},
            MissionState.LANE_DRIVING: {
                ControlMode.STOP,
                ControlMode.NORMAL_IL,
                ControlMode.LANE_FALLBACK,
            },
            MissionState.CONE_SECTION: {
                ControlMode.STOP,
                ControlMode.CONE_DRIVE_RULE,
            },
            MissionState.FIXED_OBSTACLE_SECTION: {
                ControlMode.STOP,
                ControlMode.FIXED_OBSTACLE_RULE,
            },
            MissionState.OVERTAKE_SECTION: {
                ControlMode.STOP,
                ControlMode.NORMAL_IL,
                ControlMode.LANE_FALLBACK,
                ControlMode.VEHICLE_FOLLOW,
                ControlMode.VEHICLE_OVERTAKE,
            },
            MissionState.ROUTE_SELECTION: {
                ControlMode.STOP,
                ControlMode.NORMAL_IL,
                ControlMode.LANE_FALLBACK,
            },
            MissionState.SHORTCUT_SECTION: {
                ControlMode.STOP,
                ControlMode.NORMAL_IL,
                ControlMode.SHORTCUT_RULE,
            },
        }

        for state, allowed in expected.items():
            with self.subTest(state=state):
                actual = {
                    mode
                    for mode in ControlMode
                    if manager._mode_is_allowed(state, mode)
                }
                self.assertEqual(actual, allowed)

    def test_unimplemented_mission_states_remain_stopped(self):
        future_states = (
            MissionState.FIXED_OBSTACLE_SECTION,
            MissionState.OVERTAKE_SECTION,
            MissionState.ROUTE_SELECTION,
            MissionState.SHORTCUT_SECTION,
        )
        for state in future_states:
            with self.subTest(state=state):
                manager = MissionManager(MissionManagerConfig())
                manager.context.mission_state = state
                decision = manager.update(
                    observation(
                        1.0,
                        drive_policy_valid=True,
                        lane_fallback_valid=True,
                        fixed_obstacle_detected=True,
                        vehicle_detected=True,
                        shortcut_signal_detected=True,
                    )
                )
                self.assertIs(decision.mission_state, state)
                self.assertIs(decision.control_mode, ControlMode.STOP)
                self.assertTrue(decision.stop_required)


class MissionManagerTransitionTest(unittest.TestCase):
    def setUp(self):
        self.manager = MissionManager(MissionManagerConfig())

    @staticmethod
    def _start_with_confirmed_signal(
        manager,
        *,
        drive_policy_valid,
        lane_fallback_valid,
    ):
        manager.update(
            observation(
                0.0,
                start_signal=StartSignal.RED,
                start_signal_valid=True,
            )
        )
        manager.update(
            observation(
                0.3,
                start_signal=StartSignal.RED,
                start_signal_valid=True,
            )
        )
        manager.update(
            observation(
                1.0,
                start_signal=StartSignal.GREEN,
                start_signal_valid=True,
            )
        )
        return manager.update(
            observation(
                1.3,
                start_signal=StartSignal.GREEN,
                start_signal_valid=True,
                drive_policy_valid=drive_policy_valid,
                lane_fallback_valid=lane_fallback_valid,
            )
        )

    def test_initial_state_is_wait_and_stop(self):
        decision = self.manager.update(observation(0.0))
        self.assertEqual(
            decision,
            MissionDecision(
                mission_state=MissionState.WAIT_START_SIGNAL,
                control_mode=ControlMode.STOP,
                selected_source="none",
                speed_profile="stop",
                stop_required=True,
            ),
        )

    def test_green_without_confirmed_red_never_starts(self):
        self.manager.update(
            observation(
                0.0,
                start_signal=StartSignal.GREEN,
                start_signal_valid=True,
                drive_policy_valid=True,
            )
        )
        decision = self.manager.update(
            observation(
                1.0,
                start_signal=StartSignal.GREEN,
                start_signal_valid=True,
                drive_policy_valid=True,
            )
        )

        self.assertIs(
            decision.mission_state, MissionState.WAIT_START_SIGNAL
        )
        self.assertFalse(self.manager.context.start_signal_armed)

    def test_short_red_does_not_arm_and_resets_timer(self):
        self.manager.update(
            observation(
                0.0,
                start_signal=StartSignal.RED,
                start_signal_valid=True,
            )
        )
        before = self.manager.update(
            observation(
                0.29,
                start_signal=StartSignal.RED,
                start_signal_valid=True,
            )
        )
        self.manager.update(
            observation(
                0.30,
                start_signal=StartSignal.YELLOW,
                start_signal_valid=True,
            )
        )
        restarted = self.manager.update(
            observation(
                0.50,
                start_signal=StartSignal.RED,
                start_signal_valid=True,
            )
        )

        self.assertIs(before.mission_state, MissionState.WAIT_START_SIGNAL)
        self.assertFalse(self.manager.context.start_signal_armed)
        self.assertIs(
            restarted.mission_state, MissionState.WAIT_START_SIGNAL
        )

    def test_confirmed_red_arms_but_remains_stopped(self):
        self.manager.update(
            observation(
                0.0,
                start_signal=StartSignal.RED,
                start_signal_valid=True,
            )
        )
        armed = self.manager.update(
            observation(
                0.3,
                start_signal=StartSignal.RED,
                start_signal_valid=True,
                drive_policy_valid=True,
            )
        )

        self.assertIs(armed.mission_state, MissionState.WAIT_START_SIGNAL)
        self.assertIs(armed.control_mode, ControlMode.STOP)
        self.assertTrue(self.manager.context.start_signal_armed)

    def test_confirmed_red_then_confirmed_green_starts_lane_driving(self):
        self.manager.update(
            observation(
                0.0,
                start_signal=StartSignal.RED,
                start_signal_valid=True,
            )
        )
        self.manager.update(
            observation(
                0.3,
                start_signal=StartSignal.RED,
                start_signal_valid=True,
            )
        )
        self.manager.update(
            observation(
                1.0,
                start_signal=StartSignal.GREEN,
                start_signal_valid=True,
                drive_policy_valid=True,
            )
        )
        before = self.manager.update(
            observation(
                1.299,
                start_signal=StartSignal.GREEN,
                start_signal_valid=True,
                drive_policy_valid=True,
            )
        )
        at_boundary = self.manager.update(
            observation(
                1.3,
                start_signal=StartSignal.GREEN,
                start_signal_valid=True,
                drive_policy_valid=True,
            )
        )

        self.assertIs(before.mission_state, MissionState.WAIT_START_SIGNAL)
        self.assertIs(
            at_boundary.mission_state,
            MissionState.LANE_DRIVING,
        )
        self.assertIs(at_boundary.control_mode, ControlMode.NORMAL_IL)

    def test_confirmed_green_uses_immediate_lane_source_priority(self):
        cases = (
            (True, True, ControlMode.NORMAL_IL, "drive_il", False),
            (False, True, ControlMode.LANE_FALLBACK, "lane_fallback", False),
            (False, False, ControlMode.STOP, "none", True),
        )

        for drive_valid, lane_valid, mode, source, stopped in cases:
            with self.subTest(
                drive_policy_valid=drive_valid,
                lane_fallback_valid=lane_valid,
            ):
                manager = MissionManager(MissionManagerConfig())
                decision = self._start_with_confirmed_signal(
                    manager,
                    drive_policy_valid=drive_valid,
                    lane_fallback_valid=lane_valid,
                )

                self.assertIs(
                    decision.mission_state,
                    MissionState.LANE_DRIVING,
                )
                self.assertIs(decision.control_mode, mode)
                self.assertEqual(decision.selected_source, source)
                self.assertEqual(decision.stop_required, stopped)

    def test_yellow_resets_green_confirmation_but_preserves_red_arm(self):
        self.manager.update(
            observation(
                0.0,
                start_signal=StartSignal.RED,
                start_signal_valid=True,
            )
        )
        self.manager.update(
            observation(
                0.3,
                start_signal=StartSignal.RED,
                start_signal_valid=True,
            )
        )
        self.manager.update(
            observation(
                1.0,
                start_signal=StartSignal.GREEN,
                start_signal_valid=True,
            )
        )
        yellow = self.manager.update(
            observation(
                1.2,
                start_signal=StartSignal.YELLOW,
                start_signal_valid=True,
            )
        )
        self.assertIsNone(self.manager.context.start_signal_seen_since)

        restarted = self.manager.update(
            observation(
                1.3,
                start_signal=StartSignal.GREEN,
                start_signal_valid=True,
            )
        )

        self.assertIs(yellow.mission_state, MissionState.WAIT_START_SIGNAL)
        self.assertTrue(self.manager.context.start_signal_armed)
        self.assertEqual(self.manager.context.start_signal_seen_since, 1.3)
        self.assertIs(
            restarted.mission_state, MissionState.WAIT_START_SIGNAL
        )

    def test_invalid_signal_resets_green_confirmation_but_preserves_arm(self):
        self.manager.context.start_signal_armed = True
        self.manager.update(
            observation(
                1.0,
                start_signal=StartSignal.GREEN,
                start_signal_valid=True,
            )
        )
        decision = self.manager.update(
            observation(
                1.2,
                start_signal=StartSignal.GREEN,
                start_signal_valid=False,
            )
        )

        self.assertIs(
            decision.mission_state, MissionState.WAIT_START_SIGNAL
        )
        self.assertTrue(self.manager.context.start_signal_armed)
        self.assertIsNone(self.manager.context.start_signal_seen_since)

    def test_not_safety_ready_clears_start_confirmation(self):
        self.manager.context.start_signal_armed = True
        self.manager.context.start_signal_seen_since = 1.0

        decision = self.manager.update(
            observation(
                1.1,
                start_signal=StartSignal.GREEN,
                start_signal_valid=True,
                safety_ready=False,
            )
        )

        self.assertIs(
            decision.mission_state, MissionState.WAIT_START_SIGNAL
        )
        self.assertIs(decision.control_mode, ControlMode.STOP)
        self.assertFalse(self.manager.context.start_signal_armed)
        self.assertIsNone(self.manager.context.start_signal_seen_since)

    def test_manual_start_is_immediate_and_selects_available_source(self):
        drive = manual_start(self.manager, drive=True)
        self.assertIs(drive.mission_state, MissionState.LANE_DRIVING)
        self.assertIs(drive.control_mode, ControlMode.NORMAL_IL)

        second = MissionManager(MissionManagerConfig())
        lane = manual_start(second, drive=False, lane=True)
        self.assertIs(lane.control_mode, ControlMode.LANE_FALLBACK)

        third = MissionManager(MissionManagerConfig())
        stopped = manual_start(third, drive=False, lane=False)
        self.assertIs(stopped.control_mode, ControlMode.STOP)

    def test_start_action_is_not_lost_when_mode_command_arrives_before_update(self):
        self.assertTrue(self.manager.set_manual_override("START"))
        self.assertTrue(self.manager.set_manual_override("NORMAL_IL"))

        decision = self.manager.update(
            observation(0.0, drive_policy_valid=True)
        )

        self.assertIs(
            decision.mission_state,
            MissionState.LANE_DRIVING,
        )
        self.assertIs(decision.control_mode, ControlMode.NORMAL_IL)
        self.assertEqual(self.manager.context.manual_override, "NORMAL_IL")

    def test_one_cone_frame_does_not_enter_cone_mode(self):
        manual_start(self.manager)
        one_frame = self.manager.update(
            observation(
                1.0,
                drive_policy_valid=True,
                **cone_entry_inputs(),
            )
        )
        after_dropout = self.manager.update(
            observation(1.1, drive_policy_valid=True)
        )

        self.assertIs(one_frame.control_mode, ControlMode.NORMAL_IL)
        self.assertIs(after_dropout.control_mode, ControlMode.NORMAL_IL)
        self.assertIsNone(self.manager.context.cone_seen_since)

    def test_held_valid_cone_detection_enters_at_boundary(self):
        manual_start(self.manager)
        self.manager.update(
            observation(
                2.0,
                drive_policy_valid=True,
                **cone_entry_inputs(),
            )
        )
        before = self.manager.update(
            observation(
                2.249,
                drive_policy_valid=True,
                **cone_entry_inputs(),
            )
        )
        at_boundary = self.manager.update(
            observation(
                2.25,
                drive_policy_valid=True,
                **cone_entry_inputs(),
            )
        )

        self.assertIs(before.control_mode, ControlMode.NORMAL_IL)
        self.assertIs(
            before.mission_state,
            MissionState.LANE_DRIVING,
        )
        self.assertIs(
            at_boundary.control_mode, ControlMode.CONE_DRIVE_RULE
        )
        self.assertIs(
            at_boundary.mission_state,
            MissionState.CONE_SECTION,
        )

    def test_lane_fallback_can_enter_cone_section(self):
        started = manual_start(
            self.manager,
            drive=False,
            lane=True,
        )
        self.assertIs(started.control_mode, ControlMode.LANE_FALLBACK)

        self.manager.update(
            observation(
                1.0,
                lane_fallback_valid=True,
                **cone_entry_inputs(),
            )
        )
        entered = self.manager.update(
            observation(
                1.25,
                lane_fallback_valid=True,
                **cone_entry_inputs(),
            )
        )

        self.assertIs(entered.mission_state, MissionState.CONE_SECTION)
        self.assertIs(entered.control_mode, ControlMode.CONE_DRIVE_RULE)

    def test_entry_requires_camera_count_and_ready_lidar_path(self):
        invalid_cases = (
            cone_entry_inputs(camera_count=3),
            cone_entry_inputs(camera_valid=False),
            cone_entry_inputs(lidar_path_ready=False),
            cone_entry_inputs(lidar_source_valid=False),
        )

        for inputs in invalid_cases:
            with self.subTest(inputs=inputs):
                manager = MissionManager(MissionManagerConfig())
                manual_start(manager)
                manager.update(
                    observation(1.0, drive_policy_valid=True, **inputs)
                )
                decision = manager.update(
                    observation(1.25, drive_policy_valid=True, **inputs)
                )

                self.assertIs(
                    decision.mission_state,
                    MissionState.LANE_DRIVING,
                )
                self.assertIs(decision.control_mode, ControlMode.NORMAL_IL)

    def test_interrupted_cone_entry_condition_resets_hold(self):
        manual_start(self.manager)
        self.manager.update(
            observation(
                1.0,
                drive_policy_valid=True,
                **cone_entry_inputs(),
            )
        )
        self.manager.update(
            observation(
                1.2,
                drive_policy_valid=True,
                **cone_entry_inputs(camera_count=3),
            )
        )
        not_yet = self.manager.update(
            observation(
                1.4,
                drive_policy_valid=True,
                **cone_entry_inputs(),
            )
        )

        self.assertIs(not_yet.control_mode, ControlMode.NORMAL_IL)

    def test_short_cone_dropout_does_not_exit(self):
        entered = enter_cone_mode(self.manager)
        self.assertIs(entered.control_mode, ControlMode.CONE_DRIVE_RULE)

        short = self.manager.update(
            observation(
                1.6,
                drive_policy_valid=True,
                **cone_exit_inputs(),
            )
        )
        recovered = self.manager.update(
            observation(
                2.0,
                drive_policy_valid=True,
                **cone_exit_inputs(camera_count=2),
            )
        )

        self.assertIs(short.control_mode, ControlMode.CONE_DRIVE_RULE)
        self.assertIs(recovered.control_mode, ControlMode.CONE_DRIVE_RULE)
        self.assertIsNone(self.manager.context.cone_missing_since)

    def test_three_camera_cones_prevent_exit_when_lidar_misses(self):
        enter_cone_mode(self.manager)
        inputs = cone_exit_inputs(camera_count=3)

        self.manager.update(observation(2.0, **inputs))
        decision = self.manager.update(observation(3.0, **inputs))

        self.assertIs(decision.mission_state, MissionState.CONE_SECTION)
        self.assertIs(decision.control_mode, ControlMode.CONE_DRIVE_RULE)
        self.assertIsNone(self.manager.context.cone_missing_since)

    def test_lidar_presence_prevents_exit_when_path_is_not_ready(self):
        enter_cone_mode(self.manager)
        inputs = cone_exit_inputs(lidar_present=True)

        self.manager.update(observation(2.0, **inputs))
        decision = self.manager.update(observation(3.0, **inputs))

        self.assertIs(decision.mission_state, MissionState.CONE_SECTION)
        self.assertIs(decision.control_mode, ControlMode.CONE_DRIVE_RULE)
        self.assertIsNone(self.manager.context.cone_missing_since)

    def test_invalid_sensor_input_never_counts_as_cone_absence(self):
        invalid_cases = (
            cone_exit_inputs(camera_valid=False),
            cone_exit_inputs(lidar_source_valid=False),
        )

        for inputs in invalid_cases:
            with self.subTest(inputs=inputs):
                manager = MissionManager(MissionManagerConfig())
                enter_cone_mode(manager)
                manager.update(observation(2.0, **inputs))
                decision = manager.update(observation(3.0, **inputs))

                self.assertIs(
                    decision.mission_state,
                    MissionState.CONE_SECTION,
                )
                self.assertIsNone(manager.context.cone_missing_since)

    def test_cone_exit_requires_minimum_dwell_and_missing_hold(self):
        enter_cone_mode(self.manager)
        self.manager.update(
            observation(
                1.3,
                drive_policy_valid=True,
                **cone_exit_inputs(),
            )
        )
        dwell_not_done = self.manager.update(
            observation(
                2.0,
                drive_policy_valid=True,
                **cone_exit_inputs(),
            )
        )
        both_done = self.manager.update(
            observation(
                2.25,
                drive_policy_valid=True,
                **cone_exit_inputs(),
            )
        )

        self.assertIs(
            dwell_not_done.control_mode, ControlMode.CONE_DRIVE_RULE
        )
        self.assertIs(
            dwell_not_done.mission_state,
            MissionState.CONE_SECTION,
        )
        self.assertIs(both_done.control_mode, ControlMode.NORMAL_IL)
        self.assertIs(
            both_done.mission_state,
            MissionState.LANE_DRIVING,
        )

    def test_cone_exit_missing_hold_is_independently_required(self):
        enter_cone_mode(self.manager)
        self.manager.update(
            observation(
                2.25,
                drive_policy_valid=True,
                **cone_exit_inputs(),
            )
        )
        before = self.manager.update(
            observation(
                2.949,
                drive_policy_valid=True,
                **cone_exit_inputs(),
            )
        )
        at_boundary = self.manager.update(
            observation(
                2.95,
                drive_policy_valid=True,
                **cone_exit_inputs(),
            )
        )

        self.assertIs(before.control_mode, ControlMode.CONE_DRIVE_RULE)
        self.assertIs(
            before.mission_state,
            MissionState.CONE_SECTION,
        )
        self.assertIs(at_boundary.control_mode, ControlMode.NORMAL_IL)
        self.assertIs(
            at_boundary.mission_state,
            MissionState.LANE_DRIVING,
        )

    def test_cone_exit_selects_lane_or_stop_when_drive_is_invalid(self):
        enter_cone_mode(self.manager)
        self.manager.update(
            observation(2.0, **cone_exit_inputs())
        )
        lane = self.manager.update(
            observation(
                2.7,
                lane_fallback_valid=True,
                **cone_exit_inputs(),
            )
        )
        self.assertIs(lane.control_mode, ControlMode.LANE_FALLBACK)
        self.assertIs(
            lane.mission_state,
            MissionState.LANE_DRIVING,
        )

        other = MissionManager(MissionManagerConfig())
        enter_cone_mode(other)
        other.update(
            observation(2.0, **cone_exit_inputs())
        )
        stopped = other.update(
            observation(2.7, **cone_exit_inputs())
        )
        self.assertIs(stopped.control_mode, ControlMode.STOP)
        self.assertIs(
            stopped.mission_state,
            MissionState.LANE_DRIVING,
        )

    def test_cone_reentry_cooldown_and_new_hold_are_required(self):
        enter_cone_mode(self.manager)
        self.manager.update(
            observation(
                2.0,
                drive_policy_valid=True,
                **cone_exit_inputs(),
            )
        )
        exited = self.manager.update(
            observation(
                2.7,
                drive_policy_valid=True,
                **cone_exit_inputs(),
            )
        )
        self.assertIs(exited.control_mode, ControlMode.NORMAL_IL)

        common = {"drive_policy_valid": True, **cone_entry_inputs()}
        self.manager.update(observation(2.8, **common))
        blocked = self.manager.update(observation(3.69, **common))
        cooldown_boundary = self.manager.update(
            observation(3.70, **common)
        )
        hold_boundary = self.manager.update(observation(3.95, **common))

        self.assertIs(blocked.control_mode, ControlMode.NORMAL_IL)
        self.assertIs(
            cooldown_boundary.control_mode, ControlMode.NORMAL_IL
        )
        self.assertIs(
            hold_boundary.control_mode, ControlMode.CONE_DRIVE_RULE
        )

    def test_general_failure_stops_until_fallback_is_ready(self):
        manual_start(self.manager)
        first = self.manager.update(
            observation(
                1.0,
                drive_policy_valid=False,
                lane_fallback_valid=True,
            )
        )
        before = self.manager.update(
            observation(
                1.199,
                drive_policy_valid=False,
                lane_fallback_valid=True,
            )
        )
        at_boundary = self.manager.update(
            observation(
                1.2,
                drive_policy_valid=False,
                lane_fallback_valid=True,
            )
        )

        self.assertIs(first.control_mode, ControlMode.STOP)
        self.assertIs(before.control_mode, ControlMode.STOP)
        self.assertIs(at_boundary.control_mode, ControlMode.LANE_FALLBACK)

    def test_preconfirmed_fallback_is_selected_on_first_failure_tick(self):
        manual_start(self.manager)
        self.manager.update(
            observation(
                1.0,
                drive_policy_valid=True,
                lane_fallback_valid=True,
            )
        )
        self.manager.update(
            observation(
                1.2,
                drive_policy_valid=True,
                lane_fallback_valid=True,
            )
        )
        switched = self.manager.update(
            observation(
                1.21,
                drive_policy_valid=False,
                lane_fallback_valid=True,
            )
        )

        self.assertIs(switched.control_mode, ControlMode.LANE_FALLBACK)

    def test_fallback_readiness_resets_when_fallback_is_invalid(self):
        manual_start(self.manager)
        self.manager.update(
            observation(
                1.0,
                drive_policy_valid=True,
                lane_fallback_valid=True,
            )
        )
        self.manager.update(
            observation(
                1.1,
                drive_policy_valid=True,
                lane_fallback_valid=False,
            )
        )
        first = self.manager.update(
            observation(
                1.2,
                drive_policy_valid=False,
                lane_fallback_valid=True,
            )
        )
        before = self.manager.update(
            observation(
                1.399,
                drive_policy_valid=False,
                lane_fallback_valid=True,
            )
        )
        ready = self.manager.update(
            observation(
                1.4,
                drive_policy_valid=False,
                lane_fallback_valid=True,
            )
        )

        self.assertIs(first.control_mode, ControlMode.STOP)
        self.assertIs(before.control_mode, ControlMode.STOP)
        self.assertIs(ready.control_mode, ControlMode.LANE_FALLBACK)

    def test_drive_recovery_returns_to_normal_after_hold(self):
        manual_start(self.manager, drive=False, lane=True)
        first = self.manager.update(
            observation(
                1.0,
                drive_policy_valid=True,
                lane_fallback_valid=True,
            )
        )
        before = self.manager.update(
            observation(
                1.399,
                drive_policy_valid=True,
                lane_fallback_valid=True,
            )
        )
        at_boundary = self.manager.update(
            observation(
                1.4,
                drive_policy_valid=True,
                lane_fallback_valid=True,
            )
        )

        self.assertIs(first.control_mode, ControlMode.LANE_FALLBACK)
        self.assertIs(before.control_mode, ControlMode.LANE_FALLBACK)
        self.assertIs(at_boundary.control_mode, ControlMode.NORMAL_IL)

    def test_drive_recovery_confirmation_resets_when_interrupted(self):
        manual_start(self.manager, drive=False, lane=True)
        self.manager.update(
            observation(
                1.0,
                drive_policy_valid=True,
                lane_fallback_valid=True,
            )
        )
        self.manager.update(
            observation(
                1.2,
                drive_policy_valid=False,
                lane_fallback_valid=True,
            )
        )
        self.manager.update(
            observation(
                1.3,
                drive_policy_valid=True,
                lane_fallback_valid=True,
            )
        )
        not_yet = self.manager.update(
            observation(
                1.6,
                drive_policy_valid=True,
                lane_fallback_valid=True,
            )
        )

        self.assertIs(not_yet.control_mode, ControlMode.LANE_FALLBACK)

    def test_no_valid_controller_uses_stop(self):
        manual_start(self.manager, drive=False, lane=True)
        stopped = self.manager.update(observation(1.0))
        self.assertIs(
            stopped.mission_state,
            MissionState.LANE_DRIVING,
        )
        self.assertIs(stopped.control_mode, ControlMode.STOP)
        self.assertTrue(stopped.stop_required)

    def test_lane_stop_recovery_priority(self):
        manual_start(self.manager, drive=False, lane=False)
        waiting = self.manager.update(
            observation(
                1.0,
                drive_policy_valid=True,
                lane_fallback_valid=True,
            )
        )
        fallback = self.manager.update(
            observation(
                1.2,
                drive_policy_valid=True,
                lane_fallback_valid=True,
            )
        )
        normal = self.manager.update(
            observation(
                1.4,
                drive_policy_valid=True,
                lane_fallback_valid=True,
            )
        )
        self.assertIs(waiting.control_mode, ControlMode.STOP)
        self.assertIs(fallback.control_mode, ControlMode.LANE_FALLBACK)
        self.assertIs(normal.control_mode, ControlMode.NORMAL_IL)

        lane_manager = MissionManager(MissionManagerConfig())
        manual_start(lane_manager, drive=False, lane=False)
        lane_wait = lane_manager.update(
            observation(1.0, lane_fallback_valid=True)
        )
        lane = lane_manager.update(
            observation(1.2, lane_fallback_valid=True)
        )
        self.assertIs(lane_wait.control_mode, ControlMode.STOP)
        self.assertIs(lane.control_mode, ControlMode.LANE_FALLBACK)

        cone_manager = MissionManager(MissionManagerConfig())
        manual_start(cone_manager, drive=False, lane=False)
        cone_manager.update(
            observation(
                1.0,
                **cone_entry_inputs(),
            )
        )
        cone = cone_manager.update(
            observation(
                1.25,
                drive_policy_valid=True,
                lane_fallback_valid=True,
                **cone_entry_inputs(),
            )
        )
        self.assertIs(cone.control_mode, ControlMode.CONE_DRIVE_RULE)

    def test_future_observation_flags_never_select_placeholder_modes(self):
        manual_start(self.manager, drive=True)
        placeholder_modes = {
            ControlMode.FIXED_OBSTACLE_RULE,
            ControlMode.VEHICLE_FOLLOW,
            ControlMode.VEHICLE_OVERTAKE,
            ControlMode.SHORTCUT_RULE,
        }
        for timestamp in (1.0, 2.0, 3.0):
            decision = self.manager.update(
                observation(
                    timestamp,
                    drive_policy_valid=True,
                    fixed_obstacle_detected=True,
                    vehicle_detected=True,
                    shortcut_signal_detected=True,
                    lap_crossing_detected=True,
                )
            )
            self.assertNotIn(decision.control_mode, placeholder_modes)
            self.assertIs(
                decision.mission_state,
                MissionState.LANE_DRIVING,
            )
        self.assertEqual(self.manager.context.lap_count, 0)
        self.assertFalse(self.manager.context.shortcut_used)


class ManualAndSafetyStopTest(unittest.TestCase):
    def setUp(self):
        self.manager = MissionManager(MissionManagerConfig())

    def test_forced_modes_in_wait_do_not_start_mission(self):
        for command in (
            "NORMAL_IL",
            "CONE_DRIVE_RULE",
            "LANE_FALLBACK",
            "STOP",
        ):
            manager = MissionManager(MissionManagerConfig())
            self.assertTrue(manager.set_manual_override(command))
            decision = manager.update(
                observation(
                    0.0,
                    drive_policy_valid=True,
                    lane_fallback_valid=True,
                )
            )
            self.assertIs(
                decision.mission_state, MissionState.WAIT_START_SIGNAL
            )
            self.assertIs(decision.control_mode, ControlMode.STOP)

    def test_manual_normal_requires_validity(self):
        manual_start(self.manager)
        self.manager.set_manual_override("NORMAL_IL")
        valid = self.manager.update(
            observation(1.0, drive_policy_valid=True)
        )
        invalid = self.manager.update(observation(1.1))
        self.assertIs(valid.control_mode, ControlMode.NORMAL_IL)
        self.assertIs(invalid.control_mode, ControlMode.STOP)

    def test_manual_cone_forces_integration_mode(self):
        manual_start(self.manager)
        self.manager.set_manual_override("CONE_DRIVE_RULE")
        decision = self.manager.update(observation(1.0))
        self.assertIs(decision.control_mode, ControlMode.CONE_DRIVE_RULE)
        self.assertIs(
            decision.mission_state,
            MissionState.CONE_SECTION,
        )

    def test_manual_lane_requires_validity(self):
        manual_start(self.manager)
        self.manager.set_manual_override("LANE_FALLBACK")
        invalid = self.manager.update(observation(1.0))
        valid = self.manager.update(
            observation(1.1, lane_fallback_valid=True)
        )
        self.assertIs(invalid.control_mode, ControlMode.STOP)
        self.assertIs(valid.control_mode, ControlMode.LANE_FALLBACK)

    def test_manual_stop_preserves_mission_state_and_auto_recovers(self):
        manual_start(self.manager)
        self.manager.set_manual_override("STOP")
        stopped = self.manager.update(
            observation(1.0, drive_policy_valid=True)
        )
        self.manager.set_manual_override("AUTO")
        waiting = self.manager.update(
            observation(1.1, drive_policy_valid=True)
        )
        before = self.manager.update(
            observation(1.499, drive_policy_valid=True)
        )
        resumed = self.manager.update(
            observation(1.5, drive_policy_valid=True)
        )
        self.assertIs(
            stopped.mission_state,
            MissionState.LANE_DRIVING,
        )
        self.assertIs(stopped.control_mode, ControlMode.STOP)
        self.assertIs(waiting.control_mode, ControlMode.STOP)
        self.assertIs(before.control_mode, ControlMode.STOP)
        self.assertIs(resumed.control_mode, ControlMode.NORMAL_IL)

    def test_unknown_override_is_ignored(self):
        self.assertFalse(self.manager.set_manual_override("SHORTCUT"))
        self.assertEqual(self.manager.context.manual_override, "AUTO")

    def test_safety_stop_preserves_every_active_mission_state(self):
        cases = (
            ("AUTO", dict(drive_policy_valid=True)),
            ("NORMAL_IL", dict(drive_policy_valid=True)),
            ("CONE_DRIVE_RULE", {}),
            ("LANE_FALLBACK", dict(lane_fallback_valid=True)),
            ("STOP", {}),
        )
        for command, inputs in cases:
            with self.subTest(command=command):
                manager = MissionManager(MissionManagerConfig())
                manual_start(manager, drive=True)
                manager.set_manual_override(command)
                active = manager.update(observation(1.0, **inputs))
                stopped = manager.update(
                    observation(
                        1.1,
                        safety_stop_required=True,
                        **inputs,
                    )
                )
                self.assertIs(
                    stopped.mission_state, active.mission_state
                )
                self.assertIs(stopped.control_mode, ControlMode.STOP)
                self.assertTrue(stopped.stop_required)

    def test_safety_stop_is_not_latched_and_recovers_in_same_state(self):
        manual_start(self.manager)
        stopped = self.manager.update(
            observation(
                1.0,
                safety_stop_required=True,
                drive_policy_valid=True,
            )
        )
        waiting = self.manager.update(
            observation(1.1, drive_policy_valid=True)
        )
        before = self.manager.update(
            observation(1.499, drive_policy_valid=True)
        )
        resumed = self.manager.update(
            observation(1.5, drive_policy_valid=True)
        )

        self.assertIs(
            stopped.mission_state, MissionState.LANE_DRIVING
        )
        self.assertIs(stopped.control_mode, ControlMode.STOP)
        self.assertIs(waiting.control_mode, ControlMode.STOP)
        self.assertIs(before.control_mode, ControlMode.STOP)
        self.assertIs(resumed.mission_state, MissionState.LANE_DRIVING)
        self.assertIs(resumed.control_mode, ControlMode.NORMAL_IL)

    def test_safety_stop_in_cone_is_not_recorded_as_cone_exit(self):
        enter_cone_mode(self.manager)

        stopped = self.manager.update(
            observation(1.3, safety_stop_required=True)
        )
        resumed = self.manager.update(
            observation(1.4, **cone_entry_inputs())
        )

        self.assertIs(stopped.mission_state, MissionState.CONE_SECTION)
        self.assertIs(stopped.control_mode, ControlMode.STOP)
        self.assertIsNone(self.manager._cone_exit_sec)
        self.assertIs(resumed.mission_state, MissionState.CONE_SECTION)
        self.assertIs(resumed.control_mode, ControlMode.CONE_DRIVE_RULE)

    def test_wait_safety_stop_clears_start_arm(self):
        self.manager.context.start_signal_armed = True
        self.manager.context.start_signal_seen_since = 0.0

        decision = self.manager.update(
            observation(0.1, safety_stop_required=True)
        )

        self.assertIs(
            decision.mission_state, MissionState.WAIT_START_SIGNAL
        )
        self.assertIs(decision.control_mode, ControlMode.STOP)
        self.assertFalse(self.manager.context.start_signal_armed)
        self.assertIsNone(self.manager.context.start_signal_seen_since)

    def test_removed_emergency_commands_are_rejected(self):
        self.assertFalse(
            self.manager.set_manual_override("EMERGENCY_STOP")
        )
        self.assertFalse(
            self.manager.set_manual_override("RESET_EMERGENCY")
        )

    def test_manual_start_is_not_queued_during_safety_stop(self):
        self.assertTrue(self.manager.set_manual_override("START"))
        blocked = self.manager.update(
            observation(
                0.0,
                safety_stop_required=True,
                drive_policy_valid=True,
            )
        )
        later = self.manager.update(
            observation(0.1, drive_policy_valid=True)
        )

        self.assertIs(
            blocked.mission_state, MissionState.WAIT_START_SIGNAL
        )
        self.assertIs(
            later.mission_state, MissionState.WAIT_START_SIGNAL
        )

    def test_manual_start_is_not_queued_while_safety_is_not_ready(self):
        self.assertTrue(self.manager.set_manual_override("START"))
        blocked = self.manager.update(
            observation(0.0, safety_ready=False, drive_policy_valid=True)
        )
        later = self.manager.update(
            observation(0.1, safety_ready=True, drive_policy_valid=True)
        )

        self.assertIs(
            blocked.mission_state, MissionState.WAIT_START_SIGNAL
        )
        self.assertIs(
            later.mission_state, MissionState.WAIT_START_SIGNAL
        )


class LoggingTest(unittest.TestCase):
    def test_logging_occurs_on_change_not_every_update(self):
        manager = MissionManager(
            MissionManagerConfig(status_log_period_sec=100.0)
        )
        messages = []
        manager.set_logger(messages.append)

        manager.update(observation(0.0))
        manager.update(observation(0.1))
        manager.update(observation(0.2))
        manual_start(manager, 0.3, drive=True)
        manager.update(observation(0.4, drive_policy_valid=True))

        self.assertEqual(
            len(messages),
            2,
        )
        self.assertIn("event=change", messages[0])
        self.assertIn("zone=WAIT_START_SIGNAL", messages[0])
        self.assertIn("control_mode=STOP", messages[0])
        self.assertIn("motion_request=STOP", messages[0])
        self.assertIn("stop_required=true", messages[0])
        self.assertIn("zone=LANE_DRIVING", messages[1])
        self.assertIn("control_mode=NORMAL_IL", messages[1])
        self.assertIn("selected_source=drive_il", messages[1])
        self.assertIn("motion_request=DRIVE", messages[1])
        self.assertIn("stop_required=false", messages[1])

    def test_periodic_status_is_throttled_and_contains_current_values(self):
        manager = MissionManager(
            MissionManagerConfig(status_log_period_sec=1.0)
        )
        messages = []
        manager.set_logger(messages.append)

        manager.update(observation(0.0))
        manager.update(observation(0.99))
        manager.update(observation(
            1.0,
            drive_policy_valid=True,
            lane_fallback_valid=True,
            camera_cone_valid=True,
            camera_cone_count=3,
            lidar_cone_source_valid=True,
            lidar_cone_path_ready=True,
            lidar_cone_present=True,
        ))

        self.assertEqual(len(messages), 2)
        status = messages[1]
        required_fields = (
            "event=status",
            "zone=WAIT_START_SIGNAL",
            "control_mode=STOP",
            "selected_source=none",
            "speed_profile=stop",
            "motion_request=STOP",
            "stop_required=true",
            "zone_age_sec=1.00",
            "mode_age_sec=1.00",
            "drive_policy_valid=true",
            "lane_fallback_valid=true",
            "camera_cones=3",
            "camera_valid=true",
            "lidar_path_ready=true",
            "lidar_present=true",
            "lidar_valid=true",
            "start_signal=UNKNOWN",
            "start_valid=false",
            "start_armed=false",
            "safety_ready=true",
            "safety_stop=false",
            "override=AUTO",
            "lap=0",
            "shortcut_used=false",
        )
        for field in required_fields:
            with self.subTest(field=field):
                self.assertIn(field, status)


if __name__ == "__main__":
    unittest.main()
