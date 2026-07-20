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


def enter_cone_mode(manager):
    manual_start(manager, 0.0, drive=True)
    manager.update(
        observation(
            1.0,
            drive_policy_valid=True,
            cone_detected=True,
            cone_count=2,
            cone_confidence=0.5,
        )
    )
    return manager.update(
        observation(
            1.25,
            drive_policy_valid=True,
            cone_detected=True,
            cone_count=2,
            cone_confidence=0.5,
        )
    )


class DataModelTest(unittest.TestCase):
    def test_enum_members_are_exact(self):
        self.assertEqual(
            list(MissionState.__members__),
            ["WAIT_START_SIGNAL", "RACING", "EMERGENCY_STOP"],
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
                "ROUTE_SELECT",
                "SHORTCUT",
            ],
        )
        self.assertNotIn("BOOT", MissionState.__members__)
        self.assertNotIn("FINISHED", MissionState.__members__)

    def test_observation_fields_are_exact_and_frozen(self):
        self.assertEqual(
            [item.name for item in fields(MissionObservation)],
            [
                "now_sec",
                "emergency_stop",
                "start_signal_go",
                "drive_policy_valid",
                "lane_fallback_valid",
                "cone_detected",
                "cone_confidence",
                "cone_count",
                "cone_exit_ready",
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
                "start_signal_seen_since",
                "cone_seen_since",
                "cone_missing_since",
                "cone_last_seen_sec",
                "drive_valid_since",
                "lane_fallback_valid_since",
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
                start_signal_hold_sec=0.3,
                cone_enter_hold_sec=0.25,
                cone_exit_hold_sec=0.7,
                cone_min_dwell_sec=1.0,
                cone_reenter_cooldown_sec=1.0,
                drive_recover_hold_sec=0.4,
                lane_fallback_enter_hold_sec=0.2,
                minimum_cone_count=2,
                minimum_cone_confidence=0.5,
                status_log_period_sec=1.0,
            ),
        )


class MissionManagerTransitionTest(unittest.TestCase):
    def setUp(self):
        self.manager = MissionManager(MissionManagerConfig())

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

    def test_short_start_signal_does_not_start_and_resets_timer(self):
        self.manager.update(
            observation(0.0, start_signal_go=True, drive_policy_valid=True)
        )
        short = self.manager.update(
            observation(0.29, start_signal_go=True, drive_policy_valid=True)
        )
        self.manager.update(observation(0.30, start_signal_go=False))
        restarted = self.manager.update(
            observation(0.50, start_signal_go=True, drive_policy_valid=True)
        )

        self.assertIs(short.mission_state, MissionState.WAIT_START_SIGNAL)
        self.assertIs(
            restarted.mission_state, MissionState.WAIT_START_SIGNAL
        )

    def test_held_start_signal_enters_racing_at_boundary(self):
        self.manager.update(
            observation(1.0, start_signal_go=True, drive_policy_valid=True)
        )
        before = self.manager.update(
            observation(1.299, start_signal_go=True, drive_policy_valid=True)
        )
        at_boundary = self.manager.update(
            observation(1.30, start_signal_go=True, drive_policy_valid=True)
        )

        self.assertIs(before.mission_state, MissionState.WAIT_START_SIGNAL)
        self.assertIs(at_boundary.mission_state, MissionState.RACING)
        self.assertIs(at_boundary.control_mode, ControlMode.NORMAL_IL)

    def test_manual_start_is_immediate_and_selects_available_source(self):
        drive = manual_start(self.manager, drive=True)
        self.assertIs(drive.mission_state, MissionState.RACING)
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

        self.assertIs(decision.mission_state, MissionState.RACING)
        self.assertIs(decision.control_mode, ControlMode.NORMAL_IL)
        self.assertEqual(self.manager.context.manual_override, "NORMAL_IL")

    def test_one_cone_frame_does_not_enter_cone_mode(self):
        manual_start(self.manager)
        one_frame = self.manager.update(
            observation(
                1.0,
                drive_policy_valid=True,
                cone_detected=True,
                cone_count=2,
                cone_confidence=0.5,
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
                cone_detected=True,
                cone_count=2,
                cone_confidence=0.5,
            )
        )
        before = self.manager.update(
            observation(
                2.249,
                drive_policy_valid=True,
                cone_detected=True,
                cone_count=2,
                cone_confidence=0.5,
            )
        )
        at_boundary = self.manager.update(
            observation(
                2.25,
                drive_policy_valid=True,
                cone_detected=True,
                cone_count=2,
                cone_confidence=0.5,
            )
        )

        self.assertIs(before.control_mode, ControlMode.NORMAL_IL)
        self.assertIs(
            at_boundary.control_mode, ControlMode.CONE_DRIVE_RULE
        )

    def test_low_cone_count_or_confidence_resets_entry_hold(self):
        manual_start(self.manager)
        self.manager.update(
            observation(
                1.0,
                drive_policy_valid=True,
                cone_detected=True,
                cone_count=2,
                cone_confidence=0.5,
            )
        )
        self.manager.update(
            observation(
                1.2,
                drive_policy_valid=True,
                cone_detected=True,
                cone_count=1,
                cone_confidence=0.4,
            )
        )
        not_yet = self.manager.update(
            observation(
                1.4,
                drive_policy_valid=True,
                cone_detected=True,
                cone_count=2,
                cone_confidence=0.5,
            )
        )

        self.assertIs(not_yet.control_mode, ControlMode.NORMAL_IL)

    def test_short_cone_dropout_does_not_exit(self):
        entered = enter_cone_mode(self.manager)
        self.assertIs(entered.control_mode, ControlMode.CONE_DRIVE_RULE)

        short = self.manager.update(
            observation(
                1.6,
                cone_detected=False,
                cone_exit_ready=True,
                drive_policy_valid=True,
            )
        )
        recovered = self.manager.update(
            observation(
                2.0,
                cone_detected=True,
                cone_exit_ready=True,
                drive_policy_valid=True,
            )
        )

        self.assertIs(short.control_mode, ControlMode.CONE_DRIVE_RULE)
        self.assertIs(recovered.control_mode, ControlMode.CONE_DRIVE_RULE)
        self.assertIsNone(self.manager.context.cone_missing_since)

    def test_cone_exit_requires_minimum_dwell_and_missing_hold(self):
        enter_cone_mode(self.manager)
        self.manager.update(
            observation(
                1.3,
                cone_detected=False,
                cone_exit_ready=True,
                drive_policy_valid=True,
            )
        )
        dwell_not_done = self.manager.update(
            observation(
                2.0,
                cone_detected=False,
                cone_exit_ready=True,
                drive_policy_valid=True,
            )
        )
        both_done = self.manager.update(
            observation(
                2.25,
                cone_detected=False,
                cone_exit_ready=True,
                drive_policy_valid=True,
            )
        )

        self.assertIs(
            dwell_not_done.control_mode, ControlMode.CONE_DRIVE_RULE
        )
        self.assertIs(both_done.control_mode, ControlMode.NORMAL_IL)

    def test_cone_exit_missing_hold_is_independently_required(self):
        enter_cone_mode(self.manager)
        self.manager.update(
            observation(
                2.25,
                cone_detected=False,
                cone_exit_ready=True,
                drive_policy_valid=True,
            )
        )
        before = self.manager.update(
            observation(
                2.949,
                cone_detected=False,
                cone_exit_ready=True,
                drive_policy_valid=True,
            )
        )
        at_boundary = self.manager.update(
            observation(
                2.95,
                cone_detected=False,
                cone_exit_ready=True,
                drive_policy_valid=True,
            )
        )

        self.assertIs(before.control_mode, ControlMode.CONE_DRIVE_RULE)
        self.assertIs(at_boundary.control_mode, ControlMode.NORMAL_IL)

    def test_cone_exit_selects_lane_or_stop_when_drive_is_invalid(self):
        enter_cone_mode(self.manager)
        self.manager.update(
            observation(2.0, cone_detected=False, cone_exit_ready=True)
        )
        lane = self.manager.update(
            observation(
                2.7,
                cone_detected=False,
                cone_exit_ready=True,
                lane_fallback_valid=True,
            )
        )
        self.assertIs(lane.control_mode, ControlMode.LANE_FALLBACK)

        other = MissionManager(MissionManagerConfig())
        enter_cone_mode(other)
        other.update(
            observation(2.0, cone_detected=False, cone_exit_ready=True)
        )
        stopped = other.update(
            observation(2.7, cone_detected=False, cone_exit_ready=True)
        )
        self.assertIs(stopped.control_mode, ControlMode.STOP)

    def test_cone_reentry_cooldown_and_new_hold_are_required(self):
        enter_cone_mode(self.manager)
        self.manager.update(
            observation(
                2.0,
                cone_detected=False,
                cone_exit_ready=True,
                drive_policy_valid=True,
            )
        )
        exited = self.manager.update(
            observation(
                2.7,
                cone_detected=False,
                cone_exit_ready=True,
                drive_policy_valid=True,
            )
        )
        self.assertIs(exited.control_mode, ControlMode.NORMAL_IL)

        common = dict(
            drive_policy_valid=True,
            cone_detected=True,
            cone_count=2,
            cone_confidence=0.5,
        )
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

    def test_general_failure_enters_lane_after_confirmation(self):
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

        self.assertIs(first.control_mode, ControlMode.NORMAL_IL)
        self.assertIs(before.control_mode, ControlMode.NORMAL_IL)
        self.assertIs(at_boundary.control_mode, ControlMode.LANE_FALLBACK)

    def test_general_failure_confirmation_resets_when_interrupted(self):
        manual_start(self.manager)
        self.manager.update(
            observation(
                1.0,
                drive_policy_valid=False,
                lane_fallback_valid=True,
            )
        )
        self.manager.update(
            observation(
                1.1,
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
        not_yet = self.manager.update(
            observation(
                1.3,
                drive_policy_valid=False,
                lane_fallback_valid=True,
            )
        )
        self.assertIs(not_yet.control_mode, ControlMode.NORMAL_IL)

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
        self.assertIs(stopped.mission_state, MissionState.RACING)
        self.assertIs(stopped.control_mode, ControlMode.STOP)
        self.assertTrue(stopped.stop_required)

    def test_racing_stop_recovery_priority(self):
        manual_start(self.manager, drive=False, lane=False)
        normal = self.manager.update(
            observation(
                1.0,
                drive_policy_valid=True,
                lane_fallback_valid=True,
            )
        )
        self.assertIs(normal.control_mode, ControlMode.NORMAL_IL)

        lane_manager = MissionManager(MissionManagerConfig())
        manual_start(lane_manager, drive=False, lane=False)
        lane = lane_manager.update(
            observation(1.0, lane_fallback_valid=True)
        )
        self.assertIs(lane.control_mode, ControlMode.LANE_FALLBACK)

        cone_manager = MissionManager(MissionManagerConfig())
        manual_start(cone_manager, drive=False, lane=False)
        cone_manager.update(
            observation(
                1.0,
                cone_detected=True,
                cone_count=2,
                cone_confidence=0.5,
            )
        )
        cone = cone_manager.update(
            observation(
                1.25,
                drive_policy_valid=True,
                lane_fallback_valid=True,
                cone_detected=True,
                cone_count=2,
                cone_confidence=0.5,
            )
        )
        self.assertIs(cone.control_mode, ControlMode.CONE_DRIVE_RULE)

    def test_future_observation_flags_never_select_placeholder_modes(self):
        manual_start(self.manager, drive=True)
        placeholder_modes = {
            ControlMode.FIXED_OBSTACLE_RULE,
            ControlMode.VEHICLE_FOLLOW,
            ControlMode.VEHICLE_OVERTAKE,
            ControlMode.ROUTE_SELECT,
            ControlMode.SHORTCUT,
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
        self.assertEqual(self.manager.context.lap_count, 0)
        self.assertFalse(self.manager.context.shortcut_used)


class ManualAndEmergencyTest(unittest.TestCase):
    def setUp(self):
        self.manager = MissionManager(MissionManagerConfig())

    def test_forced_modes_in_wait_do_not_start_racing(self):
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
        resumed = self.manager.update(
            observation(1.1, drive_policy_valid=True)
        )
        self.assertIs(stopped.mission_state, MissionState.RACING)
        self.assertIs(stopped.control_mode, ControlMode.STOP)
        self.assertIs(resumed.control_mode, ControlMode.NORMAL_IL)

    def test_unknown_override_is_ignored(self):
        self.assertFalse(self.manager.set_manual_override("SHORTCUT"))
        self.assertEqual(self.manager.context.manual_override, "AUTO")

    def test_emergency_stop_overrides_every_active_mode(self):
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
                manager.update(observation(1.0, **inputs))
                emergency = manager.update(
                    observation(1.1, emergency_stop=True, **inputs)
                )
                self.assertIs(
                    emergency.mission_state, MissionState.EMERGENCY_STOP
                )
                self.assertIs(emergency.control_mode, ControlMode.STOP)
                self.assertTrue(emergency.stop_required)

    def test_manual_start_and_normal_overrides_cannot_bypass_emergency(self):
        self.manager.update(observation(0.0, emergency_stop=True))
        self.assertFalse(self.manager.set_manual_override("START"))
        self.assertFalse(self.manager.set_manual_override("NORMAL_IL"))
        decision = self.manager.update(
            observation(1.0, drive_policy_valid=True)
        )
        self.assertIs(decision.mission_state, MissionState.EMERGENCY_STOP)

    def test_reset_returns_to_wait_and_does_not_auto_start(self):
        manual_start(self.manager)
        self.manager.update(observation(1.0, emergency_stop=True))
        self.assertTrue(self.manager.set_manual_override("RESET_EMERGENCY"))
        reset = self.manager.update(
            observation(1.1, drive_policy_valid=True)
        )
        self.assertIs(reset.mission_state, MissionState.WAIT_START_SIGNAL)
        self.assertIs(reset.control_mode, ControlMode.STOP)
        self.assertEqual(self.manager.context.manual_override, "AUTO")

    def test_reset_is_ineffective_while_physical_emergency_is_true(self):
        self.manager.update(observation(0.0, emergency_stop=True))
        self.manager.set_manual_override("RESET_EMERGENCY")
        still_emergency = self.manager.update(
            observation(1.0, emergency_stop=True)
        )
        cleared_without_new_reset = self.manager.update(observation(1.1))

        self.assertIs(
            still_emergency.mission_state, MissionState.EMERGENCY_STOP
        )
        self.assertIs(
            cleared_without_new_reset.mission_state,
            MissionState.EMERGENCY_STOP,
        )

        self.manager.set_manual_override("RESET_EMERGENCY")
        reset = self.manager.update(observation(1.2))
        self.assertIs(reset.mission_state, MissionState.WAIT_START_SIGNAL)

    def test_new_emergency_command_cancels_pending_reset(self):
        self.manager.update(observation(0.0, emergency_stop=True))
        self.assertTrue(
            self.manager.set_manual_override("RESET_EMERGENCY")
        )
        self.assertTrue(
            self.manager.set_manual_override("EMERGENCY_STOP")
        )

        decision = self.manager.update(observation(0.1))

        self.assertIs(
            decision.mission_state, MissionState.EMERGENCY_STOP
        )
        self.assertIs(decision.control_mode, ControlMode.STOP)

    def test_manual_emergency_is_latched_before_update(self):
        self.assertTrue(self.manager.set_manual_override("EMERGENCY_STOP"))
        self.assertFalse(self.manager.set_manual_override("AUTO"))
        emergency = self.manager.update(observation(0.0))
        self.assertIs(emergency.mission_state, MissionState.EMERGENCY_STOP)


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
            messages,
            [
                "[MISSION] state=WAIT_START_SIGNAL mode=STOP",
                "[MISSION] state=RACING mode=NORMAL_IL",
            ],
        )

    def test_periodic_status_is_throttled_and_contains_current_values(self):
        manager = MissionManager(
            MissionManagerConfig(status_log_period_sec=1.0)
        )
        messages = []
        manager.set_logger(messages.append)

        manager.update(observation(0.0))
        manager.update(observation(0.99))
        manager.update(observation(1.0))

        self.assertEqual(len(messages), 2)
        self.assertEqual(
            messages[1],
            "[MISSION] state=WAIT_START_SIGNAL mode=STOP "
            "lap=0 shortcut_used=false override=AUTO",
        )


if __name__ == "__main__":
    unittest.main()
