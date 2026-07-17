import unittest

import torch

from xycar_rl.camera_speed_models import (
    CameraSpeedActor,
    CompactCameraSpeedActor,
    denormalize_speed_command,
    initialize_temporal_actor,
    normalize_speed_command,
)
from xycar_rl.td3_bc import CameraSpeedTD3BCAgent, TD3BCConfig
from xycar_rl.train_camera_speed_bc import (
    speed_target_from_transition,
    steering_sample_weight,
    transition_rows_are_contiguous,
)


class CameraSpeedContractTest(unittest.TestCase):
    def test_steering_sample_weight_emphasizes_curves(self):
        self.assertEqual(steering_sample_weight(0.0, 4.0), 1.0)
        self.assertAlmostEqual(steering_sample_weight(0.5, 4.0), 2.0)
        self.assertAlmostEqual(steering_sample_weight(-1.0, 4.0), 5.0)

    def test_steering_sample_weight_can_emphasize_straights(self):
        self.assertEqual(steering_sample_weight(0.0, 4.0, 3.0, 0.12), 3.0)
        self.assertAlmostEqual(
            steering_sample_weight(0.5, 4.0, 3.0, 0.12),
            2.0,
        )

    def test_temporal_rows_require_exact_transition_continuity(self):
        previous = {
            "episode_id": "3",
            "step_id": "10",
            "state_timestamp_ns": "100",
            "next_timestamp_ns": "200",
        }
        current = {
            "episode_id": "3",
            "step_id": "11",
            "state_timestamp_ns": "200",
            "next_timestamp_ns": "300",
        }
        self.assertTrue(transition_rows_are_contiguous(previous, current))
        current["state_timestamp_ns"] = "201"
        self.assertFalse(transition_rows_are_contiguous(previous, current))

    def test_actor_is_camera_only_and_returns_two_actions(self):
        actor = CameraSpeedActor().eval()
        with torch.no_grad():
            output = actor(torch.rand(2, 3, 90, 160))
        self.assertEqual(tuple(output.shape), (2, 2))
        self.assertLessEqual(float(output.abs().max()), 1.0)

    def test_temporal_actor_preserves_single_frame_initial_behavior(self):
        actor = CameraSpeedActor().eval()
        temporal = initialize_temporal_actor(actor).eval()
        current = torch.rand(2, 3, 90, 160)
        previous = torch.rand(2, 3, 90, 160)
        with torch.no_grad():
            expected = actor(current)
            actual = temporal(torch.cat([previous, current], dim=1))
        self.assertEqual(temporal.temporal_frames, 2)
        self.assertTrue(torch.allclose(actual, expected, atol=1.0e-6))

    def test_compact_temporal_actor_returns_two_actions(self):
        actor = CompactCameraSpeedActor(temporal_frames=2).eval()
        with torch.no_grad():
            output = actor(torch.rand(2, 6, 90, 160))
        self.assertEqual(tuple(output.shape), (2, 2))
        self.assertLessEqual(float(output.abs().max()), 1.0)

    def test_speed_normalization_round_trip(self):
        for command in (4.0, 7.0, 9.0, 10.0):
            normalized = normalize_speed_command(command, 4.0, 10.0)
            self.assertAlmostEqual(
                denormalize_speed_command(normalized, 4.0, 10.0),
                command,
            )

    def test_straight_target_uses_maximum_speed(self):
        self.assertAlmostEqual(
            speed_target_from_transition(0.0, 0.0, 0.0),
            10.0,
        )

    def test_sharp_or_recovery_target_slows_down(self):
        sharp = speed_target_from_transition(1.0, 0.0, 0.0)
        recovery = speed_target_from_transition(0.0, 0.30, 0.0)
        self.assertAlmostEqual(sharp, 4.0)
        self.assertAlmostEqual(recovery, 4.0)

    def test_camera_speed_td3_update_is_finite(self):
        agent = CameraSpeedTD3BCAgent(
            CameraSpeedActor(), config=TD3BCConfig(policy_delay=1)
        )
        batch = {
            "image": torch.rand(2, 3, 32, 32),
            "action": torch.zeros(2, 2),
            "reward": torch.ones(2, 1) * 0.1,
            "next_image": torch.rand(2, 3, 32, 32),
            "done": torch.zeros(2, 1),
        }
        metrics = agent.update(batch)
        for value in metrics.values():
            self.assertTrue(torch.isfinite(torch.tensor(value)))

    def test_temporal_compact_camera_speed_td3_update_is_finite(self):
        agent = CameraSpeedTD3BCAgent(
            CompactCameraSpeedActor(temporal_frames=2),
            config=TD3BCConfig(policy_delay=1),
        )
        batch = {
            "image": torch.rand(2, 6, 32, 32),
            "action": torch.zeros(2, 2),
            "reward": torch.ones(2, 1) * 0.1,
            "next_image": torch.rand(2, 6, 32, 32),
            "done": torch.zeros(2, 1),
        }
        metrics = agent.update(batch)
        self.assertEqual(agent.temporal_frames, 2)
        for value in metrics.values():
            self.assertTrue(torch.isfinite(torch.tensor(value)))


if __name__ == "__main__":
    unittest.main()
