#!/usr/bin/env python3

import csv
import importlib.util
import math
import os
import tempfile
import unittest
from unittest import mock

import numpy as np
from aerial_robot_msgs.msg import FlightNav


SCRIPT_PATH = os.path.join(
    os.path.dirname(__file__),
    "..",
    "scripts",
    "trajectory_command",
    "csv_trajectory_command.py",
)
SPEC = importlib.util.spec_from_file_location("csv_trajectory_command", SCRIPT_PATH)
trajectory_command = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(trajectory_command)


class TemporaryTrajectory:
    def __init__(self, rows, columns=None):
        self.file = tempfile.NamedTemporaryFile(mode="w", newline="", suffix=".csv", delete=False)
        writer = csv.writer(self.file)
        writer.writerow(columns or trajectory_command.REQUIRED_COLUMNS)
        writer.writerows(rows)
        self.file.close()

    @property
    def path(self):
        return self.file.name

    def close(self):
        os.unlink(self.file.name)


def make_row(t, offset=0.0):
    return [t] + [offset + index for index in range(1, 13)]


class TrajectoryLoadingTest(unittest.TestCase):
    def test_irregular_times_are_resampled_and_exact_endpoint_is_included(self):
        source = TemporaryTrajectory(
            [make_row(2.0, 10.0), make_row(2.3, 20.0), make_row(2.9, 30.0)]
        )
        try:
            trajectory = trajectory_command.load_and_resample_trajectory(source.path, 2.0)
        finally:
            source.close()

        np.testing.assert_allclose(trajectory.times, [0.0, 0.5, 0.9])
        self.assertAlmostEqual(trajectory.values["x"][0], 11.0)
        self.assertAlmostEqual(trajectory.values["x"][-1], 31.0)

    def test_rejects_wrong_header(self):
        source = TemporaryTrajectory(
            [make_row(0.0), make_row(1.0)], columns=trajectory_command.REQUIRED_COLUMNS[:-1]
        )
        try:
            with self.assertRaisesRegex(trajectory_command.TrajectoryError, "columns"):
                trajectory_command.load_and_resample_trajectory(source.path, 40.0)
        finally:
            source.close()

    def test_rejects_non_increasing_time(self):
        source = TemporaryTrajectory([make_row(0.0), make_row(0.0)])
        try:
            with self.assertRaisesRegex(trajectory_command.TrajectoryError, "strictly increasing"):
                trajectory_command.load_and_resample_trajectory(source.path, 40.0)
        finally:
            source.close()

    def test_rejects_non_finite_value(self):
        row = make_row(1.0)
        row[4] = float("nan")
        source = TemporaryTrajectory([make_row(0.0), row])
        try:
            with self.assertRaisesRegex(trajectory_command.TrajectoryError, "non-finite"):
                trajectory_command.load_and_resample_trajectory(source.path, 40.0)
        finally:
            source.close()

    def test_rejects_invalid_frequency(self):
        source = TemporaryTrajectory([make_row(0.0), make_row(1.0)])
        try:
            with self.assertRaisesRegex(trajectory_command.TrajectoryError, "publish_frequency"):
                trajectory_command.load_and_resample_trajectory(source.path, 0.0)
        finally:
            source.close()


class FlightNavMappingTest(unittest.TestCase):
    SAMPLE = {
        "x": 1.0,
        "y": 2.0,
        "z": 3.0,
        "roll": 0.1,
        "pitch": 0.2,
        "yaw": 0.3,
        "vx": 4.0,
        "vy": 5.0,
        "vz": 6.0,
        "wx": 0.4,
        "wy": 0.5,
        "wz": 0.6,
    }

    def test_all_selection_combinations_set_expected_modes(self):
        for position in (False, True):
            for attitude in (False, True):
                for linear_velocity in (False, True):
                    for angular_velocity in (False, True):
                        msg = trajectory_command.make_flight_nav(
                            self.SAMPLE,
                            position,
                            attitude,
                            linear_velocity,
                            angular_velocity,
                        )
                        translation_mode = trajectory_command.navigation_mode(
                            position, linear_velocity
                        )
                        rotation_mode = trajectory_command.navigation_mode(
                            attitude, angular_velocity
                        )
                        self.assertEqual(msg.pos_xy_nav_mode, translation_mode)
                        self.assertEqual(msg.pos_z_nav_mode, translation_mode)
                        self.assertEqual(msg.roll_nav_mode, rotation_mode)
                        self.assertEqual(msg.pitch_nav_mode, rotation_mode)
                        self.assertEqual(msg.yaw_nav_mode, rotation_mode)
                        self.assertEqual(msg.control_frame, FlightNav.WORLD_FRAME)
                        self.assertEqual(msg.target, FlightNav.COG)

    def test_enabled_fields_are_copied_without_position_shift(self):
        msg = trajectory_command.make_flight_nav(self.SAMPLE, True, True, True, True)
        self.assertEqual(msg.target_pos_x, self.SAMPLE["x"])
        self.assertEqual(msg.target_pos_y, self.SAMPLE["y"])
        self.assertEqual(msg.target_pos_z, self.SAMPLE["z"])
        self.assertAlmostEqual(msg.target_roll, self.SAMPLE["roll"])
        self.assertAlmostEqual(msg.target_pitch, self.SAMPLE["pitch"])
        self.assertAlmostEqual(msg.target_yaw, self.SAMPLE["yaw"])
        self.assertAlmostEqual(msg.target_vel_x, self.SAMPLE["vx"])
        self.assertAlmostEqual(msg.target_omega_y, self.SAMPLE["wy"])

    def test_terminal_hold_zeros_enabled_rates_only(self):
        msg = trajectory_command.make_flight_nav(
            self.SAMPLE, True, True, True, True, zero_rates=True
        )
        self.assertEqual(msg.target_pos_x, self.SAMPLE["x"])
        self.assertAlmostEqual(msg.target_roll, self.SAMPLE["roll"])
        self.assertEqual(msg.target_vel_x, 0.0)
        self.assertEqual(msg.target_vel_y, 0.0)
        self.assertEqual(msg.target_vel_z, 0.0)
        self.assertEqual(msg.target_omega_x, 0.0)
        self.assertEqual(msg.target_omega_y, 0.0)
        self.assertEqual(msg.target_omega_z, 0.0)


class TrajectoryAlignmentTest(unittest.TestCase):
    def setUp(self):
        self.trajectory = trajectory_command.ResampledTrajectory(
            times=np.asarray([0.0, 1.0, 2.0]),
            values={
                "x": np.asarray([10.0, 11.0, 11.0]),
                "y": np.asarray([5.0, 5.0, 6.0]),
                "z": np.asarray([2.0, 1.9, 2.2]),
                "roll": np.asarray([0.1, 0.2, 0.3]),
                "pitch": np.asarray([-0.1, -0.2, -0.3]),
                "yaw": np.asarray([0.2, 0.3, 0.4]),
                "vx": np.asarray([1.0, 0.0, -1.0]),
                "vy": np.asarray([0.0, 1.0, 0.0]),
                "vz": np.asarray([0.1, 0.0, -0.1]),
                "wx": np.asarray([0.01, 0.02, 0.03]),
                "wy": np.asarray([0.04, 0.05, 0.06]),
                "wz": np.asarray([0.07, 0.08, 0.09]),
            },
        )

    def test_maps_first_pose_and_rotates_planar_path_and_velocity(self):
        current_position = np.asarray([100.0, 200.0, 0.5])
        current_yaw = 0.2 + math.pi / 2.0
        aligned = trajectory_command.align_trajectory_to_pose(
            self.trajectory, current_position, current_yaw, 0.30
        )

        np.testing.assert_allclose(aligned.values["x"], [100.0, 100.0, 99.0])
        np.testing.assert_allclose(aligned.values["y"], [200.0, 201.0, 201.0])
        np.testing.assert_allclose(aligned.values["z"], [0.5, 0.4, 0.7])
        np.testing.assert_allclose(
            aligned.values["yaw"], np.asarray([0.2, 0.3, 0.4]) + math.pi / 2.0
        )
        np.testing.assert_allclose(aligned.values["vx"], [0.0, -1.0, 0.0], atol=1e-15)
        np.testing.assert_allclose(aligned.values["vy"], [1.0, 0.0, -1.0], atol=1e-15)

    def test_does_not_offset_roll_pitch_or_other_rates(self):
        aligned = trajectory_command.align_trajectory_to_pose(
            self.trajectory, [1.0, 2.0, 0.5], 0.2, 0.30
        )

        for name in ("roll", "pitch", "vz", "wx", "wy", "wz"):
            np.testing.assert_array_equal(
                aligned.values[name], self.trajectory.values[name]
            )

    def test_rejects_trajectory_below_minimum_altitude(self):
        with self.assertRaisesRegex(
            trajectory_command.TrajectoryError, "minimum altitude 0.400 m.*0.410 m"
        ):
            trajectory_command.align_trajectory_to_pose(
                self.trajectory, [1.0, 2.0, 0.5], 0.2, 0.41
            )

    def test_allows_trajectory_at_minimum_altitude(self):
        aligned = trajectory_command.align_trajectory_to_pose(
            self.trajectory, [1.0, 2.0, 0.5], 0.2, 0.40
        )
        self.assertAlmostEqual(np.min(aligned.values["z"]), 0.40)

    def test_extracts_yaw_without_using_roll_or_pitch_offsets(self):
        yaw = 1.1
        self.assertAlmostEqual(
            trajectory_command.yaw_from_quaternion(
                0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)
            ),
            yaw,
        )
        with self.assertRaisesRegex(trajectory_command.TrajectoryError, "zero length"):
            trajectory_command.yaw_from_quaternion(0.0, 0.0, 0.0, 0.0)


class PlaybackStateMachineTest(unittest.TestCase):
    def test_requires_hover_and_allows_only_one_start(self):
        playback = trajectory_command.PlaybackStateMachine(2)
        success, _ = playback.start()
        self.assertFalse(success)

        playback.update_flight_state(trajectory_command.HOVER_STATE)
        success, _ = playback.start()
        self.assertTrue(success)
        success, _ = playback.start()
        self.assertFalse(success)

        self.assertEqual(playback.next_command(), (0, False))
        self.assertEqual(playback.next_command(), (1, False))
        self.assertEqual(playback.state, playback.HOLDING)
        self.assertEqual(playback.next_command(), (1, True))

    def test_flight_state_loss_aborts_running_and_holding(self):
        for commands_before_loss in (0, 1):
            playback = trajectory_command.PlaybackStateMachine(1)
            playback.update_flight_state(trajectory_command.HOVER_STATE)
            self.assertTrue(playback.start()[0])
            for _ in range(commands_before_loss):
                playback.next_command()
            self.assertTrue(playback.update_flight_state(4))
            self.assertEqual(playback.state, playback.ABORTED)
            self.assertIsNone(playback.next_command())
            self.assertFalse(playback.start()[0])


class StartSafetyTest(unittest.TestCase):
    def make_node(self, minimum_altitude=0.30):
        trajectory = trajectory_command.ResampledTrajectory(
            times=np.asarray([0.0, 1.0]),
            values={
                name: np.asarray(values)
                for name, values in {
                    "x": [0.0, 1.0],
                    "y": [0.0, 0.0],
                    "z": [0.5, 0.4],
                    "roll": [0.0, 0.0],
                    "pitch": [0.0, 0.0],
                    "yaw": [0.0, 0.0],
                    "vx": [1.0, 1.0],
                    "vy": [0.0, 0.0],
                    "vz": [-0.1, -0.1],
                    "wx": [0.0, 0.0],
                    "wy": [0.0, 0.0],
                    "wz": [0.0, 0.0],
                }.items()
            },
        )
        node = trajectory_command.CsvTrajectoryCommandNode.__new__(
            trajectory_command.CsvTrajectoryCommandNode
        )
        node.lock = trajectory_command.threading.RLock()
        node.trajectory = trajectory
        node.active_trajectory = None
        node.playback = trajectory_command.PlaybackStateMachine(len(trajectory))
        node.playback.update_flight_state(trajectory_command.HOVER_STATE)
        node.latest_cog_pose = (np.asarray([1.0, 2.0, 0.5]), 0.25)
        node.latest_odometry_time = trajectory_command.rospy.Time.from_sec(9.9)
        node.odometry_timeout = 0.5
        node.minimum_altitude = minimum_altitude
        node.nav_publisher = mock.Mock()
        return node

    def start_at_time(self, node, now=10.0):
        with mock.patch.object(
            trajectory_command.rospy.Time,
            "now",
            return_value=trajectory_command.rospy.Time.from_sec(now),
        ), mock.patch.object(trajectory_command.rospy, "loginfo"), mock.patch.object(
            trajectory_command.rospy, "logerr"
        ):
            return node._start_callback(None)

    def test_low_trajectory_is_rejected_without_publishing(self):
        node = self.make_node(minimum_altitude=0.41)
        response = self.start_at_time(node)

        self.assertFalse(response.success)
        self.assertIn("minimum altitude", response.message)
        self.assertEqual(node.playback.state, node.playback.IDLE)
        self.assertIsNone(node.active_trajectory)
        node._publish_callback(None)
        node.nav_publisher.publish.assert_not_called()

    def test_missing_or_stale_odometry_is_rejected(self):
        missing_node = self.make_node()
        missing_node.latest_cog_pose = None
        self.assertFalse(self.start_at_time(missing_node).success)

        stale_node = self.make_node()
        stale_node.latest_odometry_time = trajectory_command.rospy.Time.from_sec(9.0)
        self.assertFalse(self.start_at_time(stale_node).success)

    def test_successful_start_uses_pose_captured_at_start(self):
        node = self.make_node()
        response = self.start_at_time(node)

        self.assertTrue(response.success)
        first_sample = node.active_trajectory.sample(0)
        self.assertAlmostEqual(first_sample["x"], 1.0)
        self.assertAlmostEqual(first_sample["y"], 2.0)
        self.assertAlmostEqual(first_sample["z"], 0.5)
        self.assertAlmostEqual(first_sample["yaw"], 0.25)


if __name__ == "__main__":
    unittest.main()
