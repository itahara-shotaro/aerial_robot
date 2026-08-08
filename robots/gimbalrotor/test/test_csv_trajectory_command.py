#!/usr/bin/env python3

import csv
import importlib.util
import os
import tempfile
import unittest

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


if __name__ == "__main__":
    unittest.main()
