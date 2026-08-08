#!/usr/bin/env python3

"""Publish a spline-resampled CSV trajectory as FlightNav commands."""

import csv
import os
import threading
from dataclasses import dataclass

import numpy as np
import rospkg
import rospy
from aerial_robot_msgs.msg import FlightNav
from scipy.interpolate import CubicSpline
from std_msgs.msg import UInt8
from std_srvs.srv import Trigger, TriggerResponse


REQUIRED_COLUMNS = (
    "t",
    "x",
    "y",
    "z",
    "roll",
    "pitch",
    "yaw",
    "vx",
    "vy",
    "vz",
    "wx",
    "wy",
    "wz",
)
TRAJECTORY_COLUMNS = REQUIRED_COLUMNS[1:]
HOVER_STATE = 5


class TrajectoryError(ValueError):
    """Raised when a trajectory or its playback configuration is invalid."""


@dataclass(frozen=True)
class ResampledTrajectory:
    times: np.ndarray
    values: dict

    def __len__(self):
        return len(self.times)

    def sample(self, index):
        return {name: self.values[name][index] for name in TRAJECTORY_COLUMNS}


def resolve_trajectory_path(path):
    """Resolve relative paths from this package's trajectory-command directory."""
    if os.path.isabs(path):
        return path

    package_path = rospkg.RosPack().get_path("gimbalrotor")
    return os.path.join(package_path, "scripts", "trajectory_command", path)


def load_and_resample_trajectory(path, publish_frequency):
    """Load the fixed CSV schema and independently resample every data column."""
    if not np.isfinite(publish_frequency) or publish_frequency <= 0.0:
        raise TrajectoryError("publish_frequency must be a positive finite number")

    try:
        csv_file = open(path, "r", newline="")
    except OSError as exc:
        raise TrajectoryError("cannot open trajectory CSV '{}': {}".format(path, exc))

    with csv_file:
        reader = csv.DictReader(csv_file)
        if tuple(reader.fieldnames or ()) != REQUIRED_COLUMNS:
            raise TrajectoryError(
                "CSV columns must be exactly: {}".format(",".join(REQUIRED_COLUMNS))
            )

        rows = []
        for line_number, row in enumerate(reader, start=2):
            try:
                values = [float(row[name]) for name in REQUIRED_COLUMNS]
            except (TypeError, ValueError) as exc:
                raise TrajectoryError(
                    "invalid numeric value on CSV line {}: {}".format(line_number, exc)
                )
            if not np.all(np.isfinite(values)):
                raise TrajectoryError(
                    "non-finite numeric value on CSV line {}".format(line_number)
                )
            rows.append(values)

    if len(rows) < 2:
        raise TrajectoryError("trajectory CSV must contain at least two data rows")

    data = np.asarray(rows, dtype=float)
    source_times = data[:, 0]
    if np.any(np.diff(source_times) <= 0.0):
        raise TrajectoryError("trajectory timestamps must be strictly increasing")

    duration = source_times[-1] - source_times[0]
    sample_period = 1.0 / publish_frequency
    regular_sample_count = int(np.floor(duration / sample_period + 1.0e-12))
    relative_times = np.arange(regular_sample_count + 1, dtype=float) * sample_period
    relative_times = relative_times[relative_times <= duration]
    if not np.isclose(relative_times[-1], duration, rtol=0.0, atol=1.0e-12):
        relative_times = np.append(relative_times, duration)
    else:
        relative_times[-1] = duration

    absolute_times = source_times[0] + relative_times
    sampled_values = {}
    for column_index, name in enumerate(TRAJECTORY_COLUMNS, start=1):
        spline = CubicSpline(source_times, data[:, column_index], bc_type="natural")
        sampled_values[name] = np.asarray(spline(absolute_times), dtype=float)

    return ResampledTrajectory(times=relative_times, values=sampled_values)


def navigation_mode(include_position, include_velocity):
    if include_position and include_velocity:
        return FlightNav.POS_VEL_MODE
    if include_position:
        return FlightNav.POS_MODE
    if include_velocity:
        return FlightNav.VEL_MODE
    return FlightNav.NO_NAVIGATION


def make_flight_nav(
    sample,
    include_position,
    include_attitude,
    include_linear_velocity,
    include_angular_velocity,
    zero_rates=False,
):
    """Convert one resampled row into a FlightNav command."""
    msg = FlightNav()
    msg.control_frame = FlightNav.WORLD_FRAME
    msg.target = FlightNav.COG

    translation_mode = navigation_mode(include_position, include_linear_velocity)
    rotation_mode = navigation_mode(include_attitude, include_angular_velocity)
    msg.pos_xy_nav_mode = translation_mode
    msg.pos_z_nav_mode = translation_mode
    msg.roll_nav_mode = rotation_mode
    msg.pitch_nav_mode = rotation_mode
    msg.yaw_nav_mode = rotation_mode

    if include_position:
        msg.target_pos_x = sample["x"]
        msg.target_pos_y = sample["y"]
        msg.target_pos_z = sample["z"]

    if include_attitude:
        msg.target_roll = sample["roll"]
        msg.target_pitch = sample["pitch"]
        msg.target_yaw = sample["yaw"]

    if include_linear_velocity:
        msg.target_vel_x = 0.0 if zero_rates else sample["vx"]
        msg.target_vel_y = 0.0 if zero_rates else sample["vy"]
        msg.target_vel_z = 0.0 if zero_rates else sample["vz"]

    if include_angular_velocity:
        msg.target_omega_x = 0.0 if zero_rates else sample["wx"]
        msg.target_omega_y = 0.0 if zero_rates else sample["wy"]
        msg.target_omega_z = 0.0 if zero_rates else sample["wz"]

    return msg


class PlaybackStateMachine:
    IDLE = "idle"
    RUNNING = "running"
    HOLDING = "holding"
    ABORTED = "aborted"

    def __init__(self, sample_count):
        if sample_count < 1:
            raise TrajectoryError("resampled trajectory must contain at least one sample")
        self.sample_count = sample_count
        self.state = self.IDLE
        self.flight_state = None
        self.next_index = 0

    def update_flight_state(self, flight_state):
        self.flight_state = flight_state
        if self.state in (self.RUNNING, self.HOLDING) and flight_state != HOVER_STATE:
            self.state = self.ABORTED
            return True
        return False

    def start(self):
        if self.state != self.IDLE:
            return False, "trajectory is already running, holding, or aborted"
        if self.flight_state != HOVER_STATE:
            return False, "robot must be in Hovering state before playback starts"
        self.next_index = 0
        self.state = self.RUNNING
        return True, "trajectory playback started"

    def next_command(self):
        if self.state == self.RUNNING:
            index = self.next_index
            self.next_index += 1
            if self.next_index == self.sample_count:
                self.state = self.HOLDING
            return index, False
        if self.state == self.HOLDING:
            return self.sample_count - 1, True
        return None


class CsvTrajectoryCommandNode:
    def __init__(self):
        trajectory_file = rospy.get_param(
            "~trajectory_file", "trajectory/reach_x1.0_flightnav.csv"
        )
        self.publish_frequency = float(rospy.get_param("~publish_frequency", 40.0))
        self.include_position = bool(rospy.get_param("~include_position", True))
        self.include_attitude = bool(rospy.get_param("~include_attitude", True))
        self.include_linear_velocity = bool(
            rospy.get_param("~include_linear_velocity", True)
        )
        self.include_angular_velocity = bool(
            rospy.get_param("~include_angular_velocity", True)
        )

        if not any(
            (
                self.include_position,
                self.include_attitude,
                self.include_linear_velocity,
                self.include_angular_velocity,
            )
        ):
            raise TrajectoryError("at least one trajectory component group must be enabled")

        resolved_path = resolve_trajectory_path(trajectory_file)
        self.trajectory = load_and_resample_trajectory(
            resolved_path, self.publish_frequency
        )
        self.playback = PlaybackStateMachine(len(self.trajectory))
        self.lock = threading.RLock()

        self.nav_publisher = rospy.Publisher("uav/nav", FlightNav, queue_size=1)
        self.flight_state_subscriber = rospy.Subscriber(
            "flight_state", UInt8, self._flight_state_callback, queue_size=1
        )
        self.start_service = rospy.Service("~start", Trigger, self._start_callback)
        self.publish_timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / self.publish_frequency), self._publish_callback
        )

        rospy.loginfo(
            "loaded %d trajectory samples from %s (duration %.6f s, %.3f Hz)",
            len(self.trajectory),
            resolved_path,
            self.trajectory.times[-1],
            self.publish_frequency,
        )

    def _flight_state_callback(self, msg):
        with self.lock:
            aborted = self.playback.update_flight_state(msg.data)
        if aborted:
            rospy.logerr(
                "trajectory playback aborted because flight state changed from Hovering to %d",
                msg.data,
            )

    def _start_callback(self, _request):
        with self.lock:
            success, message = self.playback.start()
        if success:
            rospy.loginfo(message)
        else:
            rospy.logwarn(message)
        return TriggerResponse(success=success, message=message)

    def _publish_callback(self, _event):
        with self.lock:
            action = self.playback.next_command()
            if action is None:
                return
            sample_index, zero_rates = action
            sample = self.trajectory.sample(sample_index)
            msg = make_flight_nav(
                sample,
                self.include_position,
                self.include_attitude,
                self.include_linear_velocity,
                self.include_angular_velocity,
                zero_rates=zero_rates,
            )
            msg.header.stamp = rospy.Time.now()
            self.nav_publisher.publish(msg)

        if sample_index == len(self.trajectory) - 1 and not zero_rates:
            rospy.loginfo("trajectory playback finished; holding terminal setpoint")


def main():
    rospy.init_node("trajectory_command")
    try:
        CsvTrajectoryCommandNode()
    except (TrajectoryError, rospkg.ResourceNotFound) as exc:
        rospy.logfatal("failed to initialize trajectory command node: %s", exc)
        return 1
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
