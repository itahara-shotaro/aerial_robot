#!/usr/bin/env python3

"""Publish a spline-resampled CSV trajectory as FlightNav commands."""

import csv
import math
import os
import threading
from dataclasses import dataclass

import numpy as np
import rospkg
import rosgraph
import rospy
from aerial_robot_msgs.msg import FlightNav
from nav_msgs.msg import Odometry
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


def yaw_from_quaternion(x, y, z, w):
    """Extract yaw from a finite, nonzero quaternion."""
    quaternion = np.asarray((x, y, z, w), dtype=float)
    if not np.all(np.isfinite(quaternion)):
        raise TrajectoryError("odometry orientation contains a non-finite value")

    norm = np.linalg.norm(quaternion)
    if norm <= np.finfo(float).eps:
        raise TrajectoryError("odometry orientation quaternion has zero length")
    x, y, z, w = quaternion / norm
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def align_trajectory_to_pose(trajectory, position, yaw, minimum_altitude):
    """Yaw-align a trajectory's first pose to a world-frame COG pose."""
    position = np.asarray(position, dtype=float)
    if position.shape != (3,) or not np.all(np.isfinite(position)):
        raise TrajectoryError("odometry position must contain three finite values")
    if not np.isfinite(yaw):
        raise TrajectoryError("odometry yaw must be finite")
    if not np.isfinite(minimum_altitude):
        raise TrajectoryError("minimum_altitude must be finite")

    initial_yaw = trajectory.values["yaw"][0]
    yaw_offset = yaw - initial_yaw
    cos_yaw = math.cos(yaw_offset)
    sin_yaw = math.sin(yaw_offset)

    relative_x = trajectory.values["x"] - trajectory.values["x"][0]
    relative_y = trajectory.values["y"] - trajectory.values["y"][0]
    values = {name: np.array(data, copy=True) for name, data in trajectory.values.items()}
    values["x"] = position[0] + cos_yaw * relative_x - sin_yaw * relative_y
    values["y"] = position[1] + sin_yaw * relative_x + cos_yaw * relative_y
    values["z"] = position[2] + trajectory.values["z"] - trajectory.values["z"][0]
    values["yaw"] = trajectory.values["yaw"] + yaw_offset

    velocity_x = trajectory.values["vx"]
    velocity_y = trajectory.values["vy"]
    values["vx"] = cos_yaw * velocity_x - sin_yaw * velocity_y
    values["vy"] = sin_yaw * velocity_x + cos_yaw * velocity_y

    actual_minimum_altitude = float(np.min(values["z"]))
    if actual_minimum_altitude < minimum_altitude and not np.isclose(
        actual_minimum_altitude, minimum_altitude, rtol=0.0, atol=1.0e-12
    ):
        raise TrajectoryError(
            "aligned trajectory minimum altitude {:.3f} m is below the {:.3f} m "
            "safety threshold".format(actual_minimum_altitude, minimum_altitude)
        )

    return ResampledTrajectory(times=trajectory.times, values=values)


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
        robot_ns = rospy.get_param("~robot_ns", "")
        if not robot_ns:
            master = rosgraph.Master('/rostopic')
            try:
                    _, subs, _ = master.getSystemState()

            except socket.error:
                    raise ROSTopicIOException("Unable to communicate with master!")

            teleop_topics = [topic[0] for topic in subs if 'teleop_command/start' in topic[0]]
            if len(teleop_topics) == 1:
                    robot_ns = teleop_topics[0].split('/teleop')[0]
        
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
        self.minimum_altitude = float(rospy.get_param("~minimum_altitude", 0.30))
        self.odometry_timeout = float(rospy.get_param("~odometry_timeout", 0.5))
        self.odometry_topic = rospy.get_param("~odometry_topic", "uav/cog/odom")

        if not np.isfinite(self.minimum_altitude):
            raise TrajectoryError("minimum_altitude must be finite")
        if not np.isfinite(self.odometry_timeout) or self.odometry_timeout <= 0.0:
            raise TrajectoryError("odometry_timeout must be a positive finite number")

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
        self.active_trajectory = None
        self.latest_cog_pose = None
        self.latest_odometry_time = None
        self.lock = threading.RLock()

        self.nav_publisher = rospy.Publisher(robot_ns + "/uav/nav", FlightNav, queue_size=1)
        self.odometry_subscriber = rospy.Subscriber(
            robot_ns + "/" + self.odometry_topic, Odometry, self._odometry_callback, queue_size=1
        )
        self.flight_state_subscriber = rospy.Subscriber(
            robot_ns + "/flight_state", UInt8, self._flight_state_callback, queue_size=1
        )
        self.start_service = rospy.Service(robot_ns + "/start", Trigger, self._start_callback)
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

    def _odometry_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        try:
            pose = (
                np.asarray((position.x, position.y, position.z), dtype=float),
                yaw_from_quaternion(
                    orientation.x, orientation.y, orientation.z, orientation.w
                ),
            )
            if not np.all(np.isfinite(pose[0])):
                raise TrajectoryError("odometry position contains a non-finite value")
        except TrajectoryError as exc:
            rospy.logerr_throttle(1.0, "ignoring invalid COG odometry: %s", exc)
            return

        with self.lock:
            self.latest_cog_pose = pose
            self.latest_odometry_time = rospy.Time.now()

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
            if self.playback.state != self.playback.IDLE:
                success, message = self.playback.start()
            elif self.playback.flight_state != HOVER_STATE:
                success, message = self.playback.start()
            elif self.latest_cog_pose is None:
                success = False
                message = "cannot start trajectory without valid COG odometry"
            else:
                odometry_age = (rospy.Time.now() - self.latest_odometry_time).to_sec()
                if odometry_age < 0.0 or odometry_age > self.odometry_timeout:
                    success = False
                    message = (
                        "cannot start trajectory: COG odometry is stale "
                        "({:.3f} s old; limit {:.3f} s)".format(
                            odometry_age, self.odometry_timeout
                        )
                    )
                else:
                    position, yaw = self.latest_cog_pose
                    try:
                        aligned_trajectory = align_trajectory_to_pose(
                            self.trajectory, position, yaw, self.minimum_altitude
                        )
                    except TrajectoryError as exc:
                        success = False
                        message = "cannot start trajectory: {}".format(exc)
                    else:
                        success, message = self.playback.start()
                        if success:
                            self.active_trajectory = aligned_trajectory
        if success:
            rospy.loginfo(message)
        else:
            rospy.logerr(message)
        return TriggerResponse(success=success, message=message)

    def _publish_callback(self, _event):
        with self.lock:
            action = self.playback.next_command()
            if action is None:
                return
            sample_index, zero_rates = action
            sample = self.active_trajectory.sample(sample_index)
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
