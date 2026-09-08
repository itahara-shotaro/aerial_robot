#!/usr/bin/env python3
"""Align follower Fast-LIO maps to a leader map with point-to-point ICP."""

import threading

import numpy as np
import rospy
from geometry_msgs.msg import TransformStamped
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
import tf2_ros

try:
    import open3d as o3d
except ImportError:
    o3d = None

try:
    from spatialmath import SE3
    from spatialmath.base import r2q, trexp, trinterp, trlog
except ImportError:
    SE3 = None


def _as_vector(value, field_name):
    """Return a finite three-element float vector or raise ValueError."""
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError("{} must be a list of three values".format(field_name))
    vector = np.asarray(value, dtype=np.float64)
    if not np.all(np.isfinite(vector)):
        raise ValueError("{} must contain finite values".format(field_name))
    return vector


class StaticTransformEstimator(object):
    """Optional 6-DoF error-state EKF for a constant SE(3) transform."""

    def __init__(self, enabled, filter_type, initial_covariance,
                 process_covariance, measurement_covariance):
        if enabled and filter_type != 'error_state_ekf':
            raise ValueError("unsupported filter.type: {}".format(filter_type))
        self.enabled = enabled
        self._initial_covariance = initial_covariance
        self._process_covariance = process_covariance
        self._measurement_covariance = measurement_covariance
        self._estimate = None
        self._covariance = None
        self._last_update_time = None

    def update(self, measurement, update_time):
        """Return the direct measurement or updated estimate as an SE3 object."""
        measurement = SE3(measurement.A, check=False)
        if not self.enabled:
            return measurement
        if self._estimate is None:
            self._estimate = measurement
            self._covariance = self._initial_covariance.copy()
            self._last_update_time = update_time
            return self._estimate.copy()

        delta_time = max(0.0, update_time - self._last_update_time)
        predicted_covariance = self._covariance + self._process_covariance * delta_time
        # ICP output is a homogeneous transform, but its rotation can differ
        # from exact orthonormality by a few floating-point ulps.  SE3 already
        # owns the representation here, so avoid trlog's stricter recheck.
        innovation = trlog((measurement * self._estimate.inv()).A, twist=True, check=False)
        innovation_covariance = predicted_covariance + self._measurement_covariance
        gain = np.linalg.solve(innovation_covariance, predicted_covariance.T).T
        correction = gain.dot(innovation)
        self._estimate = SE3(trexp(correction), check=False) * self._estimate
        residual_gain = np.eye(6) - gain
        self._covariance = residual_gain.dot(predicted_covariance).dot(residual_gain.T)
        self._covariance += gain.dot(self._measurement_covariance).dot(gain.T)
        self._covariance = 0.5 * (self._covariance + self._covariance.T)
        self._last_update_time = update_time
        return self._estimate.copy()


class TransformRollout(object):
    """Optionally interpolate from the current output to the latest target."""

    def __init__(self, enabled, fallback_duration, minimum_duration, maximum_duration):
        self.enabled = enabled
        self._fallback_duration = fallback_duration
        self._minimum_duration = minimum_duration
        self._maximum_duration = maximum_duration
        self._start = None
        self._target = None
        self._start_time = None
        self._duration = 0.0
        self._last_target_time = None

    def value_at(self, current_time):
        if self._target is None:
            return None
        if not self.enabled or self._duration <= 0.0:
            return self._target.copy()
        fraction = float(np.clip((current_time - self._start_time) / self._duration, 0.0, 1.0))
        return SE3(trinterp(self._start.A, self._target.A, fraction), check=False)

    def set_target(self, target, current_time):
        """Retarget from the current output; an unfinished rollout is discarded."""
        target = SE3(target.A, check=False)
        if not self.enabled or self._target is None:
            self._start = target
            self._target = target
            self._start_time = current_time
            self._last_target_time = current_time
            self._duration = 0.0
            return target.copy()

        interval = current_time - self._last_target_time
        self._start = self.value_at(current_time)
        self._target = target
        self._start_time = current_time
        self._last_target_time = current_time
        self._duration = min(self._maximum_duration,
                             max(self._minimum_duration,
                                 interval if interval > 0.0 else self._fallback_duration))
        return self._start.copy()


class MultiMapIcpAlignment(object):
    """Cache a leader map and align each received follower map against it."""

    def __init__(self):
        if o3d is None:
            raise RuntimeError("Open3D is unavailable; install python3-open3d")
        if SE3 is None:
            raise RuntimeError("spatialmath-python==1.1.16 is unavailable; install package requirements")

        self._lock = threading.Lock()
        self._leader_map = None
        self._accepted_transforms = {}
        self._followers = self._read_parameters()
        self._tf_broadcaster = tf2_ros.TransformBroadcaster()

        self._leader_subscriber = rospy.Subscriber(
            self._leader_topic, PointCloud2, self._leader_callback,
            queue_size=self._subscriber_queue_size)
        self._follower_subscribers = []
        for follower in self._followers.values():
            self._follower_subscribers.append(rospy.Subscriber(
                follower['topic'], PointCloud2, self._follower_callback,
                callback_args=follower['name'], queue_size=self._subscriber_queue_size))
        self._tf_timer = rospy.Timer(
            rospy.Duration(1.0 / self._tf_publish_rate), self._publish_timer_callback)
        rospy.loginfo(
            "multi_map_icp_alignment: leader=%s, followers=%s, downsample=%s, filter=%s, rollout=%s",
            self._leader_topic, ', '.join(self._followers.keys()), self._downsample_enabled,
            self._filter_enabled, self._rollout_enabled)

    def _read_parameters(self):
        leader = rospy.get_param('~leader')
        if not isinstance(leader, dict) or not leader.get('topic'):
            raise ValueError("~leader.topic must be a non-empty topic name")
        self._leader_topic = str(leader['topic'])

        downsample = rospy.get_param('~downsample')
        self._downsample_enabled = bool(downsample.get('enabled', False))
        self._voxel_size = float(downsample.get('voxel_size', 0.0))
        if self._downsample_enabled and self._voxel_size <= 0.0:
            raise ValueError("~downsample.voxel_size must be positive when downsampling is enabled")

        registration = rospy.get_param('~registration')
        self._max_correspondence_distance = float(registration['max_correspondence_distance'])
        self._max_iteration = int(registration['max_iteration'])
        self._relative_fitness = float(registration['relative_fitness'])
        self._relative_rmse = float(registration['relative_rmse'])
        self._min_points = int(registration['min_points'])
        self._min_fitness = float(registration['min_fitness'])
        self._max_inlier_rmse = float(registration['max_inlier_rmse'])
        self._subscriber_queue_size = int(registration.get('subscriber_queue_size', 1))
        self._tf_publish_rate = float(registration.get('tf_publish_rate', 10.0))
        if (self._max_correspondence_distance <= 0.0 or self._max_iteration <= 0 or
                self._min_points < 3 or self._subscriber_queue_size <= 0 or
                self._tf_publish_rate <= 0.0 or self._min_fitness < 0.0 or
                self._max_inlier_rmse <= 0.0):
            raise ValueError("registration parameters contain an invalid non-positive value")

        filter_parameters = rospy.get_param('~filter', {})
        self._filter_enabled = bool(filter_parameters.get('enabled', False))
        self._filter_type = str(filter_parameters.get('type', 'error_state_ekf'))
        if self._filter_enabled:
            self._initial_covariance = self._covariance_from_parameters(
                filter_parameters.get('initial_covariance'), 'filter.initial_covariance')
            self._process_covariance = self._covariance_from_parameters(
                filter_parameters.get('process_covariance'), 'filter.process_covariance')
            self._measurement_covariance = self._covariance_from_parameters(
                filter_parameters.get('measurement_covariance'), 'filter.measurement_covariance')
        else:
            self._initial_covariance = np.zeros((6, 6), dtype=np.float64)
            self._process_covariance = np.zeros((6, 6), dtype=np.float64)
            self._measurement_covariance = np.zeros((6, 6), dtype=np.float64)

        rollout_parameters = rospy.get_param('~rollout', {})
        self._rollout_enabled = bool(rollout_parameters.get('enabled', False))
        self._rollout_fallback_duration = float(rollout_parameters.get('fallback_duration', 0.1))
        self._rollout_minimum_duration = float(rollout_parameters.get('minimum_duration', 0.02))
        self._rollout_maximum_duration = float(rollout_parameters.get('maximum_duration', 1.0))
        if (self._rollout_fallback_duration <= 0.0 or self._rollout_minimum_duration <= 0.0 or
                self._rollout_maximum_duration < self._rollout_minimum_duration):
            raise ValueError("rollout durations must be positive and maximum_duration >= minimum_duration")

        configured_followers = rospy.get_param('~followers')
        if not isinstance(configured_followers, list) or not configured_followers:
            raise ValueError("~followers must contain at least one follower")
        followers = {}
        for configured in configured_followers:
            if not isinstance(configured, dict):
                raise ValueError("each follower must be a mapping")
            name = str(configured.get('name', ''))
            topic = str(configured.get('topic', ''))
            if not name or not topic:
                raise ValueError("each follower requires non-empty name and topic")
            if name in followers:
                raise ValueError("follower names must be unique: {}".format(name))
            guess = configured.get('initial_guess', {})
            translation = _as_vector(guess.get('translation'),
                                     "initial_guess.translation for {}".format(name))
            rpy = _as_vector(guess.get('rotation_rpy'),
                             "initial_guess.rotation_rpy for {}".format(name))
            initial_transform = SE3.Rt(SE3.RPY(*rpy).R, translation)
            followers[name] = {
                'name': name,
                'topic': topic,
                'initial_transform': initial_transform,
                'registration_lock': threading.Lock(),
                'filter': StaticTransformEstimator(
                    self._filter_enabled, self._filter_type, self._initial_covariance,
                    self._process_covariance, self._measurement_covariance),
                'rollout': TransformRollout(
                    self._rollout_enabled, self._rollout_fallback_duration,
                    self._rollout_minimum_duration, self._rollout_maximum_duration),
            }
        return followers

    @staticmethod
    def _covariance_from_parameters(parameters, parameter_name):
        if not isinstance(parameters, dict):
            raise ValueError("{} must be a mapping".format(parameter_name))
        translation = _as_vector(parameters.get('translation'), parameter_name + '.translation')
        rotation = _as_vector(parameters.get('rotation'), parameter_name + '.rotation')
        diagonal = np.concatenate((translation, rotation))
        if np.any(diagonal <= 0.0):
            raise ValueError("{} entries must be positive variances".format(parameter_name))
        return np.diag(diagonal)

    @staticmethod
    def _frame_id(message):
        return message.header.frame_id.lstrip('/')

    def _points_from_message(self, message, source_name):
        try:
            points = np.asarray(list(point_cloud2.read_points(
                message, field_names=('x', 'y', 'z'), skip_nans=True)), dtype=np.float64)
        except Exception as error:
            rospy.logwarn_throttle(5.0, "%s has no readable XYZ fields: %s", source_name, error)
            return None
        if points.size == 0:
            rospy.logwarn_throttle(5.0, "%s has no finite XYZ points", source_name)
            return None
        points = points.reshape((-1, 3))
        if self._downsample_enabled:
            cloud = o3d.geometry.PointCloud()
            cloud.points = o3d.utility.Vector3dVector(points)
            points = np.asarray(cloud.voxel_down_sample(self._voxel_size).points)
        if len(points) < self._min_points:
            rospy.logwarn_throttle(5.0, "%s has %d usable points; need at least %d",
                                   source_name, len(points), self._min_points)
            return None
        return points

    def _leader_callback(self, message):
        frame_id = self._frame_id(message)
        if not frame_id:
            rospy.logwarn_throttle(5.0, "leader map has an empty header.frame_id")
            return
        points = self._points_from_message(message, 'leader map')
        if points is not None:
            with self._lock:
                self._leader_map = {'points': points, 'frame_id': frame_id}

    def _follower_callback(self, message, follower_name):
        follower = self._followers[follower_name]
        if not follower['registration_lock'].acquire(False):
            rospy.logdebug("ICP already running for follower %s", follower_name)
            return
        try:
            follower_frame = self._frame_id(message)
            if not follower_frame:
                rospy.logwarn_throttle(5.0, "%s map has an empty header.frame_id", follower_name)
                return
            follower_points = self._points_from_message(message, follower_name)
            if follower_points is None:
                return
            with self._lock:
                leader_map = self._leader_map
            if leader_map is None:
                rospy.logwarn_throttle(5.0, "no usable leader map received yet")
                return
            if leader_map['frame_id'] == follower_frame:
                rospy.logwarn_throttle(5.0, "leader and follower %s use the same frame_id '%s'; refusing TF",
                                       follower_name, follower_frame)
                return

            source = o3d.geometry.PointCloud()
            source.points = o3d.utility.Vector3dVector(follower_points)
            target = o3d.geometry.PointCloud()
            target.points = o3d.utility.Vector3dVector(leader_map['points'])
            criteria = o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=self._relative_fitness, relative_rmse=self._relative_rmse,
                max_iteration=self._max_iteration)
            result = o3d.pipelines.registration.registration_icp(
                source, target, self._max_correspondence_distance, follower['initial_transform'].A,
                o3d.pipelines.registration.TransformationEstimationPointToPoint(), criteria)
            if result.fitness < self._min_fitness or result.inlier_rmse > self._max_inlier_rmse:
                rospy.logwarn_throttle(5.0,
                    "ICP rejected for %s: fitness=%.3f (min %.3f), RMSE=%.3f (max %.3f)",
                    follower_name, result.fitness, self._min_fitness,
                    result.inlier_rmse, self._max_inlier_rmse)
                return

            update_time = rospy.get_time()
            measurement = SE3(result.transformation, check=False)
            filtered_transform = follower['filter'].update(measurement, update_time)
            with self._lock:
                output_transform = follower['rollout'].set_target(filtered_transform, update_time)
                accepted = {'transform': output_transform, 'parent_frame': leader_map['frame_id'],
                            'child_frame': follower_frame, 'rollout': follower['rollout']}
                self._accepted_transforms[follower_name] = accepted
            self._publish_transform(accepted)
            rospy.loginfo("ICP accepted for %s: fitness=%.3f, RMSE=%.3f%s", follower_name,
                          result.fitness, result.inlier_rmse,
                          " (filtered)" if self._filter_enabled else "")
        except RuntimeError as error:
            rospy.logwarn_throttle(5.0, "ICP failed for %s: %s", follower_name, error)
        finally:
            follower['registration_lock'].release()

    def _publish_transform(self, accepted):
        transform = accepted['transform']
        quaternion = r2q(transform.R, order='sxyz')  # spatialmath order: [w, x, y, z]
        message = TransformStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = accepted['parent_frame']
        message.child_frame_id = accepted['child_frame']
        message.transform.translation.x = float(transform.t[0])
        message.transform.translation.y = float(transform.t[1])
        message.transform.translation.z = float(transform.t[2])
        message.transform.rotation.x = float(quaternion[1])
        message.transform.rotation.y = float(quaternion[2])
        message.transform.rotation.z = float(quaternion[3])
        message.transform.rotation.w = float(quaternion[0])
        self._tf_broadcaster.sendTransform(message)

    def _publish_timer_callback(self, _event):
        current_time = rospy.get_time()
        with self._lock:
            accepted_transforms = []
            for accepted in self._accepted_transforms.values():
                transform = accepted['rollout'].value_at(current_time)
                if transform is not None:
                    outgoing = accepted.copy()
                    outgoing['transform'] = transform
                    accepted_transforms.append(outgoing)
        for accepted in accepted_transforms:
            self._publish_transform(accepted)


def main():
    rospy.init_node('multi_map_icp_alignment')
    try:
        MultiMapIcpAlignment()
    except (KeyError, TypeError, ValueError, RuntimeError) as error:
        rospy.logfatal("multi_map_icp_alignment configuration error: %s", error)
        raise
    rospy.spin()


if __name__ == '__main__':
    main()
