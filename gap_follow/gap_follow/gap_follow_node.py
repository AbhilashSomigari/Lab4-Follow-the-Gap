#!/usr/bin/env python3
"""
Lab 4: Follow the Gap.

Pipeline run on every LiDAR scan:
  1. preprocess_lidar  - clean NaN/inf, clip far returns, moving-average
                         smoothing, keep only the forward +-70 deg window.
  2. disparity extender - grow obstacle edges by half the car width so gaps
                         the car cannot physically fit through disappear.
  3. closest point + safety bubble (radius grows with speed) set to zero.
  4. find_max_gap      - longest run of consecutive free beams.
  5. find_best_point   - "better idea": centre of the deepest part of the
                         gap instead of the single farthest beam.
  6. speed from steering angle and free path ahead, smoothed steering,
     publish AckermannDriveStamped on /drive.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class GapFollowNode(Node):
    """Implement Gap Following on the car."""

    def __init__(self):
        super().__init__('gap_follow_node')

        # Topics & Subs, Pubs
        lidarscan_topic = self.declare_parameter(
            'scan_topic', '/scan').value
        drive_topic = self.declare_parameter(
            'drive_topic', '/drive').value

        # Preprocessing
        self.fov_deg = self.declare_parameter('fov_deg', 70.0).value
        self.max_range = self.declare_parameter('max_range', 6.0).value
        self.smoothing_window = self.declare_parameter(
            'smoothing_window', 5).value

        # Disparity extender
        self.car_width = self.declare_parameter('car_width', 0.31).value
        self.width_margin = self.declare_parameter('width_margin', 0.06).value
        self.disparity_threshold = self.declare_parameter(
            'disparity_threshold', 0.3).value

        # Safety bubble: radius = base + gain * speed
        self.bubble_radius = self.declare_parameter(
            'bubble_radius', 0.2).value
        self.bubble_speed_gain = self.declare_parameter(
            'bubble_speed_gain', 0.03).value

        # Gap / best point
        self.gap_threshold = self.declare_parameter(
            'gap_threshold', 2.5).value
        self.gap_threshold_ratio = self.declare_parameter(
            'gap_threshold_ratio', 0.7).value
        self.best_point_window = self.declare_parameter(
            'best_point_window', 40).value

        # Speed and steering
        self.max_steering = self.declare_parameter(
            'max_steering', 0.4189).value
        self.high_speed = self.declare_parameter('high_speed', 4.0).value
        self.mid_speed = self.declare_parameter('mid_speed', 2.5).value
        self.low_speed = self.declare_parameter('low_speed', 1.0).value
        self.straight_angle_deg = self.declare_parameter(
            'straight_angle_deg', 10.0).value
        self.turn_angle_deg = self.declare_parameter(
            'turn_angle_deg', 20.0).value
        self.time_headway = self.declare_parameter('time_headway', 1.0).value
        self.steering_alpha = self.declare_parameter(
            'steering_alpha', 0.6).value
        self.max_steering_step = self.declare_parameter(
            'max_steering_step', 0.12).value

        # Subscribe to LIDAR
        self.scan_sub = self.create_subscription(
            LaserScan,
            lidarscan_topic,
            self.lidar_callback,
            qos_profile_sensor_data
        )

        # Publish to drive
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            drive_topic,
            10
        )

        self.angle = 0.0          # last best-point angle (rad)
        self.steering = 0.0       # last published (smoothed) steering
        self.speed = 0.0          # last published speed
        self.ranges = []
        self.min_indx = 0
        self.max_indx = 0
        self.proc_ranges = np.array([])

        self.get_logger().info(
            f'Follow the Gap: {lidarscan_topic} -> {drive_topic}, '
            f'fov=+-{self.fov_deg:.0f}deg, bubble={self.bubble_radius:.2f}m'
        )

    def preprocess_lidar(self, ranges, angle_min, angle_increment):
        """
        Preprocess the LiDAR scan array.

        1. Replace NaN/inf with max_range and clip far returns.
        2. Set each value to the mean over a small window.
        3. Keep only beams within +-fov_deg; beams outside are zeroed so
           they can never be part of a gap.

        Returns the processed array (same length as the scan) and the
        first/last index of the forward window.
        """
        proc_ranges = np.array(ranges, dtype=np.float64)
        proc_ranges[~np.isfinite(proc_ranges)] = self.max_range
        proc_ranges = np.clip(proc_ranges, 0.0, self.max_range)

        w = int(self.smoothing_window)
        if w > 1:
            # Edge padding keeps the ends from being pulled toward zero.
            pad = w // 2
            padded = np.pad(proc_ranges, (pad, w - 1 - pad), mode='edge')
            proc_ranges = np.convolve(padded, np.ones(w) / w, mode='valid')

        min_angle = np.radians(-self.fov_deg)
        max_angle = np.radians(self.fov_deg)
        n = proc_ranges.size
        min_indx = int(np.clip(
            math.ceil((min_angle - angle_min) / angle_increment), 0, n - 1))
        max_indx = int(np.clip(
            math.floor((max_angle - angle_min) / angle_increment), 0, n - 1))

        proc_ranges[:min_indx] = 0.0
        proc_ranges[max_indx + 1:] = 0.0

        self.min_indx = min_indx
        self.max_indx = max_indx
        return proc_ranges, min_indx, max_indx

    def extend_disparities(self, ranges, angle_increment):
        """
        At each large jump between neighbouring beams, overwrite the beams on
        the far side with the near distance for half a car width.

        The straight line along any beam that survives is then wide enough
        for the car, and the chosen heading stays off obstacle corners.
        """
        out = ranges.copy()
        half_width = (self.car_width + self.width_margin) / 2.0
        lo_i, hi_i = self.min_indx, self.max_indx
        window = ranges[lo_i:hi_i + 1]
        edges = np.where(np.abs(np.diff(window)) > self.disparity_threshold)[0]
        for e in edges:
            i = lo_i + int(e)
            near = min(ranges[i], ranges[i + 1])
            if near <= 0.0:
                continue
            span = math.atan2(half_width, near)
            k = int(math.ceil(span / angle_increment))
            if ranges[i] < ranges[i + 1]:
                # Near obstacle on the lower-index side: extend upward.
                a, b = i + 1, min(hi_i + 1, i + 1 + k)
            else:
                # Near obstacle on the higher-index side: extend downward.
                a, b = max(lo_i, i - k + 1), i + 1
            out[a:b] = np.minimum(out[a:b], near)
        return out

    def find_max_gap(self, free_space_ranges):
        """
        Return the start index & end index of the max gap in
        free_space_ranges.

        A beam is free when it is farther than gap_threshold. Near corners
        nothing may be that far, so the threshold is capped at
        gap_threshold_ratio x the deepest beam; this keeps the gap pointed at
        the most open region instead of "every non-zero beam". Gaps of equal
        length are separated by total depth.
        """
        min_indx = self.min_indx
        max_indx = self.max_indx
        window = free_space_ranges[min_indx:max_indx + 1]

        deepest = float(np.max(window)) if window.size else 0.0
        if deepest <= 0.0:
            mid = (min_indx + max_indx) // 2
            self.proc_ranges = free_space_ranges
            return mid, mid

        threshold = min(self.gap_threshold,
                        self.gap_threshold_ratio * deepest)
        free = window > threshold

        start = min_indx
        end = min_indx
        current_start = min_indx - 1
        longest_duration = 0
        best_depth = -1.0
        for offset, is_free in enumerate(np.append(free, False)):
            i = min_indx + offset
            if is_free:
                if current_start < min_indx:
                    current_start = i
                continue
            if current_start >= min_indx:
                duration = i - current_start
                depth = float(np.sum(free_space_ranges[current_start:i]))
                if (duration > longest_duration or
                        (duration == longest_duration and depth > best_depth)):
                    longest_duration = duration
                    best_depth = depth
                    start, end = current_start, i - 1
                current_start = min_indx - 1

        self.proc_ranges = free_space_ranges
        return start, end

    def find_best_point(self, start_i, end_i, ranges, angle_min,
                        angle_increment):
        """
        Return the angle of the best point in the gap [start_i, end_i].

        "Better idea" instead of the naive farthest point: smooth the gap
        with a best_point_window moving average and aim at the centre of the
        deepest window. A long saturated stretch (e.g. a straight corridor)
        resolves to its middle, which keeps the car centred instead of
        twitching between equally far beams.
        """
        gap = ranges[start_i:end_i + 1]
        w = max(1, min(int(self.best_point_window), gap.size))
        smoothed = np.convolve(gap, np.ones(w) / w, mode='same')
        peak = smoothed.max()
        ties = np.where(smoothed >= peak - 1e-3)[0]
        # Middle of the widest contiguous block of tied maxima.
        blocks = np.split(ties, np.where(np.diff(ties) > 1)[0] + 1)
        block = max(blocks, key=len)
        best_i = start_i + int(block[len(block) // 2])

        self.angle = angle_min + best_i * angle_increment
        return self.angle

    def free_path_ahead(self, ranges, angle_min, angle_increment):
        """Length of the obstacle-free, car-width corridor straight ahead."""
        half_width = (self.car_width + self.width_margin) / 2.0
        idx = np.arange(self.min_indx, self.max_indx + 1)
        r = ranges[idx]
        a = angle_min + idx * angle_increment
        x = r * np.cos(a)
        y = r * np.sin(a)
        blocking = (r > 0.0) & (x > 0.0) & (np.abs(y) < half_width)
        if not np.any(blocking):
            return self.max_range
        return float(x[blocking].min())

    def lidar_callback(self, data):
        """Run Follow the Gap on one scan and publish the drive command."""
        ranges = data.ranges
        angle_min = data.angle_min
        angle_increment = data.angle_increment
        if len(ranges) == 0:
            return
        proc_ranges, min_indx, max_indx = self.preprocess_lidar(
            ranges, angle_min, angle_increment)
        proc_ranges = self.extend_disparities(proc_ranges, angle_increment)

        # Find closest point to LiDAR (inside the forward window)
        window = proc_ranges[min_indx:max_indx + 1]
        closest_i = min_indx + int(np.argmin(window))
        closest = float(proc_ranges[closest_i])

        # Dynamic safety bubble size based on speed
        radius = self.bubble_radius + self.bubble_speed_gain * self.speed
        if closest <= radius:
            half_angle = math.pi / 4  # already inside: blank a wide sector
        else:
            half_angle = math.asin(radius / closest)
        k = int(math.ceil(half_angle / angle_increment))

        # Eliminate all points inside 'bubble' (set them to zero)
        free_space = proc_ranges.copy()
        free_space[max(min_indx, closest_i - k):
                   min(max_indx, closest_i + k) + 1] = 0.0

        # Find max length gap
        start_i, end_i = self.find_max_gap(free_space)

        # Find the best point in the gap
        best_angle = self.find_best_point(
            start_i, end_i, free_space, angle_min, angle_increment)

        # Smooth the steering angles to avoid abrupt changes:
        # exponential moving average, then a per-scan rate limit.
        target = float(np.clip(best_angle, -self.max_steering,
                               self.max_steering))
        smoothed = (self.steering_alpha * target +
                    (1.0 - self.steering_alpha) * self.steering)
        step = float(np.clip(smoothed - self.steering,
                             -self.max_steering_step, self.max_steering_step))
        self.steering = self.steering + step

        # Increase speed if the best angle is close to zero (straight ahead),
        # and never drive faster than the free path ahead allows.
        steer_deg = abs(math.degrees(self.steering))
        if steer_deg < self.straight_angle_deg:
            speed = self.high_speed
        elif steer_deg < self.turn_angle_deg:
            speed = self.mid_speed
        else:
            speed = self.low_speed
        front = self.free_path_ahead(proc_ranges, angle_min, angle_increment)
        if self.time_headway > 0.0:
            speed = min(speed, max(self.low_speed, front / self.time_headway))
        self.speed = speed

        # Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.steering_angle = self.steering
        drive_msg.drive.speed = self.speed
        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    print('GapFollow Initialized')
    gap_follow_node = GapFollowNode()
    try:
        rclpy.spin(gap_follow_node)
    except KeyboardInterrupt:
        pass
    finally:
        gap_follow_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
