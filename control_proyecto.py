#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry


class ProyectoController(Node):
    def __init__(self):
        super().__init__('control_proyecto')
        self.get_logger().info("🚗 Nodo 'control_proyecto' iniciado")

        # Suscripciones y publicaciones
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/ego_racecar/odom', self.odom_callback, 10)

        # Parámetros Follow the Gap
        self.angle_range_deg = 100          # ±50°
        self.bubble_radius = 0.6            # m
        self.max_range = 10.0               # m
        self.window_size = 3                # suavizado LiDAR
        self.max_speed = 7.0                # 🚀 velocidad máxima definida
        self.min_speed = 1.0                # 🐢 velocidad mínima

        # Variables para control de vueltas
        self.origin_set = False
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.lap_counter = 0
        self.lap_started = False
        self.lap_start_time = self.get_clock().now()
        self.last_lap_time = self.lap_start_time
        self.lap_times = []

    def preprocess_lidar(self, ranges, angle_min, angle_increment):
        ranges = np.array(ranges)
        ranges[np.isnan(ranges)] = 0.0
        ranges[np.isinf(ranges)] = 0.0
        ranges = np.clip(ranges, 0.0, self.max_range)

        # Suavizado
        kernel = np.ones(self.window_size) / self.window_size
        smooth = np.convolve(ranges, kernel, mode='same')

        total_points = len(smooth)
        mid = total_points // 2
        deg_per_index = np.degrees(angle_increment)
        range_pts = int(self.angle_range_deg / deg_per_index / 2)

        start = mid - range_pts
        end = mid + range_pts

        return smooth[start:end], start

    def create_bubble(self, ranges):
        min_idx = np.argmin(ranges)
        min_val = ranges[min_idx]
        if min_val == 0.0:
            return ranges
        angle_radius = int(self.bubble_radius * len(ranges) / (2 * np.pi * min_val))
        bubble_start = max(0, min_idx - angle_radius)
        bubble_end = min(len(ranges) - 1, min_idx + angle_radius)
        ranges[bubble_start: bubble_end + 1] = 0.0
        return ranges

    def find_max_gap(self, ranges):
        mask = ranges > 0.0
        max_len = 0
        current_len = 0
        best_start = best_end = 0
        temp_start = 0

        for i, free in enumerate(mask):
            if free:
                if current_len == 0:
                    temp_start = i
                current_len += 1
                if current_len > max_len:
                    max_len = current_len
                    best_start = temp_start
                    best_end = i
            else:
                current_len = 0
        return best_start, best_end

    def find_best_point(self, start_idx, end_idx, ranges):
        gap = ranges[start_idx:end_idx + 1]
        rel_best_idx = np.argmax(gap)
        best_idx = start_idx + rel_best_idx
        gap_center = (start_idx + end_idx) // 2
        weighted_idx = int(0.7 * gap_center + 0.3 * best_idx)
        return weighted_idx

    def lidar_callback(self, msg):
        proc_ranges, offset = self.preprocess_lidar(
            msg.ranges, msg.angle_min, msg.angle_increment)
        proc_ranges = self.create_bubble(proc_ranges)

        gap_start, gap_end = self.find_max_gap(proc_ranges)
        best_idx = self.find_best_point(gap_start, gap_end, proc_ranges)

        mid_idx = len(proc_ranges) // 2
        angle_offset = (best_idx - mid_idx) * msg.angle_increment
        steering_angle = float(angle_offset)

        # Evaluar velocidad
        angle_deg = abs(np.degrees(steering_angle))
        gap_width = gap_end - gap_start
        is_gap_centered = abs(best_idx - mid_idx) < 5
        is_wide_gap = gap_width > 40

        if angle_deg < 5 and is_gap_centered and is_wide_gap:
            speed = self.max_speed
        elif angle_deg < 15:
            speed = 2.0
        else:
            speed = self.min_speed

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if not self.origin_set:
            self.origin_x = x
            self.origin_y = y
            self.origin_set = True
            self.get_logger().info("📍 Punto de inicio guardado para detección de vueltas.")
            return

        dist = np.sqrt((x - self.origin_x)**2 + (y - self.origin_y)**2)

        if dist < 1.5:
            if not self.lap_started:
                now = self.get_clock().now()
                lap_time = (now - self.last_lap_time).nanoseconds / 1e9

                if self.lap_counter > 0:
                    self.lap_times.append(lap_time)
                    self.get_logger().info(f"✅ Vuelta {self.lap_counter}: {lap_time:.2f} s")

                self.lap_counter += 1
                self.last_lap_time = now
                self.lap_started = True
        else:
            self.lap_started = False


def main(args=None):
    rclpy.init(args=args)
    node = ProyectoController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
