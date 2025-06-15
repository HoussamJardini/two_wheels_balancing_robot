

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import math

class ObstacleAvoidanceController(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_controller')
        
        self.scan_subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_vel_desired_subscription = self.create_subscription(
            Twist, '/cmd_vel_desired', self.cmd_vel_desired_callback, 10)
        
        self.cmd_vel_desired_publisher = self.create_publisher(
            Twist, '/cmd_vel_desired', 10)
        
        self.obstacle_distance = 1.0
        
        self.ground_height_threshold = 0.4
        self.min_valid_distance = 0.6
        
        self.user_linear_x = 0.0
        self.user_angular_z = 0.0
        self.last_user_command_time = 0.0
        
        self.latest_scan = None
        self.obstacle_detected = False
        
        self.is_rotating = False
        self.rotation_start_time = 0.0
        self.rotation_duration = 2.0
        self.target_rotation_direction = 0.0
        
        self.obstacle_detection_history = []
        self.history_length = 3
        
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.front_sector = (-math.pi/12, math.pi/12)
        self.left_sector = (math.pi/12, 0.87)
        self.right_sector = (-0.87, -math.pi/12)
        
        self.get_logger().info('SMART KEYBOARD Obstacle Avoidance initialized')
        self.get_logger().info('Acts like intelligent keyboard - sends commands to /cmd_vel_desired')
        self.get_logger().info('Balance controller unchanged - no interference!')
    
    def cmd_vel_desired_callback(self, msg):
        self.user_linear_x = msg.linear.x
        self.user_angular_z = msg.angular.z
        self.last_user_command_time = rclpy.clock.Clock().now().nanoseconds / 1e9
    
    def scan_callback(self, msg):
        self.latest_scan = msg
        self.analyze_obstacles(msg)
    
    def filter_ground_readings(self, ranges, angles):
        filtered_ranges = []
        filtered_angles = []
        
        lidar_height = 0.7
        
        for distance, angle in zip(ranges, angles):
            if distance <= 0 or np.isinf(distance) or distance < self.min_valid_distance:
                continue
            
            lidar_tilt = -0.15
            detected_height = lidar_height + distance * math.sin(lidar_tilt)
            
            conditions = [
                detected_height > self.ground_height_threshold,
                abs(angle) <= 0.87,
                distance > self.min_valid_distance and distance < 4.0,
                detected_height > 0.5,
            ]
            
            if all(conditions):
                filtered_ranges.append(distance)
                filtered_angles.append(angle)
        
        return np.array(filtered_ranges), np.array(filtered_angles)
    
    def analyze_obstacles(self, scan):
        if len(scan.ranges) == 0:
            return
        
        ranges = np.array(scan.ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))
        
        filtered_ranges, filtered_angles = self.filter_ground_readings(ranges, angles)
        
        if len(filtered_ranges) < 3:
            self.obstacle_detected = False
            return
        
        front_mask = self.angle_in_sector(filtered_angles, self.front_sector)
        front_distances = filtered_ranges[front_mask]
        
        current_obstacle = False
        if len(front_distances) >= 2:
            min_front_distance = np.min(front_distances)
            current_obstacle = min_front_distance < self.obstacle_distance
        
        self.obstacle_detection_history.append(current_obstacle)
        if len(self.obstacle_detection_history) > self.history_length:
            self.obstacle_detection_history.pop(0)
        
        if len(self.obstacle_detection_history) >= 2:
            obstacle_count = sum(self.obstacle_detection_history)
            self.obstacle_detected = obstacle_count >= 2
        else:
            self.obstacle_detected = False
    
    def angle_in_sector(self, angles, sector):
        min_angle, max_angle = sector
        return (angles >= min_angle) & (angles <= max_angle)
    
    def find_best_rotation_direction(self, scan):
        if len(scan.ranges) == 0:
            return 1.57
        
        ranges = np.array(scan.ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))
        
        filtered_ranges, filtered_angles = self.filter_ground_readings(ranges, angles)
        
        if len(filtered_ranges) == 0:
            return 1.57
        
        left_mask = self.angle_in_sector(filtered_angles, self.left_sector)
        right_mask = self.angle_in_sector(filtered_angles, self.right_sector)
        
        left_distances = filtered_ranges[left_mask]
        right_distances = filtered_ranges[right_mask]
        
        left_clearance = np.mean(left_distances) if len(left_distances) > 0 else 0.0
        right_clearance = np.mean(right_distances) if len(right_distances) > 0 else 0.0
        
        if left_clearance > right_clearance:
            self.get_logger().info(f'Rotating LEFT - clearances L:{left_clearance:.2f}m R:{right_clearance:.2f}m')
            return 1.57
        else:
            self.get_logger().info(f'Rotating RIGHT - clearances L:{left_clearance:.2f}m R:{right_clearance:.2f}m')
            return -1.57
    
    def control_loop(self):
        current_time = rclpy.clock.Clock().now().nanoseconds / 1e9
        
        if self.is_rotating:
            time_elapsed = current_time - self.rotation_start_time
            
            if time_elapsed < self.rotation_duration:
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.angular.z = self.target_rotation_direction * (1.0 / self.rotation_duration)
                self.cmd_vel_desired_publisher.publish(cmd)
                return
            else:
                self.is_rotating = False
                self.get_logger().info('90° rotation completed - resuming user control')
                self.obstacle_detection_history = []
        
        if (self.obstacle_detected and not self.is_rotating and 
            self.user_linear_x > 0.1):
            
            self.target_rotation_direction = self.find_best_rotation_direction(self.latest_scan)
            self.is_rotating = True
            self.rotation_start_time = current_time
            
            self.get_logger().warn('OBSTACLE DETECTED - Starting 90° rotation to avoid')
            
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = self.target_rotation_direction * (1.0 / self.rotation_duration)
            self.cmd_vel_desired_publisher.publish(cmd)
            return
        
        cmd = Twist()
        cmd.linear.x = self.user_linear_x
        cmd.angular.z = self.user_angular_z
        self.cmd_vel_desired_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down smart keyboard obstacle avoidance')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    #alah akbar
