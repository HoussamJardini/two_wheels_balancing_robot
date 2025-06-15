import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import math
import time

class BalanceController(Node):
    def __init__(self):
        super().__init__('balance_controller')
        
        self.sub_imu = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.sub_cmd_vel_desired = self.create_subscription(Twist, '/cmd_vel_desired', self.cmd_vel_desired_callback, 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.kp = 55.0
        self.ki = 0.2
        self.kd = 10.0
        
        self.integral = 0.0
        self.prev_error = 0.0
        self.target_angle = 0.0
        
        self.current_angle = 0.0
        self.angle_velocity = 0.0
        
        self.desired_linear_x = 0.0
        self.desired_angular_z = 0.0
        
        self.max_integral = 0.2
        self.max_velocity = 2.5
        
        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        self.imu_data_received = False
        self.consecutive_failures = 0
        
        self.startup_time = time.time()
        self.startup_delay = 1.5
        self.angle_offset = 0.0
        self.calibration_samples = 0
        self.calibration_sum = 0.0
        self.calibration_complete = False
        
        self.angle_filter_alpha = 0.7
        self.filtered_angle = 0.0
        
        self.get_logger().info('Balance Controller initialized')
    
    def cmd_vel_desired_callback(self, msg):
        self.desired_linear_x = msg.linear.x
        self.desired_angular_z = msg.angular.z
        
        if abs(msg.linear.x) > 0.1:
            self.target_angle = -msg.linear.x * 0.03
        else:
            self.target_angle = 0.0
    
    def imu_callback(self, msg):
        try:
            q = msg.orientation
            
            quat_norm = math.sqrt(q.w**2 + q.x**2 + q.y**2 + q.z**2)
            if abs(quat_norm - 1.0) > 0.1:
                self.consecutive_failures += 1
                return
            
            sinp = 2 * (q.w * q.y - q.z * q.x)
            sinp = max(-1.0, min(1.0, sinp))
            pitch = math.asin(sinp)
            
            if not self.calibration_complete and self.calibration_samples < 200:
                self.calibration_sum += pitch
                self.calibration_samples += 1
                if self.calibration_samples >= 200:
                    self.angle_offset = self.calibration_sum / self.calibration_samples
                    self.calibration_complete = True
                    self.get_logger().info(f'Calibrated - Zero angle: {math.degrees(self.angle_offset):.2f}°')
                return
            
            raw_angle = pitch - self.angle_offset
            self.filtered_angle = (self.angle_filter_alpha * self.filtered_angle + 
                                 (1 - self.angle_filter_alpha) * raw_angle)
            
            self.angle_velocity = msg.angular_velocity.y
            self.current_angle = self.filtered_angle
            self.imu_data_received = True
            self.consecutive_failures = 0
            
        except Exception as e:
            self.get_logger().error(f'IMU error: {e}')
            self.consecutive_failures += 1
    
    def control_loop(self):
        if time.time() - self.startup_time < self.startup_delay or not self.calibration_complete:
            return
            
        if not self.imu_data_received or self.consecutive_failures > 50:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.pub_cmd_vel.publish(cmd)
            return
        
        error = self.current_angle - self.target_angle
        
        self.integral += error * self.dt
        self.integral = max(-self.max_integral, min(self.max_integral, self.integral))
        
        derivative = (error - self.prev_error) / self.dt
        
        balance_output = (self.kp * error + 
                         self.ki * self.integral + 
                         self.kd * derivative)
        
        balance_output -= 0.5 * self.angle_velocity
        
        total_linear = balance_output + self.desired_linear_x
        
        total_linear = max(-self.max_velocity, min(self.max_velocity, total_linear))
        
        if abs(self.current_angle) > math.radians(45):
            total_linear = 0.0
            self.desired_angular_z = 0.0
        
        cmd = Twist()
        cmd.linear.x = float(total_linear)
        cmd.angular.z = float(self.desired_angular_z)
        self.pub_cmd_vel.publish(cmd)
        
        self.prev_error = error
        
        if hasattr(self, 'log_counter'):
            self.log_counter += 1
        else:
            self.log_counter = 0
            
        if self.log_counter % 100 == 0:
            self.get_logger().info(
                f'Angle: {math.degrees(self.current_angle):6.1f}°, '
                f'Target: {math.degrees(self.target_angle):6.1f}°, '
                f'Balance: {balance_output:6.2f}, '
                f'Output: {total_linear:6.2f}'
            )

def main(args=None):
    rclpy.init(args=args)
    node = BalanceController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down balance controller')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
