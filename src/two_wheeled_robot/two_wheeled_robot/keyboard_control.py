import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard
import threading

class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher = self.create_publisher(Twist, '/cmd_vel_desired', 10)
        self.timer = self.create_timer(0.05, self.publish_twist)
        self.pressed_keys = set()
        self.lock = threading.Lock()
        
        self.linear_speed = 2.8
        self.angular_speed = 3.9
        
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()
        
        self.get_logger().info('Keyboard control node started. Use WASD keys to control the robot.')
        self.get_logger().info('W: Forward, S: Backward, A: Rotate left, D: Rotate right')
        self.get_logger().info('ESC: Quit')
        
    def on_press(self, key):
        try:
            with self.lock:
                if key == keyboard.Key.esc:
                    self.get_logger().info('ESC pressed, shutting down...')
                    rclpy.shutdown()
                    return False
                
                if hasattr(key, 'char') and key.char:
                    char = key.char.lower()
                    if char in ['w', 'a', 's', 'd']:
                        self.pressed_keys.add(char)
                        self.get_logger().info(f'Key pressed: {char}')
                        
        except AttributeError as e:
            self.get_logger().debug(f'Special key pressed: {key}')
    
    def on_release(self, key):
        try:
            with self.lock:
                if hasattr(key, 'char') and key.char:
                    char = key.char.lower()
                    if char in self.pressed_keys:
                        self.pressed_keys.remove(char)
                        self.get_logger().info(f'Key released: {char}')
                        
        except (AttributeError, KeyError):
            pass
    
    def publish_twist(self):
        twist = Twist()
        
        with self.lock:
            if 'w' in self.pressed_keys and 's' in self.pressed_keys:
                twist.linear.x = 0.0
            elif 'w' in self.pressed_keys:
                twist.linear.x = self.linear_speed
            elif 's' in self.pressed_keys:
                twist.linear.x = -self.linear_speed
            else:
                twist.linear.x = 0.0
                
            if 'a' in self.pressed_keys and 'd' in self.pressed_keys:
                twist.angular.z = 0.0
            elif 'a' in self.pressed_keys:
                twist.angular.z = self.angular_speed
            elif 'd' in self.pressed_keys:
                twist.angular.z = -self.angular_speed
            else:
                twist.angular.z = 0.0
        
        self.publisher.publish(twist)
        
        if twist.linear.x != 0.0 or twist.angular.z != 0.0:
            self.get_logger().debug(f'Published: linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}')
    
    def destroy_node(self):
        if hasattr(self, 'listener'):
            self.listener.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down keyboard control node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    #al hamdo lilah
