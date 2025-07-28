# New Keyboard Teleop with ackermann_msgs.msg 
# Fixed_by Jisang_Yun
# Modified: Removed WASD, Added S key for Twist/Ackermann toggle

import curses
import os
import signal
import time
import threading
from collections import defaultdict

try:
    from pynput import keyboard
    PYNPUT_AVAILABLE = True
except ImportError:
    PYNPUT_AVAILABLE = False
    print("pynput not available. Install with: pip install pynput")

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist


class TextWindow():
    """Curses-based text window for clean GUI display"""
    
    def __init__(self, stdscr, lines=10):
        self._screen = stdscr
        self._screen.nodelay(True)
        curses.curs_set(0)  # Hide cursor
        self._num_lines = lines
        
        # Initialize colors if available
        if curses.has_colors():
            curses.start_color()
            curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)
            curses.init_pair(2, curses.COLOR_YELLOW, curses.COLOR_BLACK)
            curses.init_pair(3, curses.COLOR_RED, curses.COLOR_BLACK)
            curses.init_pair(4, curses.COLOR_CYAN, curses.COLOR_BLACK)

    def clear(self):
        self._screen.clear()

    def write_line(self, lineno, message, color_pair=0):
        """Write a line of text to the screen"""
        if lineno < 0 or lineno >= self._num_lines:
            return
        
        height, width = self._screen.getmaxyx()
        y = int((height / self._num_lines) * lineno)
        x = 2
        
        for i, text in enumerate(message.split('\n')):
            if y + i < height - 1:  # Ensure we don't write past screen bounds
                text = text.ljust(min(len(text), width - x - 1))
                try:
                    if color_pair > 0:
                        self._screen.addstr(y + i, x, text, curses.color_pair(color_pair))
                    else:
                        self._screen.addstr(y + i, x, text)
                except curses.error:
                    pass  # Ignore errors from writing to screen edges

    def draw_title(self, title="F1TENTH Keyboard Teleop"):
        """Draw title without border"""
        height, width = self._screen.getmaxyx()
        title_x = max(2, (width - len(title)) // 2)
        self._screen.addstr(0, title_x, title, curses.color_pair(4))

    def refresh(self):
        self._screen.refresh()


class PynputCursesKeyTeleop(Node):
    def __init__(self, interface):
        super().__init__('pynput_curses_key_teleop')
        
        if not PYNPUT_AVAILABLE:
            self.get_logger().error("pynput library is required. Install with: pip install pynput")
            return
        
        self._interface = interface
        self._hz = 10.0
        self._forward_rate = 0.8
        self._backward_rate = 0.5
        self._rotation_rate = 1.0
        self._running = True
        
        # Message type toggle (True for Ackermann, False for Twist)
        self._use_ackermann = True
        
        # Thread-safe key state tracking
        self._key_states = defaultdict(bool)
        self._state_lock = threading.Lock()
        
        # Key mappings to movement (linear, angular) - Only arrow keys
        self._key_mappings = {
            keyboard.Key.up: (1.0, 0.0),      # forward
            keyboard.Key.down: (-1.0, 0.0),   # backward
            keyboard.Key.left: (0.0, 5.0),    # left turn
            keyboard.Key.right: (0.0, -5.0),  # right turn
        }
        
        self._linear = 0.0
        self._angular = 0.0
        self._active_keys = []
        
        # ROS publishers
        self._pub_ackermann = self.create_publisher(
            AckermannDriveStamped, '/ackermann_cmd', qos_profile_system_default)
        self._pub_twist = self.create_publisher(
            Twist, '/cmd_vel', qos_profile_system_default)
        
        # Start keyboard listener
        self._start_keyboard_listener()

    def _start_keyboard_listener(self):
        """Start the keyboard listener in a separate thread"""
        def on_press(key):
            # Handle quit command
            if key == keyboard.KeyCode.from_char('q') or key == keyboard.KeyCode.from_char('Q'):
                self.get_logger().info("Quit key pressed. Shutting down...")
                self._running = False
                return False
            
            # Handle message type toggle
            if key == keyboard.KeyCode.from_char('s') or key == keyboard.KeyCode.from_char('S'):
                self._use_ackermann = not self._use_ackermann
                msg_type = "Ackermann" if self._use_ackermann else "Twist"
                self.get_logger().info(f"Switched to {msg_type} mode")
                return
            
            # Handle movement keys
            if key in self._key_mappings:
                with self._state_lock:
                    self._key_states[key] = True

        def on_release(key):
            if key in self._key_mappings:
                with self._state_lock:
                    self._key_states[key] = False

        # Start listener WITHOUT suppress - allows normal keyboard usage elsewhere
        self._listener = keyboard.Listener(
            on_press=on_press,
            on_release=on_release
        )
        self._listener.start()

    def run(self):
        """Main run loop"""
        while self._running:
            self._calculate_velocity()
            self._publish()
            time.sleep(1.0 / self._hz)

    def _calculate_velocity(self):
        """Calculate velocity based on currently pressed keys"""
        linear = 0.0
        angular = 0.0
        active_keys = []
        
        with self._state_lock:
            current_states = dict(self._key_states)
        
        for key, is_pressed in current_states.items():
            if is_pressed and key in self._key_mappings:
                l, a = self._key_mappings[key]
                linear += l
                angular += a
                
                # Add to active keys for display
                key_name = self._get_key_name(key)
                active_keys.append(key_name)
        
        # Apply rate scaling
        if linear > 0:
            linear *= self._forward_rate
        elif linear < 0:
            linear *= self._backward_rate
        
        angular *= self._rotation_rate
        
        # Clamp values to reasonable ranges
        linear = max(-3.0, min(3.0, linear))
        angular = max(-5.0, min(5.0, angular))
        
        self._linear = linear
        self._angular = angular
        self._active_keys = active_keys

    def _get_key_name(self, key):
        """Convert key to readable string"""
        key_names = {
            keyboard.Key.up: '↑',
            keyboard.Key.down: '↓', 
            keyboard.Key.left: '←',
            keyboard.Key.right: '→'
        }
        
        if key in key_names:
            return key_names[key]
        elif hasattr(key, 'char') and key.char:
            return key.char.upper()
        else:
            return str(key)

    def _make_ackermann_msg(self, speed, steering_angle):
        """Create Ackermann message"""
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = speed
        msg.drive.steering_angle = steering_angle
        return msg

    def _make_twist_msg(self, linear_vel, angular_vel):
        """Create Twist message"""
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel
        return msg

    def _publish(self):
        """Publish control commands and update GUI"""
        # Clear and update interface
        self._interface.clear()
        self._interface.draw_title()
        
        # Display current status
        active_str = ' '.join(self._active_keys) if self._active_keys else 'None'
        
        # Message type display
        msg_type = "Ackermann" if self._use_ackermann else "Twist"
        msg_color = 1 if self._use_ackermann else 2
        self._interface.write_line(2, f"Mode: {msg_type}", msg_color)
        
        # Speed display with color coding
        speed_color = 1 if self._linear > 0 else (3 if self._linear < 0 else 0)
        self._interface.write_line(3, f"Speed:    {self._linear:+6.2f} m/s", speed_color)
        
        # Steering display with color coding  
        steer_color = 2 if abs(self._angular) > 0 else 0
        self._interface.write_line(4, f"Steering: {self._angular:+6.2f} rad/s", steer_color)
        
        # Active keys display
        self._interface.write_line(5, f"Keys:     {active_str}")
        
        # Controls display
        self._interface.write_line(7, "Controls")
        self._interface.write_line(8, "↑: Forward      ↓: Backward")
        self._interface.write_line(9, "←: Left         →: Right")
        self._interface.write_line(10, "S: Toggle Mode  Q: Quit")
        
        # Additional info
        if self._active_keys:
            self._interface.write_line(12, "● DRIVING", 1)
        else:
            self._interface.write_line(12, "○ STOPPED", 2)
            
        self._interface.write_line(13, f"Rate: {self._hz} Hz | You can type in other apps!")
        
        # Topic info
        topic = "/ackermann_cmd" if self._use_ackermann else "/cmd_vel"
        self._interface.write_line(14, f"Publishing to: {topic}")
        
        self._interface.refresh()
        
        # Publish appropriate message type
        if self._use_ackermann:
            ackermann_msg = self._make_ackermann_msg(self._linear, self._angular)
            self._pub_ackermann.publish(ackermann_msg)
        else:
            twist_msg = self._make_twist_msg(self._linear, self._angular)
            self._pub_twist.publish(twist_msg)

    def stop(self):
        """Clean shutdown"""
        self._running = False
        if hasattr(self, '_listener'):
            self._listener.stop()


def execute(stdscr):
    """Main execution function for curses"""
    if not PYNPUT_AVAILABLE:
        stdscr.addstr(0, 0, "Error: pynput library is required")
        stdscr.addstr(1, 0, "Install with: pip install pynput")
        stdscr.addstr(2, 0, "Press any key to exit...")
        stdscr.refresh()
        stdscr.getch()
        return
    
    rclpy.init()
    
    try:
        app = PynputCursesKeyTeleop(TextWindow(stdscr, lines=20))
        app.run()
    except KeyboardInterrupt:
        pass
    finally:
        if 'app' in locals():
            app.stop()
        rclpy.shutdown()


def main():
    """Main entry point"""
    if not PYNPUT_AVAILABLE:
        print("Error: pynput library is required")
        print("Install with: pip install pynput")
        return
    
    try:
        curses.wrapper(execute)
    except KeyboardInterrupt:
        print("\nShutdown completed.")


if __name__ == '__main__':
    main()
