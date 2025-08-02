#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2015 Enrique Fernandez
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Enrique Fernandez nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#
# Authors:
#   * Enrique Fernandez
#   * Jeremie Deray (artivis)

import signal
import tkinter

from geometry_msgs.msg import Twist, Vector3
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import numpy
import rclpy
from rclpy.node import Node


class MouseTeleop(Node):

    def __init__(self):
        super().__init__('mouse_teleop')

        # Retrieve params:
        self._frequency = self.declare_parameter('frequency', 0.0).value
        self._scale = self.declare_parameter('scale', 1.0).value
        self._holonomic = self.declare_parameter('holonomic', False).value
        self._max_steering_angle = self.declare_parameter('max_steering_angle', 0.5).value  # radians
        self._wheelbase = self.declare_parameter('wheelbase', 2.5).value  # meters

        # Create publishers for both message types
        self._pub_twist = self.create_publisher(Twist, 'cmd_vel', 10)
        self._pub_ackermann = self.create_publisher(AckermannDriveStamped, 'ackermann_cmd', 10)

        # Message type toggle (True = Ackermann, False = Twist)
        self._use_ackermann = True

        # Initialize twist components to zero:
        self._v_x = 0.0
        self._v_y = 0.0
        self._w = 0.0

        # Initialize ackermann components to zero:
        self._speed = 0.0
        self._steering_angle = 0.0

        # Initialize mouse position (x, y) to None (unknown); it's initialized
        # when the mouse button is pressed on the _start callback that handles
        # that event:
        self._x = None
        self._y = None

        # Create window:
        self._root = tkinter.Tk()
        self._root.title('Mouse Teleop - Ackermann Mode')

        # Make window non-resizable:
        self._root.resizable(0, 0)

        # Create canvas:
        self._canvas = tkinter.Canvas(self._root, bg='white')

        # Create canvas objects:
        self._canvas.create_arc(0, 0, 0, 0, fill='red', outline='red',
                                width=1, style=tkinter.PIESLICE, start=90.0, tag='w')
        self._canvas.create_line(0, 0, 0, 0, fill='blue', width=4, tag='v_x')

        if self._holonomic:
            self._canvas.create_line(0, 0, 0, 0, fill='blue', width=4, tag='v_y')

        # Create canvas text objects:
        self._text_v_x = tkinter.StringVar()
        if self._holonomic:
            self._text_v_y = tkinter.StringVar()
        self._text_w = tkinter.StringVar()
        self._text_mode = tkinter.StringVar()

        self._label_v_x = tkinter.Label(self._root, anchor=tkinter.W, textvariable=self._text_v_x)
        if self._holonomic:
            self._label_v_y = tkinter.Label(
                self._root, anchor=tkinter.W, textvariable=self._text_v_y)
        self._label_w = tkinter.Label(self._root, anchor=tkinter.W, textvariable=self._text_w)
        self._label_mode = tkinter.Label(self._root, anchor=tkinter.W, textvariable=self._text_mode, 
                                        fg='red', font=('Arial', 10, 'bold'))

        self._update_display_text()

        self._label_mode.pack()
        self._label_v_x.pack()
        if self._holonomic:
            self._label_v_y.pack()
        self._label_w.pack()

        # Add instruction label
        instruction_text = "Press 's' to toggle between Ackermann and Twist modes"
        self._label_instruction = tkinter.Label(self._root, text=instruction_text, 
                                               anchor=tkinter.W, fg='gray')
        self._label_instruction.pack()

        # Bind event handlers:
        self._canvas.bind('<Button-1>', self._start)
        self._canvas.bind('<ButtonRelease-1>', self._release)

        self._canvas.bind('<Configure>', self._configure)

        if self._holonomic:
            self._canvas.bind('<B1-Motion>', self._mouse_motion_linear)
            self._canvas.bind('<Shift-B1-Motion>', self._mouse_motion_angular)

            self._root.bind('<Shift_L>', self._change_to_motion_angular)
            self._root.bind('<KeyRelease-Shift_L>', self._change_to_motion_linear)
        else:
            self._canvas.bind('<B1-Motion>', self._mouse_motion_angular)

        # Bind 's' key to toggle message type
        self._root.bind('<KeyPress-s>', self._toggle_message_type)
        self._root.focus_set()  # Make sure the window can receive key events

        self._canvas.pack()

        # If frequency is positive, use synchronous publishing mode:
        if self._frequency > 0.0:
            # Create timer for the given frequency to publish messages:
            period = 1.0 / self._frequency

            self._timer = self.create_timer(period, self._publish_command)

        # Handle ctrl+c on the window
        self._root.bind('<Control-c>', self._quit)

        # Nasty polling-trick to handle ctrl+c in terminal
        self._root.after(50, self._check)
        signal.signal(2, self._handle_signal)

        # Log initial state
        self.get_logger().info('Mouse teleop started in Ackermann mode. Press "s" to toggle.')

        # Start window event manager main loop:
        self._root.mainloop()

    def _toggle_message_type(self, event):
        """Toggle between Ackermann and Twist message types"""
        self._use_ackermann = not self._use_ackermann
        
        if self._use_ackermann:
            self._root.title('Mouse Teleop - Ackermann Mode')
            self.get_logger().info('Switched to Ackermann mode')
        else:
            self._root.title('Mouse Teleop - Twist Mode')
            self.get_logger().info('Switched to Twist mode')
        
        self._update_display_text()

    def _update_display_text(self):
        """Update the display text based on current mode"""
        if self._use_ackermann:
            self._text_mode.set('Mode: ACKERMANN')
            self._text_v_x.set('speed = %0.2f m/s' % self._speed)
            self._text_w.set('steering = %0.2f deg' % numpy.rad2deg(self._steering_angle))
        else:
            self._text_mode.set('Mode: TWIST')
            if self._holonomic:
                self._text_v_x.set('v_x = %0.2f m/s' % self._v_x)
                self._text_v_y.set('v_y = %0.2f m/s' % self._v_y)
                self._text_w.set('w   = %0.2f deg/s' % numpy.rad2deg(self._w))
            else:
                self._text_v_x.set('v = %0.2f m/s' % self._v_x)
                self._text_w.set('w = %0.2f deg/s' % numpy.rad2deg(self._w))

    def _quit(self, ev):
        self._root.quit()

    def __del__(self):
        self._root.quit()

    def _check(self):
        self._root.after(50, self._check)

    def _handle_signal(self, signum, frame):
        self._quit(None)

    def _start(self, event):
        self._x, self._y = event.y, event.x

        self._y_linear = self._y_angular = 0

        self._v_x = self._v_y = self._w = 0.0
        self._speed = self._steering_angle = 0.0

    def _release(self, event):
        self._v_x = self._v_y = self._w = 0.0
        self._speed = self._steering_angle = 0.0

        self._send_motion()

    def _configure(self, event):
        self._width, self._height = event.height, event.width

        self._c_x = self._height / 2.0
        self._c_y = self._width / 2.0

        self._r = min(self._height, self._width) * 0.25

    def _mouse_motion_linear(self, event):
        self._v_x, self._v_y = self._relative_motion(event.y, event.x)

        if self._use_ackermann:
            # Convert to Ackermann parameters
            self._speed = self._v_x
            self._steering_angle = self._v_y * self._max_steering_angle
        
        self._send_motion()

    def _mouse_motion_angular(self, event):
        dx, dy = self._relative_motion(event.y, event.x)
        
        if self._use_ackermann:
            # For Ackermann: dx = speed, dy = steering angle
            self._speed = dx
            self._steering_angle = dy * self._max_steering_angle
        else:
            # For Twist: dx = linear velocity, dy = angular velocity
            self._v_x = dx
            self._w = dy

        self._send_motion()

    def _update_coords(self, tag, x0, y0, x1, y1):
        x0 += self._c_x
        y0 += self._c_y

        x1 += self._c_x
        y1 += self._c_y

        self._canvas.coords(tag, (x0, y0, x1, y1))

    def _draw_v_x(self, v):
        x = -v * float(self._width)

        self._update_coords('v_x', 0, 0, 0, x)

    def _draw_v_y(self, v):
        y = -v * float(self._height)

        self._update_coords('v_y', 0, 0, y, 0)

    def _draw_w(self, w):
        x0 = y0 = -self._r
        x1 = y1 = self._r

        self._update_coords('w', x0, y0, x1, y1)

        if self._use_ackermann:
            # For Ackermann, show steering angle
            angle_deg = w * numpy.rad2deg(1.0)  # w is already in degrees for display
        else:
            # For Twist, show angular velocity
            angle_deg = w * numpy.rad2deg(self._scale)

        self._canvas.itemconfig('w', extent=angle_deg)

    def _send_motion(self):
        if self._use_ackermann:
            self._draw_v_x(self._speed)
            self._draw_w(numpy.rad2deg(self._steering_angle))
        else:
            self._draw_v_x(self._v_x)
            if self._holonomic:
                self._draw_v_y(self._v_y)
            self._draw_w(self._w)

        self._update_display_text()

        if self._use_ackermann:
            self._publish_ackermann()
        else:
            self._publish_twist()

    def _publish_twist(self):
        """Publish Twist message"""
        v_x = self._v_x * self._scale
        v_y = self._v_y * self._scale
        w = self._w * self._scale

        lin = Vector3(x=v_x, y=v_y, z=0.0)
        ang = Vector3(x=0.0, y=0.0, z=w)

        twist = Twist(linear=lin, angular=ang)
        self._pub_twist.publish(twist)

    def _publish_ackermann(self):
        """Publish Ackermann message"""
        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.header.stamp = self.get_clock().now().to_msg()
        ackermann_msg.header.frame_id = "base_link"
        
        ackermann_msg.drive.speed = self._speed * self._scale
        ackermann_msg.drive.steering_angle = self._steering_angle
        
        # Optional: calculate steering_angle_velocity if needed
        # ackermann_msg.drive.steering_angle_velocity = 0.0
        
        self._pub_ackermann.publish(ackermann_msg)

    def _publish_command(self):
        """Timer callback to publish commands"""
        self._send_motion()

    def _relative_motion(self, x, y):
        dx = self._x - x
        dy = self._y - y

        dx /= float(self._width)
        dy /= float(self._height)

        dx = max(-1.0, min(dx, 1.0))
        dy = max(-1.0, min(dy, 1.0))

        return dx, dy

    def _change_to_motion_linear(self, event):
        if self._y is not None:
            y = event.x

            self._y_angular = self._y - y
            self._y = self._y_linear + y

    def _change_to_motion_angular(self, event):
        if self._y is not None:
            y = event.x

            self._y_linear = self._y - y
            self._y = self._y_angular + y


def main():
    try:
        rclpy.init()

        node = MouseTeleop()

        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
