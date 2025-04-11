#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import cv2
import numpy as np
from cv_bridge import CvBridge
import time
import serial

class DeviceShifuDriver:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('deviceshifu_driver', anonymous=True)
        
        # Initialize parameters
        self.linear_speed = rospy.get_param('~linear_speed', 0.5)
        self.angular_speed = rospy.get_param('~angular_speed', 0.2)
        self.camera_fps = rospy.get_param('~camera_fps', 30)
        
        rospy.loginfo(f'Initialized with speeds - Linear: {self.linear_speed}, Angular: {self.angular_speed}')
        
        # Create publishers
        # 使用与官方驱动相同的话题名称
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.image_pub = rospy.Publisher('camera/image', Image, queue_size=10)
        
        # Create subscriber - for receiving control commands
        self.command_sub = rospy.Subscriber('remote_command', Int32, self.command_callback)
            
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Initialize camera
        self.init_camera()
        
        # Initialize serial port for direct control
        self.init_serial()
        
        # Create rate for publishing images
        self.rate = rospy.Rate(self.camera_fps)
        
        # Initialize movement state
        self.current_command = 0  # 0: stop, 1: forward, 2: backward, 3: left, 4: right
        self.is_moving = False
        self.movement_timer = None
        
        rospy.loginfo('DeviceShifu driver initialized')

    def init_camera(self):
        """Initialize camera"""
        try:
            self.cap = cv2.VideoCapture(0)
            if self.cap.isOpened():
                rospy.loginfo('Successfully connected to camera')
                self.has_camera = True
            else:
                raise Exception("Failed to open camera")
        except Exception as e:
            rospy.logwarn(f'Camera initialization failed: {str(e)}')
            rospy.loginfo('Using simulated image')
            self.has_camera = False
            self.cap = None

    def init_serial(self):
        """Initialize serial port for direct control"""
        try:
            self.ser = serial.Serial(
                port='/dev/ttyUSB0',  # 根据实际串口设备修改
                baudrate=115200,
                timeout=1
            )
            rospy.loginfo('Successfully connected to serial port')
            self.has_serial = True
        except Exception as e:
            rospy.logwarn(f'Serial port initialization failed: {str(e)}')
            self.has_serial = False
            self.ser = None

    def send_control_command(self, cmd):
        """Send control command directly to the robot"""
        if not self.has_serial:
            return
            
        try:
            # 根据命令生成控制指令
            if cmd == 0:    # Stop
                command = "STOP\n"
            elif cmd == 1:  # Forward
                command = f"FORWARD {self.linear_speed}\n"
            elif cmd == 2:  # Backward
                command = f"BACKWARD {self.linear_speed}\n"
            elif cmd == 3:  # Turn left
                command = f"LEFT {self.angular_speed}\n"
            elif cmd == 4:  # Turn right
                command = f"RIGHT {self.angular_speed}\n"
            else:
                return
                
            # 发送命令
            self.ser.write(command.encode())
            rospy.loginfo(f'Sent command: {command.strip()}')
        except Exception as e:
            rospy.logerr(f'Failed to send command: {str(e)}')

    def publish_image(self):
        """Publish image data"""
        if self.has_camera:
            # Use real camera
            ret, frame = self.cap.read()
            if ret:
                try:
                    msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = "camera_frame"
                    self.image_pub.publish(msg)
                    rospy.logdebug('Published real camera image')
                except Exception as e:
                    rospy.logerr(f'Failed to publish image: {str(e)}')
        else:
            # Publish simulated image
            try:
                # Create solid color image (#39c5bb)
                image = np.zeros((480, 720, 3), dtype=np.uint8)
                # Note: OpenCV uses BGR format
                image[:] = [187, 197, 57]  # BGR format of #39c5bb
                
                msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "camera_frame"
                self.image_pub.publish(msg)
            except Exception as e:
                rospy.logerr(f'Failed to publish simulated image: {str(e)}')

    def command_callback(self, msg):
        """Process control commands"""
        self.current_command = msg.data
        self.is_moving = True
        
        # Cancel existing timer if any
        if self.movement_timer:
            self.movement_timer.shutdown()
        
        # Create new timer for continuous movement
        self.movement_timer = rospy.Timer(rospy.Duration(0.1), self.movement_callback)
        
        # Send command directly to the robot
        self.send_control_command(self.current_command)
        
        rospy.loginfo(f'Received command: {msg.data}')

    def movement_callback(self, event):
        """Handle continuous movement"""
        if not self.is_moving:
            return
            
        # Send command directly to the robot
        self.send_control_command(self.current_command)
        
        # Also publish to cmd_vel for compatibility
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        
        if self.current_command == 0:    # Stop
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.is_moving = False
            self.movement_timer.shutdown()
            rospy.loginfo('Stopping movement')
        elif self.current_command == 1:  # Forward
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
            rospy.loginfo(f'Moving forward with speed: {self.linear_speed}')
        elif self.current_command == 2:  # Backward
            cmd.linear.x = -self.linear_speed
            cmd.angular.z = 0.0
            rospy.loginfo(f'Moving backward with speed: {self.linear_speed}')
        elif self.current_command == 3:  # Turn left
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed
            rospy.loginfo(f'Turning left with speed: {self.angular_speed}')
        elif self.current_command == 4:  # Turn right
            cmd.linear.x = 0.0
            cmd.angular.z = -self.angular_speed
            rospy.loginfo(f'Turning right with speed: {self.angular_speed}')
        else:
            rospy.logwarn(f'Unknown command: {self.current_command}')
            return
            
        self.cmd_vel_pub.publish(cmd)
        rospy.logdebug(f'Published cmd_vel - linear: {cmd.linear.x}, angular: {cmd.angular.z}')

    def run(self):
        """Run node"""
        try:
            while not rospy.is_shutdown():
                self.publish_image()
                self.rate.sleep()
        except Exception as e:
            rospy.logerr(f'Error occurred: {str(e)}')
        finally:
            if self.has_camera and self.cap is not None:
                self.cap.release()
            if self.has_serial and self.ser is not None:
                self.ser.close()
            if self.movement_timer:
                self.movement_timer.shutdown()

def main():
    try:
        driver = DeviceShifuDriver()
        driver.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main() 