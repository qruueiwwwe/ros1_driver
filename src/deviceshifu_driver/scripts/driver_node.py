#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import time
from flask import Flask, request, jsonify
import threading
import os

class DeviceShifuDriver:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('deviceshifu_driver', anonymous=True)
        
        # Initialize parameters
        self.linear_speed = rospy.get_param('~linear_speed', 0.5)
        self.angular_speed = rospy.get_param('~angular_speed', 0.2)
        self.http_port = rospy.get_param('~http_port', 5000)
        
        rospy.loginfo(f'Initialized with speeds - Linear: {self.linear_speed}, Angular: {self.angular_speed}')
        
        # Create publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Initialize movement state
        self.current_command = 0  # 0: stop, 1: forward, 2: backward, 3: left, 4: right
        self.is_moving = False
        self.movement_timer = None
        
        # Initialize Flask app
        self.app = Flask(__name__)
        self.setup_routes()
        
        # Start HTTP server in a separate thread
        self.http_thread = threading.Thread(target=self.run_http_server)
        self.http_thread.daemon = True
        self.http_thread.start()
        
        rospy.loginfo('DeviceShifu driver initialized with HTTP server on port %d', self.http_port)

    def setup_routes(self):
        @self.app.route('/move', methods=['POST'])
        def move():
            try:
                data = request.get_json()
                command = data.get('command')
                if command is not None:
                    self.current_command = int(command)
                    self.is_moving = True
                    
                    if self.movement_timer:
                        self.movement_timer.shutdown()
                    
                    self.movement_timer = rospy.Timer(rospy.Duration(0.1), self.movement_callback)
                    return jsonify({'status': 'success', 'command': command})
                else:
                    return jsonify({'status': 'error', 'message': 'No command provided'}), 400
            except Exception as e:
                return jsonify({'status': 'error', 'message': str(e)}), 500

        @self.app.route('/stop', methods=['POST'])
        def stop():
            try:
                self.current_command = 0
                self.is_moving = False
                if self.movement_timer:
                    self.movement_timer.shutdown()
                return jsonify({'status': 'success', 'message': 'Robot stopped'})
            except Exception as e:
                return jsonify({'status': 'error', 'message': str(e)}), 500

        @self.app.route('/status', methods=['GET'])
        def status():
            return jsonify({
                'status': 'success',
                'is_moving': self.is_moving,
                'current_command': self.current_command,
                'linear_speed': self.linear_speed,
                'angular_speed': self.angular_speed
            })

    def run_http_server(self):
        # 获取容器IP地址
        container_ip = os.environ.get('POD_IP', '0.0.0.0')
        rospy.loginfo(f'Starting HTTP server on {container_ip}:{self.http_port}')
        self.app.run(host=container_ip, port=self.http_port)

    def movement_callback(self, event):
        """Handle continuous movement"""
        if not self.is_moving:
            return
            
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
                time.sleep(0.1)
        except Exception as e:
            rospy.logerr(f'Error occurred: {str(e)}')
        finally:
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