#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Float32
from sensor_msgs.msg import Imu
import time
from flask import Flask, request, jsonify
import threading
import os
import socket
import json

class DeviceShifuDriver:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('deviceshifu_driver', anonymous=True)
        
        # Initialize parameters
        self.linear_speed = rospy.get_param('~linear_speed', 0.5)
        self.angular_speed = rospy.get_param('~angular_speed', 0.2)
        self.http_port = rospy.get_param('~http_port', 5000)
        
        # 添加速度限制
        self.max_linear_speed = 1.0
        self.min_linear_speed = 0.1
        self.max_angular_speed = 0.5
        self.min_angular_speed = 0.1
        
        rospy.loginfo(f'Initialized with speeds - Linear: {self.linear_speed}, Angular: {self.angular_speed}')
        
        # Create publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Initialize movement state
        self.current_command = 0  # 0: stop, 1: forward, 2: backward, 3: left, 4: right
        self.is_moving = False
        self.movement_timer = None
        
        # Initialize sensor data
        self.sensor_data = {
            'imu': None,
            'power_voltage': None,
            'imu_low': None
        }
        
        # Create subscribers for sensor data
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.power_voltage_sub = rospy.Subscriber('/PowerVoltage', Float32, self.power_voltage_callback)
        self.imu_low_sub = rospy.Subscriber('/imu_low', Imu, self.imu_low_callback)
        
        # Initialize Flask app
        self.app = Flask(__name__)
        self.setup_routes()
        
        # Start HTTP server in a separate thread
        self.http_thread = threading.Thread(target=self.run_http_server)
        self.http_thread.daemon = True
        self.http_thread.start()
        
        rospy.loginfo('DeviceShifu driver initialized with HTTP server on port %d', self.http_port)

    def imu_callback(self, msg):
        """Callback for IMU data"""
        self.sensor_data['imu'] = {
            'orientation': {
                'x': msg.orientation.x,
                'y': msg.orientation.y,
                'z': msg.orientation.z,
                'w': msg.orientation.w
            },
            'angular_velocity': {
                'x': msg.angular_velocity.x,
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z
            },
            'linear_acceleration': {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z
            }
        }

    def imu_low_callback(self, msg):
        """Callback for low frequency IMU data"""
        self.sensor_data['imu_low'] = {
            'orientation': {
                'x': msg.orientation.x,
                'y': msg.orientation.y,
                'z': msg.orientation.z,
                'w': msg.orientation.w
            },
            'angular_velocity': {
                'x': msg.angular_velocity.x,
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z
            },
            'linear_acceleration': {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z
            }
        }

    def power_voltage_callback(self, msg):
        """Callback for power voltage data"""
        self.sensor_data['power_voltage'] = msg.data

    def setup_routes(self):
        @self.app.route('/move', methods=['POST'])
        def move():
            try:
                data = request.get_json()
                command = data.get('command')
                # 获取可选的速度参数
                linear_speed = data.get('linear_speed')
                angular_speed = data.get('angular_speed')
                
                if command is not None:
                    self.current_command = int(command)
                    self.is_moving = True
                    
                    # 如果提供了速度参数，更新速度
                    if linear_speed is not None:
                        self.linear_speed = max(min(float(linear_speed), self.max_linear_speed), self.min_linear_speed)
                    if angular_speed is not None:
                        self.angular_speed = max(min(float(angular_speed), self.max_angular_speed), self.min_angular_speed)
                    
                    if self.movement_timer:
                        self.movement_timer.shutdown()
                    
                    self.movement_timer = rospy.Timer(rospy.Duration(0.1), self.movement_callback)
                    return jsonify({
                        'status': 'success', 
                        'command': command,
                        'current_speeds': {
                            'linear': self.linear_speed,
                            'angular': self.angular_speed
                        }
                    })
                else:
                    return jsonify({'status': 'error', 'message': 'No command provided'}), 400
            except Exception as e:
                return jsonify({'status': 'error', 'message': str(e)}), 500

        @self.app.route('/speed', methods=['POST'])
        def set_speed():
            try:
                data = request.get_json()
                linear_speed = data.get('linear_speed')
                angular_speed = data.get('angular_speed')
                
                if linear_speed is not None:
                    self.linear_speed = max(min(float(linear_speed), self.max_linear_speed), self.min_linear_speed)
                if angular_speed is not None:
                    self.angular_speed = max(min(float(angular_speed), self.max_angular_speed), self.min_angular_speed)
                
                return jsonify({
                    'status': 'success',
                    'current_speeds': {
                        'linear': self.linear_speed,
                        'angular': self.angular_speed
                    }
                })
            except Exception as e:
                return jsonify({'status': 'error', 'message': str(e)}), 500

        @self.app.route('/speed', methods=['GET'])
        def get_speed():
            return jsonify({
                'status': 'success',
                'current_speeds': {
                    'linear': self.linear_speed,
                    'angular': self.angular_speed
                },
                'speed_limits': {
                    'linear': {
                        'min': self.min_linear_speed,
                        'max': self.max_linear_speed
                    },
                    'angular': {
                        'min': self.min_angular_speed,
                        'max': self.max_angular_speed
                    }
                }
            })

        @self.app.route('/stop', methods=['POST'])
        def stop():
            try:
                self.current_command = 0
                self.is_moving = False
                if self.movement_timer:
                    self.movement_timer.shutdown()
                
                # 发送停止命令到ROS话题
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                cmd.linear.z = 0.0
                cmd.angular.x = 0.0
                cmd.angular.y = 0.0
                cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd)
                
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
                'angular_speed': self.angular_speed,
                'sensor_data': self.sensor_data
            })

        @self.app.route('/sensors', methods=['GET'])
        def sensors():
            """Get all sensor data"""
            return jsonify({
                'status': 'success',
                'sensor_data': self.sensor_data
            })

        @self.app.route('/imu', methods=['GET'])
        def imu():
            """Get IMU data"""
            return jsonify({
                'status': 'success',
                'imu_data': self.sensor_data['imu']
            })

        @self.app.route('/power_voltage', methods=['GET'])
        def power_voltage():
            """Get power voltage data"""
            return jsonify({
                'status': 'success',
                'voltage': self.sensor_data['power_voltage']
            })

    def run_http_server(self):
        try:
            # 获取容器IP地址
            container_ip = os.environ.get('POD_IP', '0.0.0.0')
            hostname = socket.gethostname()
            local_ip = socket.gethostbyname(hostname)
            
            rospy.loginfo(f'Container IP: {container_ip}')
            rospy.loginfo(f'Hostname: {hostname}')
            rospy.loginfo(f'Local IP: {local_ip}')
            rospy.loginfo(f'Starting HTTP server on 0.0.0.0:{self.http_port}')
            
            # 使用0.0.0.0确保服务器监听所有网络接口
            self.app.run(host='0.0.0.0', port=self.http_port, debug=True, use_reloader=False)
        except Exception as e:
            rospy.logerr(f'Failed to start HTTP server: {str(e)}')

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