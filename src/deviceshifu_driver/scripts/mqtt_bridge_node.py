#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import paho.mqtt.client as mqtt
import json
import yaml
import os
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, Imu, LaserScan, CompressedImage
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import cv2
import numpy as np
from threading import Lock

class MQTTBridgeNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('mqtt_bridge_node', anonymous=True)
        
        # Load configuration from params.yaml
        self.config = self.load_config()
        
        # Initialize MQTT client
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        
        # Connect to MQTT broker
        self.connect_to_broker()
        
        # Dictionary to store ROS publishers and subscribers
        self.ros_publishers = {}
        self.ros_subscribers = {}
        
        # Thread-safe lock for MQTT client
        self.mqtt_lock = Lock()
        
        # Setup ROS-MQTT bridges
        self.setup_bridges()
        
        rospy.loginfo('MQTT Bridge node initialized')

    def load_config(self):
        """Load configuration from params.yaml"""
        config_path = os.path.join(os.path.dirname(__file__), '../../config/params.yaml')
        rospy.loginfo(f'Loading configuration from {config_path}')
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            self.validate_config(config)
            return config
        except Exception as e:
            rospy.logerr(f'Error loading configuration: {str(e)}')
            raise

    def validate_config(self, config):
        """Validate configuration"""
        required_keys = ['broker', 'bridge']
        for key in required_keys:
            if key not in config:
                raise ValueError(f'Missing required configuration key: {key}')
        
        if 'host' not in config['broker'] or 'port' not in config['broker']:
            raise ValueError('Missing broker host or port configuration')
        
        if 'ros2mqtt' not in config['bridge'] or 'mqtt2ros' not in config['bridge']:
            raise ValueError('Missing bridge configuration')

    def connect_to_broker(self):
        """Connect to MQTT broker with retry mechanism"""
        max_retries = 5
        retry_delay = 2
        
        for attempt in range(max_retries):
            try:
                self.mqtt_client.connect(
                    self.config['broker']['host'],
                    self.config['broker']['port']
                )
                self.mqtt_client.loop_start()
                rospy.loginfo('Successfully connected to MQTT broker')
                return
            except Exception as e:
                rospy.logwarn(f'Failed to connect to MQTT broker (attempt {attempt + 1}/{max_retries}): {str(e)}')
                if attempt < max_retries - 1:
                    time.sleep(retry_delay)
        
        rospy.logerr('Failed to connect to MQTT broker after maximum retries')
        raise ConnectionError('Could not connect to MQTT broker')

    def setup_bridges(self):
        """Setup ROS-MQTT bridges based on configuration"""
        # Setup ROS to MQTT bridges
        for bridge in self.config['bridge']['ros2mqtt']:
            ros_topic = bridge['ros_topic']
            mqtt_topic = bridge['mqtt_topic']
            
            # Create ROS subscriber based on topic type
            msg_type = self.get_message_type(ros_topic)
            if msg_type:
                self.ros_subscribers[ros_topic] = rospy.Subscriber(
                    ros_topic,
                    msg_type,
                    lambda msg, topic=ros_topic, mqtt_topic=mqtt_topic: 
                    self.ros_to_mqtt_callback(msg, topic, mqtt_topic)
                )
                rospy.loginfo(f'Created ROS to MQTT bridge: {ros_topic} -> {mqtt_topic}')

        # Setup MQTT to ROS bridges
        for bridge in self.config['bridge']['mqtt2ros']:
            mqtt_topic = bridge['mqtt_topic']
            ros_topic = bridge['ros_topic']
            
            # Subscribe to MQTT topic
            self.mqtt_client.subscribe(mqtt_topic)
            rospy.loginfo(f'Subscribed to MQTT topic: {mqtt_topic}')

    def get_message_type(self, topic):
        """Determine ROS message type based on topic"""
        if '/cmd_vel' in topic:
            return Twist
        elif '/imu' in topic:
            return Imu
        elif '/odom' in topic:
            return Odometry
        elif '/scan' in topic:
            return LaserScan
        elif '/PowerVoltage' in topic:
            return Float32
        elif '/camera' in topic:
            if 'compressed' in topic:
                return CompressedImage
            return Image
        return None

    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Callback when MQTT client connects to broker"""
        if rc == 0:
            rospy.loginfo('Connected to MQTT broker')
            # Resubscribe to all topics
            for bridge in self.config['bridge']['mqtt2ros']:
                self.mqtt_client.subscribe(bridge['mqtt_topic'])
        else:
            rospy.logerr(f'Failed to connect to MQTT broker with code {rc}')

    def on_mqtt_disconnect(self, client, userdata, rc):
        """Callback when MQTT client disconnects"""
        if rc != 0:
            rospy.logwarn('Unexpected disconnection from MQTT broker')
            # Attempt to reconnect
            self.connect_to_broker()

    def on_mqtt_message(self, client, userdata, msg):
        """Callback when MQTT message is received"""
        try:
            # Find corresponding ROS topic
            ros_topic = None
            for bridge in self.config['bridge']['mqtt2ros']:
                if bridge['mqtt_topic'] == msg.topic:
                    ros_topic = bridge['ros_topic']
                    break
            
            if ros_topic:
                # Convert MQTT message to ROS message and publish
                if ros_topic == '/cmd_vel':
                    twist_msg = Twist()
                    data = json.loads(msg.payload)
                    twist_msg.linear.x = data.get('linear', {}).get('x', 0.0)
                    twist_msg.angular.z = data.get('angular', {}).get('z', 0.0)
                    
                    if ros_topic not in self.ros_publishers:
                        self.ros_publishers[ros_topic] = rospy.Publisher(
                            ros_topic, Twist, queue_size=10
                        )
                    self.ros_publishers[ros_topic].publish(twist_msg)
                    rospy.logdebug(f'Published to ROS topic {ros_topic}: {twist_msg}')
                
        except Exception as e:
            rospy.logerr(f'Error processing MQTT message: {str(e)}')

    def ros_to_mqtt_callback(self, msg, ros_topic, mqtt_topic):
        """Callback when ROS message is received"""
        try:
            with self.mqtt_lock:
                if isinstance(msg, Twist):
                    data = {
                        'linear': {'x': msg.linear.x, 'y': msg.linear.y, 'z': msg.linear.z},
                        'angular': {'x': msg.angular.x, 'y': msg.angular.y, 'z': msg.angular.z}
                    }
                    self.mqtt_client.publish(mqtt_topic, json.dumps(data))
                
                elif isinstance(msg, Imu):
                    data = {
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
                    self.mqtt_client.publish(mqtt_topic, json.dumps(data))
                
                elif isinstance(msg, Odometry):
                    data = {
                        'pose': {
                            'position': {
                                'x': msg.pose.pose.position.x,
                                'y': msg.pose.pose.position.y,
                                'z': msg.pose.pose.position.z
                            },
                            'orientation': {
                                'x': msg.pose.pose.orientation.x,
                                'y': msg.pose.pose.orientation.y,
                                'z': msg.pose.pose.orientation.z,
                                'w': msg.pose.pose.orientation.w
                            }
                        },
                        'twist': {
                            'linear': {
                                'x': msg.twist.twist.linear.x,
                                'y': msg.twist.twist.linear.y,
                                'z': msg.twist.twist.linear.z
                            },
                            'angular': {
                                'x': msg.twist.twist.angular.x,
                                'y': msg.twist.twist.angular.y,
                                'z': msg.twist.twist.angular.z
                            }
                        }
                    }
                    self.mqtt_client.publish(mqtt_topic, json.dumps(data))
                
                elif isinstance(msg, LaserScan):
                    data = {
                        'ranges': msg.ranges[:],
                        'angle_min': msg.angle_min,
                        'angle_max': msg.angle_max,
                        'angle_increment': msg.angle_increment,
                        'range_min': msg.range_min,
                        'range_max': msg.range_max
                    }
                    self.mqtt_client.publish(mqtt_topic, json.dumps(data))
                
                elif isinstance(msg, Float32):
                    self.mqtt_client.publish(mqtt_topic, json.dumps({'value': msg.data}))
                
                elif isinstance(msg, (Image, CompressedImage)):
                    if isinstance(msg, CompressedImage):
                        # Already compressed, just send the data
                        self.mqtt_client.publish(mqtt_topic, msg.data)
                    else:
                        # Convert to compressed image
                        np_arr = np.frombuffer(msg.data, np.uint8)
                        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                        _, compressed_data = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
                        self.mqtt_client.publish(mqtt_topic, compressed_data.tobytes())
                
                else:
                    # Default conversion for unknown types
                    data = self.msg_to_dict(msg)
                    self.mqtt_client.publish(mqtt_topic, json.dumps(data))
            
        except Exception as e:
            rospy.logerr(f'Error processing ROS message: {str(e)}')

    def msg_to_dict(self, msg):
        """Convert ROS message to dictionary"""
        return {attr: getattr(msg, attr) for attr in dir(msg) 
                if not attr.startswith('_') and not callable(getattr(msg, attr))}

    def run(self):
        """Run node"""
        try:
            rospy.spin()
        except Exception as e:
            rospy.logerr(f'Error occurred: {str(e)}')
        finally:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()

def main():
    try:
        bridge_node = MQTTBridgeNode()
        bridge_node.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main() 