#!/usr/bin/env python3

import rclpy
import sys
import tf2_ros
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, VehicleCommand,ActuatorMotors, SensorCombined,VehicleOdometry
from tf_transformations import euler_from_quaternion
from rclpy.callback_groups import ReentrantCallbackGroup,MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu



class frames_publish(Node):
   
    def __init__(self):
        
        super().__init__('frames_publisher')     
        self.get_logger().info('Node created')       
                                    
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.callback_group_1=MutuallyExclusiveCallbackGroup()
        self.callback_group_2=MutuallyExclusiveCallbackGroup()
        self.callback_group_3=MutuallyExclusiveCallbackGroup()
        self.callback_group_4=MutuallyExclusiveCallbackGroup()
        self.callback_group_5=MutuallyExclusiveCallbackGroup()
        self.callback_group_6=ReentrantCallbackGroup()

        self.publisher_2=self.create_publisher(Float32MultiArray,'/controller_data',10)
        
        self.subscription_1 = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry',self.sensor_callback_1,qos_profile=qos_profile,callback_group=self.callback_group_1)
        self.subscription_2=self.create_subscription(SensorCombined,'fmu/out/sensor_combined',self.sensor_callback_2,qos_profile=qos_profile,callback_group=self.callback_group_2)

        self.timer_=self.create_timer(0.01, self.callback_1,callback_group=self.callback_group_3)
        self.timer_2=self.create_timer(0.01, self.callback_2,callback_group=self.callback_group_4)
        self.timer_3=self.create_timer(0.01, self.callback_3,callback_group=self.callback_group_5)
        self.timer_4=self.create_timer(0.01, self.callback_4,callback_group=self.callback_group_6)

        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)  

        self.x1=0.0
        self.x2=0.0
        self.x3=0.0
        self.x4=0.0
        self.x5=0.0
        self.x6=0.0
        self.x7=0.0
        self.x8=0.0
        self.x9=0.0
        self.x10=0.0
        self.x11=0.0
        self.x12=0.0

        self.pos_x=0.0
        self.pos_y=0.0
        self.pos_z=0.0 

        self.roll_rate=0.0
        self.pitch_rate=0.0
        self.yaw_rate=0.0

        self.vel_x=0.0
        self.vel_y=0.0
        self.vel_z=0.0

    def callback_1(self):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'base_link'
        tf_msg.child_frame_id = 'vehicle'
        tf_msg.transform.translation.x =float(self.pos_x)
        tf_msg.transform.translation.y =float(self.pos_y)
        tf_msg.transform.translation.z =float(self.pos_z)
        tf_msg.transform.rotation.x = 0.706825
        tf_msg.transform.rotation.y = 0.707388
        tf_msg.transform.rotation.z = 0.000563
        tf_msg.transform.rotation.w = 0.000563
        self.br.sendTransform(tf_msg)
    
    def callback_2(self):
            tf_msg = TransformStamped()
            tf_msg.header.stamp = self.get_clock().now().to_msg()
            tf_msg.header.frame_id = 'body_iris'
            tf_msg.child_frame_id = 'body_new'
            tf_msg.transform.translation.x =0.0
            tf_msg.transform.translation.y =0.0
            tf_msg.transform.translation.z =0.1
            tf_msg.transform.rotation.x = 1.0
            tf_msg.transform.rotation.y = 0.0
            tf_msg.transform.rotation.z = 0.0
            tf_msg.transform.rotation.w = 0.0
            self.br.sendTransform(tf_msg)   

    def callback_3(self):
        roll,pitch,yaw=0.0,0.0,0.0
        try:
            transform = self.tf_buffer.lookup_transform('base_link', 'body_new', rclpy.time.Time())
            roll,pitch,yaw=euler_from_quaternion([transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z,transform.transform.rotation.w])
            # print(roll,pitch,yaw)

        except Exception as e:
                self.get_logger().error(f"Error looking up or publishing transform: {str(e)}")
        self.x1=float(roll)
        self.x2=float(pitch)
        self.x3=float(yaw)
        # print(type(self.x10))
 
    def callback_4(self):  
        self.x4=float(self.roll_rate)
        self.x5=float(self.pitch_rate)
        self.x6=float(self.yaw_rate)
        self.x7=float(self.pos_x)
        self.x8=float(self.pos_y)
        self.x9=float(self.pos_z)
        self.x10=float(self.vel_x)
        self.x11=float(self.vel_y)
        self.x12=float(self.vel_z)     
        vector = Float32MultiArray()
        # vector.data=[self.x10]
        # vector.data = [ self.x4, self.x5, self.x6, self.x7, self.x8, self.x9, self.x10, self.x11, self.x12] 
        vector.data = [ self.x1,self.x2,self.x3,self.x4, self.x5, self.x6, self.x7, self.x8, self.x9, self.x10, self.x11, self.x12] 
        # print(vector)
        self.publisher_2.publish(vector)

    def sensor_callback_1(self,msg):
        # Handle the sensor data here
        self.vel_x=msg.velocity[1]
        self.vel_y=msg.velocity[0]
        self.vel_z=-msg.velocity[2]
        self.pos_x=msg.position[1]
        self.pos_y=msg.position[0]
        self.pos_z=-msg.position[2]
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'vehicle'
        tf_msg.child_frame_id = 'body_iris'
        tf_msg.transform.translation.x =0.0
        tf_msg.transform.translation.y =0.0
        tf_msg.transform.translation.z =0.0
        tf_msg.transform.rotation.w = float(msg.q[0])
        tf_msg.transform.rotation.x = float(msg.q[1])
        tf_msg.transform.rotation.y = float(msg.q[2])
        tf_msg.transform.rotation.z = float(msg.q[3])
        self.br.sendTransform(tf_msg)
    
    def sensor_callback_2(self,msg):
        self.roll_rate=msg.gyro_rad[0]
        self.pitch_rate=-msg.gyro_rad[1]
        self.yaw_rate=-msg.gyro_rad[2]
      


def main():

    rclpy.init(args=sys.argv)                                       # initialisation
    frames_publish_class = frames_publish()                                     # creating a new object for class 'frames_publish'
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(frames_publish_class)

    # Spin in background
    try:
        executor.spin()
    finally:
        # Ensure shutdown
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()