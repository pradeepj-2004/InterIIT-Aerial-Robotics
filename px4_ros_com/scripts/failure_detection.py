#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import SensorCombined  
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class failure(Node):
    def __init__(self):
        super().__init__('motor_failure_detection')

        # Taking the same qos_profile of MicroXRCEAgent
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(SensorCombined, '/fmu/out/sensor_combined',self.sensor_callback,qos_profile=qos_profile)
        
        # Initalising Variables
        self.roll_rate=0.0
        self.pitch_rate=0.0
        self.previous_roll=0.0
        self.previous_pitch=0.0
        self.i = 0

        #Creating timer
        self.timer_=self.create_timer(0.1,self.callback)

        self._logger.info("Motor Failure Detection Node started")



    def sensor_callback(self,msg):
        self.roll_rate=msg.gyro_rad[0]
        self.pitch_rate=msg.gyro_rad[1]

        if self.i==0:
            self.previous_roll=self.roll_rate
            self.previous_pitch=self.pitch_rate
            self._logger.info("IMU Data is receiving")
            self.i=1


    def callback(self):
          if self.i== 1:
            change_roll=self.roll_rate-self.previous_roll
            change_pitch=self.pitch_rate-self.previous_pitch

            self.previous_roll=self.roll_rate
            self.previous_pitch=self.pitch_rate



            if  (change_roll>0) and  (abs(change_roll)>1.0) and (abs(change_pitch)>0.3) and (change_pitch<0) :
                print("roll rate change:",change_roll)
                print("pitch rate change:",change_pitch)
                print('motor1 failed')
                self.i=2

            elif   change_roll<0 and  abs(change_roll)>1.0 and change_pitch<0 and  abs(change_pitch)>0.3 :
                print("roll rate change:",change_roll)
                print("pitch rate change:",change_pitch)
                print('motor3 failed')
                self.i=2    
            elif  change_roll>0 and  abs(change_roll)>1.0 and change_pitch>0 and  abs(change_pitch)>0.3 :
                print("roll rate change:",change_roll)
                print("pitch rate change:",change_pitch)
                print('motor4 failed')
                self.i=2    

            elif  change_roll<0 and  abs(change_roll)>1.0 and (abs(change_pitch)>0.3) and change_pitch>0 :
                print("roll rate change:",change_roll)
                print("pitch rate change:",change_pitch)
                print('motor2 failed')
                self.i=2  


def main(args=None):
    rclpy.init(args=args)
    node = failure()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__' :
    main()



