#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, VehicleCommand,VehicleOdometry,TrajectorySetpoint,ActuatorMotors, SensorCombined
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Float32MultiArray
import time
import math

class ActuatorControl(Node):
    """Node for controlling a motor."""

    def __init__(self) -> None:
        super().__init__('actuator_control_node')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.get_logger().info('Node created:')


        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.actuator_publisher = self.create_publisher(ActuatorMotors, '/fmu/in/actuator_motors', qos_profile)
        self.subscription = self.create_subscription(SensorCombined, '/fmu/out/sensor_combined',self.sensor_callback,qos_profile=qos_profile)
        self.subscription_2 = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry',self.sensor_callback_2,qos_profile=qos_profile)
        self.subscription_3 = self.create_subscription(Float32MultiArray,'/controller_data',self.sensor_data,10)
        self.timer = self.create_timer(0.01, self.MotorControlpid)

        #Flags for motor failure detection
        self.roll_rate=0.0
        self.pitch_rate=0.0
        self.previous_roll=0.0
        self.previous_pitch=0.0
        self.i1 = 10

        self.x1= 0.0
        self.x2= 0.0
        self.x3= 0.0
        self.x4= 0.0
        self.x5= 0.0
        self.x6= 0.0
        self.x7= 0.0
        self.x8= 0.0
        self.x9= 0.0
        self.x10= 0.0
        self.x11= 0.0
        self.x12= 0.0

        self.thrust = 0.0
        self.t_roll = 0.0
        self.t_pitch= 0.0
        self.takeoff_height = -15.0
        self.get_logger().info(f"Takeoff position: {-self.takeoff_height} m")

    def sensor_data(self,msg):
        self.x1=msg.data[0]
        self.x2=msg.data[1]
        self.x3=msg.data[2]
        self.x4=msg.data[3]
        self.x5=msg.data[4]
        self.x6=msg.data[5]
        self.x7=msg.data[6]
        self.x8=msg.data[7]
        self.x9=msg.data[8]
        self.x10=msg.data[9]
        self.x11=msg.data[10]
        self.x12=msg.data[11]
    
        
    def sensor_callback_2(self,msg):
        self.yaw,self.pitch,self.roll=euler_from_quaternion([msg.q[0],msg.q[1],msg.q[2],msg.q[3]])

    def sensor_callback(self,msg):

        self.roll_rate=msg.gyro_rad[0]
        self.pitch_rate=msg.gyro_rad[1]
        self.yaw_rate = msg.gyro_rad[2]
        self.vertical_acceleration = msg.accelerometer_m_s2[2]
        
        if self.i1== 10:
            error_r=self.roll_rate-self.previous_roll
            error_p=self.pitch_rate-self.previous_pitch
            self.previous_roll=self.roll_rate
            self.previous_pitch=self.pitch_rate

            if  (error_r>0) and  (abs(error_r)>0.1) and (abs(error_p)>0.06) and (error_p<0) :
                print('motor1 failed')
                self.i1=1

            elif   error_r<0 and  abs(error_r)>0.15 and error_p<0 and  abs(error_p)>0.1 :
                print('motor3 failed')
                self.i1=3   

            elif  error_r>0 and  abs(error_r)>0.1 and error_p>0 and  abs(error_p)>0.1 :
                print('motor4 failed')
                self.i1=4    

            elif  error_r<0 and  abs(error_r)>0.0975 and (abs(error_p)>0.1) and error_p>0 :
                print('motor2 failed')
                self.i1=2  

    def motor_control(self,w1,w2,w3,w4):
        msg = ActuatorMotors()
        '''Motor thrust control for motor 1'''
        msg.control = [0.0] * 12  
        msg.reversible_flags = 0 
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.timestamp_sample = msg.timestamp
        msg.reversible_flags = 0000
        msg.control[0] = w1 
        msg.control[1] = w2 
        msg.control[2] = w3
        msg.control[3] = w4

        self.actuator_publisher.publish(msg)


    def relu(self,x):
        if x>=0:
            y = x
        else:
            y = 0
        return y
    def relu_2(self,x):
        if x>=0:
            y = x
            if (x>1):
                y=1
        else:
            y = 0
        return y

    def relu_3(self,x):
        if x>=0.6:
            y = x
            if (x>1):
                y=0.6
        else:
            y = 0.6
        return y
    
    def MotorControlpid(self):
        self.arm()
        
        if(self.i1!=10):
            self.publish_offboard_control_heartbeat_signal(False,True)
            self.engage_offboard_mode()

        if(self.i1==10):
            self.publish_offboard_control_heartbeat_signal(True,False)
            self.engage_offboard_mode()
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)


        match self.i1:
            case 1:
            
                kr= 0.23
                eta= 1.5
                wn=0.005
                height_d = 10

                self.t_roll =  (151351*(math.tan(self.x2)*math.cos(self.x1)**2 + math.tan(self.x2)*math.sin(self.x1)**2)*(- self.x12*wn**2 + 2*eta*(height_d - self.x9)*wn + 49/5))/(40*(11339*math.cos(self.x1)*math.cos(self.x2)*math.sin(self.x1) - 27500*math.cos(self.x1)**2*math.cos(self.x2) + 19720*math.cos(self.x1)**3*math.cos(self.x2)*math.tan(self.x2) + 19720*math.cos(self.x1)*math.cos(self.x2)*math.sin(self.x1)**2*math.tan(self.x2))) - (29*(27500*math.cos(self.x1) - 11339*math.sin(self.x1))*((1000*kr*self.x4)/29 + (26*self.x5*self.x6)/29 - wn**2*self.x4 + math.sin(self.x1)*math.tan(self.x2)*((1000*kr*self.x5)/29 - (26*self.x4*self.x6)/29) - ((self.x6*math.cos(self.x1) + self.x5*math.sin(self.x1))*(self.x5*math.cos(self.x1) - self.x6*math.sin(self.x1)))/math.cos(self.x2)**2 - math.tan(self.x2)*(self.x5*math.cos(self.x1) - self.x6*math.sin(self.x1))*(self.x4 + self.x6*math.cos(self.x1)*math.tan(self.x2) + self.x5*math.sin(self.x1)*math.tan(self.x2)) - 2*eta*wn*self.x1 + (200*kr*self.x6*math.cos(self.x1)*math.tan(self.x2))/11))/(1000*(11339*math.sin(self.x1) - 27500*math.cos(self.x1) + 19720*math.cos(self.x1)**2*math.tan(self.x2) + 19720*math.sin(self.x1)**2*math.tan(self.x2))) - (29*(11339*math.cos(self.x1)*math.tan(self.x2) + 27500*math.sin(self.x1)*math.tan(self.x2))*(self.x5*wn**2 + 2*eta*self.x2*wn - math.cos(self.x1)*((1000*kr*self.x5)/29 - (26*self.x4*self.x6)/29) - (self.x6*math.cos(self.x1) + self.x5*math.sin(self.x1))*(self.x4 + self.x6*math.cos(self.x1)*math.tan(self.x2) + self.x5*math.sin(self.x1)*math.tan(self.x2)) + (200*kr*self.x6*math.sin(self.x1))/11))/(1000*(11339*math.sin(self.x1) - 27500*math.cos(self.x1) + 19720*math.cos(self.x1)**2*math.tan(self.x2) + 19720*math.sin(self.x1)**2*math.tan(self.x2)))
                self.t_pitch = (14297*math.sin(self.x1)*((1000*kr*self.x4)/29 + (26*self.x5*self.x6)/29 - wn**2*self.x4 + math.sin(self.x1)*math.tan(self.x2)*((1000*kr*self.x5)/29 - (26*self.x4*self.x6)/29) - ((self.x6*math.cos(self.x1) + self.x5*math.sin(self.x1))*(self.x5*math.cos(self.x1) - self.x6*math.sin(self.x1)))/math.cos(self.x2)**2 - math.tan(self.x2)*(self.x5*math.cos(self.x1) - self.x6*math.sin(self.x1))*(self.x4 + self.x6*math.cos(self.x1)*math.tan(self.x2) + self.x5*math.sin(self.x1)*math.tan(self.x2)) - 2*eta*wn*self.x1 + (200*kr*self.x6*math.cos(self.x1)*math.tan(self.x2))/11))/(25*(11339*math.sin(self.x1) - 27500*math.cos(self.x1) + 19720*math.cos(self.x1)**2*math.tan(self.x2) + 19720*math.sin(self.x1)**2*math.tan(self.x2))) - (29*(986*math.cos(self.x1)*math.tan(self.x2) - 1375)*(self.x5*wn**2 + 2*eta*self.x2*wn - math.cos(self.x1)*((1000*kr*self.x5)/29 - (26*self.x4*self.x6)/29) - (self.x6*math.cos(self.x1) + self.x5*math.sin(self.x1))*(self.x4 + self.x6*math.cos(self.x1)*math.tan(self.x2) + self.x5*math.sin(self.x1)*math.tan(self.x2)) + (200*kr*self.x6*math.sin(self.x1))/11))/(50*(11339*math.sin(self.x1) - 27500*math.cos(self.x1) + 19720*math.cos(self.x1)**2*math.tan(self.x2) + 19720*math.sin(self.x1)**2*math.tan(self.x2))) - (151351*math.sin(self.x1)*(- self.x12*wn**2 + 2*eta*(height_d - self.x9)*wn + 49/5))/(40*(11339*math.cos(self.x1)*math.cos(self.x2)*math.sin(self.x1) - 27500*math.cos(self.x1)**2*math.cos(self.x2) + 19720*math.cos(self.x1)**3*math.cos(self.x2)*math.tan(self.x2) + 19720*math.cos(self.x1)*math.cos(self.x2)*math.sin(self.x1)**2*math.tan(self.x2)))
                self.thrust =  (307*(- self.x12*wn**2 + 2*eta*(height_d - self.x9)*wn + 49/5))/(200*math.cos(self.x1)*math.cos(self.x2)) 

                if self.thrust>14.0:
                    self.thrust=self.thrust 
                elif self.thrust<0.0 or self.x9<1.0:
                    self.thrust=0.0

                b=5.84*10**(-6)
                self.t_yaw  = 0.17 * self.thrust - 1.33 *self.t_roll  + 0.782*self.t_pitch

                self.t2 = self.relu((0.25 *self.thrust + 1.9608*self.t_roll - 1.129*self.t_pitch - 1.470*self.t_yaw))
                self.t3 = self.relu((0.25 *self.thrust - 1.9608*self.t_roll - 1.129*self.t_pitch + 1.470*self.t_yaw))
                self.t4 = self.relu((0.25 *self.thrust + 1.9608*self.t_roll + 1.129*self.t_pitch + 1.470*self.t_yaw))

                self.t2 = (1/1000)*(math.sqrt(self.t2/b)-100)
                self.t3 = (1/1000)*(math.sqrt(self.t3/b)-100)
                self.t4 = (1/1000)*(math.sqrt(self.t4/b)-100)
                
                self.t2 = self.relu_2(self.t2)
                self.t3 = self.relu_2(self.t3)
                self.t4 = self.relu_2(self.t4)

                print("t2: {:.2f}  t3: {:.2f}  t4: {:.2f}  t_roll: {:.2f}  t_pitch: {:.2f} Roll: {:.6f} Pitch: {:.6f} Thrust: {:.2f}  Body_accn: {:.2f} Position {:.2f}".format(
                    self.t2, self.t3, self.t4, self.t_roll, self.t_pitch,self.x1,self.x2, self.thrust,self.acceleration_error,self.x9))
                # print(self.x1, self.x2, self.x3,self.x4,self.x5,self.x6,self.x7,self.x8, self.x9,self.x10, self.x11, self.x12)
                self.motor_control(0.0 , self.t2 , self.t3 , self.t4)

            case 2:
                
                self.motor_control(1.0,0.0,0.0,1.0)
            case 3:
                
                self.motor_control(1.0,0.0,0.0,1.0)
            case 4:
                
                self.motor_control(0.0,1.0,1.0,0.0)

    def publish_offboard_control_heartbeat_signal(self,a,b):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = a
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.direct_actuator = b
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)
    
    def publish_position_setpoint(self, x: float, y: float, z: float):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        # self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")


    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)


def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    actuator_control = ActuatorControl()
    rclpy.spin(actuator_control)
    actuator_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
