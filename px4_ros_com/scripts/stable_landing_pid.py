#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, VehicleCommand,VehicleOdometry,TrajectorySetpoint,ActuatorMotors, SensorCombined
from tf_transformations import euler_from_quaternion
import time
import math
import tf2_ros
from std_msgs.msg import Float32MultiArray


"""When the running the code ,drone will takeoff automatically and hold
at particular altitude till it detect the motor failure. The control logic is currently implemented for 1st motor failure """

class StableLanding(Node):
    """Node for controlling a motor."""

    def __init__(self) -> None:
        super().__init__('actuator_control_node')

        #Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.get_logger().info('Node created:')


        #Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.actuator_publisher = self.create_publisher(ActuatorMotors, '/fmu/in/actuator_motors', qos_profile)

        self.subscription = self.create_subscription(SensorCombined, '/fmu/out/sensor_combined',self.sensor_callback,qos_profile=qos_profile)
        self.subscription_2 = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry',self.sensor_callback_2,qos_profile=qos_profile)
        self.subscription_3 = self.create_subscription(Float32MultiArray, '/controller_data',self.controller_callback,10)

        self.timer = self.create_timer(0.01, self.MotorControlpid)

        self.x1=0.0
        self.x2=0.0
        self.x9=0.0
        self.x12=0.0
        
        #Flags for motor failure detection
        self.roll_rate=0.0
        self.pitch_rate=0.0
        self.previous_roll=0.0
        self.previous_pitch=0.0
        self.i1 = 10

        #Flags for roll PID
        self.roll_error = 0.0
        self.error_r_i = 0.0
        self.error_r_d=0.0
        self.roll_error_prev=0.0

        self.k=0 #Flag value for counting number of iterations roll is stable
        self.p=0 #Flag value for counting number of iterations pitch is stable

        #Flags for pitch PID
        self.pitch_error = 0.0
        self.error_p_i=0.0
        self.error_p_d=0.0
        self.pitch_error_prev = 0.0

        # Flags for vel PID
        self.vel_error=0.0
        self.error_a_i=0.0
        self.error_a_d=0.0
        self.vel_error_prev=0.0

        
        #Initial PID Gains

        self.kpr = 1.7
        self.kdr = 0.37
        self.kir = 0.01

        self.kpp = 0.7
        self.kdp = 0.37 
        self.kip = 0.02

        self.kpa=0.2
        self.kda=0.11
        self.kia=0.0

        self.thrust= 10.0
        self.t_roll=0.0
        self.t_pitch=0.0

        self.takeoff_height = -50.0
        self.get_logger().info(f"Takeoff position: {-self.takeoff_height} m")
       
    def sensor_callback_2(self,msg):

        #Function to gather vehicle odometry data
        self.yaw,self.pitch,self.roll=euler_from_quaternion([msg.q[0],msg.q[1],msg.q[2],msg.q[3]])
        self.pitch= -self.pitch
        self.zpos=msg.position[2]
        self.zpos=-self.zpos

    def controller_callback(self,msg):
        self.x1=msg.data[0]
        self.x2=msg.data[1]
        self.x9=msg.data[8]
        self.x12=msg.data[11]
        # print(self.x1,self.x2)


    def sensor_callback(self,msg):

        #Function to detect Motor Failure
        self.roll_rate=msg.gyro_rad[0]
        self.pitch_rate=msg.gyro_rad[1]
        self.yaw_rate = msg.gyro_rad[2]
        self.vertical_acceleration = msg.accelerometer_m_s2[2]
        
        if self.i1== 10:
            error_r=self.roll_rate-self.previous_roll
            error_p=self.pitch_rate-self.previous_pitch
            self.previous_roll=self.roll_rate
            self.previous_pitch=self.pitch_rate

            #Conditions for failure of different rotors
            if  (error_r>0) and  (abs(error_r)>0.1) and (abs(error_p)>0.06) and (error_p<0) :
                self._logger.info('motor1 failed')
                self.i1=1

            elif   error_r<0 and  abs(error_r)>0.15 and error_p<0 and  abs(error_p)>0.1 :
                self._logger.info('motor3 failed')
                self.i1=3   

            elif  error_r>0 and  abs(error_r)>0.1 and error_p>0 and  abs(error_p)>0.1 :
                self._logger.info('motor4 failed')
                self.i1=4    

            elif  error_r<0 and  abs(error_r)>0.0975 and (abs(error_p)>0.1) and error_p>0 :
                self._logger.info('motor2 failed')
                self.i1=2  

    def motor_control(self,w1,w2,w3,w4):
        #Controls indiviudal speed of motor
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
        #Function to map negative total thrust values to 0
        if x>=0:
            y = x
        else:
            y = 0
        return y
    
    def relu_2(self,x):
        #Function to map individual motor thrust values between 0 and 1
        if x>=0:
            y = x
            if (x>1):
                y=1
        else:
            y = 0
        return y

    
    def MotorControlpid(self):
        self.arm()
        
        #To Start takeoff of drone initally for testing
        if(self.i1!=10):
            self.publish_offboard_control_heartbeat_signal(False,True)
            self.engage_offboard_mode()

        #To Start offboard control after motor failure starts
        if(self.i1==10):
            self.publish_offboard_control_heartbeat_signal(True,False)
            self.engage_offboard_mode()
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)


        #Implementation of control algorithm for motor 1 failure
        match self.i1:
            case 1:
                
                #Calculating PID output roll_error
                #########################################################
                self.roll_error = (0.0 - self.x1)
                self.error_r_i=self.error_r_i+self.roll_error
                self.error_r_d = self.roll_error-self.roll_error_prev
                self.roll_error_prev = self.roll_error

                #########################################################

                #Calculating PID pitch_error
                self.pitch_error = (0.0 - self.x2)
                self.error_p_i=self.error_p_i+self.pitch_error
                self.error_p_d = self.pitch_error - self.pitch_error_prev
                self.pitch_error_prev = self.pitch_error

                ##########################################################



                # print(self.e_v)
                # #Condition to increase the thrust as the roll and pitch error setlles 
                # if (self.x1<0.2 and self.x1>-0.2 and self.x2<0.2 and self.x2>-0.2 ):
                #     self.k=self.k+1     
                # if (self.k>=20):

                        #Calculating PID Velocity Error
                self.x9=(self.x9/19)

                self.vel_error= 0.0 -self.x9
                self.error_a_i=self.error_a_i+self.vel_error
                self.error_a_d=self.vel_error-self.vel_error_prev
                self.vel_error_prev=self.vel_error

                self.e_v=self.kpa*(self.vel_error)+self.kda*(self.error_a_d/0.01)# + self.kia*(self.error_a_i)
                self.thrust= self.thrust-self.e_v

                if self.thrust>16:
                    self.thrust=16
                elif self.thrust<12:
                    self.thrust=11.0

                if self.x1>0.3 or self.x1<-0.3:
                    self.thrust=10

                self.t_roll = self.kpr *( self.roll_error) + self.kdr * (self.error_r_d/0.01) + self.kir*(self.error_r_i)
                self.t_pitch = self.kpp * (self.pitch_error) + self.kdp * (self.error_p_d/0.01) + self.kip*(self.error_p_i)
                # print(self.thrust,self.e_v)

                b=8.0*10**(-6)  #Thrust Coefficient

                # self.t_yaw  = -29.76 * self.thrust + 135.29 *self.t_roll  -233.34 *self.t_pitch

                # self.t2 = self.relu((0.25 *self.thrust + 1.1364*self.t_roll - 1.9608*self.t_pitch + 0.0084*self.t_yaw))
                # self.t3 = self.relu((0.25 *self.thrust + 1.1364*self.t_roll + 1.9608*self.t_pitch - 0.0084*self.t_yaw))
                # self.t4 = self.relu((0.25 *self.thrust - 1.1364*self.t_roll - 1.9608*self.t_pitch - 0.0084*self.t_yaw))

                self.t_yaw  = ( -1.1364 *self.t_roll - 0.5*self.t_pitch + 0.25 * self.thrust) /1.6
                self.t2 = self.relu((1.1364*self.t_roll + 0.5*self.t_pitch - 1.6*self.t_yaw + 0.25 *self.thrust ))
                self.t3 = self.relu((1.1364*self.t_roll - 0.5*self.t_pitch + 1.6*self.t_yaw + 0.25 *self.thrust ))
                self.t4 = self.relu((-1.1364*self.t_roll+ 0.5*self.t_pitch + 1.6*self.t_yaw + 0.25 *self.thrust))

                self.t2 = self.relu_2((1/1000)*(math.sqrt(self.t2/b)-100))
                self.t3 = self.relu_2((1/1000)*(math.sqrt(self.t3/b)-100))
                self.t4 = self.relu_2((1/1000)*(math.sqrt(self.t4/b)-100))
                

                if (abs(self.zpos)>1.0):
                    #Providing PID output to the individual motors
                    print("t2: {:.2f}  t3: {:.2f}  t4: {:.2f}  t_roll: {:.2f}  t_pitch: {:.2f} Thrust {:.2f} Roll: {:.2f} Pitch: {:.2f} Position {:.2f} Velocity{:.2f}".format(
                    self.t2, self.t3, self.t4, self.t_roll, self.t_pitch,self.thrust,self.x1,self.x2,self.x9,self.x12))    
                    self.motor_control(0.0 , self.t2 , self.t3 , self.t4)
                else:
                    #To stop all the rotors after successful landing
                    self._logger.info("Landed successfully")
                    self.i1=15
                    self.motor_control(0.0,0.0,0.0,0.0)
               
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
        #Function for publishing location of the drone
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079  
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)


def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    actuator_control = StableLanding()
    rclpy.spin(actuator_control)
    actuator_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        #Main function
        main()
    except Exception as e:
        print(e)
