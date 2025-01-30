#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class MotorfailInjection(Node):

    def __init__(self):
        super().__init__('motor_fail_injection')
        self.publisher_ = self.create_publisher(Int32, '/motor_failure/motor_number', 10)
        self.declare_parameter('motor_number', 0)
        motor_number=self.get_parameter('motor_number').get_parameter_value().integer_value
        msg = Int32()
        msg.data = motor_number
        self._logger.info(f"Motor {motor_number} failed")
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    motor_fail = MotorfailInjection()
    
    try:
        rclpy.spin(motor_fail)
    except KeyboardInterrupt:
        pass
    motor_fail.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
