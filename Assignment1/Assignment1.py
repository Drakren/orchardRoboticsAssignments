#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class AudibotStraightLine(Node):
    def __init__(self):
        super().__init__('audibot_straight_line')
        
       #publishers
        self.speed_pub = self.create_publisher(Float64, '/audibot/speed_cmd', 10)
        
        # 1 Hz timer (1s interval) to beat the audibot timeout
        self.timer_period = 1  
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.get_logger().info('Audibot straight line node initialized. Publishing at 1Hz...')

    def timer_callback(self):
        speed_msg = Float64()
        speed_msg.data = 1.0 
        
        self.speed_pub.publish(speed_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AudibotStraightLine()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down program')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()