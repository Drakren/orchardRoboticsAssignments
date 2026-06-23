#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import Point
from std_msgs.msg import UInt32, Float64
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class AudibotWaypointTraversal(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')

        #Publishers
        self.speed = self.create_publisher(Float64, '/audibot/speed_cmd', 10) 
        self.gear = self.create_publisher(UInt32, '/audibot/gear_cmd',10)
        self.steer = self.create_publisher(Float64, '/audibot/steering_cmd', 10)

        #TF listener
        self.tf_buffer = Buffer()
        self.tf_listen = TransformListener(self.tf_buffer, self)

        self.target_x = None
        self.target_y = None

        # self.waypoints = [(20.0, 10.0),
        #                   (40.0, -10.0),
        #                   (-10.0, 20.0)]
        
        # self.currentWaypoint = 0
        # self.missionCompleted = False

        #Subscriber
        self.missionPointSub = self.create_subscription(Point, '/audibot/customwaypoints', self.coordinate, 10)

        #PID constants
        self.kp = 17.8
        self.ki = 0.0
        self.kd = 0.1
        self.prev_error = 0.0
        self.integral = 0.0

        #timer
        self.timer = 0.1 #publishing data at 10Hz
        self.timercallback = self.create_timer(self.timer, self.controller)
        self.get_logger().info('Node started.')

    def coordinate(self,msg):
        self.target_x = msg.x
        self.target_y = msg.y
        self.get_logger().info(f"Data received X: {msg.x} and Y: {msg.y}")


    def eulerfromquat(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    

    def normalizeAngle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    

    #controller
    def controller(self):
        gearmsg = UInt32()
        gearmsg.data = 0

        speedmsg = Float64()
        # speedmsg.data = 6.0

        steeringmsg = Float64()
        steeringmsg.data = 0.0

        if self.target_x is None or self.target_y is None:
            speedmsg.data = 0.0
            self.gear.publish(gearmsg)
            self.speed.publish(speedmsg)
            self.steer.publish(steeringmsg)
            return

        # if self.missionCompleted:
        #     speedmsg.data =0.0
        #     self.gear.publish(gearmsg)
        #     self.steer.publish(steeringmsg)
        #     self.speed.publish(speedmsg)
        #     return
        
        #getting state from TF

        try:
            t = self.tf_buffer.lookup_transform('world','base_footprint',rclpy.time.Time())
            curr_x = t.transform.translation.x
            curr_y = t.transform.translation.y

            curr_yaw = self.eulerfromquat(t.transform.rotation)

        except TransformException:
            return
        
        dist = math.sqrt((curr_x - self.target_x)**2 + (curr_y - self.target_y)**2)

        if dist < 0.5:
            self.get_logger().info(f'Mission Complete!')
            self.target_x = None
            self.target_y = None
            return
            

        targ_yaw = math.atan2(float(self.target_y) - curr_y, float(self.target_x) - curr_x)

        error = self.normalizeAngle(targ_yaw-curr_yaw)

        self.integral += self.timer*self.integral
        derivative = (error - self.prev_error)/self.timer

        correction = (self.kp*error + self.kd*derivative + self.ki*self.integral)
        self.prev_error = error

        correction = max(min(correction, 10.68), -10.68)

        steeringmsg.data = correction
        speedmsg.data = 6.0

        self.gear.publish(gearmsg)
        self.steer.publish(steeringmsg)
        self.speed.publish(speedmsg)


def main(args=None):
    rclpy.init(args=args)
    node = AudibotWaypointTraversal()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
