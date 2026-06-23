#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading

from geometry_msgs.msg import Point

class MissionPlanner(Node):
    def __init__(self):
        super().__init__('Mission_Planner')

        self.missionPub = self.create_publisher(Point, '/audibot/customwaypoints', 10)

        self.input_thread = threading.Thread(target=self.missionPlan, daemon=True)
        self.input_thread.start()

    def missionPlan(self):
        while(rclpy.ok()):
            try:
                targ_x = input("X coordinate: ")
                targ_y = input("Y coordinate: ")

                coordinate = Point()
                coordinate.x = float(targ_x)
                coordinate.y = float(targ_y)
                coordinate.z = 0.0

                self.missionPub.publish(coordinate)
                self.get_logger().info(f'Command transmitted: X={coordinate.x}, Y={coordinate.y}')


            except ValueError:
                self.get_logger().error("Invalid input! Please enter numbers only.")
            except (EOFError, KeyboardInterrupt):
                break

    
def main(args=None):
    rclpy.init(args=args)
    node = MissionPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()