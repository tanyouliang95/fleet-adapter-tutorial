# Simple testing pub node to pub a nav goal to mir_fleet_manager

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import _thread
import time


class nav_goal_publisher(Node):

    def __init__(self):
        self.node_name = "nav_goal_publisher_ros2"
        super().__init__(self.node_name)
        self.nav_pub = self.create_publisher(PoseStamped, '/mir_fleet_manager/waypoint_goal')
        _thread.start_new_thread(self.MockPublisher, tuple())

    def pub_goal(self):
        self.get_logger().info(' Publish random goal!!!')

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = 17.0
        pose_stamped.pose.position.y = 10.0
        pose_stamped.pose.position.z = 0.0
        pose_stamped.pose.orientation.w = 1.0

        self.nav_pub.publish(pose_stamped)

    def MockPublisher(self):
        self.get_logger().info("Running pub thread.....")
        time.sleep(1)
        while(1):
            input("## Enter to Pub ->") 
            self.pub_goal()
            time.sleep(1)


def main(args=None):
    rclpy.init(args=args)
    node = nav_goal_publisher()
    node.get_logger().info('Mock Pub is Running....')

    rclpy.spin(node)
    node.destroy_node()
    node.get_logger().info('Byebye =)')
    rclpy.shutdown()

if __name__ == '__main__':
    main()
