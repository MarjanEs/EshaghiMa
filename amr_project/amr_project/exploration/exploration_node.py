import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import (
    OccupancyGrid,
    Odometry
)

from geometry_msgs.msg import (
    PoseStamped
)

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy
)

from .frontier_explorer import (
    FrontierExplorer
)


class ExplorationNode(Node):

    def __init__(self):

        super().__init__(
            'exploration_node'
        )

        self.explorer = (
            FrontierExplorer()
        )

        self.map_msg = None

        self.robot_x = None
        self.robot_y = None

        self.last_goal = None

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # For now this publishes the selected
        # exploration pose.
        self.goal_pub = (
            self.create_publisher(
                PoseStamped,
                '/exploration_goal',
                10
            )
        )

        # Look for a new exploration goal
        # every 2 seconds.
        self.timer = (
            self.create_timer(
                2.0,
                self.explore
            )
        )

        self.get_logger().info(
            'Frontier exploration started'
        )


    def map_callback(
        self,
        msg
    ):

        # This map comes from SLAM.
        self.map_msg = msg


    def odom_callback(
        self,
        msg
    ):

        self.robot_x = (
            msg.pose.pose.position.x
        )

        self.robot_y = (
            msg.pose.pose.position.y
        )


    def explore(self):

        if self.map_msg is None:
            return

        if self.robot_x is None:
            return

        goal = (
            self.explorer.select_frontier(
                self.map_msg,
                self.robot_x,
                self.robot_y
            )
        )

        if goal is None:

            self.get_logger().info(
                'No frontiers found.'
            )

            return

        goal_x, goal_y = goal

        # Do not repeatedly publish almost
        # exactly the same goal.
        # if self.last_goal is not None:

        #     distance = math.hypot(
        #         goal_x - self.last_goal[0],
        #         goal_y - self.last_goal[1]
        #     )

        #     if distance < 0.30:
        #         return

        goal_msg = PoseStamped()

        goal_msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        goal_msg.header.frame_id = 'map'

        goal_msg.pose.position.x = (
            goal_x
        )

        goal_msg.pose.position.y = (
            goal_y
        )

        goal_msg.pose.orientation.w = 1.0

        self.goal_pub.publish(
            goal_msg
        )

        self.last_goal = (
            goal_x,
            goal_y
        )

        self.get_logger().info(
            f'New frontier goal: '
            f'{goal_x:.2f}, '
            f'{goal_y:.2f}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = ExplorationNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()