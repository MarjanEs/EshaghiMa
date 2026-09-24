import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from rclpy.node import Node
from amr_project.utils.occupancy_grid import OccupancyGridMap

from nav_msgs.msg import (
    OccupancyGrid,
    Odometry
)
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy
)
from geometry_msgs.msg import Pose

from geometry_msgs.msg import PoseArray

from .particle_filter import (
    ParticleFilter
)


class LocalizationNode(Node):

    def __init__(self):

        super().__init__(
            'localization_node'
        )
        self.grid = None
        self.map_msg = None
        self.map_received = False
        self.latest_scan = None

        self.filter = (
            ParticleFilter(
                num_particles=300
            )
        )
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.map_received = False

        self.prev_x = None
        self.prev_y = None
        self.prev_theta = None

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

        self.pose_array_pub = (
            self.create_publisher(
                PoseArray,
                '/particle_cloud',
                10
            )
        )

        self.timer = (
            self.create_timer(
                0.5,
                self.publish_particles
            )
        )

        self.get_logger().info(
            'Particle Filter started'
        )
        self.latest_scan = None

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
    def scan_callback(self, msg):

        self.latest_scan = msg

        if self.grid is None or not self.map_received:
            return

        self.filter.measurement_update(
            msg,
            self.grid
        )
    # def scan_callback(self, msg):

    #     self.latest_scan = msg

    #     if self.grid is None or not self.map_received:
    #         return

    #     # Test laser matching at the known reference pose
    #     true_error = self.filter.pose_scan_error(
            
    #         msg,
    #         self.grid
    #     )

    #     self.get_logger().info(
    #         f'TRUE POSE scan error = {true_error:.3f}'
    #     )

    #     self.filter.measurement_update(
    #         msg,
    #         self.grid
    #     )

    def map_callback(
        self,
        msg
    ):

    

        self.get_logger().info(
            'MAP CALLBACK CALLED'
        )

        if self.map_received:
            return

        self.map_msg = msg

        self.get_logger().info(
            'CREATING GRID'
        )

        self.grid = OccupancyGridMap(msg)

        self.get_logger().info(
            'GRID CREATED'
        )
        origin_x = (
            msg.info.origin.position.x
        )

        origin_y = (
            msg.info.origin.position.y
        )

        xmax = (
            origin_x
            + msg.info.width
            * msg.info.resolution
        )

        ymax = (
            origin_y
            + msg.info.height
            * msg.info.resolution
        )

        self.filter.initialize_particles(
            origin_x,
            xmax,
            origin_y,
            ymax,
            self.grid
        )

        self.map_received = True

        self.get_logger().info(
            'Particles initialized'
        )

    # def odom_callback(
    #     self,
    #     msg
    # ):
    #     if self.grid is None:
    #           return
        
    #     # Current odometry position
    #     x = msg.pose.pose.position.x
    #     y = msg.pose.pose.position.y

        # Current odometry orientation
        # q = msg.pose.pose.orientation
        
        # theta = math.atan2(
        #     2.0 * (q.w * q.z + q.x * q.y),
        #     1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        # )
        # # First odometry message
        # if self.prev_x is None:

        #     self.prev_x = x
        #     self.prev_y = y
        #     self.prev_theta = theta
        #     return
        
        # # Calculate movement,  Change in odometry
        # dx = x - self.prev_x
        # dy = y - self.prev_y
        # dtheta = theta - self.prev_theta

        # Keep angle between -pi and +pi,
        #Normalize angle to [-pi, pi]
        # dtheta = math.atan2(
        #     math.sin(dtheta),
        #     math.cos(dtheta)
        # )

        # # Update particles
        # self.filter.motion_update(
        #     dx,
        #     dy,
        #     dtheta
        # )
        # # Save current odometry for next callback
        # self.prev_x = x
        # self.prev_y = y
        # self.prev_theta = theta
    def odom_callback(self, msg):

        if self.grid is None:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        theta = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        # First odometry message
        if self.prev_x is None:
            self.prev_x = x
            self.prev_y = y
            self.prev_theta = theta
            return

        # Odometry movement in world/odom coordinates
        dx_world = x - self.prev_x
        dy_world = y - self.prev_y

        # Convert movement into the robot's local coordinates
        dx_local = (
            math.cos(self.prev_theta) * dx_world
            + math.sin(self.prev_theta) * dy_world
        )

        dy_local = (
            -math.sin(self.prev_theta) * dx_world
            + math.cos(self.prev_theta) * dy_world
        )

        dtheta = theta - self.prev_theta

        dtheta = math.atan2(
            math.sin(dtheta),
            math.cos(dtheta)
        )

        self.filter.motion_update(
            dx_local,
            dy_local,
            dtheta
        )

        self.prev_x = x
        self.prev_y = y
        self.prev_theta = theta
        

    def publish_particles(
        self
    ):

        pose_array = PoseArray()

        pose_array.header.frame_id = (
            'map'
        )

        for p in self.filter.particles:

            pose = Pose()

            pose.position.x = p.x
            pose.position.y = p.y

            pose.orientation.z = math.sin(p.theta / 2.0)
            pose.orientation.w = math.cos(p.theta / 2.0)

            pose_array.poses.append(
                pose
            )

        self.pose_array_pub.publish(
            pose_array
        )

        if len(self.filter.particles) == 0:
            return

        x, y, theta = self.filter.estimate_pose()

        self.get_logger().info(
            f'Estimated pose: '
            f'{x:.2f}, {y:.2f}'
        )


def main():

    rclpy.init()

    node = LocalizationNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()
