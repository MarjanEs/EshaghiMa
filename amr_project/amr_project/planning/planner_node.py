import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

from amr_project.planning.astar import AStarPlanner
from amr_project.planning.potential_field import PotentialFieldPlanner
from amr_project.planning.waypoint_follower import WaypointFollower
from amr_project.utils.occupancy_grid import OccupancyGridMap

from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import DurabilityPolicy
from tf2_ros import Buffer, TransformListener, TransformException


class PlannerNode(Node):

    def __init__(self):

        super().__init__('planner_node')
        self.have_goal = False
        self.path_active = False
        self.goal_x = 0.0
        self.goal_y = 0.0

        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
        self.tf_buffer,
        self
        )

        # =========================================================
        # Parameters
        # =========================================================

        self.declare_parameter(
            'map_topic',
            '/map'
        )

        self.declare_parameter(
            'odom_topic',
            '/odom'
        )

        self.declare_parameter(
            'scan_topic',
            '/scan'
        )

        self.declare_parameter(
            'goal_topic',
            '/goal_pose'
        )

        self.declare_parameter(
            'cmd_vel_topic',
            '/cmd_vel'
        )

        self.declare_parameter(
            'global_path_topic',
            '/global_path'
        )

        self.declare_parameter(
            'waypoints_topic',
            '/waypoints'
        )

        self.declare_parameter(
            'goal_tolerance',
            0.25
        )

        self.declare_parameter(
            'waypoint_tolerance',
            0.30
        )

        self.declare_parameter(
            'attractive_gain',
            1.5
        )

        self.declare_parameter(
            'repulsive_gain',
            0.35
        )

        self.declare_parameter(
            'obstacle_distance_threshold',
            0.60
        )

        self.declare_parameter(
            'max_linear_velocity',
            0.30
        )

        self.declare_parameter(
            'max_angular_velocity',
            1.0
        )

        self.declare_parameter(
            'min_linear_velocity',
            0.05
        )

        self.declare_parameter(
            'control_frequency',
            10.0
        )

        self.declare_parameter(
            'obstacle_inflation_radius',
            0.20
        )

        # =========================================================
        # Read parameters
        # =========================================================

        self.map_topic = self.get_parameter(
            'map_topic'
        ).value

        self.odom_topic = self.get_parameter(
            'odom_topic'
        ).value

        self.scan_topic = self.get_parameter(
            'scan_topic'
        ).value

        self.goal_topic = self.get_parameter(
            'goal_topic'
        ).value

        self.cmd_vel_topic = self.get_parameter(
            'cmd_vel_topic'
        ).value

        self.global_path_topic = self.get_parameter(
            'global_path_topic'
        ).value

        self.waypoints_topic = self.get_parameter(
            'waypoints_topic'
        ).value

        self.goal_tolerance = self.get_parameter(
            'goal_tolerance'
        ).value

        self.waypoint_tolerance = self.get_parameter(
            'waypoint_tolerance'
        ).value

        attractive_gain = self.get_parameter(
            'attractive_gain'
        ).value

        repulsive_gain = self.get_parameter(
            'repulsive_gain'
        ).value

        obstacle_distance_threshold = (
            self.get_parameter(
                'obstacle_distance_threshold'
            ).value
        )

        self.max_linear_velocity = (
            self.get_parameter(
                'max_linear_velocity'
            ).value
        )

        self.max_angular_velocity = (
            self.get_parameter(
                'max_angular_velocity'
            ).value
        )

        self.min_linear_velocity = (
            self.get_parameter(
                'min_linear_velocity'
            ).value
        )

        control_frequency = self.get_parameter(
            'control_frequency'
        ).value

        self.obstacle_inflation_radius = (
            self.get_parameter(
                'obstacle_inflation_radius'
            ).value
        )

        # =========================================================
        # Planners
        # =========================================================

        self.astar = AStarPlanner()

        self.potential_field = PotentialFieldPlanner(
            attractive_gain=attractive_gain,
            repulsive_gain=repulsive_gain,
            obstacle_distance_threshold=(
                obstacle_distance_threshold
            )
        )

        self.waypoint_follower = WaypointFollower(
            waypoint_tolerance=self.waypoint_tolerance
        )

        # =========================================================
        # Robot state
        # =========================================================

        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None

        # =========================================================
        # Goal
        # =========================================================

        self.goal_x = None
        self.goal_y = None
        self.goal_frame = None

        self.have_goal = False

        # =========================================================
        # Map
        # =========================================================

        self.map_msg = None
        self.grid = None

        # =========================================================
        # Laser scan
        # =========================================================

        self.scan = None

        # =========================================================
        # Global path
        # =========================================================

        self.global_path = []

        # =========================================================
        # Subscribers
        # =========================================================

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            map_qos
        )

        self.odom_subscriber = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )

        self.goal_subscriber = self.create_subscription(
            PoseStamped,
            self.goal_topic,
            self.goal_callback,
            10
        )

        # =========================================================
        # Publishers
        # =========================================================

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )

        self.global_path_publisher = self.create_publisher(
            Path,
            self.global_path_topic,
            10
        )

        self.waypoints_publisher = self.create_publisher(
            MarkerArray,
            self.waypoints_topic,
            10
        )

        # =========================================================
        # Control timer
        # =========================================================

        timer_period = 1.0 / control_frequency

        self.timer = self.create_timer(
            timer_period,
            self.control_loop
        )

        # =========================================================
        # Startup message
        # =========================================================

        self.get_logger().info(
            'A* + Potential Field Planner started.'
        )

        self.get_logger().info(
            f'Waiting for map on {self.map_topic}'
        )

    # =============================================================
    # MAP CALLBACK
    # =============================================================
    def map_callback(self, msg):

        self.map_msg = msg

        self.grid = OccupancyGridMap(msg)

        self.get_logger().info(
            f'Map received: '
            f'{msg.info.width} x {msg.info.height}, '
            f'resolution={msg.info.resolution:.3f} m'
        )

        #if self.have_goal and not self.path_active:

            #self.plan_global_path()

    # =============================================================
    # ODOM CALLBACK
    # =============================================================
    def odom_callback(self, msg):

        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

        except TransformException:
            return

        self.robot_x = transform.transform.translation.x
        self.robot_y = transform.transform.translation.y

        q = transform.transform.rotation

        self.robot_yaw = self.quaternion_to_yaw(
            q.x,
            q.y,
            q.z,
            q.w
        )
    # def odom_callback(self, msg):

    #     self.robot_x = msg.pose.pose.position.x
    #     self.robot_y = msg.pose.pose.position.y

    #     q = msg.pose.pose.orientation

    #     self.robot_yaw = self.quaternion_to_yaw(
    #         q.x,
    #         q.y,
    #         q.z,
    #         q.w
    #     )

    # =============================================================
    # LASER CALLBACK
    # =============================================================

    def scan_callback(self, msg):

        self.scan = msg

    # =============================================================
    # GOAL CALLBACK
    # =============================================================

    def goal_callback(self, msg):

        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_frame = msg.header.frame_id

        self.have_goal = True

        self.get_logger().info(
            f'New goal received: '
            f'x={self.goal_x:.2f}, '
            f'y={self.goal_y:.2f}, '
            f'frame={self.goal_frame}'
        )

        if self.grid is None:

            self.get_logger().warn(
                'Goal received, but no map is available yet.'
            )

            return

        self.plan_global_path()

    # =============================================================
    # GLOBAL A* PLANNING
    # =============================================================

    def plan_global_path(self):

        if self.grid is None:

            self.get_logger().warn(
                'Cannot plan: no map available.'
            )

            return

        if self.robot_x is None:

            self.get_logger().warn(
                'Cannot plan: no robot odometry available.'
            )

            return

        if self.goal_x is None:

            return

        # ---------------------------------------------------------
        # Convert world coordinates to grid cells
        # ---------------------------------------------------------

        start = self.grid.world_to_grid(
            self.robot_x,
            self.robot_y
        )

        goal = self.grid.world_to_grid(
            self.goal_x,
            self.goal_y
        )

        # ---------------------------------------------------------
        # Validate start and goal
        # ---------------------------------------------------------

        if not self.grid.is_inside(
            start[0],
            start[1]
        ):

            self.get_logger().error(
                'Robot position is outside the map.'
            )

            return

        if not self.grid.is_inside(
            goal[0],
            goal[1]
        ):

            self.get_logger().error(
                'Goal position is outside the map.'
            )

            return

        # ---------------------------------------------------------
        # Inflate obstacles
        # ---------------------------------------------------------

        planning_grid = self.create_inflated_grid()

        # ---------------------------------------------------------
        # A*
        # ---------------------------------------------------------

        path_cells = self.astar.plan(
            start,
            goal,
            planning_grid
        )

        if not path_cells:

            self.get_logger().error(
                'A* could not find a path.'
            )

            self.stop_robot()

            return

        # ---------------------------------------------------------
        # Convert cells to world coordinates
        # ---------------------------------------------------------

        path_world = []

        for cell in path_cells:

            point = planning_grid.grid_to_world(
                cell[0],
                cell[1]
            )

            path_world.append(point)
        # Keep every 6th A* point so the controller
        # looks farther ahead instead of chasing
        # every 5 cm grid cell.

        if len(path_world) > 2:

            simplified_path = path_world[::6]

            # Always keep the final goal.
            if simplified_path[-1] != path_world[-1]:
                simplified_path.append(
                    path_world[-1]
                )

            path_world = simplified_path
        self.global_path = path_world    
        # ---------------------------------------------------------
        # Simplify path
        # ---------------------------------------------------------

        #path_world = self.simplify_path(
            #path_world
        #)
        path_world = path_world

        self.global_path = path_world

        # ---------------------------------------------------------
        # Send path to waypoint follower
        # ---------------------------------------------------------

        self.waypoint_follower.set_path(
            path_world
        )

        # ---------------------------------------------------------
        # Publish visualization
        # ---------------------------------------------------------

        self.publish_global_path(
            path_world
        )

        self.publish_waypoints(
            path_world
        )

        self.get_logger().info(
            f'A* path created: '
            f'{len(path_world)} waypoints.'
        )

    # =============================================================
    # CREATE INFLATED GRID
    # =============================================================

    def create_inflated_grid(self):

        class PlanningGrid:
            pass

        planning_grid = PlanningGrid()

        planning_grid.width = self.grid.width
        planning_grid.height = self.grid.height
        planning_grid.resolution = self.grid.resolution
        planning_grid.origin_x = self.grid.origin_x
        planning_grid.origin_y = self.grid.origin_y

        planning_grid.data = (
            self.grid.data.copy()
        )

        inflation_cells = int(
            math.ceil(
                self.obstacle_inflation_radius
                /
                self.grid.resolution
            )
        )

        occupied_cells = np_where(
            planning_grid.data
        )

        for x, y in occupied_cells:

            for dx in range(
                -inflation_cells,
                inflation_cells + 1
            ):

                for dy in range(
                    -inflation_cells,
                    inflation_cells + 1
                ):

                    nx = x + dx
                    ny = y + dy

                    if (
                        0 <= nx < planning_grid.width
                        and
                        0 <= ny < planning_grid.height
                    ):

                        distance = math.hypot(
                            dx,
                            dy
                        )

                        if distance <= inflation_cells:

                            planning_grid.data[
                                ny,
                                nx
                            ] = 100

        planning_grid.is_inside = (
            lambda x, y:
            0 <= x < planning_grid.width
            and
            0 <= y < planning_grid.height
        )

        planning_grid.is_occupied = (
            lambda x, y:
            (
                not planning_grid.is_inside(x, y)
                or
                planning_grid.data[y, x] == -1
                or
                planning_grid.data[y, x] > 50
            )
        )

        planning_grid.world_to_grid = (
            lambda x, y:
            (
                int(
                    (x - planning_grid.origin_x)
                    /
                    planning_grid.resolution
                ),
                int(
                    (y - planning_grid.origin_y)
                    /
                    planning_grid.resolution
                )
            )
        )

        planning_grid.grid_to_world = (
            lambda x, y:
            (
                planning_grid.origin_x
                +
                (x + 0.5)
                *
                planning_grid.resolution,

                planning_grid.origin_y
                +
                (y + 0.5)
                *
                planning_grid.resolution
            )
        )

        return planning_grid

    # =============================================================
    # CONTROL LOOP
    # =============================================================

    def control_loop(self):

        # ---------------------------------------------------------
        # Need robot pose
        # ---------------------------------------------------------

        if (
            self.robot_x is None
            or self.robot_y is None
            or self.robot_yaw is None
        ):

            return

        # ---------------------------------------------------------
        # No goal
        # ---------------------------------------------------------

        if not self.have_goal:

            self.stop_robot()

            return

        # ---------------------------------------------------------
        # Check final goal distance
        # ---------------------------------------------------------

        distance_to_goal = math.hypot(
            self.goal_x - self.robot_x,
            self.goal_y - self.robot_y
        )

        if distance_to_goal <= self.goal_tolerance:

            self.get_logger().info(
                'Goal reached.'
            )

            self.stop_robot()

            self.have_goal = False

            self.global_path = []

            self.waypoint_follower.set_path([])

            return

        # ---------------------------------------------------------
        # Need global path
        # ---------------------------------------------------------

        if not self.global_path:

            self.stop_robot()

            return

        # ---------------------------------------------------------
        # Need laser scan
        # ---------------------------------------------------------

        if self.scan is None:

            self.stop_robot()

            return

        # ---------------------------------------------------------
        # Get current waypoint
        # ---------------------------------------------------------

        waypoint = self.waypoint_follower.update(
            self.robot_x,
            self.robot_y
        )

        if waypoint is None:

            self.get_logger().info(
                'All waypoints completed.'
            )

            self.stop_robot()

            return

        # ---------------------------------------------------------
        # Attractive force toward waypoint
        # ---------------------------------------------------------

        attractive_fx, attractive_fy = (
            self.potential_field.attractive_force(
                (
                    self.robot_x,
                    self.robot_y
                ),
                waypoint
            )
        )

        # ---------------------------------------------------------
        # Repulsive force from LaserScan
        # ---------------------------------------------------------

        repulsive_robot_x, repulsive_robot_y = (
            self.potential_field.repulsive_force(
                self.scan
            )
        )

        # ---------------------------------------------------------
        # Transform repulsive force
        # from robot frame to odom frame
        # ---------------------------------------------------------

        cos_yaw = math.cos(
            self.robot_yaw
        )

        sin_yaw = math.sin(
            self.robot_yaw
        )

        repulsive_world_x = (
            repulsive_robot_x * cos_yaw
            -
            repulsive_robot_y * sin_yaw
        )

        repulsive_world_y = (
            repulsive_robot_x * sin_yaw
            +
            repulsive_robot_y * cos_yaw
        )

        # ---------------------------------------------------------
        # Total force
        # ---------------------------------------------------------

        total_fx = (
            attractive_fx
            +
            repulsive_world_x
        )

        total_fy = (
            attractive_fy
            +
            repulsive_world_y
        )

        # ---------------------------------------------------------
        # Convert force to velocity
        # ---------------------------------------------------------

        self.publish_velocity(
            total_fx,
            total_fy
        )

    # =============================================================
    # VELOCITY CONTROL
    # =============================================================

    def publish_velocity(
        self,
        force_x,
        force_y
    ):

        desired_angle = math.atan2(
            force_y,
            force_x
        )

        angle_error = self.normalize_angle(
            desired_angle - self.robot_yaw
        )

        # ---------------------------------------------------------
        # Angular velocity
        # ---------------------------------------------------------

        angular_velocity = (
            2.0 * angle_error
        )

        angular_velocity = self.clamp(
            angular_velocity,
            -self.max_angular_velocity,
            self.max_angular_velocity
        )

        # ---------------------------------------------------------
        # Linear velocity
        # ---------------------------------------------------------

        force_magnitude = math.hypot(
            force_x,
            force_y
        )

        linear_velocity = min(
            self.max_linear_velocity,
            force_magnitude
        )

        # Slow down while turning.

        turning_factor = max(
            0.0,
            1.0
            -
            abs(angle_error) / math.pi
        )

        linear_velocity *= turning_factor

        # Don't drive forward strongly while
        # facing the wrong direction.

        if abs(angle_error) > math.pi / 2.0:

            linear_velocity = 0.0

        elif (
            linear_velocity > 0.0
            and
            linear_velocity < self.min_linear_velocity
        ):

            linear_velocity = (
                self.min_linear_velocity
            )

        # ---------------------------------------------------------
        # Publish
        # ---------------------------------------------------------

        cmd = Twist()

        cmd.linear.x = linear_velocity
        cmd.angular.z = angular_velocity

        self.cmd_vel_publisher.publish(
            cmd
        )

    # =============================================================
    # PUBLISH GLOBAL PATH
    # =============================================================

    def publish_global_path(
        self,
        path
    ):

        msg = Path()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.header.frame_id = 'map'

        for x, y in path:

            pose = PoseStamped()

            pose.header = msg.header

            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            pose.pose.orientation.w = 1.0

            msg.poses.append(
                pose
            )

        self.global_path_publisher.publish(
            msg
        )

    # =============================================================
    # PUBLISH WAYPOINTS
    # =============================================================

    def publish_waypoints(
        self,
        path
    ):

        marker_array = MarkerArray()

        for index, (x, y) in enumerate(path):

            marker = Marker()

            marker.header.frame_id = 'map'

            marker.header.stamp = (
                self.get_clock().now().to_msg()
            )

            marker.ns = 'waypoints'

            marker.id = index

            marker.type = Marker.SPHERE

            marker.action = Marker.ADD

            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.05

            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.10
            marker.scale.y = 0.10
            marker.scale.z = 0.10

            marker_array.markers.append(
                marker
            )

        self.waypoints_publisher.publish(
            marker_array
        )

    # =============================================================
    # STOP ROBOT
    # =============================================================

    def stop_robot(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0

        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0

        self.cmd_vel_publisher.publish(
            cmd
        )

    # =============================================================
    # ANGLE
    # =============================================================

    @staticmethod
    def normalize_angle(angle):

        while angle > math.pi:

            angle -= 2.0 * math.pi

        while angle < -math.pi:

            angle += 2.0 * math.pi

        return angle

    # =============================================================
    # CLAMP
    # =============================================================

    @staticmethod
    def clamp(
        value,
        minimum,
        maximum
    ):

        return max(
            minimum,
            min(
                maximum,
                value
            )
        )

    # =============================================================
    # QUATERNION TO YAW
    # =============================================================

    @staticmethod
    def quaternion_to_yaw(
        x,
        y,
        z,
        w
    ):

        sin_yaw = 2.0 * (
            w * z
            +
            x * y
        )

        cos_yaw = 1.0 - 2.0 * (
            y * y
            +
            z * z
        )

        return math.atan2(
            sin_yaw,
            cos_yaw
        )


# =============================================================
# NUMPY HELPER
# =============================================================

def np_where(data):

    result = []

    height = data.shape[0]
    width = data.shape[1]

    for y in range(height):

        for x in range(width):

            if data[y, x] > 50:

                result.append(
                    (x, y)
                )

    return result


# =============================================================
# MAIN
# =============================================================

def main(args=None):

    rclpy.init(args=args)

    node = PlannerNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.stop_robot()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
