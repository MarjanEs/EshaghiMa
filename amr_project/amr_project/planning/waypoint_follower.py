import math


class WaypointFollower:

    def __init__(
        self,
        waypoint_tolerance=0.30
    ):

        self.waypoints = []

        self.current_index = 0

        self.tolerance = waypoint_tolerance

    # ---------------------------------------------------------
    # Set new path
    # ---------------------------------------------------------

    def set_path(self, path):

        self.waypoints = list(path)

        self.current_index = 0

    # ---------------------------------------------------------
    # Check whether path is finished
    # ---------------------------------------------------------

    def path_finished(self):

        return (
            self.current_index
            >= len(self.waypoints)
        )

    # ---------------------------------------------------------
    # Current waypoint
    # ---------------------------------------------------------

    def current_waypoint(self):

        if self.path_finished():
            return None

        return self.waypoints[
            self.current_index
        ]

    # ---------------------------------------------------------
    # Update waypoint
    # ---------------------------------------------------------

    def update(
        self,
        robot_x,
        robot_y
    ):

        if self.path_finished():
            return None

        waypoint = self.current_waypoint()

        dx = (
            waypoint[0]
            - robot_x
        )

        dy = (
            waypoint[1]
            - robot_y
        )

        distance = math.hypot(
            dx,
            dy
        )

        if distance <= self.tolerance:

            self.current_index += 1

            if self.path_finished():
                return None

        return self.current_waypoint()
