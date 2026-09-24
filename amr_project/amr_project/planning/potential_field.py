import math


class PotentialFieldPlanner:

    def __init__(
        self,
        attractive_gain=1.5,
        repulsive_gain=1.0,
        obstacle_distance_threshold=1.0
    ):

        self.k_att = attractive_gain

        self.k_rep = repulsive_gain

        self.obstacle_distance_threshold = (
            obstacle_distance_threshold
        )

    # ---------------------------------------------------------
    # Attractive force
    # ---------------------------------------------------------

    def attractive_force(
        self,
        robot,
        target
    ):

        dx = target[0] - robot[0]

        dy = target[1] - robot[1]

        fx = self.k_att * dx

        fy = self.k_att * dy

        return fx, fy

    # ---------------------------------------------------------
    # Repulsive force
    # ---------------------------------------------------------
    def repulsive_force(
        self,
        scan
    ):

        fx = 0.0
        fy = 0.0

        threshold = self.obstacle_distance_threshold

        for i, distance in enumerate(scan.ranges):

            if not math.isfinite(distance):
                continue

            if distance <= 0.05:
                continue

            if distance >= threshold:
                continue

            angle = (
                scan.angle_min
                + i * scan.angle_increment
            )

            # Normalize laser angle to [-pi, pi]
            angle = math.atan2(
                math.sin(angle),
                math.cos(angle)
            )

            # --------------------------------------------------
            # Ignore obstacles mostly behind the robot.
            # We care primarily about the forward 180 degrees.
            # --------------------------------------------------
            if abs(angle) > math.pi / 2.0:
                continue

            magnitude = (
                self.k_rep
                *
                (
                    1.0 / distance
                    - 1.0 / threshold
                )
                /
                (distance * distance)
            )

            # Prevent individual laser beams from dominating.
            magnitude = min(
                magnitude,
                0.10
            )

            obstacle_fx = (
                -magnitude * math.cos(angle)
            )

            obstacle_fy = (
                -magnitude * math.sin(angle)
            )

            # --------------------------------------------------
            # IMPORTANT FOR CORRIDORS
            #
            # Side walls should mainly push the robot sideways,
            # not strongly backwards.
            # --------------------------------------------------
            side_factor = abs(
                math.sin(angle)
            )

            front_factor = abs(
                math.cos(angle)
            )

            fx += obstacle_fx * front_factor

            fy += obstacle_fy * (
                0.35 + 0.65 * side_factor
            )

        # ------------------------------------------------------
        # Limit total repulsive force
        # ------------------------------------------------------

        total = math.hypot(
            fx,
            fy
        )

        max_repulsive_force = 0.50

        if total > max_repulsive_force:

            scale = (
                max_repulsive_force
                /
                total
            )

            fx *= scale
            fy *= scale

        return fx, fy

    
    # def repulsive_force(
    #     self,
    #     scan
    # ):

    #     fx = 0.0

    #     fy = 0.0

    #     threshold = (
    #         self.obstacle_distance_threshold
    #     )

    #     for i, distance in enumerate(
    #         scan.ranges
    #     ):

    #         if not math.isfinite(distance):
    #             continue

    #         if distance <= 0.05:
    #             continue

    #         if distance >= threshold:
    #             continue

    #         angle = (
    #             scan.angle_min
    #             +
    #             i * scan.angle_increment
    #         )

    #         # Standard potential-field
    #         # repulsive magnitude.

    #         magnitude = (
    #             self.k_rep
    #             *
    #             (
    #                 1.0 / distance
    #                 -
    #                 1.0 / threshold
    #             )
    #             /
    #             (distance * distance)
    #         )

    #         # Force points away from obstacle.

    #         fx -= (
    #             magnitude
    #             * math.cos(angle)
    #         )

    #         fy -= (
    #             magnitude
    #             * math.sin(angle)
    #         )

    #     return fx, fy
