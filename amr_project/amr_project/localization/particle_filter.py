import random
import math
import numpy as np
import math
from .particle import Particle


class ParticleFilter:

    def __init__(self, num_particles=100):

        self.num_particles = num_particles

        self.particles = []

    def initialize_particles(
        self,
        xmin,
        xmax,
        ymin,
        ymax,
        grid
    ):

        self.particles = []

        while len(self.particles) < self.num_particles:

            x = random.uniform(xmin, xmax)
            y = random.uniform(ymin, ymax)

            gx, gy = grid.world_to_grid(
                x,
                y
            )

            if grid.is_occupied(
                gx,
                gy
            ):
                continue

            theta = random.uniform(
                -math.pi,
                math.pi
            )

            particle = Particle(
                x,
                y,
                theta
            )

            particle.weight = (
                1.0 / self.num_particles
            )

            self.particles.append(
                particle
            ) 
    def motion_update(self, dx_local, dy_local, dtheta):

        for p in self.particles:

            # Each particle interprets the robot movement
            # according to its own possible orientation.
            c = math.cos(p.theta)
            s = math.sin(p.theta)

            dx_map = (
                c * dx_local
                - s * dy_local
            )

            dy_map = (
                s * dx_local
                + c * dy_local
            )

            p.x += (
                dx_map
                + random.gauss(0.0, 0.02)
            )

            p.y += (
                dy_map
                + random.gauss(0.0, 0.02)
            )

            p.theta += (
                dtheta
                + random.gauss(0.0, 0.01)
            )

            # Keep theta between -pi and +pi
            p.theta = math.atan2(
                math.sin(p.theta),
                math.cos(p.theta)
            )
    # def motion_update(
    #     self,
    #     dx,
    #     dy,
    #     dtheta
    # ):

    #     for p in self.particles:

    #         p.x += dx + random.gauss(
    #             0.0,
    #             0.02
    #         )

    #         p.y += dy + random.gauss(
    #             0.0,
    #             0.02
    #         )

    #         p.theta += (
    #             dtheta
    #             + random.gauss(
    #                 0.0,
    #                 0.01
    #             )
    #         )

    def normalize(self):

        total = sum(
            p.weight
            for p in self.particles
        )

        if total <= 0.0:

            uniform = 1.0 / self.num_particles

            for p in self.particles:
                p.weight = uniform

            return

        for p in self.particles:

            p.weight /= total   

    def estimate_pose(self):

        if not self.particles:
            return 0.0, 0.0, 0.0

        self.normalize()

        # Find the highest-weight particle.
        best = max(
            self.particles,
            key=lambda p: p.weight
        )

        # Use particles near the strongest hypothesis.
        # This avoids averaging several different rooms together.
        cluster_radius = 1.0

        cluster = []

        for p in self.particles:

            distance = math.hypot(
                p.x - best.x,
                p.y - best.y
            )

            if distance <= cluster_radius:
                cluster.append(p)

        # Fallback
        if len(cluster) == 0:
            return best.x, best.y, best.theta

        total_weight = sum(
            p.weight for p in cluster
        )

        if total_weight <= 0.0:
            return best.x, best.y, best.theta

        x = sum(
            p.x * p.weight
            for p in cluster
        ) / total_weight

        y = sum(
            p.y * p.weight
            for p in cluster
        ) / total_weight

        sin_sum = sum(
            math.sin(p.theta) * p.weight
            for p in cluster
        )

        cos_sum = sum(
            math.cos(p.theta) * p.weight
            for p in cluster
        )

        theta = math.atan2(
            sin_sum,
            cos_sum
        )

        return x, y, theta    

    # def estimate_pose(self):

    #     if not self.particles:
    #         return 0.0, 0.0, 0.0

    #     self.normalize()

    #     x = sum(
    #         p.x * p.weight
    #         for p in self.particles
    #     )

    #     y = sum(
    #         p.y * p.weight
    #         for p in self.particles
    #     )

    #     sin_sum = sum(
    #         math.sin(p.theta) * p.weight
    #         for p in self.particles
    #     )

    #     cos_sum = sum(
    #         math.cos(p.theta) * p.weight
    #         for p in self.particles
    #     )

    #     theta = math.atan2(
    #         sin_sum,
    #         cos_sum
    #     )

    #     return x, y, theta
    
    # def estimate_pose(self):

    #     if not self.particles:

    #         return (
    #             0.0,
    #             0.0,
    #             0.0
    #         )

    #     x = sum(
    #         p.x * p.weight
    #         for p in self.particles
    #     )

    #     y = sum(
    #         p.y * p.weight
    #         for p in self.particles
    #     )

    #     theta = sum(
    #         p.theta * p.weight
    #         for p in self.particles
    #     )

    #     return (
    #         x,
    #         y,
    #         theta
    #     )
    def cast_ray(
        self,
        x,
        y,
        angle,
        grid
    ):

        step = 0.05

        distance = 0.0

        while distance < 5.5:

            rx = (
                x
                + distance
                * math.cos(angle)
            )

            ry = (
                y
                + distance
                * math.sin(angle)
            )

            gx, gy = (
               grid.world_to_grid(
                   rx,
                   ry
               )
           )

            if grid.is_occupied(
               gx,
               gy
            ):
               return distance

            distance += step

        return 5.5
    
    # def pose_scan_error(self, x, y, theta, scan, grid):

    #     num_beams = 15

    #     beam_indices = np.linspace(
    #         0,
    #         len(scan.ranges) - 1,
    #         num_beams,
    #         dtype=int
    #     )

    #     max_range = min(scan.range_max, 5.5)
    #     laser_x = x + 0.45 * math.cos(theta)
    #     laser_y = y + 0.45 * math.sin(theta)

    #     total_error = 0.0
    #     valid_beams = 0

    #     print("\n========== TRUE POSE LASER TEST ==========")
    #     print(f"Base position:  x={x:.2f}, y={y:.2f}, theta={theta:.2f}")
    #     print(f"Laser position: x={laser_x:.2f}, y={laser_y:.2f}")

    #     for idx in beam_indices:

    #         measured = scan.ranges[idx]

    #         if math.isnan(measured):
    #             continue

    #         if math.isinf(measured):
    #             measured = max_range

    #         if measured < scan.range_min:
    #             continue

    #         measured = min(measured, max_range)

    #         angle = (
    #             theta
    #             + scan.angle_min
    #             + idx * scan.angle_increment
    #         )

    #         expected = self.cast_ray(
    #             laser_x,
    #             laser_y,
    #             angle,
    #             grid
    #         )

    #         error = abs(expected - measured)
    #         print(
    #             f"beam={idx:3d}  "
    #             f"angle={math.degrees(angle):7.1f}  "
    #             f"measured={measured:5.2f}  "
    #             f"expected={expected:5.2f}  "
    #             f"error={error:5.2f}"
    #         )
    #         total_error += error
    #         valid_beams += 1

    #     print("==========================================\n")
    #     if valid_beams == 0:
    #         return 9999.0

    #     return total_error / valid_beams

    
    # def measurement_update(self, scan, grid):
    #     if scan is None or len(scan.ranges) == 0:
    #         return

    #     # Use more laser beams across the scan
    #     num_beams = 15

    #     beam_indices = np.linspace(
    #         0,
    #         len(scan.ranges) - 1,
    #         num_beams,
    #         dtype=int
    #     )

    #     sigma = 0.4
    #     max_range = min(scan.range_max, 5.5)

    #     for p in self.particles:

    #         weight = 1.0
    #         valid_beams = 0

    #         for idx in beam_indices:

    #             measured = scan.ranges[idx]

    #             # NaN is unusable
    #             if math.isnan(measured):
    #                 continue

    #             # inf means the laser saw no obstacle
    #             if math.isinf(measured):
    #                 measured = max_range

    #             # Ignore invalid very-short measurements
    #             if measured < scan.range_min:
    #                 continue

    #             measured = min(measured, max_range)

    #             angle = (
    #                p.theta
    #                + scan.angle_min
    #                + idx * scan.angle_increment
    #             )

    #             # Laser is 0.45 m in front of the robot
    #             laser_offset = 0.45

    #             laser_x = p.x + laser_offset * math.cos(p.theta)
    #             laser_y = p.y + laser_offset * math.sin(p.theta)

    #             expected = self.cast_ray(
    #                 laser_x,
    #                 laser_y,
    #                 angle,
    #                 grid
    #             )

    #             error = expected - measured


    #             likelihood = math.exp(
    #                 -(error ** 2) / (2 * sigma ** 2)
    #             )

    #             # Prevent one beam from making weight exactly zero
    #             likelihood = max(likelihood, 0.001)

    #             weight *= likelihood
    #             valid_beams += 1

    #         if valid_beams > 0:
    #             p.weight = weight
    #         else:
    #             p.weight = 1.0

    #     self.normalize()
    #     #self.resample()
    #     # Effective number of particles
    #     neff = 1.0 / sum(
    #         p.weight ** 2
    #         for p in self.particles
    #     )

    #     # Resample only when necessary
    #     if neff < self.num_particles / 2.0:
    #         self.resample()
    def measurement_update(self, scan, grid):

        if scan is None or len(scan.ranges) == 0:
            return

        # Use more actual obstacle measurements.
        num_beams = 30

        beam_indices = np.linspace(
            0,
            len(scan.ranges) - 1,
            num_beams,
            dtype=int
        )

        sigma = 0.35
        max_range = min(scan.range_max, 5.5)

        for p in self.particles:

            log_weight = 0.0
            valid_beams = 0

            for idx in beam_indices:

                measured = scan.ranges[idx]

                # Ignore invalid values.
                if math.isnan(measured):
                    continue

                # IMPORTANT:
                # Ignore beams where no obstacle was detected.
                # We want obstacle hits to distinguish locations.
                if math.isinf(measured):
                    continue

                if measured < scan.range_min:
                    continue

                measured = min(
                    measured,
                    max_range
                )

                # Direction of this laser beam for this particle.
                angle = (
                    p.theta
                    + scan.angle_min
                    + idx * scan.angle_increment
                )

                # Front laser is 0.45 m ahead of base_link.
                laser_offset = 0.45

                laser_x = (
                    p.x
                    + laser_offset * math.cos(p.theta)
                )

                laser_y = (
                    p.y
                    + laser_offset * math.sin(p.theta)
                )

                expected = self.cast_ray(
                    laser_x,
                    laser_y,
                    angle,
                    grid
                )

                error = expected - measured

                # Gaussian laser likelihood.
                # Add log likelihood instead of multiplying
                # many tiny numbers together.
                log_weight += (
                    -(error ** 2)
                    / (2.0 * sigma ** 2)
                )

                valid_beams += 1

            if valid_beams > 0:

                # Average prevents particle weight from depending
                # too strongly on the number of valid beams.
                log_weight /= valid_beams

                # Avoid numerical underflow.
                log_weight = max(
                    log_weight,
                    -50.0
                )

                p.weight = math.exp(
                    log_weight
                )

            else:
                p.weight = 1e-12

        self.normalize()

        neff = 1.0 / sum(
            p.weight ** 2
            for p in self.particles
        )

        if neff < self.num_particles * 0.8:
            self.resample() 
               
    def resample(self):

        if len(self.particles) == 0:
            return

        weights = np.array(
            [p.weight for p in self.particles],
            dtype=float
        )

        weights[~np.isfinite(weights)] = 0.0

        total = np.sum(weights)

        if total <= 0.0:
            weights = np.ones(
                len(self.particles)
            ) / len(self.particles)
        else:
            weights /= total

        indices = np.random.choice(
            len(self.particles),
            len(self.particles),
            p=weights
        )

        new_particles = []

        uniform_weight = 1.0 / len(self.particles)

        for i in indices:

            old = self.particles[i]

            new_particle = Particle(
                old.x + random.gauss(0.0, 0.03),
                old.y + random.gauss(0.0, 0.03),
                old.theta + random.gauss(0.0, 0.02)
            )

            new_particle.weight = uniform_weight

            new_particles.append(new_particle)

        self.particles = new_particles