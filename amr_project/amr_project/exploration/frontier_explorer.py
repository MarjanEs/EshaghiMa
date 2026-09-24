import math
from collections import deque


class FrontierExplorer:

    def __init__(self):

        # Minimum number of connected frontier cells
        # required before we consider it a useful frontier.
        self.min_frontier_size = 5


    def index(self, x, y, width):

        return y * width + x


    def in_bounds(self, x, y, width, height):

        return (
            0 <= x < width
            and 0 <= y < height
        )


    def get_value(self, map_msg, x, y):

        width = map_msg.info.width
        height = map_msg.info.height

        if not self.in_bounds(
            x, y, width, height
        ):
            return 100

        i = self.index(
            x,
            y,
            width
        )

        return map_msg.data[i]


    def is_free(self, map_msg, x, y):

        value = self.get_value(
            map_msg,
            x,
            y
        )

        return 0 <= value <= 20


    def is_unknown(self, map_msg, x, y):

        return (
            self.get_value(
                map_msg,
                x,
                y
            ) == -1
        )


    def is_frontier(self, map_msg, x, y):

        # A frontier must itself be free space.
        if not self.is_free(
            map_msg,
            x,
            y
        ):
            return False

        # It must touch at least one unknown cell.
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):

                if dx == 0 and dy == 0:
                    continue

                nx = x + dx
                ny = y + dy

                if self.is_unknown(
                    map_msg,
                    nx,
                    ny
                ):
                    return True

        return False


    def find_frontier_cells(self, map_msg):

        width = map_msg.info.width
        height = map_msg.info.height

        frontier_cells = []

        for y in range(height):
            for x in range(width):

                if self.is_frontier(
                    map_msg,
                    x,
                    y
                ):
                    frontier_cells.append(
                        (x, y)
                    )

        return frontier_cells


    def cluster_frontiers(
        self,
        map_msg,
        frontier_cells
    ):

        frontier_set = set(
            frontier_cells
        )

        visited = set()

        clusters = []

        for start in frontier_cells:

            if start in visited:
                continue

            queue = deque([start])

            visited.add(start)

            cluster = []

            while queue:

                current = queue.popleft()

                cluster.append(current)

                cx, cy = current

                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):

                        if dx == 0 and dy == 0:
                            continue

                        neighbor = (
                            cx + dx,
                            cy + dy
                        )

                        if (
                            neighbor in frontier_set
                            and neighbor not in visited
                        ):

                            visited.add(
                                neighbor
                            )

                            queue.append(
                                neighbor
                            )

            if (
                len(cluster)
                >= self.min_frontier_size
            ):
                clusters.append(
                    cluster
                )

        return clusters


    def grid_to_world(
        self,
        map_msg,
        gx,
        gy
    ):

        resolution = (
            map_msg.info.resolution
        )

        origin_x = (
            map_msg.info.origin.position.x
        )

        origin_y = (
            map_msg.info.origin.position.y
        )

        wx = (
            origin_x
            + (gx + 0.5) * resolution
        )

        wy = (
            origin_y
            + (gy + 0.5) * resolution
        )

        return wx, wy


    def cluster_center(
        self,
        map_msg,
        cluster
    ):

        gx = sum(
            p[0] for p in cluster
        ) / len(cluster)

        gy = sum(
            p[1] for p in cluster
        ) / len(cluster)

        return self.grid_to_world(
            map_msg,
            gx,
            gy
        )


    def select_frontier(
        self,
        map_msg,
        robot_x,
        robot_y
    ):

        cells = (
            self.find_frontier_cells(
                map_msg
            )
        )

        clusters = (
            self.cluster_frontiers(
                map_msg,
                cells
            )
        )

        if len(clusters) == 0:
            return None

        best_goal = None
        best_score = float('inf')

        for cluster in clusters:

            wx, wy = (
                self.cluster_center(
                    map_msg,
                    cluster
                )
            )

            distance = math.hypot(
                wx - robot_x,
                wy - robot_y
            )

            # Prefer nearby frontiers,
            # but slightly favour larger frontiers.
            score = (
                distance
                - 0.02 * len(cluster)
            )

            if score < best_score:

                best_score = score

                best_goal = (
                    wx,
                    wy
                )

        return best_goal