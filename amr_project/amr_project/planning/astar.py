import heapq
import math


class AStarPlanner:

    def __init__(self):

        self.neighbor_cost = 1.0

    # ---------------------------------------------------------
    # Heuristic
    # ---------------------------------------------------------

    def heuristic(self, a, b):

        return math.hypot(
            a[0] - b[0],
            a[1] - b[1]
        )

    # ---------------------------------------------------------
    # Neighbors
    # ---------------------------------------------------------

    def get_neighbors(self, node):

        x, y = node

        return [
            (x + 1, y),
            (x - 1, y),
            (x, y + 1),
            (x, y - 1),

            # Diagonal movement
            (x + 1, y + 1),
            (x + 1, y - 1),
            (x - 1, y + 1),
            (x - 1, y - 1)
        ]

    # ---------------------------------------------------------
    # Movement cost
    # ---------------------------------------------------------

    def movement_cost(self, current, neighbor):

        dx = abs(
            neighbor[0] - current[0]
        )

        dy = abs(
            neighbor[1] - current[1]
        )

        if dx == 1 and dy == 1:
            return math.sqrt(2.0)

        return 1.0

    # ---------------------------------------------------------
    # A* planner
    # ---------------------------------------------------------

    def plan(self, start, goal, grid):

        if not grid.is_inside(
            start[0],
            start[1]
        ):
            return []

        if not grid.is_inside(
            goal[0],
            goal[1]
        ):
            return []

        if grid.is_occupied(
            start[0],
            start[1]
        ):
            return []

        if grid.is_occupied(
            goal[0],
            goal[1]
        ):
            return []

        open_set = []

        heapq.heappush(
            open_set,
            (
                self.heuristic(start, goal),
                start
            )
        )

        came_from = {}

        g_score = {
            start: 0.0
        }

        closed_set = set()

        while open_set:

            _, current = heapq.heappop(
                open_set
            )

            if current in closed_set:
                continue

            closed_set.add(current)

            # -------------------------------------------------
            # Goal reached
            # -------------------------------------------------

            if current == goal:

                return self.reconstruct_path(
                    came_from,
                    current
                )

            # -------------------------------------------------
            # Explore neighbors
            # -------------------------------------------------

            for neighbor in self.get_neighbors(
                current
            ):

                if neighbor in closed_set:
                    continue

                if grid.is_occupied(
                    neighbor[0],
                    neighbor[1]
                ):
                    continue

                # Prevent diagonal corner cutting.
                if (
                    neighbor[0] != current[0]
                    and neighbor[1] != current[1]
                ):

                    cell_1 = (
                        neighbor[0],
                        current[1]
                    )

                    cell_2 = (
                        current[0],
                        neighbor[1]
                    )

                    if (
                        grid.is_occupied(
                            cell_1[0],
                            cell_1[1]
                        )
                        or
                        grid.is_occupied(
                            cell_2[0],
                            cell_2[1]
                        )
                    ):
                        continue

                tentative_g = (
                    g_score[current]
                    +
                    self.movement_cost(
                        current,
                        neighbor
                    )
                )

                if (
                    neighbor not in g_score
                    or
                    tentative_g < g_score[neighbor]
                ):

                    came_from[neighbor] = current

                    g_score[neighbor] = tentative_g

                    f_score = (
                        tentative_g
                        +
                        self.heuristic(
                            neighbor,
                            goal
                        )
                    )

                    heapq.heappush(
                        open_set,
                        (
                            f_score,
                            neighbor
                        )
                    )

        return []

    # ---------------------------------------------------------
    # Reconstruct path
    # ---------------------------------------------------------

    def reconstruct_path(
        self,
        came_from,
        current
    ):

        path = [current]

        while current in came_from:

            current = came_from[current]

            path.append(current)

        path.reverse()

        return path
