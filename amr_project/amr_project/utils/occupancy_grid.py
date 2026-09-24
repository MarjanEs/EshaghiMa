import numpy as np


class OccupancyGridMap:

    def __init__(self, msg):

        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution

        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y

        self.data = np.array(
            msg.data,
            dtype=np.int16
        ).reshape(
            self.height,
            self.width
        )

    def world_to_grid(self, x, y):

        grid_x = int(
            (x - self.origin_x) / self.resolution
        )

        grid_y = int(
            (y - self.origin_y) / self.resolution
        )

        return grid_x, grid_y

    def grid_to_world(self, x, y):

        world_x = (
            self.origin_x
            + (x + 0.5) * self.resolution
        )

        world_y = (
            self.origin_y
            + (y + 0.5) * self.resolution
        )

        return world_x, world_y

    def is_inside(self, x, y):

        return (
            0 <= x < self.width
            and
            0 <= y < self.height
        )

    def is_occupied(self, x, y):

        if not self.is_inside(x, y):
            return True

        value = self.data[y, x]

        # Occupied OR unknown
        if value == -1:
            return True

        if value > 50:
            return True

        return False
