"""Small online flood-fill planner for the 6x6 PacBot maze.

Unknown edges are treated as traversable.  The motion controller remains
responsible for validating a route with live sensors; this planner only ranks
locally observed branches.
"""
from collections import deque


class FloodFillPlanner:
    SIZE = 6
    # 0=east, 1=north, 2=west, 3=south
    DX = (1, 0, -1, 0)
    DY = (0, -1, 0, 1)

    def __init__(self, x=0, y=0, heading=0):
        self.x = x
        self.y = y
        self.heading = heading % 4
        self.walls = {}  # (x, y, absolute direction) -> True/False
        self.last_distances = {}

    def _inside(self, x, y):
        return 0 <= x < self.SIZE and 0 <= y < self.SIZE

    def _edge(self, x, y, direction):
        return (x, y, direction % 4)

    def set_wall(self, x, y, direction, value):
        direction %= 4
        self.walls[self._edge(x, y, direction)] = bool(value)
        nx = x + self.DX[direction]
        ny = y + self.DY[direction]
        if self._inside(nx, ny):
            self.walls[self._edge(nx, ny, (direction + 2) % 4)] = bool(value)

    def observe(self, front=None, left=None, right=None):
        """Record known walls relative to the current cell.

        Values are True=wall, False=open, None=unknown.
        """
        for relative, value in ((0, front), (1, left), (-1, right)):
            if value is not None:
                self.set_wall(self.x, self.y,
                              self.heading + relative, value)

    def rotate(self, relative_quarters):
        self.heading = (self.heading + int(relative_quarters)) % 4

    def advance(self):
        nx = self.x + self.DX[self.heading]
        ny = self.y + self.DY[self.heading]
        if self._inside(nx, ny):
            self.x, self.y = nx, ny

    def distances(self):
        """Flood distances to any east-border goal cell."""
        distances = {(x, y): None
                     for x in range(self.SIZE) for y in range(self.SIZE)}
        queue = deque()
        for y in range(self.SIZE):
            distances[(self.SIZE - 1, y)] = 0
            queue.append((self.SIZE - 1, y))

        while queue:
            x, y = queue.popleft()
            for direction in range(4):
                nx = x + self.DX[direction]
                ny = y + self.DY[direction]
                if not self._inside(nx, ny):
                    continue
                if self.walls.get(self._edge(x, y, direction)) is True:
                    continue
                if distances[(nx, ny)] is None:
                    distances[(nx, ny)] = distances[(x, y)] + 1
                    queue.append((nx, ny))
        self.last_distances = distances
        return distances

    def choose(self, relative_options):
        """Return the best relative direction among observed open options."""
        distances = self.distances()
        ranked = []
        for relative in relative_options:
            direction = (self.heading + relative) % 4
            if self.walls.get(self._edge(self.x, self.y, direction)) is True:
                continue
            nx = self.x + self.DX[direction]
            ny = self.y + self.DY[direction]
            if self._inside(nx, ny):
                ranked.append((distances[(nx, ny)], relative))
        if not ranked:
            return None
        ranked.sort(key=lambda item: (item[0] is None, item[0],
                                      0 if item[1] > 0 else 1))
        return ranked[0][1]
