from typing import List
from libcpp cimport bool

cdef int[6][2] DIRECTIONS = [
    [2, 0],
    [1, 1],
    [-1, 1],
    [-2, 0],
    [-1, -1],
    [1, -1],
]

cdef class HexCoordinate:
    cdef public int x, y

    # By default in doublewidth coordinates (see, e.g., https://www.redblobgames.com/grids/hexagons/#coordinates-doubled)
    def __init__(self, int x, int y):
        if (x % 2 == 0) != (y % 2 == 0):
            raise Exception('Wrong hex coordinate!')

        self.x = x
        self.y = y

    cpdef (int, int, int) to_cube(self):
        cdef int x = (self.x - self.y) / 2
        cdef int z = self.y
        cdef int y = -x - z
        return (x, y, z)

    @staticmethod
    def from_cube((int, int, int) cube) -> "HexCoordinate":
        cdef int col = 2 * cube[0] + cube[2]
        cdef int row = cube[2]
        return HexCoordinate(col, row)

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return "HexCoord(x={}, y={})".format(self.x, self.y)

    def __lt__(self, HexCoordinate other):
        if self.y < other.y:
            return True
        elif self.y == other.y and self.x < other.x:
            return True
        else:
            return False

    def __le__(self, HexCoordinate other):
        return self.__lt__(other) or self.__eq__(other)

    def __ne__(self, HexCoordinate other):
        return not self.__eq__(other)

    def __gt__(self, HexCoordinate other):
        return not self.__le__(other)

    def __ge__(self, HexCoordinate other):
        return not self.__lt__(other)

    def __eq__(self, HexCoordinate other):
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))

cpdef bint _is_feasible_coordinate(int x, int y, int height, int width):
    if y < 0 or y >= height:
        return False

    if x % 2 == 0 and y % 2 == 0:
        if 0 <= x <= (width - 1)*2:
            return True
        else:
            return False
    elif x % 2 != 0 and x % 2 != 0:
        if 1 <= x <= (width - 1) * 2 + 1:
            return True
        else:
            return False
    else:
        return False


def _list_of_occupied_by_obstacles_hexes(int time, int additional_radius, obstacles, map):
    occupied_hexes = set()
    cdef int obstacle_x
    cdef int obstacle_y
    cdef int obstacle_i, i, j, ring_radius
    cdef int radius
    cdef int r

    cdef HexCoordinate cube

    for obstacle_i in range(len(obstacles)):

        obstacle_x = obstacles[obstacle_i].x
        obstacle_y = obstacles[obstacle_i].y

        radius = additional_radius + 1
        occupied_hexes.add(HexCoordinate(obstacle_x, obstacle_y))

        for ring_radius in range(1, radius):
            r = ring_radius

            cube = HexCoordinate(obstacle_x + DIRECTIONS[4][0]*r, obstacle_y + DIRECTIONS[4][1]*r) # step radius hexes to the right

            for i in range(len(DIRECTIONS)):
                for j in range(r):
                    if _is_feasible_coordinate(cube.x, cube.y, map.height, map.width):
                        occupied_hexes.add(cube)
                    cube = HexCoordinate(cube.x + DIRECTIONS[i][0], cube.y + DIRECTIONS[i][1])

    return occupied_hexes
