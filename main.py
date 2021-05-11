import functools
import itertools
import queue
import random
import time
from dataclasses import dataclass
from math import ceil
from typing import List, Dict, Tuple, Set, Union

import simplejson
from igraph import *
import networkx as nx
import numpy as np


DEBUG = False


@dataclass
class Request:
    start_point: "HexCoordinate"
    end_point: "HexCoordinate"
    start_time: int

    def __init__(self, start: Union[Tuple[int, int], "HexCoordinate"], end: Union[Tuple[int, int], "HexCoordinate"], start_time: int):
        if isinstance(start, HexCoordinate):
            self.start_point = start
        else:
            self.start_point = HexCoordinate(x=start[0], y=start[1])

        if isinstance(end, HexCoordinate):
            self.end_point = end
        else:
            self.end_point = HexCoordinate(x=end[0], y=end[1])

        self.start_time = start_time


@dataclass
class GDP:
    max_time: int
    penalty: float


class Flightplan:
    ONE_TICK_TO_LAND = False

    def __init__(self, points: List[Tuple[int, "HexCoordinate"]]):
        self.start_time = min([x[0] for x in points])
        self._end_time = max([x[0] for x in points])

        self.points: Dict[(int, HexCoordinate)] = {}
        self._fill_points(points)

    def _fill_points(self, points: List[Tuple[int, "HexCoordinate"]]):
        for (time, point) in points:
            self.points[time] = point

    @property
    def end_time(self):
        if self.ONE_TICK_TO_LAND:
            return self._end_time + 1
        else:
            return self._end_time

    @end_time.setter
    def end_time(self, value):
        self._end_time = value

    def is_present_at_time(self, time: int) -> bool:
        return time >= self.start_time and time <= self.end_time

    def position_at_time(self, time: int) -> "HexCoordinate":
        if not self.is_present_at_time(time):
            raise Exception('Not present at time {}'.format(time))

        if self.ONE_TICK_TO_LAND and time not in self.points:
            return self.points[time-1]

        return self.points[time]

    def smoothed(self) -> List[Tuple[int, "HexCoordinate"]]:
        smoothed_points: List[Tuple[int, "HexCoordinate"]] = []

        orig_points = [(t, x) for (t, x) in sorted(self.points.items(), key=lambda x : x[0])]

        i = 0
        smoothed_points.append(orig_points[0])

        while i < len(orig_points) - 1:
            current_point = orig_points[i][1]

            found_any_point = False
            for j in reversed(range(i+1, len(orig_points))):
                other_point = orig_points[j][1]

                line = HexHelper.line_drawing(current_point, other_point)
                if line == [x[1] for x in orig_points[i:j+1]]:
                    found_any_point = True
                    smoothed_points.append(orig_points[j])
                    i = j
                    break

            if not found_any_point:
                smoothed_points.append(orig_points[i+1])

        return smoothed_points


@functools.total_ordering
class HexCoordinate:
    # By default in doublewidth coordinates (see, e.g., https://www.redblobgames.com/grids/hexagons/#coordinates-doubled)
    def __init__(self, x: int, y: int):
        if (x % 2 == 0) != (y % 2 == 0):
            raise Exception('Wrong hex coordinate!')

        self.x: int = x
        self.y: int = y

    def to_cube(self) -> Tuple[int, int, int]:
        x = (self.x - self.y) / 2
        z = self.y
        y = -x - z
        return (int(x), int(y), z)

    @staticmethod
    def from_cube(cube: Tuple[int, int, int]) -> "HexCoordinate":
        col = 2 * cube[0] + cube[2]
        row = cube[2]
        return HexCoordinate(col, row)

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return "HexCoord(x={}, y={})".format(self.x, self.y)

    def __lt__(self, other: "HexCoordinate") -> bool:
        if self.y < other.y:
            return True
        elif self.y == other.y and self.x < other.x:
            return True
        else:
            return False

    def __eq__(self, other: "HexCoordinate") -> bool:
        return self.x == other.x and self.y == other.y

    def __hash__(self) -> int:
        return hash((self.x, self.y))


class HexMap:
    DIRECTIONS = [
        [2, 0],
        [1, 1],
        [-1, 1],
        [-2, 0],
        [-1, -1],
        [1, -1],
    ]

    # We are using double-width horizontal layout as specified in https://www.redblobgames.com/grids/hexagons/
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height

    def int_time(self, node_id: int) -> int:
        time = math.floor(node_id / self.number_of_nodes)

        return time

    def int_to_coord(self, node_id: int) -> HexCoordinate:
        if node_id >= self.number_of_nodes:
            node_id = node_id - math.floor(node_id / self.number_of_nodes) * self.number_of_nodes # add tests for this

        y = math.floor(node_id / self.width)
        x = node_id - y * self.width

        if y % 2 == 0:
            x = x * 2
        else:
            x = x * 2 + 1

        return HexCoordinate(x=x, y=y)

    def coord_to_int(self, coord: HexCoordinate) -> int:
        if coord.x % 2 == 0 and coord.y % 2 == 0:
            return round(coord.y * self.width + coord.x / 2)
        else:
            return round(coord.y * self.width + math.floor(coord.x / 2))

    def get_neighbours(self, coord: HexCoordinate) -> List[HexCoordinate]:
        neighbours: List[HexCoordinate] = []

        for (x_offset, y_offset) in self.DIRECTIONS:
            if self._is_feasible_coordinate(coord.x + x_offset, coord.y + y_offset):
                neighbours.append(HexCoordinate(coord.x + x_offset, coord.y + y_offset))

        return neighbours

    def is_feasible_coordinate(self, coord: HexCoordinate) -> bool:
        return self._is_feasible_coordinate(coord.x, coord.y)

    def _is_feasible_coordinate(self, x: int, y: int) -> bool:
        if y < 0 or y >= self.height:
            return False

        if x % 2 == 0 and y % 2 == 0:
            if 0 <= x <= (self.width - 1)*2:
                return True
            else:
                return False
        elif x % 2 != 0 and x % 2 != 0:
            if 1 <= x <= (self.width - 1) * 2 + 1:
                return True
            else:
                return False
        else:
            return False

    @property
    def number_of_nodes(self) -> int:
        return self.width*self.height

    def spiral(self, center: HexCoordinate, radius: int) -> List[HexCoordinate]:
        results: List[HexCoordinate] = []

        results.append(center)

        for i in range(2, radius+1):
            results.extend(self.ring(center, i))

        return results

    def ring(self, center: HexCoordinate, radius: int) -> List[HexCoordinate]:
        r = radius - 1
        results: List[HexCoordinate] = []

        cube = HexCoordinate(center.x + self.DIRECTIONS[4][0]*r, center.y + self.DIRECTIONS[4][1]*r) # step radius hexes to the right

        for i in range(len(self.DIRECTIONS)):
            for j in range(r):
                results.append(cube)
                cube = HexCoordinate(cube.x + self.DIRECTIONS[i][0], cube.y + self.DIRECTIONS[i][1])

        return results


class PathPlanner:
    SOURCE_NODE_ID = -1
    SINK_NODE_ID = -2

    def __init__(self, obstacles: List[HexCoordinate], requests: List[Request], map_width, map_height,
                 hex_radius_m, drone_radius_m, gdp, city_map=None):
        self.flightplans: List[Flightplan] = []
        self.obstacles = obstacles
        self.requests = requests
        self.map = HexMap(width=map_width, height=map_height)
        self.hex_radius_m = hex_radius_m
        self.drone_radius_m = drone_radius_m
        self.drone_radius_hex = ceil(drone_radius_m/hex_radius_m)
        self.gdp = gdp

        self.city_map = city_map

    def _get_all_neighbours(self, coordinate: HexCoordinate, time: int, request: Request) -> List[Tuple[int, float]]:
        viable_neighbours_ids: List[Tuple[int, float]] = []

        occupied = self.list_of_occupied_hexes(time + 1)
        if self.jumping_can_occur():
            drones_movements = self.list_of_pending_drone_movements(time + 1)

        neighbours = self.map.get_neighbours(coordinate)
        neighbours_ids = [self.map.coord_to_int(x) for x in neighbours]

        for (neigh_i, node_neighbour_id) in enumerate(neighbours_ids):
            if neighbours[neigh_i] not in occupied:
                if not (self.jumping_can_occur() and (neighbours[neigh_i], coordinate) in drones_movements):
                    viable_neighbours_ids.append(
                        (self.time_ext_node_id(node_neighbour_id, time + 1), 1)
                    )

        if coordinate not in occupied:
            viable_neighbours_ids.append(
                (self.time_ext_node_id(self.map.coord_to_int(coordinate), time + 1), 1)  # hovering
            )

        if coordinate == request.end_point:
            viable_neighbours_ids.append(
                (self.SINK_NODE_ID, 0)  # sink
            )

        return viable_neighbours_ids

    def _find_shortest_path(self, request: Request) -> List[int]:
        self._visited_nodes = set()
        self._nodes_weights = {}
        self._queue = queue.PriorityQueue()
        self._previous_nodes = {}

        # self._queue.put((0, self.SOURCE_NODE_ID))

        for i in range(self.gdp.max_time+1):
            weight = i*self.gdp.penalty
            occupied = self.list_of_occupied_hexes(request.start_time + i)
            coord = self.time_ext_node_id(self.map.coord_to_int(request.start_point), request.start_time + i)

            if request.start_point not in occupied:
                self._queue.put((weight, coord))
                self._nodes_weights[coord] = weight
                self._previous_nodes[coord] = self.SOURCE_NODE_ID

        while not self._queue.empty():
            (weight, current_node_id) = self._queue.get()

            if current_node_id not in self._visited_nodes:
                if current_node_id == self.SINK_NODE_ID:
                    break

                current_node_weight = self._nodes_weights[current_node_id]
                current_node_coord = self.map.int_to_coord(current_node_id)
                current_node_time = self.map.int_time(current_node_id)

                neighbours = self._get_all_neighbours(current_node_coord, current_node_time, request)

                for (neighbour_id, weight) in neighbours:
                    if neighbour_id not in self._nodes_weights or weight + current_node_weight < self._nodes_weights[neighbour_id]:
                        self._nodes_weights[neighbour_id] = weight + current_node_weight
                        self._previous_nodes[neighbour_id] = current_node_id
                        self._queue.put((weight + current_node_weight, neighbour_id))

                self._visited_nodes.add(current_node_id)

        path = []
        previous_node = self._previous_nodes[self.SINK_NODE_ID]

        while previous_node != self.SOURCE_NODE_ID:
            path.append(previous_node)
            previous_node = self._previous_nodes[previous_node]

        path.reverse()

        return path

    def time_ext_node_id(self, node_id: int, time: int) -> int:
        return node_id + self.map.number_of_nodes * time

    def list_of_pending_drone_movements(self, time: int) -> List[Tuple[HexCoordinate, HexCoordinate]]:
        pending_movements: List[Tuple[HexCoordinate, HexCoordinate]] = []

        for plan in self.flightplans:
            if plan.is_present_at_time(time - 1) and plan.is_present_at_time(time):
                pending_movements.append(
                    (plan.position_at_time(time - 1), plan.position_at_time(time))
                )

        return pending_movements

    def jumping_can_occur(self):
        return self.drone_radius_hex == 1

    def list_of_occupied_hexes(self, time: int, render_mode: bool = False) -> List[HexCoordinate]:
        occupied_hexes: List[HexCoordinate] = []

        for plan in self.flightplans:
            if plan.is_present_at_time(time):
                radius = self.drone_radius_hex * 2 - 1 if not render_mode else self.drone_radius_hex
                hexes_covered_by_flightplan = [x for x in self.map.spiral(plan.position_at_time(time), radius) if self.map.is_feasible_coordinate(x)]
                occupied_hexes.extend(hexes_covered_by_flightplan)

        for obstacle in self.obstacles:
            radius = self.drone_radius_hex if not render_mode else 1
            hexes_covered_by_obstacle = [x for x in self.map.spiral(obstacle, radius) if
                                           self.map.is_feasible_coordinate(x)]
            occupied_hexes.extend(hexes_covered_by_obstacle)

        return occupied_hexes

    def resolve_all(self) -> List[Flightplan]:
        for i, request in enumerate(self.requests):
            new_flightplan = self.resolve_request(request)
            self.flightplans.append(new_flightplan)
            print('{} out of {}'.format(i, len(self.requests)))

        return self.flightplans

    def resolve_request(self, request: Request) -> Flightplan:
        path = self._find_shortest_path(request)

        points = []
        if self.map.int_time(path[0]) != request.start_time:
            for i in range(self.map.int_time(path[0]) - request.start_time):
                points.append((i + request.start_time, self.map.int_to_coord(path[0])))

        for i, node_id in enumerate(path):
            points.append((self.map.int_time(node_id), self.map.int_to_coord(node_id)))

        new_flightplan = Flightplan(points=points)

        return new_flightplan

    def get_data_as_dict(self):
        data = {
            'map': {
                'width': self.map.width,
                'height': self.map.height
            },
            'drones_radius': self.drone_radius_hex,
            'flightplans': [],
            'occupied_hexes': {},
            'obstacles': [(x.x, x.y) for x in self.obstacles]
        }

        min_time = math.inf
        max_time = -math.inf

        for flightplan in self.flightplans:
            flightplan_points = [(time, point.x, point.y) for (time, point) in flightplan.points.items()]

            data['flightplans'].append(flightplan_points)

            flightplan_min_time = min([x[0] for x in flightplan_points])
            flightplan_max_time = max([x[0] for x in flightplan_points])

            if flightplan_min_time < min_time:
                min_time = flightplan_min_time

            if flightplan_max_time > max_time:
                max_time = flightplan_max_time

        for time in range(min_time, max_time + 1):
            data['occupied_hexes'][time] = [(point.x, point.y) for point in
                                            self.list_of_occupied_hexes(time, render_mode=True)]

        if self.city_map is not None:
            data['heights'] = self.city_map.heights_map.tolist()
            data['size'] = self.city_map.hexagonal_map_size()
            data['hex_r_px'] = self.city_map.hex_radius_in_px

        return data


class HexHelper:
    @staticmethod
    def cube_to_axial(x, y, z) -> Tuple[float, float]:
        q = x
        r = z
        return (q, r)

    @staticmethod
    def axial_to_cube(q: float, r: float) -> Tuple[float, float, float]:
        x = q
        z = r
        y = -x - z
        return (x, y, z)

    @staticmethod
    def doublewidth_to_cube(col: float, row: float) -> Tuple[float, float, float]:
        x = (col - row) / 2
        z = row
        y = -x - z
        return (x, y, z)

    @staticmethod
    def cube_to_doublewidth(x: float, y: float, z: float) -> Tuple[float, float]:
        col = 2 * x + z
        row = z
        return (col, row)

    @staticmethod
    def cube_round(x, y, z):
        rx = round(x)
        ry = round(y)
        rz = round(z)

        x_diff = abs(rx - x)
        y_diff = abs(ry - y)
        z_diff = abs(rz - z)

        if x_diff > y_diff and x_diff > z_diff:
            rx = -ry - rz
        elif y_diff > z_diff:
            ry = -rx - rz
        else:
            rz = -rx - ry

        return (rx, ry, rz)

    @staticmethod
    def _lerp(a, b, t):  # for floats
        return a + (b - a) * t

    @classmethod
    def _cube_lerp(cls, a, b, t):  # for hexes
        return (cls._lerp(a[0] + 1e-6, b[0], t),
                cls._lerp(a[1] + 2e-6, b[1], t),
                cls._lerp(a[2] - 3e-6, b[2], t))

    @classmethod
    def line_drawing(cls, c1: HexCoordinate, c2: HexCoordinate) -> List[HexCoordinate]:
        n = cls.hex_distance(c1, c2)

        results = []
        if n > 0:
            for i in range(n + 1):
                point = cls._cube_lerp(c1.to_cube(), c2.to_cube(), 1.0 / n * i)
                coord = HexCoordinate.from_cube(cls.cube_round(*point))
                results.append(coord)
        else:
            results = [c1, c2]

        return results

    @staticmethod
    def hex_distance(point1: HexCoordinate, point2: HexCoordinate) -> int:
        dx = abs(point1.x - point2.x)
        dy = abs(point1.y - point2.y)
        return round(dy + max(0, (dx - dy) / 2))


class CityMap:
    # Maybe add support for hex < pixel
    def __init__(self, heights: np.array, pixel_size_m: float, hex_radius_m: float, flight_height: float):
        self.heights_map: np.array = heights
        self.pixel_size_m = pixel_size_m
        self.hex_radius_m = hex_radius_m
        self.flight_height = flight_height

        if self.pixel_size_m > 4*self.hex_radius_m:
            raise NotImplementedError("This implementation does not support the case where hexagons are smaller than pixels. Hexagons should have radius at least 4 times pixel size")

    @property
    def hex_radius_in_px(self) -> float:
        return self.hex_radius_m/self.pixel_size_m

    def _axial_hex_round(self, q: float, r: float) -> HexCoordinate:
        cube = HexHelper.axial_to_cube(q, r)
        cube_rounded = HexHelper.cube_round(*cube)
        x, y = HexHelper.cube_to_doublewidth(*cube_rounded)

        if abs(x - int(x)) > 0.00001 or abs(y - int(y)) > 0.00001:
            raise Exception("Error rounding coordinates")

        return HexCoordinate(int(x), int(y))

    def hexagonal_map_size(self) -> Tuple[int, int]:
        map_width_m = self.heights_map.shape[1] * self.pixel_size_m
        map_height_m = self.heights_map.shape[0] * self.pixel_size_m

        hex_width = math.sqrt(3) * self.hex_radius_m
        hex_height = 2 * self.hex_radius_m

        # TODO: if odd or even then add +1
        map_width_in_hex = math.ceil(map_width_m/hex_width) + 1 # +1 to have extra hex on both sides
        map_height_in_hex = math.ceil(map_height_m/(hex_height*3/4)) + 1 # 3/4 because horizontal spacing between two hexagons is 3/4 h

        return (
            map_width_in_hex,
            map_height_in_hex
        )

    def coord_to_hex(self, x: int, y: int) -> HexCoordinate:
        q = (math.sqrt(3) / 3 * x - 1. / 3 * y) / self.hex_radius_in_px
        r = (2. / 3 * y) / self.hex_radius_in_px

        return self._axial_hex_round(q, r)

    def obstacles(self) -> List[HexCoordinate]:
        obstacles_list = set()

        for x in range(self.heights_map.shape[1]):
            for y in range(self.heights_map.shape[0]):
                if self.heights_map[y, x] > self.flight_height:
                    obstacles_list.add(self.coord_to_hex(x, y))

        return list(obstacles_list)

    def requests_to_hex_grid(self, requests: List) -> List[Request]:
        converted_requests = []

        for request in requests:
            coord_from = self.coord_to_hex(request['from'][0], request['from'][1])
            coord_to = self.coord_to_hex(request['to'][0], request['to'][1])
            converted_requests.append(Request(coord_from, coord_to, request['start_time']))

        return converted_requests


def run_ny_map():
    requests_fname = 'ny_1_requests_1'
    heights_fname = 'ny_1_heights'
    map_details_fname = 'ny_1_details'

    with open('./data/{}.json'.format(requests_fname), 'r') as requests_f, open('./data/{}.json'.format(heights_fname), 'r') as heights_f, open('./data/{}.json'.format(map_details_fname), 'r') as map_f:
        map_data = simplejson.load(map_f)
        heights_raw = np.array(simplejson.load(heights_f))
        city_map = CityMap(heights=heights_raw, pixel_size_m=map_data["pixel_size_m"], hex_radius_m=map_data["hex_radius_m"], flight_height=map_data["flight_height"])

        width = city_map.hexagonal_map_size()[0]
        height = city_map.hexagonal_map_size()[1]

        data = simplejson.load(requests_f)
        requests_data = data['requests']

        requests = city_map.requests_to_hex_grid(requests_data)

        obstacles = city_map.obstacles()
        planner = PathPlanner(obstacles=obstacles, requests=requests, map_width=width, map_height=height,
                              drone_radius_m=1, hex_radius_m=1,
                              gdp=GDP(max_time=data['gdp']['max_time'], penalty=data['gdp']['penalty']), city_map=city_map)
        planner.resolve_all()

        data = planner.get_data_as_dict()

        with open('./results/ny_results.json', 'w') as wf:
            simplejson.dump(data, wf)


def run_simple():
    fname = 'requests'
    fname = 'requests_towards_each_other'
    fname = 'requests_towards_each_other_radius'
    # fname = 'requests_towards_radius_gdp'
    # fname = 'requests_towards_radius_gdp_longer'
    # fname = 'requests_worst_case'

    with open('./data/{}.json'.format(fname), 'r') as f:
        width = 7
        height = 7

        data = simplejson.load(f)
        requests_data = data['requests']

        requests = [Request(x['from'], x['to'], x['start_time']) for x in requests_data]
        # requests = []
        # for i in range(20):
        #     requests.append(Request((0, 4), (12, 4), i))
        #
        # for i in range(20):
        #     requests.append(Request((4, 0), (4, 6), i))
        #     requests.append(Request((6, 0), (6, 6), i))
        #     # requests.append(Request((8, 0), (8, 6), i))
        #     # requests.append(Request((2, 0), (2, 6), i))
        #
        # # random.shuffle(requests)

        obstacles_data = data['obstacles']
        obstacles = [HexCoordinate(x=x[0], y=x[1]) for x in obstacles_data]
        planner = PathPlanner(obstacles=obstacles, requests=requests, map_width=width, map_height=height,
                              drone_radius_m=data['radius'], hex_radius_m=1,
                              gdp=GDP(max_time=data['gdp']['max_time'], penalty=data['gdp']['penalty']))
        planner.resolve_all()

        data = planner.get_data_as_dict()

        with open('./results/results.json', 'w') as wf:
            simplejson.dump(data, wf)


def main():
    run_ny_map()


if __name__ == '__main__':
    main()
