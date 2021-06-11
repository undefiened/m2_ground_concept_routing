import functools
import itertools
import queue
import random
import time
from dataclasses import dataclass
from math import ceil
from typing import List, Dict, Tuple, Set, Union, Optional

import simplejson
from igraph import *
import networkx as nx
import numpy as np

from misc_functions import _is_feasible_coordinate, _list_of_occupied_by_obstacles_hexes, HexCoordinate, _hex_distance

DEBUG = False


class PathNotFoundException(Exception):
    """Raises if there is no path for the request"""


@dataclass
class Request:
    start_point: "HexCoordinate"
    end_point: "HexCoordinate"
    start_time: int
    drone_radius_m: Optional[float]
    time_uncertainty: int
    speed_hex: float

    def __init__(self, start: Union[Tuple[int, int], "HexCoordinate"], end: Union[Tuple[int, int], "HexCoordinate"], start_time: int, drone_radius_m = None, time_uncertainty=None, speed_hex=1):
        if isinstance(start, HexCoordinate):
            self.start_point = start
        else:
            self.start_point = HexCoordinate(x=start[0], y=start[1])

        if isinstance(end, HexCoordinate):
            self.end_point = end
        else:
            self.end_point = HexCoordinate(x=end[0], y=end[1])

        self.start_time = start_time

        if self.start_time < 0:
            raise Exception("The requested start time cannot be negative")

        self.drone_radius_m = drone_radius_m

        self.time_uncertainty = time_uncertainty

        if speed_hex > 1:
            raise NotImplementedError('Speed higher than 1 hex per unit of time is not yet supported!')

        self.speed_hex = speed_hex

    def is_radius_defined(self) -> bool:
        return self.drone_radius_m is not None

    def has_time_uncertainty(self) -> bool:
        return self.time_uncertainty is not None

    @staticmethod
    def from_dict(dict: Dict) -> "Request":
        kwargs = {}
        if 'radius' in dict:
            kwargs['drone_radius_m'] = dict['radius']

        if 'time_uncertainty' in dict:
            kwargs['time_uncertainty'] = dict['time_uncertainty']

        if 'speed_hex' in dict:
            kwargs['speed_hex'] = dict['speed_hex']

        return Request(dict['from'], dict['to'], dict['start_time'], **kwargs)

    def to_dict(self) -> Dict:
        data = {
            'from': [self.start_point.x, self.start_point.y],
            'to': [self.end_point.x, self.end_point.y],
            'start_time': self.start_time,
        }

        if self.drone_radius_m is not None:
            data['radius'] = self.drone_radius_m

        if self.time_uncertainty is not None:
            data['time_uncertainty'] = self.time_uncertainty

        if self.speed_hex != 1:
            data['speed_hex'] = self.speed_hex

        return data

@dataclass
class GDP:
    max_time: int
    penalty: float


class Flightplan:
    ONE_TICK_TO_LAND = False

    def __init__(self, points: List[Tuple[int, "HexCoordinate"]], radius_hex, time_uncertainty, speed_hex):
        self.start_time = min([x[0] for x in points])
        self._end_time = max([x[0] for x in points])

        self.points: Dict[(int, HexCoordinate)] = {}
        self._fill_points(points)
        self.radius_hex = radius_hex
        self.time_uncertainty = time_uncertainty
        self.speed_hex = speed_hex
        self.time_to_pass_hex = math.ceil(1/self.speed_hex)

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

    def is_present_at_time_with_uncertainty(self, time: int, additional_uncertainty: int) -> bool:
        return time >= self.start_time - additional_uncertainty and time <= self.end_time + self.time_uncertainty

    def position_at_time(self, time: int) -> List["HexCoordinate"]:
        if not self.is_present_at_time(time):
            raise Exception('Not present at time {}'.format(time))

        if self.speed_hex == 1:
            if self.ONE_TICK_TO_LAND and time not in self.points:
                return [self.points[time-1], ]

            return [self.points[time], ]
        else:
            if time in self.points:
                return [self.points[time], ]
            else:
                keys = sorted(self.points.keys())

                for i in range(len(keys)-1):
                    key_1 = keys[i]
                    key_2 = keys[i+1]
                    if key_1 < time < key_2:
                        return [self.points[key_1], self.points[key_2]]

                raise Exception('A point in the path is missing')

    def sorted_points(self) -> List[Tuple[int, "HexCoordinate"]]:
        return sorted(self.points.items(), key=lambda x: x[0])

    def smoothed(self, epsilon=1) -> List[Tuple[int, "HexCoordinate"]]:
        smoothed1 = self._basic_smoothing()
        smoothed2 = self._rdp_smoothing(epsilon=epsilon)

        if len(smoothed1) < len(smoothed2):
            return smoothed1
        elif len(smoothed2) < len(smoothed1):
            return smoothed2

        return smoothed2

    @staticmethod
    def _distance_to_line(external_point, line_point_1, line_point_2) -> float:
        x0, y0 = external_point[1].get_euclidean_position()
        x1, y1 = line_point_1[1].get_euclidean_position()
        x2, y2 = line_point_2[1].get_euclidean_position()

        return abs((x2 - x1)*(y1 - y0) - (x1 - x0)*(y2 - y1))/math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    @classmethod
    def _douglas_peucker(cls, points: List[Tuple[int, "HexCoordinate"]], epsilon: float) -> List[Tuple[int, HexCoordinate]]:
        dmax = 0
        index = 0

        for i in range(1, len(points) - 1):
            d = cls._distance_to_line(points[i], points[0], points[-1])
            if d > dmax:
                index = i
                dmax = d

        if dmax > epsilon:
            rec_results_1 = cls._douglas_peucker(points[:index], epsilon)
            rec_results_2 = cls._douglas_peucker(points[index:], epsilon)
            rec_results_1.pop()
            rec_results_1.extend(rec_results_2)

            return rec_results_1
        else:
            return [points[0], points[-1]]

    def _rdp_smoothing(self, epsilon: float) -> List[Tuple[int, HexCoordinate]]:
        # Ramer–Douglas–Peucker smoothing, see https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm
        orig_points = self.sorted_points()
        return self._douglas_peucker(orig_points, epsilon)

    def _basic_smoothing(self) -> List[Tuple[int, "HexCoordinate"]]:
        smoothed_points: List[Tuple[int, "HexCoordinate"]] = []

        orig_points = self.sorted_points()

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

    @functools.lru_cache(100000)
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
        if coord.x < 0 or coord.y < 0:
            raise Exception("Coordinates cannot be negative")

        if coord.x % 2 == 0 and coord.y % 2 == 0:
            return round(coord.y * self.width + coord.x / 2)
        else:
            return round(coord.y * self.width + math.floor(coord.x / 2))

    @functools.lru_cache(10000)
    def get_neighbours(self, coord: HexCoordinate) -> List[HexCoordinate]:
        neighbours: List[HexCoordinate] = []

        for (x_offset, y_offset) in self.DIRECTIONS:
            if _is_feasible_coordinate(coord.x + x_offset, coord.y + y_offset, self.height, self.width):
                neighbours.append(HexCoordinate(coord.x + x_offset, coord.y + y_offset))

        return neighbours

    def is_feasible_coordinate(self, coord: HexCoordinate) -> bool:
        return _is_feasible_coordinate(coord.x, coord.y, self.height, self.width)

    def _is_feasible_coordinate(self, x: int, y: int) -> bool:
        return _is_feasible_coordinate(x, y, self.height, self.width)

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

    DEVIATION_PENALTY_DISTANCE_MULTIPLIER = 0.000001

    def __init__(self, obstacles: List[HexCoordinate], map_width, map_height,
                 hex_radius_m, default_drone_radius_m, gdp, city_map=None, punish_deviation=False):
        self.flightplans: List[Flightplan] = []
        self._obstacles = obstacles
        self.requests: List[Request] = []
        self._map = HexMap(width=map_width, height=map_height)
        self.hex_radius_m = hex_radius_m
        self.default_drone_radius_m = default_drone_radius_m
        self.default_drone_radius_hex = self._radius_m_to_hex(default_drone_radius_m)
        self.gdp = gdp
        self.punish_deviation = punish_deviation
        self.city_map = city_map

    @property
    def map(self) -> HexMap:
        return self._map

    @property
    def obstacles(self) -> List[HexCoordinate]:
        return self._obstacles

    # @obstacles.setter
    # def obstacles(self, obstacles: List[HexCoordinate]):
    #     self._obstacles = obstacles
    #     self._list_of_occupied_by_obstacles_hexes_cython.cache_clear()

    def _radius_m_to_hex(self, radius_m: float) -> int:
        return ceil(radius_m / self.hex_radius_m)

    def _request_radius_hex(self, request: Request) -> int:
        if request.is_radius_defined():
            return self._radius_m_to_hex(request.drone_radius_m)
        else:
            return self.default_drone_radius_hex

    def _request_time_uncertainty(self, request: Request) -> int:
        if request.has_time_uncertainty():
            return request.time_uncertainty
        else:
            return 0

    def _request_time_to_pass_hex(self, request: Request) -> int:
        return math.ceil(1/request.speed_hex)

    def _get_deviation_penalty(self, to_coord: HexCoordinate, request: Request):
        if not self.punish_deviation:
            return 0

        # line = HexHelper.line_drawing(request.start_point, request.end_point)
        #
        # min_dist = min([HexHelper.hex_distance(point, to_coord) for point in line])
        #
        # return min_dist*self.DEVIATION_PENALTY_DISTANCE_MULTIPLIER

        x0, y0 = to_coord.get_euclidean_position()
        x1, y1 = request.start_point.get_euclidean_position()
        x2, y2 = request.end_point.get_euclidean_position()

        return (abs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1)) / math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2))*self.DEVIATION_PENALTY_DISTANCE_MULTIPLIER

    def _get_all_neighbours(self, coordinate: HexCoordinate, time: int, request: Request) -> List[Tuple[int, float]]:
        viable_neighbours_ids: List[Tuple[int, float]] = []

        occupied = self._list_of_occupied_hexes_for_request(time + self._request_time_to_pass_hex(request), request)
        if self.jumping_can_occur():
            drones_movements = self.list_of_pending_drone_movements(time + self._request_time_to_pass_hex(request))

        neighbours = self._map.get_neighbours(coordinate)
        neighbours_ids = [self._map.coord_to_int(x) for x in neighbours]

        for (neigh_i, node_neighbour_id) in enumerate(neighbours_ids):
            if neighbours[neigh_i] not in occupied:
                if not (self.jumping_can_occur() and (neighbours[neigh_i], coordinate) in drones_movements):
                    viable_neighbours_ids.append(
                        (self.time_ext_node_id(node_neighbour_id, time + self._request_time_to_pass_hex(request)), 1 + self._get_deviation_penalty(self._map.int_to_coord(node_neighbour_id), request))
                    )

        if coordinate not in occupied:
            viable_neighbours_ids.append(
                (self.time_ext_node_id(self._map.coord_to_int(coordinate), time + 1), 1)  # hovering
            )

        if coordinate == request.end_point:
            viable_neighbours_ids.append(
                (self.SINK_NODE_ID, 0)  # sink
            )

        return viable_neighbours_ids

    def _is_destination_reachable(self, request: Request) -> bool:
        G = nx.Graph()

        G.add_nodes_from(range(self._map.number_of_nodes))

        start_point_id = self._map.coord_to_int(request.start_point)
        end_point_id = self._map.coord_to_int(request.end_point)
        edges = []

        occupied = self._list_of_occupied_by_obstacles_hexes(0, self._request_radius_hex(request)-1)

        for i in range(self._map.number_of_nodes):
            coord = self._map.int_to_coord(i)
            if coord not in occupied:
                neighbours = self._map.get_neighbours(self._map.int_to_coord(i))

                for neighbour_coord in neighbours:
                    neighbour_id = self._map.coord_to_int(neighbour_coord)
                    if neighbour_coord not in occupied:
                        edges.append((i, neighbour_id))

        G.add_edges_from(edges)

        return nx.has_path(G, start_point_id, end_point_id)

    def _find_shortest_path(self, request: Request) -> List[int]:
        self._visited_nodes = set()
        self._nodes_weights = {}
        self._queue = queue.PriorityQueue()
        self._previous_nodes = {}

        # self._queue.put((0, self.SOURCE_NODE_ID))

        for i in range(self.gdp.max_time+1):
            weight = i*self.gdp.penalty
            occupied = self._list_of_occupied_hexes_for_request(request.start_time + i, request)
            coord = self.time_ext_node_id(self._map.coord_to_int(request.start_point), request.start_time + i)

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
                current_node_coord = self._map.int_to_coord(current_node_id)
                current_node_time = self._map.int_time(current_node_id)

                neighbours = self._get_all_neighbours(current_node_coord, current_node_time, request)

                for (neighbour_id, weight) in neighbours:
                    if neighbour_id not in self._nodes_weights or weight + current_node_weight < self._nodes_weights[neighbour_id]:
                        self._nodes_weights[neighbour_id] = weight + current_node_weight
                        self._previous_nodes[neighbour_id] = current_node_id
                        self._queue.put((weight + current_node_weight, neighbour_id))

                self._visited_nodes.add(current_node_id)

        if self.SINK_NODE_ID not in self._previous_nodes:
            raise PathNotFoundException("Cannot find a path for request {}".format(request))

        path = []
        previous_node = self._previous_nodes[self.SINK_NODE_ID]

        while previous_node != self.SOURCE_NODE_ID:
            path.append(previous_node)
            previous_node = self._previous_nodes[previous_node]

        path.reverse()

        return path

    def time_ext_node_id(self, node_id: int, time: int) -> int:
        return node_id + self._map.number_of_nodes * time

    def list_of_pending_drone_movements(self, time: int) -> List[Tuple[HexCoordinate, HexCoordinate]]:
        pending_movements: List[Tuple[HexCoordinate, HexCoordinate]] = []

        for plan in self.flightplans:
            if plan.is_present_at_time(time - 1) and plan.is_present_at_time(time):
                pending_movements.append(
                    (plan.position_at_time(time - 1)[0], plan.position_at_time(time)[0])
                )

        return pending_movements

    @functools.lru_cache(10)
    def jumping_can_occur(self):
        radii_hex = [self._request_radius_hex(request) for request in self.requests]
        return radii_hex.count(1) > 1

    def list_of_occupied_hexes(self, time: int) -> Set[HexCoordinate]:
        return self._list_of_occupied_hexes(time, 0, 0)

    def _list_of_occupied_hexes_for_request(self, time: int, request: Request) -> Set[HexCoordinate]:
        return self._list_of_occupied_hexes(time, self._request_radius_hex(request)-1, self._request_time_uncertainty(request))

    def _list_of_occupied_by_drones_hexes(self, time: int, additional_radius: int, additional_time_uncertainty: int) -> Set[HexCoordinate]:
        occupied_hexes = set()

        for plan in self.flightplans:
            # for time_uncertainty in range(max(-additional_time_uncertainty, time - plan.end_time), min(plan.time_uncertainty + 1, time - plan.start_time)):
            for time_uncertainty in range(max(-additional_time_uncertainty, time - plan.end_time), min(plan.time_uncertainty + 1, time - plan.start_time + 1)):
                if plan.is_present_at_time(time):
                    radius = plan.radius_hex + additional_radius
                    for position in plan.position_at_time(time - time_uncertainty):
                        hexes_covered_by_flightplan = [x for x in self._map.spiral(position, radius) if
                                                       self._map.is_feasible_coordinate(x)]
                        occupied_hexes.update(hexes_covered_by_flightplan)

        return occupied_hexes

    def _list_of_occupied_by_obstacles_hexes(self, time: int, additional_radius: int) -> Set[HexCoordinate]:
        # return self._list_of_occupied_by_obstacles_hexes_native(time, additional_radius)
        return self._list_of_occupied_by_obstacles_hexes_cython(time, additional_radius)

    @functools.lru_cache(maxsize=None)
    def _list_of_occupied_by_obstacles_hexes_cython(self, time: int, additional_radius: int) -> Set[HexCoordinate]:
        occupied_hexes = _list_of_occupied_by_obstacles_hexes(time, additional_radius, self._obstacles, self._map)
        return occupied_hexes

    def _list_of_occupied_by_obstacles_hexes_native(self, time: int, additional_radius: int) -> Set[HexCoordinate]:
        occupied_hexes = set()

        for obstacle in self._obstacles:
            radius = additional_radius + 1
            hexes_covered_by_obstacle = [x for x in self._map.spiral(obstacle, radius) if
                                         self._map.is_feasible_coordinate(x)]
            occupied_hexes.update(hexes_covered_by_obstacle)

        return occupied_hexes

    def _list_of_occupied_hexes(self, time: int, additional_radius: int, additional_time_uncertainty: int) -> Set[HexCoordinate]:
        occupied_hexes: Set[HexCoordinate] = set()

        occupied_hexes.update(self._list_of_occupied_by_drones_hexes(time, additional_radius, additional_time_uncertainty))

        occupied_hexes.update(self._list_of_occupied_by_obstacles_hexes(time, additional_radius))

        return occupied_hexes

    def resolve_requests(self, requests: List[Request]) -> List[Flightplan]:
        for i, request in enumerate(requests):
            self.resolve_request(request)
            print('{} out of {}'.format(i+1, len(requests)))

        return self.flightplans

    def resolve_request(self, request: Request) -> Flightplan:
        self.requests.append(request)
        self.jumping_can_occur.cache_clear()
        if not self._is_destination_reachable(request):
            raise PathNotFoundException("The destination is not reachable for request {}".format(request))

        path = self._find_shortest_path(request)

        points = []
        if self._map.int_time(path[0]) != request.start_time:
            for i in range(self._map.int_time(path[0]) - request.start_time):
                points.append((i + request.start_time, self._map.int_to_coord(path[0])))

        for i, node_id in enumerate(path):
            points.append((self._map.int_time(node_id), self._map.int_to_coord(node_id)))

        new_flightplan = Flightplan(points=points, radius_hex=self._request_radius_hex(request), time_uncertainty=self._request_time_uncertainty(request), speed_hex=request.speed_hex)

        self.flightplans.append(new_flightplan)
        return new_flightplan

    def get_data_as_dict(self):
        data = {
            'map': {
                'width': self._map.width,
                'height': self._map.height
            },
            'drones_radius': self.default_drone_radius_hex,
            'flightplans': [],
            'occupied_hexes': {},
            'obstacles': [(x.x, x.y) for x in self._obstacles],
            'smoothed_flightplans': [],
        }

        min_time = math.inf
        max_time = -math.inf

        for flightplan in self.flightplans:
            flightplan_points = [(time, point.x, point.y) for (time, point) in flightplan.points.items()]

            data['flightplans'].append(flightplan_points)
            data['smoothed_flightplans'].append([(time, point.x, point.y) for (time, point) in flightplan.smoothed()])

            flightplan_min_time = min([x[0] for x in flightplan_points])
            flightplan_max_time = max([x[0] for x in flightplan_points])

            if flightplan_min_time < min_time:
                min_time = flightplan_min_time

            if flightplan_max_time > max_time:
                max_time = flightplan_max_time

        for time in range(min_time, max_time + 1):
            data['occupied_hexes'][time] = [(point.x, point.y) for point in
                                            self.list_of_occupied_hexes(time)]

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
    @functools.lru_cache(1000)
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
        return _hex_distance(point1, point2)


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
    requests_fname = 'ny_1_requests_random_2'
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
        planner = PathPlanner(obstacles=obstacles, map_width=width, map_height=height,
                              default_drone_radius_m=2, hex_radius_m=1,
                              gdp=GDP(max_time=data['gdp']['max_time'], penalty=data['gdp']['penalty']), city_map=city_map, punish_deviation=True)
        planner.resolve_requests(requests)

        data = planner.get_data_as_dict()

        with open('./results/ny_results.json', 'w') as wf:
            simplejson.dump(data, wf)


# def run_artificial():
#     requests_fname = 'ny_1_requests_2'
#     heights_fname = 'artificial_1_heights'
#     map_details_fname = 'ny_1_details'


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
                              default_drone_radius_m=data['radius'], hex_radius_m=1,
                              gdp=GDP(max_time=data['gdp']['max_time'], penalty=data['gdp']['penalty']), punish_deviation=True)
        planner.resolve_requests()

        data = planner.get_data_as_dict()

        with open('./results/results.json', 'w') as wf:
            simplejson.dump(data, wf)


def main():
    run_ny_map()


if __name__ == '__main__':
    main()
