import functools
import itertools
import queue
import random
import time
from dataclasses import dataclass
from math import ceil
from typing import List, Dict, Tuple

import simplejson
from igraph import *
import networkx as nx


DEBUG = False


@dataclass
class Request:
    start_point: "HexCoordinate"
    end_point: "HexCoordinate"
    start_time: int

    def __init__(self, start: Tuple[int, int], end: Tuple[int, int], start_time: int):
        self.start_point = HexCoordinate(x=start[0], y=start[1])
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


@functools.total_ordering
class HexCoordinate:
    def __init__(self, x: int, y: int):
        if (x % 2 == 0) != (y % 2 == 0):
            raise Exception('Wrong hex coordinate!')

        self.x = x
        self.y = y

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

    @staticmethod
    def distance_between_points(point1: HexCoordinate, point2: HexCoordinate) -> int:
        dx = abs(point1.x - point2.x)
        dy = abs(point1.y - point2.y)
        return round(dy + max(0, (dx - dy) / 2))

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

        for i in range(6):
            for j in range(r):
                results.append(cube)
                cube = HexCoordinate(cube.x + self.DIRECTIONS[i][0], cube.y + self.DIRECTIONS[i][1])

        return results


class PathPlanner:
    TIME_MULTIPLIER = 40 # obsolete since implementation of Dijkstra without building the whole graph
    SOURCE_NODE_ID = -1
    SINK_NODE_ID = -2

    def __init__(self, obstacles: List[HexCoordinate], requests: List[Request], map_width, map_height,
                 hex_radius_m, drone_radius_m, gdp):
        self.flightplans: List[Flightplan] = []
        self.obstacles = obstacles
        self.requests = requests
        self.map = HexMap(width=map_width, height=map_height)
        self.hex_radius_m = hex_radius_m
        self.drone_radius_m = drone_radius_m
        self.drone_radius_hex = ceil(drone_radius_m/hex_radius_m)
        self.gdp = gdp

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

    def _build_graph(self, time_from: int, time_to: int, request: Request) -> (nx.DiGraph, int, int):
        # create nodes and add edges in one pass
        number_of_layers = time_to - time_from
        graph = nx.DiGraph()

        all_nodes = range(self.map.number_of_nodes * number_of_layers + 2)
        graph.add_nodes_from(all_nodes)

        destination_node_id = self.map.coord_to_int(request.end_point)
        start_node_id = self.map.coord_to_int(request.start_point)

        edges = []

        source = all_nodes[-2]
        sink = all_nodes[-1]
        for i in range(number_of_layers-1):
            time = time_from + i

            occupied = self.list_of_occupied_hexes(time + 1)

            if self.jumping_can_occur():
                drones_movements = self.list_of_pending_drone_movements(time + 1)

            for node_id in range(self.map.number_of_nodes):
                node_coord = self.map.int_to_coord(node_id)
                node_neighbours = self.map.get_neighbours(node_coord)
                node_neighbours_ids = [self.map.coord_to_int(x) for x in node_neighbours]

                for (neigh_i, node_neighbour_id) in enumerate(node_neighbours_ids):
                    if node_neighbours[neigh_i] not in occupied:
                        if not (self.jumping_can_occur() and (node_neighbours[neigh_i], node_coord) in drones_movements):
                            edges.append(
                                (self.time_ext_node_id(node_id, i), self.time_ext_node_id(node_neighbour_id, i + 1), 1)
                            )

                if node_coord not in occupied:
                    edges.append(
                        (self.time_ext_node_id(node_id, i), self.time_ext_node_id(node_id, i + 1), 1) # hovering
                    )

            edges.append(
                (self.time_ext_node_id(destination_node_id, i), sink, 0) # sink
            )

            if i < self.gdp.max_time and request.start_point not in occupied:
                edges.append(
                    (source, self.time_ext_node_id(start_node_id, i), i*self.gdp.penalty)  # source
                )

        edges.append(
            (source, self.time_ext_node_id(start_node_id, 0), 0) # source
        )

        graph.add_weighted_edges_from(edges)

        return graph, source, sink

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

    def old_resolve_request(self, request: Request) -> Flightplan:
        distance = self.map.distance_between_points(request.start_point, request.end_point)

        graph, start_id, end_id = self._build_graph(request.start_time, request.start_time + self.TIME_MULTIPLIER*distance, request)
        path = nx.shortest_path(graph, start_id, end_id)

        points = []
        if self.map.int_time(path[1]) != 0:
            for i in range(self.map.int_time(path[1])):
                points.append((i + request.start_time, self.map.int_to_coord(path[1])))

        for i, node_id in enumerate(path[1:-1]):
            points.append((self.map.int_time(node_id) + request.start_time, self.map.int_to_coord(node_id)))

        new_flightplan = Flightplan(points=points)

        return new_flightplan

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

        return data


def main():
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


if __name__ == '__main__':
    main()
