import functools
import itertools
import time
from dataclasses import dataclass
from math import ceil
from typing import List, Dict, Tuple

import simplejson
from igraph import *
import networkx as nx



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
    def __init__(self, points: List[Tuple[int, "HexCoordinate"]]):
        self.start_time = min([x[0] for x in points])
        self.end_time = max([x[0] for x in points])

        self.points: Dict[(int, HexCoordinate)] = {}
        self._fill_points(points)

    def _fill_points(self, points: List[Tuple[int, "HexCoordinate"]]):
        for (time, point) in points:
            self.points[time] = point

    def is_present_at_time(self, time: int) -> bool:
        return time >= self.start_time and time <= self.end_time + 1

    def position_at_time(self, time: int) -> "HexCoordinate":
        if not self.is_present_at_time(time):
            raise Exception('Not present at time {}'.format(time))

        if time not in self.points:
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
    TIME_MULTIPLIER = 10 # how many layers to create in the graph

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
                                (self.time_ext_node_id(node_id, time), self.time_ext_node_id(node_neighbour_id, time + 1), 1)
                            )

                if node_coord not in occupied:
                    edges.append(
                        (self.time_ext_node_id(node_id, time), self.time_ext_node_id(node_id, time + 1), 1) # hovering
                    )

            edges.append(
                (self.time_ext_node_id(destination_node_id, time), sink, 0) # sink
            )

            if i < self.gdp.max_time and request.start_point not in occupied:
                edges.append(
                    (source, self.time_ext_node_id(start_node_id, time), i*self.gdp.penalty)  # source
                )

        edges.append(
            (source, self.time_ext_node_id(start_node_id, time_from), 0) # source
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
        # TODO: reserve the next hex as well to avoid diving issue
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
        for request in self.requests:
            new_flightplan = self.resolve_request(request)
            self.flightplans.append(new_flightplan)

        return self.flightplans

    def resolve_request(self, request: Request) -> Flightplan:
        distance = self.map.distance_between_points(request.start_point, request.end_point)

        graph, start_id, end_id = self._build_graph(request.start_time, request.start_time + self.TIME_MULTIPLIER*distance, request)
        path = nx.shortest_path(graph, start_id, end_id)

        points = []
        if self.map.int_time(path[1]) != request.start_time:
            for i in range(self.map.int_time(path[1]) - request.start_time):
                points.append((i + request.start_time, self.map.int_to_coord(path[1])))

        for i, node_id in enumerate(path[1:-1]):
            points.append((self.map.int_time(node_id), self.map.int_to_coord(node_id)))

        new_flightplan = Flightplan(points=points)

        return new_flightplan


def main():
    fname = 'requests'
    fname = 'requests_towards_each_other'
    fname = 'requests_towards_each_other_radius'
    fname = 'requests_towards_radius_gdp'
    fname = 'requests_towards_radius_gdp_longer'

    with open('./data/{}.json'.format(fname), 'r') as f:
        data = simplejson.load(f)
        requests_data = data['requests']
        requests = [Request(x['from'], x['to'], x['start_time']) for x in requests_data]
        obstacles_data = data['obstacles']
        obstacles = [HexCoordinate(x=x[0], y=x[1]) for x in obstacles_data]
        planner = PathPlanner(obstacles=obstacles, requests=requests, map_width=7, map_height=7,
                              drone_radius_m=data['radius'], hex_radius_m=1,
                              gdp=GDP(max_time=data['gdp']['max_time'], penalty=data['gdp']['penalty']))
        flightplans = planner.resolve_all()

        data = {
            'map': {
                'width': 7,
                'height': 7
            },
            'drones_radius': planner.drone_radius_hex,
            'flightplans': [],
            'occupied_hexes': {},
            'obstacles': obstacles_data
        }

        min_time = math.inf
        max_time = -math.inf

        for flightplan in flightplans:
            flightplan_points = [(time, point.x, point.y) for (time, point) in flightplan.points.items()]

            data['flightplans'].append(flightplan_points)

            flightplan_min_time = min([x[0] for x in flightplan_points])
            flightplan_max_time = max([x[0] for x in flightplan_points])

            if flightplan_min_time < min_time:
                min_time = flightplan_min_time

            if flightplan_max_time > max_time:
                max_time = flightplan_max_time

        for time in range(min_time, max_time + 1):
            data['occupied_hexes'][time] = [(point.x, point.y) for point in planner.list_of_occupied_hexes(time, render_mode=True)]

        with open('./results/results.json', 'w') as wf:
            simplejson.dump(data, wf)


if __name__ == '__main__':
    main()
