import functools
import itertools
import time
from dataclasses import dataclass
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
        return time >= self.start_time and time <= self.end_time

    def position_at_time(self, time: int) -> "HexCoordinate":
        if not self.is_present_at_time(time):
            raise Exception('Not present at time {}'.format(time))

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
    # We are using double-width horizontal layout as specified in https://www.redblobgames.com/grids/hexagons/
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height

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
        offsets = [
            [-1, -1],
            [1, -1],
            [2, 0],
            [-2, 0],
            [-1, 1],
            [1, 1],
        ]

        neighbours: List[HexCoordinate] = []

        for (x_offset, y_offset) in offsets:
            if self._is_feasible_coordinate(coord.x + x_offset, coord.y + y_offset):
                neighbours.append(HexCoordinate(coord.x + x_offset, coord.y + y_offset))

        return neighbours

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


class PathPlanner:
    TIME_MULTIPLIER = 10 # how many layers to create in the graph

    def __init__(self, obstacles: List[HexCoordinate], requests: List[Request], map_width, map_height):
        self.flightplans: List[Flightplan] = []
        self.obstacles = obstacles
        self.requests = requests
        self.map = HexMap(width=map_width, height=map_height)

    def _build_graph(self, time_from: int, time_to: int, request: Request) -> (nx.DiGraph, int, int):
        # create nodes and add edges in one pass
        # TODO: make punishment for ground delay different than for hovering
        number_of_layers = time_to - time_from
        graph = nx.DiGraph()

        all_nodes = range(self.map.number_of_nodes * number_of_layers + 2)
        graph.add_nodes_from(all_nodes)

        destination_node_id = self.map.coord_to_int(request.end_point)
        start_node_id = self.map.coord_to_int(request.start_point)

        edges = []

        for i in range(number_of_layers-1):
            time = time_from + i

            occupied = self.list_of_occupied_hexes(time + 1)

            for node_id in range(self.map.number_of_nodes):
                node_neighbours = self.map.get_neighbours(self.map.int_to_coord(node_id))
                node_neighbours_ids = [self.map.coord_to_int(x) for x in node_neighbours]

                for (neigh_i, node_neighbour_id) in enumerate(node_neighbours_ids):
                    if node_neighbours[neigh_i] not in occupied:
                        edges.append((node_id + self.map.number_of_nodes*i, node_neighbour_id + self.map.number_of_nodes*(i+1)))

                edges.append((node_id + self.map.number_of_nodes*i, node_id + self.map.number_of_nodes*(i+1)))

            edges.append((destination_node_id + self.map.number_of_nodes*i, all_nodes[-1]))

        edges.append((all_nodes[-2], start_node_id))

        graph.add_edges_from(edges)

        return graph, all_nodes[-2], all_nodes[-1]

    def list_of_occupied_hexes(self, time: int) -> List[HexCoordinate]:
        # TODO: reserve the next hex as well to avoid diving issue
        occupied_hexes: List[HexCoordinate] = []

        for plan in self.flightplans:
            if plan.is_present_at_time(time):
                occupied_hexes.append(plan.position_at_time(time))

        for obstacle in self.obstacles:
            occupied_hexes.append(obstacle)

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
        for i, node_id in enumerate(path[1:-1]):
            points.append((i+request.start_time, self.map.int_to_coord(node_id)))

        new_flightplan = Flightplan(points=points)

        return new_flightplan


def benchmark_graph_libraries():
    start_time = time.time()

    map_size = [100, 100]
    number_of_layers = 300

    # g = Graph(directed=True)
    # g.add_vertices(map_size[0]*map_size[1]*number_of_layers)
    G = nx.DiGraph()

    G.add_nodes_from(range(map_size[0] * map_size[1] * number_of_layers))

    edges = []

    for i in range(number_of_layers - 2):
        print(i)
        for t in range(map_size[0] * map_size[1]):
            id1 = i * map_size[0] * map_size[1] + t

            edges.append([id1, (i + 1) * map_size[0] * map_size[1] + t + 1])
            edges.append([id1, (i + 1) * map_size[0] * map_size[1] + t + 2])
            edges.append([id1, (i + 1) * map_size[0] * map_size[1] + t + 3])
            edges.append([id1, (i + 1) * map_size[0] * map_size[1] + t + 4])
            edges.append([id1, (i + 1) * map_size[0] * map_size[1] + t + 5])
            edges.append([id1, (i + 1) * map_size[0] * map_size[1] + t + 6])

    G.add_edges_from(edges)
    # g.add_edges(edges)
    print("--- %s seconds ---" % (time.time() - start_time))
    print('aaa')


def main():
    with open('./data/requests.json', 'r') as f:
        data = simplejson.load(f)
        requests_data = data['requests']
        requests = [Request(x['from'], x['to'], x['start_time']) for x in requests_data]
        obstacles_data = data['obstacles']
        obstacles = [HexCoordinate(x=x[0], y=x[1]) for x in obstacles_data]
        planner = PathPlanner(obstacles=obstacles, requests=requests, map_width=7, map_height=7)
        flightplans = planner.resolve_all()

        data = {
            'map': {
                'width': 7,
                'height': 7
            },
            'flightplans': [],
            'obstacles': obstacles_data
        }

        for flightplan in flightplans:
            data['flightplans'].append([(time, point.x, point.y) for (time, point) in flightplan.points.items()])

        with open('./results/results.json', 'w') as wf:
            simplejson.dump(data, wf)


if __name__ == '__main__':
    main()
