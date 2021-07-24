from abc import ABC, abstractmethod
from dataclasses import dataclass
from functools import lru_cache
from typing import Tuple, List, TYPE_CHECKING

import networkx as nx
import pyclipper
from pyclipper import PyclipperOffset, scale_to_clipper, scale_from_clipper

if TYPE_CHECKING:
    from ground_routing.street_network import SNFlightplan


PCO_SCALING_FACTOR = 10 ** 4


class PathNotFoundException(Exception):
    """Raises if there is no path for the request"""


@dataclass
class GDP:
    time_step: int
    max_time: int
    penalty: float


NO_GDP = GDP(1, 0, 1000)


@dataclass
class Request:
    id: str
    origin: Tuple[float, float]
    destination: Tuple[float, float]
    time_uncertainty_s: int
    speed_m_s: float
    departure_time: int
    uncertainty_radius_m: float
    gdp: GDP

    def __hash__(self):
        return hash(repr(self))


@dataclass
class TurnParams:
    angle_from: float
    angle_to: float
    speed_knot: float
    time_cost: float

    @property
    def penalized(self):
        return self.time_cost > 0


class TurnParamsTable:
    def __init__(self, turn_costs: List[TurnParams]):
        self.turn_params_table = turn_costs
        self.turn_params_table.sort(key=lambda x: x.angle_from)

        self._check_table_consistency()

    @property
    def penalized_turn_angle(self):
        return

    def _check_table_consistency(self):
        if self.turn_params_table[0].angle_from != 0 or self.turn_params_table[-1].angle_to != 180:
            raise Exception('Wrong turn costs!')

        for i in range(len(self.turn_params_table) - 1):
            c1 = self.turn_params_table[i]
            c2 = self.turn_params_table[i + 1]

            if c1.angle_to != c2.angle_from:
                raise Exception('Wrong turn costs!')

    def get_turn_params(self, angle: float) -> TurnParams:
        if angle < 0:
            angle = abs(angle)

        while angle >= 180:
            angle -= 180

        for params in self.turn_params_table:
            if params.angle_from <= angle < params.angle_to:
                return params

    def get_turn_cost_s(self, angle: float) -> float:
        return self.get_turn_params(angle).time_cost

    def get_turn_speed_knots(self, angle: float) -> float:
        return self.get_turn_params(angle).speed_knot

    def is_turn_penalized(self, angle: float) -> float:
        return self.get_turn_params(angle).penalized


class Flightplan:
    @dataclass
    class Waypoint:
        x: float
        y: float
        norm_x: float
        norm_y: float
        time: int
        turning: bool = False

    id: str
    waypoints: List[Waypoint]
    departure_time: int
    destination_time: int
    layer: int

    def __init__(self, id: str, waypoints: List[Waypoint], time_uncertainty_s: int, speed_m_s: float, uncertainty_radius_m: float, layer: int = None):
        self.waypoints = waypoints
        self.departure_time = min([x.time for x in waypoints])
        self.destination_time = max([x.time for x in waypoints])
        self.time_uncertainty_s = time_uncertainty_s
        self.speed_m_s = speed_m_s
        self.uncertainty_radius_m = uncertainty_radius_m
        self.id = id
        self.layer = layer

    def position_range(self, time) -> List[Waypoint]:
        if time < self.departure_time or time > self.destination_time + self.time_uncertainty_s:
            raise Exception("Flightplan {} does not exist during time {}".format(self, time))

        position_range_waypoints = []
        time_from = max(time - self.time_uncertainty_s, self.departure_time)
        time_to = time

        first_waypoint = max([i for i, x in enumerate(self.waypoints) if x.time <= time_from])
        waypoints_later_than_time_to = [i for i, x in enumerate(self.waypoints) if x.time >= time_to]
        if waypoints_later_than_time_to:
            last_waypoint = min(waypoints_later_than_time_to)
        else:
            last_waypoint = len(self.waypoints)-1

        if self.waypoints[first_waypoint].time == time_from:
            w1 = self.waypoints[first_waypoint]
            position_range_waypoints.append(self.Waypoint(
                x=w1.x,
                y=w1.y,
                norm_x=w1.norm_x,
                norm_y=w1.norm_y,
                time=time
            ))
        else:
            w1 = self.waypoints[first_waypoint]
            w2 = self.waypoints[first_waypoint+1]
            ratio = (time_from-w1.time)/(w2.time-w1.time)
            position_range_waypoints.append(self.Waypoint(
                x=w1.x + ratio*(w2.x-w1.x),
                y=w1.y + ratio*(w2.y-w1.y),
                norm_x=w1.norm_x + ratio*(w2.norm_x-w1.norm_x),
                norm_y=w1.norm_y + ratio*(w2.norm_y-w1.norm_y),
                time=time
            ))

        for i in range(first_waypoint+1, last_waypoint):
            w1 = self.waypoints[i]
            position_range_waypoints.append(self.Waypoint(
                x=w1.x,
                y=w1.y,
                norm_x=w1.norm_x,
                norm_y=w1.norm_y,
                time=time
            ))

        if self.waypoints[last_waypoint].time <= time_to:
            w1 = self.waypoints[last_waypoint]
            position_range_waypoints.append(self.Waypoint(
                x=w1.x,
                y=w1.y,
                norm_x=w1.norm_x,
                norm_y=w1.norm_y,
                time=time
            ))
        else:
            w1 = self.waypoints[last_waypoint-1]
            w2 = self.waypoints[last_waypoint]
            ratio = (time_to-w1.time)/(w2.time-w1.time)
            position_range_waypoints.append(self.Waypoint(
                x=w1.x + ratio*(w2.x-w1.x),
                y=w1.y + ratio*(w2.y-w1.y),
                norm_x=w1.norm_x + ratio*(w2.norm_x-w1.norm_x),
                norm_y=w1.norm_y + ratio*(w2.norm_y-w1.norm_y),
                time=time
            ))

        return position_range_waypoints

    @classmethod
    def from_graph_path(cls, graph: nx.Graph, path: List[str], request: Request, layer: int = None) -> "Flightplan":
        waypoints = []
        previous_node_finish_time = request.departure_time

        waypoints.append(cls.Waypoint(graph.nodes[path[0]]['x'], graph.nodes[path[0]]['y'], graph.nodes[path[0]]['norm_x'], graph.nodes[path[0]]['norm_y'], request.departure_time, True))

        time_cost = 0
        for i in range(len(path)-1):
            end_node = graph.nodes[path[i+1]]
            time_cost += graph.edges[path[i], path[i + 1]]['time_cost']
            if end_node['turning'] and not graph.nodes[path[i]]['turning']:
                waypoints.append(cls.Waypoint(end_node['x'], end_node['y'], end_node['norm_x'], end_node['norm_y'], previous_node_finish_time + time_cost, True))

                previous_node_finish_time = previous_node_finish_time + time_cost
                time_cost = 0

        return cls(request.id, waypoints, request.time_uncertainty_s, request.speed_m_s, request.uncertainty_radius_m, layer=layer)

    @classmethod
    def from_sn_flightplan(cls, graph: nx.Graph, sn_flightplan: "SNFlightplan", layer: int = None):
        waypoints = []

        path = sn_flightplan.nodes
        path_keys = list(path.keys())

        ground_delay = 0
        for i in range(len(path) - 1):
            if path[path_keys[i]] != path[path_keys[i + 1]]:
                ground_delay = i
                break

        previous_node_finish_time = path_keys[ground_delay]

        waypoints.append(
            cls.Waypoint(graph.nodes[path[path_keys[ground_delay]]]['x'], graph.nodes[path[path_keys[ground_delay]]]['y'], graph.nodes[path[path_keys[ground_delay]]]['norm_x'],
                         graph.nodes[path[path_keys[ground_delay]]]['norm_y'], previous_node_finish_time, True))

        for i in range(ground_delay, len(path) - 1):
            if graph.has_edge(path[path_keys[i]], path[path_keys[i + 1]]):
                time = graph.edges[path[path_keys[i]], path[path_keys[i + 1]]]['time_cost']
            else:
                time = 0

            if (graph.nodes[path[path_keys[i]]]['x'] != graph.nodes[path[path_keys[i+1]]]['x'] or graph.nodes[path[path_keys[i]]]['y'] != graph.nodes[path[path_keys[i+1]]]['y']):
                end_node = graph.nodes[path[path_keys[i + 1]]]

                waypoints.append(cls.Waypoint(end_node['x'], end_node['y'], end_node['norm_x'], end_node['norm_y'],
                                              previous_node_finish_time + time, graph.nodes[path[path_keys[i + 1]]]['turning']))

            previous_node_finish_time = previous_node_finish_time + time

        return cls(sn_flightplan.id, waypoints, sn_flightplan.request.time_uncertainty_s, sn_flightplan.request.speed_m_s, sn_flightplan.uncertainty_radius_m, layer=layer)


class Geofence(ABC):
    time: Tuple[int, int]

    @abstractmethod
    def contains_point(self, point: Tuple[float, float], offset_m: float) -> bool:
        pass

    @abstractmethod
    def get_pyclipper_geometry(self) -> List:
        pass

    def exists_at_time(self, time: int) -> bool:
        return self.time[0] <= time <= self.time[1]


class DiskGeofence(Geofence):
    radius_m: float
    center: Tuple[float, float]

    def __init__(self, time: Tuple[int, int], radius_m: float, center: Tuple[float, float]):
        self.time = time
        self.radius_m = radius_m
        self.center = center

    def contains_point(self, point, offset_m):
        return abs(self.center[0] - point[0]) ** 2 + abs(self.center[1] - point[1]) ** 2 <= (self.radius_m + offset_m) ** 2

    @lru_cache(1)
    def get_pyclipper_geometry(self) -> List:
        pco = PyclipperOffset()
        pco.AddPath(scale_to_clipper([self.center, ], PCO_SCALING_FACTOR), pyclipper.JT_ROUND, pyclipper.ET_OPENSQUARE)

        buffer = scale_from_clipper(pco.Execute(self.radius_m * PCO_SCALING_FACTOR),
                                    PCO_SCALING_FACTOR)

        return buffer
