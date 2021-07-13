from dataclasses import dataclass
from typing import Tuple, List, TYPE_CHECKING

import networkx as nx

if TYPE_CHECKING:
    from street_network.path_planner import SNFlightplan


class PathNotFoundException(Exception):
    """Raises if there is no path for the request"""

@dataclass
class GDP:
    max_time: int
    penalty: float


@dataclass
class Request:
    origin: Tuple[float, float]
    destination: Tuple[float, float]
    time_uncertainty_s: int
    speed_m_s: float
    departure_time: int
    uncertainty_radius_m: float
    gdp: GDP


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
        while angle > 180:
            angle -= 180

        while angle < 0:
            angle += 180

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

    waypoints: List[Waypoint]
    departure_time: int
    destination_time: int

    def __init__(self, waypoints: List[Waypoint], time_uncertainty_s: int, speed_m_s: float):
        self.waypoints = waypoints
        self.departure_time = min([x.time for x in waypoints])
        self.destination_time = max([x.time for x in waypoints])
        self.time_uncertainty_s = time_uncertainty_s
        self.speed_m_s = speed_m_s

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
        # for i in range(len(self.waypoints)-1):
        #     if self.waypoints[i].time <= time_from < self.waypoints[i + 1].time:
        #         w1 = self.waypoints[i]
        #         w2 = self.waypoints[i+1]
        #         ratio = (time_from-w1.time)/(w2.time-w1.time)
        #         position_range_waypoints.append(self.Waypoint(
        #             x=w1.x + ratio*(w2.x-w1.x),
        #             y=w1.y + ratio*(w2.y-w1.y),
        #             norm_x=w1.norm_x + ratio*(w2.norm_x-w1.norm_x),
        #             norm_y=w1.norm_y + ratio*(w2.norm_y-w1.norm_y),
        #             time=time
        #         ))
        #
        #     if self.waypoints[i].time > time_from and self.waypoints[i].time < time_to:
        #         w1 = self.waypoints[i]
        #         position_range_waypoints.append(self.Waypoint(
        #             x=w1.x,
        #             y=w1.y,
        #             norm_x=w1.norm_x,
        #             norm_y=w1.norm_y,
        #             time=time
        #         ))
        #
        #     if self.waypoints[i].time > time_from and self.waypoints[i+1].time > time_from and self.waypoints[i].time < time_to and self.waypoints[i+1].time <= time_to:
        #         w1 = self.waypoints[i]
        #         position_range_waypoints.append(self.Waypoint(
        #             x=w1.x,
        #             y=w1.y,
        #             norm_x=w1.norm_x,
        #             norm_y=w1.norm_y,
        #             time=time
        #         ))
        #
        #     if self.waypoints[i].time <= time_to < self.waypoints[i + 1].time:
        #         w1 = self.waypoints[i]
        #         w2 = self.waypoints[i + 1]
        #         ratio = (time_to - w1.time) / (w2.time - w1.time)
        #         position_range_waypoints.append(self.Waypoint(
        #             x=w1.x + ratio * (w2.x - w1.x),
        #             y=w1.y + ratio * (w2.y - w1.y),
        #             norm_x=w1.norm_x + ratio * (w2.norm_x - w1.norm_x),
        #             norm_y=w1.norm_y + ratio * (w2.norm_y - w1.norm_y),
        #             time=time
        #         ))
        #
        # if time_to >= self.waypoints[-1].time:
        #     w1 = self.waypoints[-1]
        #     position_range_waypoints.append(self.Waypoint(
        #         x=w1.x,
        #         y=w1.y,
        #         norm_x=w1.norm_x,
        #         norm_y=w1.norm_y,
        #         time=time
        #     ))

        return position_range_waypoints

    @classmethod
    def from_graph_path(cls, graph: nx.Graph, path: List[str], request: Request) -> "Flightplan":
        waypoints = []
        previous_node_finish_time = request.departure_time

        waypoints.append(cls.Waypoint(graph.nodes[path[0]]['x'], graph.nodes[path[0]]['y'], graph.nodes[path[0]]['norm_x'], graph.nodes[path[0]]['norm_y'], request.departure_time))

        length = 0
        for i in range(len(path)-1):
            # node1 = graph.nodes[path[i]]
            end_node = graph.nodes[path[i+1]]
            length += graph.edges[path[i], path[i + 1]]['length']
            if end_node['turning']:
                time = length/request.speed_m_s

                waypoints.append(cls.Waypoint(end_node['x'], end_node['y'], end_node['norm_x'], end_node['norm_y'], previous_node_finish_time + time))

                previous_node_finish_time = previous_node_finish_time + time
                length = 0

        return cls(waypoints, request.time_uncertainty_s, request.speed_m_s)

    @classmethod
    def from_sn_flightplan(cls, graph: nx.Graph, sn_flightplan: "SNFlightplan"):
        waypoints = []
        previous_node_finish_time = sn_flightplan.start_time

        path = sn_flightplan.nodes

        waypoints.append(
            cls.Waypoint(graph.nodes[path[0]]['x'], graph.nodes[path[0]]['y'], graph.nodes[path[0]]['norm_x'],
                         graph.nodes[path[0]]['norm_y'], sn_flightplan.start_time))

        for i in range(len(path) - 1):
            # node1 = graph.nodes[path[i]]
            end_node = graph.nodes[path[i + 1]]

            length = graph.edges[path[i], path[i + 1]]['length']
            time = length / sn_flightplan.request.speed_m_s

            waypoints.append(cls.Waypoint(end_node['x'], end_node['y'], end_node['norm_x'], end_node['norm_y'],
                                          previous_node_finish_time + time))

            previous_node_finish_time = previous_node_finish_time + time

        return cls(waypoints, sn_flightplan.request.time_uncertainty_s, sn_flightplan.request.speed_m_s)
