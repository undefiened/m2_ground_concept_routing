import copy
import itertools
import math
import queue
from dataclasses import dataclass
from functools import lru_cache
from typing import Tuple, List, Dict, Set, Union

import numpy as np
import scipy
from geomet import wkt
from geopy import distance
import networkx as nx
from scipy.spatial import KDTree

from ground_routing.common import PathNotFoundException, GDP, Request, TurnParamsTable, Geofence, Flightplan


def plot_three_points(n1, n2, n3):
    import matplotlib.pyplot as plt

    N = 50
    x = [n1['norm_x'], n2['norm_x'], n3['norm_x']]
    y = [n1['norm_y'], n2['norm_y'], n3['norm_y']]

    plt.scatter(x, y)
    plt.show()


class SNRequest:
    id: str
    start_node: str
    destination_node: str
    start_time: int
    time_uncertainty_s: int
    uncertainty_radius_m: float
    speed_m_s: float
    gdp: GDP

    def __init__(self, id: str, start_node: str, destination_node: str, start_time: int, drone_radius_m: float, speed_m_s: float, time_uncertainty_s: int = 0, gdp: GDP = None):
        self.start_node = start_node
        self.destination_node = destination_node
        self.start_time = start_time
        self.uncertainty_radius_m = drone_radius_m
        self.speed_m_s = speed_m_s
        self.time_uncertainty_s = time_uncertainty_s
        self.gdp = gdp
        self.id = id

    def __str__(self):
        return 'Request(from "{}" to "{}" at T={} with uncertainty {}s and {}m, speed {}m/s and GDP {})'.format(
            self.start_node, self.destination_node, self.start_time, self.time_uncertainty_s, self.uncertainty_radius_m,
            self.speed_m_s, self.gdp
        )

    def __hash__(self):
        return hash(repr(self))


class SNFlightplan:
    id: str
    nodes: Dict[int, str]
    start_time: int
    end_time: int
    speed_node: int
    time_uncertainty: int
    uncertainty_radius_m: float
    request: SNRequest
    weight: float

    def __init__(self, id: str, nodes: List[Tuple[int, str]], speed_node: int, time_uncertainty: int, radius_m: float, request: SNRequest):
        self.nodes = {}
        self._fill_nodes(nodes)
        self.start_time = min([x[0] for x in nodes])
        self.end_time = max([x[0] for x in nodes])
        self.speed_node = speed_node
        self.time_uncertainty = time_uncertainty
        self.uncertainty_radius_m = radius_m
        self.request = request
        self.id = id

    def _fill_nodes(self, nodes: List[Tuple[int, str]]):
        for node in nodes:
            self.nodes[node[0]] = node[1]

    def is_present_at_time(self, time: int) -> bool:
        return time >= self.start_time and time <= self.end_time

    def is_present_at_time_with_uncertainty(self, time: int, additional_uncertainty: int) -> bool:
        return time >= self.start_time - additional_uncertainty and time <= self.end_time + self.time_uncertainty

    def position_at_time(self, time: int) -> List[str]:
        if not self.is_present_at_time(time):
            raise Exception('Not present at time {}'.format(time))

        if self.speed_node == 1:
            if time in self.nodes:
                return [self.nodes[time], ]
            else:
                keys = sorted(self.nodes.keys())

                for i in range(len(keys) - 1):
                    key_1 = keys[i]
                    key_2 = keys[i + 1]
                    if key_1 < time < key_2:
                        return [self.nodes[key_1], self.nodes[key_2]]

        else:
            if time in self.nodes:
                return [self.nodes[time], ]
            elif time < self.start_time:
                return [self.nodes[self.start_time], ]
            elif time > self.end_time:
                return [self.nodes[self.end_time], ]
            else:
                keys = sorted(self.nodes.keys())

                for i in range(len(keys)-1):
                    key_1 = keys[i]
                    key_2 = keys[i+1]
                    if key_1 < time < key_2:
                        return [self.nodes[key_1], self.nodes[key_2]]

                raise Exception('A point in the path is missing')


class StreetNetwork:
    @dataclass
    class BoundingBox:
        left: float
        right: float
        bottom: float
        top: float

    MERGE_THRESHOLD_MULTIPLIER = 0.3

    def __init__(self, network: nx.Graph, turn_params_table: TurnParamsTable, recompute_lengths: bool = False, geometry_graph: nx.Graph = None):
        self.original_network: nx.Graph = network
        self._convert_attributes_to_numbers()

        if recompute_lengths:
            self._recompute_edges_lengths()

        self._compute_normalized_positions()
        self.turn_params_table = turn_params_table
        self.geometry_graph = geometry_graph
        # self._compute_distances()

    def _recompute_edges_lengths(self):
        for u, v in self.original_network.edges:
            self.original_network[u][v]['length'] = distance.distance(
                (self.original_network.nodes[u]['y'], self.original_network.nodes[u]['x']),
                (self.original_network.nodes[v]['y'], self.original_network.nodes[v]['x'])
            ).m

    def _convert_attributes_to_numbers(self):
        for node in self.original_network.nodes:
            self.original_network.nodes[node]['x'] = float(self.original_network.nodes[node]['x'])
            self.original_network.nodes[node]['y'] = float(self.original_network.nodes[node]['y'])

        for u, v in self.original_network.edges:
            self.original_network[u][v]['length'] = float(self.original_network[u][v]['length'])

    def coordinate_to_norm(self, lat, lon) -> Tuple[float, float]:
        bounding_box = self._bounding_box_lrbt()

        height = distance.distance((bounding_box.bottom, bounding_box.left), (bounding_box.top, bounding_box.left)).m
        width = distance.distance((bounding_box.bottom, bounding_box.left), (bounding_box.bottom, bounding_box.right)).m

        norm_x = ((lon - bounding_box.left) / (
                    bounding_box.right - bounding_box.left)) * width
        norm_y = ((lat - bounding_box.bottom) / (
                    bounding_box.top - bounding_box.bottom)) * height

        return norm_x, norm_y

    def _compute_normalized_positions(self):
        bounding_box = self._bounding_box_lrbt()

        height = distance.distance((bounding_box.bottom, bounding_box.left), (bounding_box.top, bounding_box.left)).m
        width = distance.distance((bounding_box.bottom, bounding_box.left), (bounding_box.bottom, bounding_box.right)).m

        for node in self.original_network.nodes:
            norm_x = ((self.original_network.nodes[node]['x'] - bounding_box.left)/(bounding_box.right-bounding_box.left)) * width
            norm_y = ((self.original_network.nodes[node]['y'] - bounding_box.bottom)/(bounding_box.top-bounding_box.bottom)) * height

            self.original_network.nodes[node]['norm_x'] = norm_x
            self.original_network.nodes[node]['norm_y'] = norm_y

    def _bounding_box_lrbt(self):
        left = min([self.original_network.nodes[x]['x'] for x in self.original_network.nodes])
        right = max([self.original_network.nodes[x]['x'] for x in self.original_network.nodes])
        bottom = min([self.original_network.nodes[x]['y'] for x in self.original_network.nodes])
        top = max([self.original_network.nodes[x]['y'] for x in self.original_network.nodes])

        return self.BoundingBox(left=left, right=right, top=top, bottom=bottom)

    @classmethod
    def from_graphml_file(cls, filename: str, turn_params_table: TurnParamsTable, recompute_lengths: bool = False):
        network = nx.read_graphml(filename)
        return cls(network, recompute_lengths=recompute_lengths, turn_params_table=turn_params_table)

    @classmethod
    def from_graphml_string(cls, graphml: str, turn_params_table: TurnParamsTable, recompute_lengths: bool = False):
        network = nx.parse_graphml(graphml)
        return cls(network, turn_params_table=turn_params_table, recompute_lengths=recompute_lengths)

    def get_subdivided_network(self, edge_length) -> nx.DiGraph:
        bounding_box = self._bounding_box_lrbt()
        height = distance.distance((bounding_box.bottom, bounding_box.left), (bounding_box.top, bounding_box.left)).m
        width = distance.distance((bounding_box.bottom, bounding_box.left), (bounding_box.bottom, bounding_box.right)).m

        subdivided_network = nx.DiGraph()
        # subdivided_network.add_nodes_from(self.original_network)

        for node in self.original_network.nodes:
            new_nodes = []

            new_node_params = {}
            new_node_params['x'] = self.original_network.nodes[node]['x']
            new_node_params['y'] = self.original_network.nodes[node]['y']
            new_node_params['norm_x'] = self.original_network.nodes[node]['norm_x']
            new_node_params['norm_y'] = self.original_network.nodes[node]['norm_y']
            new_node_params['turning'] = True

            for v, u in self.original_network.edges(node):
                new_node_name = '{}@{}'.format(u, v)
                new_nodes.append((new_node_name, new_node_params))

            additional_edges = []
            any_turn_penalized = False
            for (v1, u1), (v2, u2) in itertools.combinations(self.original_network.edges(node), 2):
                n1 = self.original_network.nodes[node]
                n2 = self.original_network.nodes[u1]

                n3 = self.original_network.nodes[u2]

                if u1 == node or u2 == node or v2 != node:
                    raise Exception

                angle = abs(180 - (math.atan2(n3['norm_y'] - n1['norm_y'], n3['norm_x'] - n1['norm_x']) - math.atan2(n2['norm_y'] - n1['norm_y'], n2['norm_x'] - n1['norm_x'])) * 180/math.pi)

                additional_edges.append(('{}@{}'.format(u1, v1), '{}@{}'.format(u2, v2), {'turn_cost_edge': True, 'time_cost': self.turn_params_table.get_turn_cost_s(angle), 'penalized': self.turn_params_table.is_turn_penalized(angle)}))
                additional_edges.append(('{}@{}'.format(u2, v2), '{}@{}'.format(u1, v1), {'turn_cost_edge': True, 'time_cost': self.turn_params_table.get_turn_cost_s(angle), 'penalized': self.turn_params_table.is_turn_penalized(angle)}))

                if self.turn_params_table.is_turn_penalized(angle):
                    any_turn_penalized = True

            if any_turn_penalized:
                subdivided_network.add_nodes_from(new_nodes)
                subdivided_network.add_edges_from(additional_edges)
            else:
                subdivided_network.add_node(node, **new_node_params)

        for (u, v) in self.original_network.edges:
            original_length_m = self.original_network[u][v]['length']
            original_u, original_v = u, v
            if u not in subdivided_network.nodes:
                u = '{}@{}'.format(original_v, original_u)

            if v not in subdivided_network.nodes:
                v = '{}@{}'.format(original_u, original_v)

            subdivide_into = math.ceil(original_length_m/edge_length)

            if original_length_m % edge_length > 0 and original_length_m % edge_length < self.MERGE_THRESHOLD_MULTIPLIER*edge_length:
                subdivide_into = subdivide_into - 1

            if subdivide_into > 1:
                additional_nodes = []
                additional_edges = []

                for i in range(subdivide_into-1):
                    this_node = '{}_{}_{}'.format(u, v, i)
                    previous_node = '{}_{}_{}'.format(u, v, i-1) if i != 0 else u
                    current_length = (i+1)*edge_length

                    if 'geometry' in self.original_network.edges[original_u, original_v]:
                        geometry = self.original_network.edges[original_u, original_v]['geometry']
                        geometry = wkt.loads(geometry)
                        if geometry['type'] != 'LineString':
                            raise Exception('Not supported!')

                        if abs(geometry['coordinates'][0][0] - subdivided_network.nodes[u]['x']) > 0.000001 or abs(geometry['coordinates'][0][1] - subdivided_network.nodes[u]['y']) > 0.000001:
                            geometry['coordinates'].reverse()

                        lengths = []

                        for j in range(len(geometry['coordinates'])-1):
                            c1 = geometry['coordinates'][j]
                            c2 = geometry['coordinates'][j+1]
                            length = distance.great_circle((c1[1], c1[0]), (c2[1], c2[0])).m
                            lengths.append(length)

                        sum_length = 0
                        new_ratio = 0
                        selected_geometry_edge_i = 0

                        for geometry_edge_i, length in enumerate(lengths):
                            sum_length += length
                            if sum_length > current_length:
                                selected_geometry_edge_i = geometry_edge_i
                                new_ratio = (current_length-(sum_length - length))/length
                                break

                        ci1 = geometry['coordinates'][selected_geometry_edge_i]
                        ci2 = geometry['coordinates'][selected_geometry_edge_i + 1]
                        lon = ci1[0] + (ci2[0] - ci1[0]) * new_ratio
                        lat = ci1[1] + (ci2[1] - ci1[1]) * new_ratio
                        norm_x = ((lon - bounding_box.left) / (
                                    bounding_box.right - bounding_box.left)) * width
                        norm_y = ((lat - bounding_box.bottom) / (
                                    bounding_box.top - bounding_box.bottom)) * height

                    else:
                        ratio = (current_length / original_length_m)

                        u_x = subdivided_network.nodes[u]['x']
                        u_y = subdivided_network.nodes[u]['y']
                        u_norm_x = subdivided_network.nodes[u]['norm_x']
                        u_norm_y = subdivided_network.nodes[u]['norm_y']
                        v_x = subdivided_network.nodes[v]['x']
                        v_y = subdivided_network.nodes[v]['y']
                        v_norm_x = subdivided_network.nodes[v]['norm_x']
                        v_norm_y = subdivided_network.nodes[v]['norm_y']
                        lon = u_x + (v_x - u_x)*ratio
                        lat = u_y + (v_y - u_y)*ratio
                        norm_x = u_norm_x + (v_norm_x - u_norm_x) * ratio
                        norm_y = u_norm_y + (v_norm_y - u_norm_y) * ratio

                    additional_nodes.append((this_node, {'x': lon, 'y': lat, 'norm_x': norm_x, 'norm_y': norm_y, 'turning': False}))

                    additional_edges.append((previous_node, this_node, {'length': edge_length, 'turn_cost_edge': False, 'time_cost': 0}))
                    additional_edges.append((this_node, previous_node, {'length': edge_length, 'turn_cost_edge': False, 'time_cost': 0}))

                length = original_length_m % edge_length
                if length < self.MERGE_THRESHOLD_MULTIPLIER*edge_length:
                    length = length + edge_length

                additional_edges.append(('{}_{}_{}'.format(u, v, subdivide_into-2), v,  {'length': length, 'turn_cost_edge': False, 'time_cost': 0}))
                additional_edges.append((v, '{}_{}_{}'.format(u, v, subdivide_into-2),  {'length': length, 'turn_cost_edge': False, 'time_cost': 0}))

                subdivided_network.add_nodes_from(additional_nodes)
                subdivided_network.add_edges_from(additional_edges)
            else:
                subdivided_network.add_edges_from([
                    (u, v, {'length': original_length_m, 'turn_cost_edge': False, 'time_cost': 0}),
                    (v, u, {'length': original_length_m, 'turn_cost_edge': False, 'time_cost': 0})
                ])

        return subdivided_network


class PathPlanner:
    SOURCE_NODE_ID = 'SOURCE'
    SINK_NODE_ID = 'DESTINATION'

    street_network: StreetNetwork
    permanent_sn_flightplans: List[SNFlightplan]
    temporary_sn_flightplans: List[SNFlightplan]
    planned_flightplans: List[Flightplan]
    timestep_s: int
    network: nx.DiGraph
    edge_length_m: float
    _geofences: List[Geofence]

    def __init__(self, street_network: StreetNetwork, timestep_s: int, edge_length_m: float, default_gdp: GDP, geofences: List[Geofence] = []):
        self.street_network = street_network
        self.default_gdp = default_gdp
        self.permanent_sn_flightplans = []
        self.temporary_sn_flightplans = []
        self.timestep_s = timestep_s
        self.edge_length_m = edge_length_m
        self._geofences = geofences

        self.network = self.street_network.get_subdivided_network(edge_length_m)

        self.planned_flightplans = []

    @property
    def flightplans(self):
        flightplans = []
        flightplans.extend(self.permanent_sn_flightplans)
        flightplans.extend(self.temporary_sn_flightplans)
        return flightplans

    @property
    def geofences(self):
        return self._geofences

    def add_geofence(self, geofence: Geofence):
        self._list_of_occupied_nodes_for_request.cache_clear()
        self._geofences.append(geofence)

    def add_geofences(self, geofences: List[Geofence]):
        self._list_of_occupied_nodes_for_request.cache_clear()
        self._geofences.extend(geofences)

    def clear_geofences(self):
        self._list_of_occupied_nodes_for_request.cache_clear()
        self._geofences = []

    @lru_cache(1)
    def _get_network_kd_tree(self) -> Tuple[KDTree, List[str]]:
        nodes = list(self.network.nodes(data=True))
        coords_list = np.array([(x[1]['norm_x'], x[1]['norm_y']) for x in nodes])
        names_list = [x[0] for x in nodes]

        res = KDTree(coords_list)

        return res, names_list

    def _get_nodes_within_radius(self, center: Tuple[float, float], radius_m: float) -> List[str]:
        kd_tree, nodes = self._get_network_kd_tree()

        res = kd_tree.query_ball_point(center, radius_m)

        return [nodes[x] for x in res]


    @lru_cache(200)
    def _list_of_occupied_nodes_for_request(self, time: int, request_uncertainty_radius_m: float, request_time_uncertainty_ticks: int) -> Set[str]:
        occupied_nodes = set()

        for plan in self.flightplans:
            for time_uncertainty in range(max(-request_time_uncertainty_ticks, time - plan.end_time),
                                          min(plan.time_uncertainty + 1, time - plan.start_time + 1)):
                if plan.is_present_at_time_with_uncertainty(time, request_time_uncertainty_ticks):
                    radius = plan.uncertainty_radius_m + request_uncertainty_radius_m
                    for position_node in plan.position_at_time(time - time_uncertainty):
                        position_node = self.network.nodes[position_node]
                        position = (position_node['norm_x'], position_node['norm_y'])

                        nodes_covered_by_flightplan = self._get_nodes_within_radius(position, radius)

                        occupied_nodes.update(nodes_covered_by_flightplan)

        for plan in self.planned_flightplans:
            s = max(-request_time_uncertainty_ticks, time - plan.destination_time)
            e = min(plan.time_uncertainty_s + 1, time - plan.departure_time + 1)
            for time_uncertainty in range(s, e):
                if plan.is_present_at_time_with_uncertainty(time, request_time_uncertainty_ticks):
                    radius = plan.uncertainty_radius_m + request_uncertainty_radius_m
                    position = plan.position_at_time(time - time_uncertainty)
                    position = (position.norm_x, position.norm_y)

                    nodes_covered_by_flightplan = self._get_nodes_within_radius(position, radius)

                    occupied_nodes.update(nodes_covered_by_flightplan)

        nodes_covered_by_geofences = []
        current_geofences = [x for x in self._geofences if x.exists_at_time(time)]
        if len(current_geofences) > 0:
            for node, data in self.network.nodes(data=True):
                for geofence in current_geofences:
                    if geofence.contains_point((data['norm_x'], data['norm_y']), request_uncertainty_radius_m):
                        nodes_covered_by_geofences.append(node)
                        break

        occupied_nodes.update(nodes_covered_by_geofences)

        return occupied_nodes

    @lru_cache(200)
    def list_of_pending_drone_movements(self, time: int) -> List[Tuple[str, str]]:
        pending_movements = []

        for plan in self.flightplans:
            if plan.is_present_at_time(time - 1) and plan.is_present_at_time(time):
                pending_movements.append(
                    (plan.position_at_time(time - 1)[0], plan.position_at_time(time)[0])
                )

        return pending_movements

    def _request_time_uncertainty_to_ticks(self, request: SNRequest) -> int:
        return math.ceil(request.time_uncertainty_s / self.timestep_s)

    @staticmethod
    def time_ext_node_id(node: str, time: int) -> str:
        return '{}:{}'.format(node, time)
    
    @staticmethod
    def _get_node_time(time_ext_node_id: str) -> int:
        return int(time_ext_node_id.split(':')[1])

    @staticmethod
    def _get_node_id(time_ext_node_id) -> str:
        return time_ext_node_id.split(':')[0].split('/')[0]

    @classmethod
    def _is_node_turning(cls, time_ext_node_id: str) -> bool:
        return '@' in cls._get_node_id(time_ext_node_id).split('_')[-1]

    @lru_cache(3)
    def _request_time_to_pass_node(self, request: Union[SNRequest, Request]) -> int:
        return self._time_to_pass_node_with_speed(request.speed_m_s)

    @lru_cache(3)
    def _time_to_pass_node_with_speed(self, speed_m_s: float) -> int:
        return math.ceil(self.timestep_s/(self.edge_length_m / speed_m_s))

    def _request_gdp_or_default(self, request: SNRequest) -> GDP:
        if request.gdp is not None:
            return request.gdp
        else:
            return self.default_gdp

    def _get_edge_penalty(self, node1, node2) -> float:
        if node1 == node2:
            return 0
        else:
            return self.network[node1][node2]['length']/1000000

    def _get_all_neighbours(self, node_id: str, time: int, request: SNRequest) -> List[Tuple[str, float]]:
        nodes = list(self._tmp_time_cost_enhanced_network.neighbors(node_id))
        nodes_costs = {}

        for node in nodes:
            nodes_costs[node] = self._tmp_time_cost_enhanced_network._succ[node_id][node]['time_cost']

        max_lookup_time = self._time_to_pass_node_with_speed(request.speed_m_s)

        for node in nodes:
            if nodes_costs[node] > max_lookup_time:
                max_lookup_time = nodes_costs[node]

        occupied = {}
        pending_movements = {}

        for i in range(1, max(self._time_to_pass_node_with_speed(request.speed_m_s), max_lookup_time) + 1):
            occupied[i] = self._list_of_occupied_nodes_for_request(int(time + i), request.uncertainty_radius_m, self._request_time_uncertainty_to_ticks(request))
            pending_movements[i] = self.list_of_pending_drone_movements(time + i)

        neighbours = []
        for node in nodes:
            time_cost = nodes_costs[node]
            available = True

            for i in range(1, time_cost + 1):
                if not(node not in occupied[i] and node_id not in occupied[i] and (node, node_id) not in pending_movements[i]):
                    available = False

            if available:
                neighbours.append((self.time_ext_node_id(node, time + time_cost), time_cost)) # add + get_edge_penalty() if needed

        if node_id == request.destination_node:
            neighbours.append((self.SINK_NODE_ID, 0))

        return neighbours

    def _find_shortest_path(self, request: SNRequest) -> Tuple[List[str], float]:
        self._tmp_time_cost_enhanced_network = self.get_time_cost_enhanced_network(request.speed_m_s)
        self._list_of_occupied_nodes_for_request.cache_clear()
        self._request_time_to_pass_node.cache_clear()
        self._time_to_pass_node_with_speed.cache_clear()
        self.list_of_pending_drone_movements.cache_clear()

        self.visited_nodes = set()
        self._nodes_weights = {}
        self._queue = queue.PriorityQueue()
        self._previous_nodes = {}

        # self._queue.put((0, self.SOURCE_NODE_ID))

        for i in range(0, self._request_gdp_or_default(request).max_time + 1, self._request_gdp_or_default(request).time_step):
            weight = i * self._request_gdp_or_default(request).penalty
            occupied = self._list_of_occupied_nodes_for_request(int(request.start_time + i), request.uncertainty_radius_m, self._request_time_uncertainty_to_ticks(request))
            node = self.time_ext_node_id(request.start_node, int(request.start_time + i))

            if request.start_node not in occupied:
                self._queue.put((weight, node))
                self._nodes_weights[node] = weight
                self._previous_nodes[node] = self.SOURCE_NODE_ID

        while not self._queue.empty():
            (weight, current_node_id) = self._queue.get()

            if current_node_id not in self.visited_nodes:
                if current_node_id == self.SINK_NODE_ID:
                    break

                current_node_weight = self._nodes_weights[current_node_id]
                current_node_time = self._get_node_time(current_node_id)

                neighbours = self._get_all_neighbours(self._get_node_id(current_node_id), current_node_time, request)

                for (neighbour_id, weight) in neighbours:
                    if neighbour_id not in self._nodes_weights or weight + current_node_weight < self._nodes_weights[
                        neighbour_id]:
                        self._nodes_weights[neighbour_id] = weight + current_node_weight
                        self._previous_nodes[neighbour_id] = current_node_id
                        self._queue.put((weight + current_node_weight, neighbour_id))

                self.visited_nodes.add(current_node_id)

        if self.SINK_NODE_ID not in self._previous_nodes:
            raise PathNotFoundException("Cannot find a path for request {}".format(request))

        path = []
        previous_node = self._previous_nodes[self.SINK_NODE_ID]

        while previous_node != self.SOURCE_NODE_ID:
            path.append(previous_node)
            previous_node = self._previous_nodes[previous_node]

        path.reverse()

        weight = self._nodes_weights[path[-1]]

        self.visited_nodes = None
        self._nodes_weights = None
        self._queue = None
        self._previous_nodes = None

        return path, weight

    def is_destination_reachable(self, request: SNRequest):
        return nx.has_path(self.network, request.start_node, request.destination_node)

    def find_closest_node(self, x: float, y: float, starting_node_in_subdivided_graph: bool = False, speed_m_s: float = None) -> str:
        d = distance.great_circle

        if starting_node_in_subdivided_graph:
            nodes = self.network.nodes()
        else:
            nodes = [node for node, data in self.network.nodes(data=True) if data['turning']]

        return min(nodes, key=lambda node: d(
            (self.network.nodes[node]['y'], self.network.nodes[node]['x']),
            (y, x)
        ).m)

    def convert_request_to_sn(self, request: Request, starting_node_in_subdivided_graph: bool = False) -> SNRequest:
        start_node = self.find_closest_node(request.origin[0], request.origin[1], starting_node_in_subdivided_graph, request.speed_m_s)
        end_node = self.find_closest_node(request.destination[0], request.destination[1], starting_node_in_subdivided_graph, request.speed_m_s)
        sn_request = SNRequest(request.id, start_node, end_node, request.departure_time, request.uncertainty_radius_m, request.speed_m_s, request.time_uncertainty_s, request.gdp)
        return sn_request

    def resolve_request(self, request: Union[Request, SNRequest]):
        self.list_of_pending_drone_movements.cache_clear()
        if isinstance(request, Request):
            request = self.convert_request_to_sn(request)

        if not self.is_destination_reachable(request):
            raise PathNotFoundException('There is no path!')

        path, weight = self._find_shortest_path(request)

        points = []
        if self._get_node_time(path[0]) != request.start_time:
            for i in range(self._get_node_time(path[0]) - request.start_time):
                points.append((i + request.start_time, self._get_node_id(path[0])))

        for i, node_id in enumerate(path):
            points.append((self._get_node_time(node_id), self._get_node_id(node_id)))

        new_flightplan = SNFlightplan(id=request.id, nodes=points, speed_node=math.floor(1 / self._time_to_pass_node_with_speed(request.speed_m_s)),
                                      time_uncertainty=self._request_time_uncertainty_to_ticks(request), radius_m=request.uncertainty_radius_m, request=request)
        new_flightplan.weight = weight

        return new_flightplan

    def resolve_requests(self, requests: List[SNRequest]) -> List[SNFlightplan]:
        self.temporary_sn_flightplans = []
        for request in requests:
            try:
                flightplan = self.resolve_request(request)
                self.temporary_sn_flightplans.append(flightplan)
            except PathNotFoundException as e:
                print(e)

        return self.temporary_sn_flightplans

    def add_flightplan(self, flightplan: SNFlightplan):
        self.list_of_pending_drone_movements.cache_clear()
        self._list_of_occupied_nodes_for_request.cache_clear()
        self.permanent_sn_flightplans.append(flightplan)

    @lru_cache(3)
    def get_time_cost_enhanced_network(self, speed_m_s):
        new_network = copy.deepcopy(self.network)

        for u, v in new_network.edges:
            edge = new_network.edges[u, v]
            if not edge['turn_cost_edge']:
                edge['time_cost'] = self._time_to_pass_node_with_speed(speed_m_s)

        return new_network

    def _clear_cache(self):
        self._list_of_occupied_nodes_for_request.cache_clear()
        self._request_time_to_pass_node.cache_clear()
        self._time_to_pass_node_with_speed.cache_clear()
        self.list_of_pending_drone_movements.cache_clear()
