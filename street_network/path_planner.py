import math
import queue
from dataclasses import dataclass
from typing import Tuple, List, Dict, Set, Union

from geopy import distance
import networkx as nx

from common import PathNotFoundException, GDP, Request


class SNRequest:
    start_node: str
    destination_node: str
    start_time: int
    time_uncertainty_s: int
    uncertainty_radius_m: float
    speed_m_s: float
    gdp: GDP

    def __init__(self, start_node: str, destination_node: str, start_time: int, drone_radius_m: float, speed_m_s: float, time_uncertainty_s: int = 0, gdp: GDP = None):
        self.start_node = start_node
        self.destination_node = destination_node
        self.start_time = start_time
        self.uncertainty_radius_m = drone_radius_m
        self.speed_m_s = speed_m_s
        self.time_uncertainty_s = time_uncertainty_s
        self.gdp = gdp


class SNFlightplan:
    nodes: Dict[int, str]
    start_time: int
    end_time: int
    speed_node: int
    time_uncertainty: int
    uncertainty_radius_m: float
    request: SNRequest
    weight: float

    def __init__(self, nodes: List[Tuple[int, str]], speed_node: int, time_uncertainty: int, radius_m: float, request: SNRequest):
        self.nodes = {}
        self._fill_nodes(nodes)
        self.start_time = min([x[0] for x in nodes])
        self.end_time = max([x[0] for x in nodes])
        self.speed_node = speed_node
        self.time_uncertainty = time_uncertainty
        self.uncertainty_radius_m = radius_m
        self.request = request

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
            return [self.nodes[time], ]
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

    MERGE_THRESHOLD_MULTIPLIER = 0.1

    def __init__(self, network: nx.Graph, recompute_lengths: bool=False):
        self.original_network: nx.Graph = network
        self._convert_attributes_to_numbers()

        if recompute_lengths:
            self._recompute_edges_lengths()

        self._compute_normalized_positions()
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
    def from_graphml_file(cls, filename, recompute_lengths: bool = False):
        network = nx.read_graphml(filename)
        return cls(network, recompute_lengths=recompute_lengths)

    @classmethod
    def from_graphml_string(cls, graphml, recompute_lengths: bool = False):
        network = nx.parse_graphml(graphml)
        return cls(network, recompute_lengths)

    def get_subdivided_network(self, edge_length) -> nx.DiGraph:
        subdivided_network = nx.DiGraph()
        subdivided_network.add_nodes_from(self.original_network)

        for node in subdivided_network:
            subdivided_network.nodes[node]['x'] = self.original_network.nodes[node]['x']
            subdivided_network.nodes[node]['y'] = self.original_network.nodes[node]['y']
            subdivided_network.nodes[node]['norm_x'] = self.original_network.nodes[node]['norm_x']
            subdivided_network.nodes[node]['norm_y'] = self.original_network.nodes[node]['norm_y']
            subdivided_network.nodes[node]['turning'] = True

        for (u, v) in self.original_network.edges:
            original_length_m = self.original_network[u][v]['length']

            subdivide_into = math.ceil(original_length_m/edge_length)

            if original_length_m % edge_length > 0 and original_length_m % edge_length < self.MERGE_THRESHOLD_MULTIPLIER*edge_length:
                subdivide_into = subdivide_into - 1

            if subdivide_into > 1:
                additional_nodes = []
                additional_edges = []

                for i in range(subdivide_into-1):
                    this_node = '{}_{}_{}'.format(u, v, i)
                    previous_node = '{}_{}_{}'.format(u, v, i-1) if i != 0 else u
                    u_x = subdivided_network.nodes[u]['x']
                    u_y = subdivided_network.nodes[u]['y']
                    u_norm_x = subdivided_network.nodes[u]['norm_x']
                    u_norm_y = subdivided_network.nodes[u]['norm_y']
                    v_x = subdivided_network.nodes[v]['x']
                    v_y = subdivided_network.nodes[v]['y']
                    v_norm_x = subdivided_network.nodes[v]['norm_x']
                    v_norm_y = subdivided_network.nodes[v]['norm_y']
                    lat = u_x + (v_x - u_x)*((i+1)*edge_length/original_length_m)
                    lon = u_y + (v_y - u_y)*((i+1)*edge_length/original_length_m)
                    norm_x = u_norm_x + (v_norm_x - u_norm_x) * ((i + 1) * edge_length / original_length_m)
                    norm_y = u_norm_y + (v_norm_y - u_norm_y) * ((i + 1) * edge_length / original_length_m)
                    additional_nodes.append((this_node, {'x': lat, 'y': lon, 'norm_x': norm_x, 'norm_y': norm_y, 'turning': False}))

                    additional_edges.append((previous_node, this_node, {'length': edge_length}))
                    additional_edges.append((this_node, previous_node, {'length': edge_length}))

                length = original_length_m % edge_length
                if length < self.MERGE_THRESHOLD_MULTIPLIER*edge_length:
                    length = length + edge_length

                additional_edges.append(('{}_{}_{}'.format(u, v, subdivide_into-2), v,  {'length': length}))
                additional_edges.append((v, '{}_{}_{}'.format(u, v, subdivide_into-2),  {'length': length}))

                subdivided_network.add_nodes_from(additional_nodes)
                subdivided_network.add_edges_from(additional_edges)
            else:
                subdivided_network.add_edges_from([
                    (u, v, {'length': original_length_m}),
                    (v, u, {'length': original_length_m})
                ])

        return subdivided_network


class PathPlanner:
    SOURCE_NODE_ID = 'SOURCE'
    SINK_NODE_ID = 'DESTINATION'

    street_network: StreetNetwork
    permanent_flightplans: List[SNFlightplan]
    temporary_flightplans: List[SNFlightplan]
    timestep_s: int
    network: nx.DiGraph
    edge_length_m: float

    def __init__(self, street_network: StreetNetwork, timestep_s: int, edge_length_m: float, default_gdp: GDP):
        self.street_network = street_network
        self.default_gdp = default_gdp
        self.permanent_flightplans = []
        self.temporary_flightplans = []
        self.timestep_s = timestep_s
        self.edge_length_m = edge_length_m

        self.network = self.street_network.get_subdivided_network(edge_length_m)

    @property
    def flightplans(self):
        flightplans = []
        flightplans.extend(self.permanent_flightplans)
        flightplans.extend(self.temporary_flightplans)
        return flightplans

    def _list_of_occupied_nodes_for_request(self, time: int, request: SNRequest) -> Set[str]:
        def squared_distance(p1, p2):
            return (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2

        occupied_nodes = set()

        for plan in self.flightplans:
            for time_uncertainty in range(max(-self._request_time_uncertainty_to_ticks(request), time - plan.end_time),
                                          min(plan.time_uncertainty + 1, time - plan.start_time + 1)):
                if plan.is_present_at_time_with_uncertainty(time, self._request_time_uncertainty_to_ticks(request)):
                    radius = plan.uncertainty_radius_m + request.uncertainty_radius_m
                    for position_node in plan.position_at_time(time - time_uncertainty):
                        position = (self.network.nodes[position_node]['norm_x'], self.network.nodes[position_node]['norm_y'])
                        nodes_covered_by_flightplan = [node for node in self.network.nodes if squared_distance(position, (self.network.nodes[node]['norm_x'], self.network.nodes[node]['norm_y'])) < radius**2]
                        occupied_nodes.update(nodes_covered_by_flightplan)

        return occupied_nodes

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
        return time_ext_node_id.split(':')[0]

    def _request_time_to_pass_node(self, request: SNRequest) -> int:
        return math.ceil(self.timestep_s/(self.edge_length_m / request.speed_m_s))

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
        occupied = self._list_of_occupied_nodes_for_request(time + self._request_time_to_pass_node(request), request)

        pending_movements = self.list_of_pending_drone_movements(time + self._request_time_to_pass_node(request))

        nodes = self.network.neighbors(node_id)
        time_augmented_nodes = [self.time_ext_node_id(x, time + self._request_time_to_pass_node(request)) for x in nodes]
        time_augmented_nodes.append(self.time_ext_node_id(node_id, time + self._request_time_to_pass_node(request)))

        neighbours = []
        for node in time_augmented_nodes:
            if self._get_node_id(node) not in occupied and (self._get_node_id(node), node_id) not in pending_movements:
                neighbours.append((node, 1 + self._get_edge_penalty(node_id, self._get_node_id(node))))

        if self._get_node_id(node_id) == request.destination_node:
            neighbours.append((self.SINK_NODE_ID, 0))

        return neighbours

    def _find_shortest_path(self, request: SNRequest) -> Tuple[List[str], float]:
        self.visited_nodes = set()
        self._nodes_weights = {}
        self._queue = queue.PriorityQueue()
        self._previous_nodes = {}

        # self._queue.put((0, self.SOURCE_NODE_ID))

        for i in range(self._request_gdp_or_default(request).max_time + 1):
            weight = i * self._request_gdp_or_default(request).penalty
            occupied = self._list_of_occupied_nodes_for_request(request.start_time + i, request)
            node = self.time_ext_node_id(request.start_node, request.start_time + i)

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

        return path, self._nodes_weights[path[-1]]

    def is_destination_reachable(self, request: SNRequest):
        return nx.has_path(self.network, request.start_node, request.destination_node)

    def find_closest_node(self, x: float, y: float) -> str:
        return min([node for node in self.network.nodes], key=lambda node: distance.distance(
            (self.network.nodes[node]['y'], self.network.nodes[node]['x']),
            (y, x)
        ))

    def convert_request_to_sn(self, request: Request) -> SNRequest:
        start_node = self.find_closest_node(request.origin[0], request.origin[1])
        end_node = self.find_closest_node(request.destination[0], request.destination[1])
        sn_request = SNRequest(start_node, end_node, request.departure_time, request.uncertainty_radius_m, request.speed_m_s, request.time_uncertainty_s, request.gdp)
        return sn_request

    def resolve_request(self, request: Union[Request, SNRequest]):
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

        new_flightplan = SNFlightplan(nodes=points, speed_node=math.floor(1 / self._request_time_to_pass_node(request)),
                                      time_uncertainty=self._request_time_uncertainty_to_ticks(request), radius_m=request.uncertainty_radius_m, request=request)
        new_flightplan.weight = weight

        return new_flightplan

    def resolve_requests(self, requests: List[SNRequest]) -> List[SNFlightplan]:
        self.temporary_flightplans = []
        for request in requests:
            flightplan = self.resolve_request(request)
            self.temporary_flightplans.append(flightplan)

        return self.temporary_flightplans

    def add_flightplan(self, flightplan: SNFlightplan):
        self.permanent_flightplans.append(flightplan)
