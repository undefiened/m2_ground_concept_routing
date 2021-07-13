import itertools
from dataclasses import dataclass
from enum import Enum, unique
from typing import Tuple, List, Dict

import networkx as nx
import numpy as np
import pyclipper
from networkx import NetworkXNoPath

from common import GDP, Request, Flightplan, TurnParamsTable
from street_network.path_planner import PathPlanner as SNPathPlanner, StreetNetwork, SNRequest
from pyclipper import *
from mip import Model, xsum, minimize, BINARY
import pyproj

PCO_SCALING_FACTOR = 10 ** 6

@dataclass
class RegionExtent:
    lat: Tuple[float, float]
    lon: Tuple[float, float]


@dataclass
class Layer:
    @unique
    class Type(Enum):
        NETWORK = 0
        HEXAGONAL = 1

    altitude_m: float
    type: Type
    path_planner: SNPathPlanner = None


class RoutePlanner:
    """
    Only street network layers are supported at the time
    """
    layers: List[Layer]
    DELTA_T = 1

    @dataclass
    class FlightplanToColor:
        available_layers: List[int]
        path: List[str]
        flightplan: Flightplan
        sn_request: SNRequest

    def __init__(self, layers: List[Layer], turn_params_table: TurnParamsTable):
        self.layers = sorted(layers, key=lambda x: x.altitude_m)
        self.turn_params_table = turn_params_table

        # self.street_network_filename = street_network_filename
        # self.timestep_s = timestep_s
        # self.edge_length_s = edge_length_m
        # self.default_gdp = default_gdp

    # def _init_layers(self):
    #     for layer in self.layers:
    #         if layer.type == Layer.Type.NETWORK:
    #             layer.path_planner = self._init_network_path_planner(layer)
    #         elif layer.type == Layer.Type.HEXAGONAL:
    #             raise NotImplementedError('Hexagonal resolution is not implemented yet')
    #             layer.path_planner = self._init_hexagonal_path_planner(layer)

    # def _init_network_path_planner(self, layer: Layer):
    #     sn = StreetNetwork.from_graphml_file(self.street_network_filename)
    #     return SNPathPlanner(sn, self.timestep_s, self.edge_length_s, self.default_gdp)
    #
    # def _init_hexagonal_path_planner(self, layer: Layer):
    #     raise NotImplementedError()

    @property
    def top_layer(self) -> Layer:
        return max(self.layers, key=lambda x: x.altitude_m)

    def _find_shortest_path_in_top_layer(self, request: SNRequest):
        flightplan = nx.shortest_path(self.top_layer.path_planner.network, request.start_node, request.destination_node, 'length')
        flightplan_length = nx.path_weight(self.top_layer.path_planner.network, flightplan, 'length')
        return flightplan, flightplan_length

    def _available_layers_intersect(self, route1: FlightplanToColor, route2: FlightplanToColor) -> bool:
        return bool(set(route1.available_layers) & set(route2.available_layers))

    def _get_flightplan_buffer(self, route: FlightplanToColor):
        points = []

        for waypoint in route.flightplan.waypoints:
            points.append((waypoint.norm_x, waypoint.norm_y))

        buffer = self._get_points_buffer(points, route.sn_request.uncertainty_radius_m)
        return buffer

    @staticmethod
    def _get_points_buffer(points: List[Tuple[float, float]], radius: float) -> List:
        pco = PyclipperOffset()
        pco.AddPath(scale_to_clipper(points, PCO_SCALING_FACTOR), pyclipper.JT_SQUARE, pyclipper.ET_OPENSQUARE)

        buffer = scale_from_clipper(pco.Execute(radius * PCO_SCALING_FACTOR),
                                    PCO_SCALING_FACTOR)

        return buffer

    @staticmethod
    def _do_two_buffers_intersect(buffer1, buffer2) -> bool:
        pco = Pyclipper()
        b1 = scale_to_clipper(buffer1, PCO_SCALING_FACTOR)
        for p in b1:
            pco.AddPath(p, pyclipper.PT_SUBJECT, True)

        b2 = scale_to_clipper(buffer2, PCO_SCALING_FACTOR)
        for p in b2:
            pco.AddPath(p, pyclipper.PT_CLIP, True)

        res = pco.Execute(pyclipper.CT_INTERSECTION, pyclipper.PFT_EVENODD, pyclipper.PFT_EVENODD)

        return len(res) > 0

    def _initial_intersection_check(self, route1: FlightplanToColor, route2: FlightplanToColor) -> bool:
        buffer1 = self._get_flightplan_buffer(route1)
        buffer2 = self._get_flightplan_buffer(route2)

        return self._do_two_buffers_intersect(buffer1, buffer2)

    @staticmethod
    def _intersection_in_time(route1: FlightplanToColor, route2: FlightplanToColor) -> Tuple[int, int]:
        t_from = max(route1.flightplan.departure_time, route2.flightplan.departure_time)
        t_to = min(route1.flightplan.destination_time + route1.flightplan.time_uncertainty_s, route2.flightplan.destination_time + route2.flightplan.time_uncertainty_s)

        if t_from > t_to:
            return None
        else:
            return t_from, t_to

    def _find_intersection_by_simulation(self, route1, route2, intersection_in_time):
        for t in np.arange(intersection_in_time[0], intersection_in_time[1] + self.DELTA_T / 1000, self.DELTA_T):
            pos1 = [(x.norm_x, x.norm_y) for x in route1.flightplan.position_range(t)]
            pos2 = [(x.norm_x, x.norm_y) for x in route2.flightplan.position_range(t)]

            buffer1 = self._get_points_buffer(pos1, route1.sn_request.uncertainty_radius_m)
            buffer2 = self._get_points_buffer(pos2, route2.sn_request.uncertainty_radius_m)

            if self._do_two_buffers_intersect(buffer1, buffer2):
                return True

        return False

    def _two_routes_intersect(self, route1: FlightplanToColor, route2: FlightplanToColor) -> bool:
        intersection_in_time = self._intersection_in_time(route1, route2)
        if intersection_in_time and self._available_layers_intersect(route1, route2) and self._initial_intersection_check(route1, route2):
            return self._find_intersection_by_simulation(route1, route2, intersection_in_time)
        else:
            return False

    def _prepare_flightplans_to_color(self, requests: List[Request]) -> List[FlightplanToColor]:
        flightplans_to_color = []

        for request in requests:
            sn_request = self.top_layer.path_planner.convert_request_to_sn(request)
            shortest_path, shortest_path_length = self._find_shortest_path_in_top_layer(sn_request)

            available_layers = []

            for i, layer in enumerate(self.layers):
                layer_available = True
                try:
                    nx.path_weight(layer.path_planner.network, shortest_path, 'length')
                except NetworkXNoPath as e:
                    layer_available = False

                if layer_available:
                    available_layers.append(i)

            flightplans_to_color.append(
                self.FlightplanToColor(available_layers, shortest_path, Flightplan.from_graph_path(self.top_layer.path_planner.network, shortest_path, request), sn_request)
            )

        return flightplans_to_color

    def _find_flightplans_intersections(self, flightplans_to_color: List[FlightplanToColor]) -> List[Tuple[int, int]]:
        flightplans_intersection = []

        for pair in itertools.combinations(range(len(flightplans_to_color)), 2):
            d1 = flightplans_to_color[pair[0]]
            d2 = flightplans_to_color[pair[1]]

            if self._two_routes_intersect(d1, d2):
                flightplans_intersection.append((pair[0], pair[1]))

        return flightplans_intersection

    def _attempt_coloring(self, requests: List[Request]) -> Tuple[List[int], List[FlightplanToColor]]:
        flightplans_to_color = self._prepare_flightplans_to_color(requests)
        requests_intersections = self._find_flightplans_intersections(flightplans_to_color)
        colored_flightplans = self._color_flightplans(flightplans_to_color, requests_intersections)

        return colored_flightplans, flightplans_to_color

    def _color_flightplans(self, requests_to_color: List[FlightplanToColor], requests_intersection: List[Tuple[int, int]]) -> List[int]:
        number_of_drones = len(requests_to_color)
        number_of_layers = len(self.layers)

        m = Model("Coloring")
        d = [[m.add_var(var_type=BINARY) for j in range(number_of_layers)] for i in range(number_of_drones)]
        l_used = [m.add_var(var_type=BINARY) for j in range(number_of_layers)]

        m.objective = minimize(xsum(l_used[j] for j in range(number_of_layers)))
        for i in range(number_of_drones):
            m += xsum(d[i][j] for j in range(number_of_layers)) == 1

            for j in range(number_of_layers):
                if j not in requests_to_color[i].available_layers:
                    m += d[i][j] == 0

        for u, v in requests_intersection:
            for j in range(number_of_layers):
                m += d[u][j] + d[v][j] <= l_used[j]

        m.optimize()

        if m.num_solutions:
            selected_layers = []
            for i in range(number_of_drones):
                selected = [j for j in range(number_of_layers) if d[i][j].x >= 0.99]
                selected_layers.append(selected[0])

            return selected_layers
        else:
            return None

    def convert_flightplans_to_M2_scenarios(self, flightplans: List[Flightplan], layers: List[int]) -> List[str]:
        pass

    def resolve_requests(self, requests: List[Request]):
        """
        TODO: find shortest path on the top layer.
        Find all layers where the shortest path is fine.
        Solve IP to assign all requests to different hex layers (where possible)
        Use python-mip https://docs.python-mip.com/en/latest/examples.html + https://projects.coin-or.org/Cbc solver (may switch to Gurobi if needed)
        Start with hex shortest paths probably. Then if there is time, switch to https://pypi.org/project/pyclipper/ and straight routes.
        """
        layers, flightplans = self._attempt_coloring(requests)

        if layers:
            return self.convert_flightplans_to_M2_scenarios([x.flightplan for x in flightplans], layers)
        else:
            layers = []
            flightplans = []

            for flightplan in flightplans:
                layers_flightplans = []

                for layer in self.layers:
                    layers_flightplans.append(layer.path_planner.resolve_request(flightplan.sn_request))

                best_flightplan_layer, best_flightplan = min(enumerate(layers_flightplans), key=lambda x: x[1].weight)
                self.layers[best_flightplan_layer].path_planner.add_flightplan(best_flightplan)
                layers.append(best_flightplan_layer)
                flightplans.append(Flightplan.from_sn_flightplan(self.layers[best_flightplan_layer].path_planner.network, best_flightplan))

            return self.convert_flightplans_to_M2_scenarios(flightplans, layers)

    def convert_flightplans_to_M2_scenarios(self, flightplans: List[Flightplan], layers: List[int]) -> List[str]:
        scenarios = []
        for i in range(len(flightplans)):
            scenarios.append(self.convert_flightplan_to_M2_scenario(flightplans[i], layers[i], i))

        return scenarios

    def convert_flightplan_to_M2_scenario(self, flightplan: Flightplan, layer: int, id: int) -> str:
        geodesic = pyproj.Geod(ellps='WGS84')

        scenario = ""
        scenario += "00:00:00>HOLD\n"
        scenario += "00:00:00>PAN 48.223775 16.337976\n"
        scenario += "00:00:00>ZOOM 50\n"
        scenario += "00:00:00>VIS MAP TILEDMAP\n"

        source = flightplan.waypoints[0]
        spd = 30
        initial_heading, _, _ = geodesic.inv(source.x, source.y, flightplan.waypoints[1].x, flightplan.waypoints[1].y)

        scenario += "00:00:00>CRE D{id} M600 {lat} {lon} {hdg} {alt} {spd}\n".format(id=id, lat=source.y, lon=source.x, hdg=initial_heading, alt=self.layers[layer].altitude_m * 3.281, spd=spd)

        scenario += "00:00:00>ADDWPT D{id} {lat} {lon} {alt} {spd}\n".format(id=id, lat=flightplan.waypoints[0].y,
                                                                             lon=flightplan.waypoints[0].x,
                                                                             alt=self.layers[
                                                                                     layer].altitude_m * 3.281,
                                                                             spd=spd)

        last_was_turn = False

        for i in range(1, len(flightplan.waypoints)):
            waypoint = flightplan.waypoints[i]

            if i < len(flightplan.waypoints) - 1:
                heading1, _, _ = geodesic.inv(flightplan.waypoints[i-1].x, flightplan.waypoints[i-1].y, flightplan.waypoints[i].x,
                                                     flightplan.waypoints[i].y)
                heading2, _, _ = geodesic.inv(flightplan.waypoints[i].x, flightplan.waypoints[i].y, flightplan.waypoints[i+1].x,
                                                     flightplan.waypoints[i+1].y)

                if self.turn_params_table.is_turn_penalized(abs(heading1 - heading2)):
                    if not last_was_turn:
                        scenario += "00:00:00>ADDWPT D{id} FLYTURN\n".format(id=id)
                        scenario += "00:00:00>ADDWPT D{id} TURNSPEED {turn_speed}\n".format(id=id, turn_speed=self.turn_params_table.get_turn_speed_knots(abs(heading1 - heading2)))

                        last_was_turn = True
                else:
                    scenario += "00:00:00>ADDWPT D{id} FLYBY\n".format(id=id)
                    last_was_turn = False


            scenario += "00:00:00>ADDWPT D{id} {lat} {lon} {alt} {spd}\n".format(id=id, lat=waypoint.y,
                                                                                 lon=waypoint.x,
                                                                                 alt=self.layers[
                                                                                         layer].altitude_m * 3.281,
                                                                                 spd=spd)

        scenario += "00:00:00>LNAV D{id} ON\n".format(id=id)
        scenario += "00:00:00>VNAV D{id} ON\n".format(id=id)

        return scenario
