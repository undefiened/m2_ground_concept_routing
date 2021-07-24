import datetime
import itertools
import math
from dataclasses import dataclass
from enum import Enum, unique
from typing import Tuple, List

import networkx as nx
import numpy
import numpy as np
import pyclipper
from networkx import NetworkXNoPath

from ground_routing.common import Request, Flightplan, TurnParamsTable, PathNotFoundException, Geofence, \
    PCO_SCALING_FACTOR
from ground_routing.street_network.path_planner import PathPlanner as SNPathPlanner, SNRequest
from pyclipper import *
from mip import Model, xsum, minimize, BINARY
import pyproj



@dataclass
class RegionExtent:
    lat: Tuple[float, float]
    lon: Tuple[float, float]


@dataclass
class Layer:
    @unique
    class Type(Enum):
        NETWORK = 0
        # HEXAGONAL = 1

    altitude_m: float
    type: Type
    path_planner: SNPathPlanner = None


class RoutePlanner:
    """
    Only street network layers are supported at the time
    """
    layers: List[Layer]
    _geofences: List[Geofence]
    DELTA_T = 1

    @dataclass
    class FlightplanToColor:
        available_layers: List[int]
        path: List[str]
        flightplan: Flightplan
        sn_request: SNRequest

    def __init__(self, layers: List[Layer], turn_params_table: TurnParamsTable, geofences: List[Geofence] = []):
        self.layers = sorted(layers, key=lambda x: x.altitude_m)
        self.turn_params_table = turn_params_table
        self._geofences = geofences

    @property
    def top_layer(self) -> Layer:
        return max(self.layers, key=lambda x: x.altitude_m)

    def _find_shortest_path_in_top_layer(self, request: SNRequest):
        time_cost_enhanced_network = self.top_layer.path_planner.get_time_cost_enhanced_network(request.speed_m_s)
        flightplan = nx.shortest_path(time_cost_enhanced_network, request.start_node, request.destination_node, 'time_cost')
        flightplan_length = nx.path_weight(time_cost_enhanced_network, flightplan, 'time_cost')
        return flightplan, flightplan_length

    def _available_layers_intersect(self, route1: FlightplanToColor, route2: FlightplanToColor) -> bool:
        return bool(set(route1.available_layers) & set(route2.available_layers))

    def _get_flightplan_buffer(self, route: Flightplan):
        points = []

        for waypoint in route.waypoints:
            points.append((waypoint.norm_x, waypoint.norm_y))

        buffer = self._get_points_buffer(points, route.uncertainty_radius_m)
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
        buffer1 = self._get_flightplan_buffer(route1.flightplan)
        buffer2 = self._get_flightplan_buffer(route2.flightplan)

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
                    nx.path_weight(layer.path_planner.get_time_cost_enhanced_network(request.speed_m_s), shortest_path, 'time_cost')
                except NetworkXNoPath as e:
                    layer_available = False

                if layer_available:
                    available_layers.append(i)

            flightplans_to_color.append(
                self.FlightplanToColor(available_layers, shortest_path, Flightplan.from_graph_path(self.top_layer.path_planner.get_time_cost_enhanced_network(sn_request.speed_m_s), shortest_path, request), sn_request)
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

    def _test_overall_intersection_with_geofence(self, flightplan: Flightplan, geofence: Geofence) -> bool:
        if flightplan.departure_time > geofence.time[1] or geofence.time[0] > flightplan.destination_time:
            return False

        buffer1 = self._get_flightplan_buffer(flightplan)
        buffer2 = geofence.get_pyclipper_geometry()

        return self._do_two_buffers_intersect(buffer1, buffer2)

    def _test_intersection_with_geofence_by_simulation(self, flightplan: Flightplan, geofence: Geofence) -> bool:
        for t in np.arange(flightplan.departure_time, flightplan.destination_time + self.DELTA_T / 1000, self.DELTA_T):
            if geofence.exists_at_time(t):
                pos1 = [(x.norm_x, x.norm_y) for x in flightplan.position_range(t)]

                buffer1 = self._get_points_buffer(pos1, flightplan.uncertainty_radius_m)
                buffer2 = geofence.get_pyclipper_geometry()

                if self._do_two_buffers_intersect(buffer1, buffer2):
                    return True

        return False

    def _flightplan_intersects_geofence(self, flightplan: Flightplan, geofence: Geofence) -> bool:
        return self._test_overall_intersection_with_geofence(flightplan, geofence) \
               and self._test_intersection_with_geofence_by_simulation(flightplan, geofence)

    def _find_intersections_with_geofences(self, flightplans_to_color: List[FlightplanToColor]) -> List[bool]:
        flightplans_intersect = []
        for flightplan in flightplans_to_color:
            flightplan_intersects = False

            for geofence in self._geofences:
                if self._flightplan_intersects_geofence(flightplan.flightplan, geofence):
                    flightplan_intersects = True

            flightplans_intersect.append(flightplan_intersects)

        return flightplans_intersect

    def _attempt_coloring(self, requests: List[Request]) -> Tuple[List[int], List[FlightplanToColor]]:
        flightplans_to_color = self._prepare_flightplans_to_color(requests)
        intersections_with_geofences = self._find_intersections_with_geofences(flightplans_to_color)

        if any(intersections_with_geofences):
            return None, flightplans_to_color

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

    def resolve_requests(self, requests: List[Request], skip_coloring=False):
        """
        TODO: find shortest path on the top layer.
        Find all layers where the shortest path is fine.
        Solve IP to assign all requests to different hex layers (where possible)
        Use python-mip https://docs.python-mip.com/en/latest/examples.html + https://projects.coin-or.org/Cbc solver (may switch to Gurobi if needed)
        Start with hex shortest paths probably. Then if there is time, switch to https://pypi.org/project/pyclipper/ and straight routes.
        """
        print('ATTEMPTING COLORING')
        if not skip_coloring:
            layers, flightplans = self._attempt_coloring(requests)

            for i in range(len(flightplans)):
                flightplans[i].flightplan.layer = layers[i]
        else:
            flightplans = self._prepare_flightplans_to_color(requests)

        if not skip_coloring and layers:
            print('COLORING WAS SUCCESSFUL, THE SOLUTION IS OPTIMAL')
            return flightplans, layers
        else:
            print('COLORING FAILED, USING TIME-EXPANDED NETWORK')
            layers = []
            te_flightplans = []

            for i, flightplan in enumerate(flightplans):
                print('PLANNING FLIGHTPLAN {}'.format(i))
                layers_flightplans = []

                fp = None
                for layer in self.layers:
                    try:
                        fp = layer.path_planner.resolve_request(flightplan.sn_request)
                    except PathNotFoundException as e:
                        print(str(e) + "in layer {}".format(layer))

                    layers_flightplans.append(fp)

                best_flightplan_layer, best_flightplan = min(enumerate(layers_flightplans), key=lambda x: x[1].weight if x[1] is not None else math.inf)
                if best_flightplan is not None:
                    self.layers[best_flightplan_layer].path_planner.add_flightplan(best_flightplan)
                    layers.append(best_flightplan_layer)
                    te_flightplans.append(Flightplan.from_sn_flightplan(self.layers[best_flightplan_layer].path_planner.get_time_cost_enhanced_network(flightplan.sn_request.speed_m_s), best_flightplan, layer=best_flightplan_layer))

            return te_flightplans, layers

    def convert_flightplans_to_M2_scenarios(self, flightplans: List[Flightplan], layers: List[int]) -> List[str]:
        scenarios = []
        header = ""
        header += "00:00:00>HOLD\n"
        header += "00:00:00>ASAS ON\n"
        header += "00:00:00>PAN 48.223775 16.337976\n"
        header += "00:00:00>ZOOM 50\n"
        header += "00:00:00>VIS MAP TILEDMAP\n"
        header += "00:00:00>ZONER 0.0161987\n"
        header += "00:00:00>ZONEDH 1\n"

        scenarios.append(header)
        for i in range(len(flightplans)):
            scenarios.append(self.convert_flightplan_to_M2_scenario(flightplans[i], layers[i], i))

        return scenarios

    def convert_flightplan_to_M2_scenario(self, flightplan: Flightplan, layer: int, id: int) -> str:
        EVERY_NON_TURNING = 7

        geodesic = pyproj.Geod(ellps='WGS84')

        scenario = ""

        source = flightplan.waypoints[0]

        turning = [x.turning for x in flightplan.waypoints]

        series_of_non_turning = []

        start = 0
        started = False
        for i, t in enumerate(turning):
            if t:
                if started:
                    started = False
                    series_of_non_turning.append((start, i-1))
            else:
                if not started:
                    start = i
                    started = True

        waypoints_to_remove = []

        for serie in series_of_non_turning:
            serie_size = serie[1] - serie[0] + 1
            if serie_size > EVERY_NON_TURNING:
                number_of_waypoints_to_keep = math.floor(serie_size / EVERY_NON_TURNING)
                waypoints_to_keep = numpy.floor(numpy.linspace(serie[0], serie[1], number_of_waypoints_to_keep + 2)[1:-1])
            else:
                waypoints_to_keep = []

            for i in range(serie[0], serie[1]+1):
                if i not in waypoints_to_keep:
                    waypoints_to_remove.append(i)

        waypoints = [x for i, x in enumerate(flightplan.waypoints) if i not in waypoints_to_remove]

        waypoints_to_remove = []

        for i in range(len(waypoints) - 1):
            if abs(waypoints[i].x - waypoints[i+1].x) < 0.00001 and abs(waypoints[i].y - waypoints[i+1].y) < 0.00001:
                waypoints_to_remove.append(i)

        waypoints = [x for i, x in enumerate(waypoints) if i not in waypoints_to_remove]

        spd = 30
        initial_heading, _, _ = geodesic.inv(source.x, source.y, waypoints[1].x, waypoints[1].y)

        time = '0' + str(datetime.timedelta(seconds=flightplan.departure_time))

        for i in range(len(waypoints)):
            scenario += "00:00:00>DEFWPT D{id}_{i},{lat},{lon},FIX\n".format(id=id, i=i, lat=waypoints[i].y,
                                                                             lon=waypoints[i].x)

        scenario += "{time}>CRE D{id} M600 {lat} {lon} {hdg} {alt} {spd}\n".format(id=id, lat=source.y, lon=source.x, hdg=initial_heading, alt=self.layers[layer].altitude_m * 3.281, spd=spd, time=time)

        scenario += "{time}>ADDWPT D{id} D{id}_{i} {alt} {spd}\n".format(id=id, i=0, lat=waypoints[0].y,
                                                                             lon=waypoints[0].x,
                                                                             alt=self.layers[
                                                                                     layer].altitude_m * 3.281,
                                                                             spd=spd, time=time)

        last_was_turn = False

        for i in range(1, len(waypoints)):
            waypoint = waypoints[i]

            if i < len(waypoints) - 1:
                heading1, _, _ = geodesic.inv(waypoints[i-1].x, waypoints[i-1].y, waypoints[i].x,
                                                     waypoints[i].y)
                heading2, _, _ = geodesic.inv(waypoints[i].x, waypoints[i].y, waypoints[i+1].x,
                                                     waypoints[i+1].y)

                if self.turn_params_table.is_turn_penalized(abs(heading1 - heading2)) and (waypoints[i-1].turning or waypoints[i].turning or waypoints[i+1].turning):
                    if not last_was_turn:
                        scenario += "{time}>ADDWPT D{id} FLYTURN\n".format(id=id, time=time)
                        scenario += "{time}>ADDWPT D{id} TURNSPEED {turn_speed}\n".format(id=id, turn_speed=self.turn_params_table.get_turn_speed_knots(abs(heading1 - heading2)), time=time)

                        last_was_turn = True
                else:
                    if last_was_turn:
                        scenario += "{time}>ADDWPT D{id} FLYBY\n".format(id=id, time=time)
                        last_was_turn = False

            else:
                last_was_turn = False
                scenario += "{time}>ADDWPT D{id} FLYBY\n".format(id=id, time=time)

            if last_was_turn:
                scenario += "{time}>ADDWPT D{id} D{id}_{i} {alt} {spd}\n".format(id=id, i=i, lat=waypoint.y,
                                                                             lon=waypoint.x,
                                                                             alt=self.layers[
                                                                                     layer].altitude_m * 3.281,
                                                                             spd=spd, time=time)
            else:
                scenario += "{time}>ADDWPT D{id} D{id}_{i} {alt} ,\n".format(id=id, i=i, lat=waypoint.y,
                                                                                     lon=waypoint.x,
                                                                                     alt=self.layers[
                                                                                             layer].altitude_m * 3.281,
                                                                                     spd=spd, time=time)

            if not last_was_turn:
                scenario += "{time}>RTA D{id} D{id}_{i} {time2}\n".format(id=id, i=i, time=time, time2=str(datetime.timedelta(seconds=waypoint.time)))

        scenario += "{time}>D{id} ATDIST {lat} {lon} 0.01 DEL D{id}\n".format(id=id, lat=waypoints[-1].y, lon=waypoints[-1].x, time=time)

        scenario += "{time}>LNAV D{id} ON\n".format(id=id, time=time)
        scenario += "{time}>VNAV D{id} ON\n".format(id=id, time=time)

        return scenario
