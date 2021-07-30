import itertools
import math
import pickle
from typing import List, Set, Tuple

import pyproj
from geopy import distance

import bluesky as bs
import numpy as np
from ground_routing.common import GDP, DiskGeofence, Geofence, Request, PathNotFoundException
from ground_routing.planner import Layer, Flightplan, RoutePlanner
from ground_routing.street_network.path_planner import PathPlanner

from bluesky import stack
from bluesky.traffic.asas import ConflictResolution


def init_plugin():
    # Addtional initilisation code
    # Configuration parameters

    config = {
        # The name of your plugin
        'plugin_name': 'GROUNDCONFLICTRESOLUTION',
        # The type of this plugin. For now, only simulation plugins are possible.
        'plugin_type': 'sim'
    }

    return config


class GroundConflictResolution(ConflictResolution):
    LAYER_TOLERANCE_M = 10
    GEOFENCE_TIME_S = 120
    FLIGHTPLAN_DELAY_S = 8

    layers: List[Layer]
    flightplans: List[Flightplan]
    geofenced_conflicts_list: Set[Tuple[str, str]]
    slowed_down_drones_list: Set[str]
    route_planner: RoutePlanner

    def __init__(self):
        super().__init__()
        with open('../routing_hex/cache.pickle', 'rb') as f:
            d = pickle.load(f)
            self.route_planner = d['route_planner']
            self.flightplans = d['flightplans']
            self.layers = self.route_planner.layers

        self.geofenced_conflicts_list = set()
        self.slowing_down_drones_list = set()
        self.slowed_down_drones_list = set()

    def _get_original_drone_id(self, id: str):
        return int(id[1:])

    def _add_temporary_geofence(self, idss, idxs) -> Tuple[int, Geofence]:
        layer_id, layer = self._get_drone_layer(idxs[0])
        layer_id2, layer2 = self._get_drone_layer(idxs[1])
        assert layer_id == layer_id2

        lats = bs.traf.lat[list(idxs)]
        lons = bs.traf.lon[list(idxs)]


        radius1 = self._get_flightplan_by_id(idss[0]).uncertainty_radius_m
        radius2 = self._get_flightplan_by_id(idss[1]).uncertainty_radius_m

        if radius2 < radius1:
            radius1, radius2 = radius2, radius1
            lats = reversed(lats)
            lons = reversed(lons)

        x1, y1 = self.layers[0].path_planner.street_network.coordinate_to_norm(lats[0], lons[0])
        x2, y2 = self.layers[0].path_planner.street_network.coordinate_to_norm(lats[1], lons[1])

        D = np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

        if D + radius1 <= radius2:
            geofence_radius = radius2
            geofence_center = (x1, y1)
        else:
            geofence_radius = (D + radius1 + radius2)/2
            a = 0.5 + (radius2 - radius1)/(2 * D)
            geofence_center = ((1 - a) * x1 + a * x2, (1 - a) * y1 + a * y2)

        new_geofence = DiskGeofence((bs.sim.simt, bs.sim.simt + self.GEOFENCE_TIME_S), geofence_radius, geofence_center)

        layer.path_planner.add_geofence(new_geofence)

        return layer_id, new_geofence

    def _get_flightplan_idx_by_id(self, id: str) -> int:
        for i, flightplan in enumerate(self.flightplans):
            if flightplan.id == id:
                return i

        raise Exception

    def _get_flightplan_by_id(self, id: str) -> Flightplan:
        return self.flightplans[self._get_flightplan_idx_by_id(id)]

    def _get_drone_layer(self, idx) -> Tuple[int, Layer]:
        for i, layer in enumerate(self.layers):
            if abs(layer.altitude_m - bs.traf.alt[idx]) < self.LAYER_TOLERANCE_M:
                return i, layer

    def _clear_lists_from_resolved_conflicts(self, drones_in_conflicts: Set[str]):
        drones_to_remove = []

        for drone in self.slowed_down_drones_list:
            if drone not in drones_in_conflicts:
                drones_to_remove.append(drone)

        for drone in drones_to_remove:
            self.slowed_down_drones_list.remove(drone)

        conflicts_to_remove = []

        for geofenced_conflict in self.geofenced_conflicts_list:
            if geofenced_conflict[0] not in drones_in_conflicts or geofenced_conflict[1] not in drones_in_conflicts:
                conflicts_to_remove.append(geofenced_conflict)

        for conflict in conflicts_to_remove:
            self.geofenced_conflicts_list.remove(conflict)

    def resolve(self, conf, ownship, intruder):
        alt = ownship.ap.alt
        newtrack = ownship.ap.trk
        newgscapped = ownship.ap.tas
        newvs = ownship.ap.vs

        drones_in_conflicts = set([x for i, x in enumerate(bs.traf.id) if bs.traf.cr.active[i]])

        self._clear_lists_from_resolved_conflicts(drones_in_conflicts)

        affected_layers = set()
        to_deconflict = set()
        new_geofences = []

        for idx in list(itertools.compress(range(len(bs.traf.cr.active)), bs.traf.cr.active)):
            # Find the pairs in which IDX is involved in a conflict
            ids_pairs = [x for x in conf.confpairs if x[0] == bs.traf.id[idx]]
            idx_pairs = [(bs.traf.id.index(x[0]), bs.traf.id.index(x[1])) for x in ids_pairs]

            if bs.traf.id[idx] not in self.slowed_down_drones_list and bs.traf.id[idx] not in self.slowing_down_drones_list:
                self.slowing_down_drones_list.add(bs.traf.id[idx])

            if bs.traf.id[idx] not in self.slowed_down_drones_list:
                if ownship.ap.tas[idx] < 0.1: # the drone has stopped
                    self.slowed_down_drones_list.add(bs.traf.id[idx])
                    self.slowing_down_drones_list.remove(bs.traf.id[idx])
                    to_deconflict.add(bs.traf.id[idx])

                newgscapped[idx] = 0

            for i, pair in enumerate(ids_pairs):
                if pair not in self.geofenced_conflicts_list:
                    affected_layer, new_geofence = self._add_temporary_geofence(pair, idx_pairs[i])
                    affected_layers.add(affected_layer)
                    new_geofences.append((affected_layer, new_geofence))
                    self.geofenced_conflicts_list.add(pair)

        self.reroute_other_drones(affected_layers, to_deconflict, new_geofences)
        self.reroute_conflict_drones(affected_layers, to_deconflict)

        return newtrack, newgscapped, newvs, alt

    # def _find_next_best_node(self, current_position: Tuple[float, float], current_heading: float, layer_id: int, speed_m_s: float) -> Tuple[float, float]:
    #     network = self.route_planner.layers[layer_id].path_planner.get_time_cost_enhanced_network(speed_m_s)
    #     d = distance.great_circle
    #     geodesic = pyproj.Geod(ellps='WGS84')
    #
    #     best_node = None
    #     best_node_dist = math.inf
    #
    #     for node, data in network.nodes(data=True):
    #         heading, _, _ = geodesic.inv(current_position[0], current_position[1], data['x'], data['y'])
    #
    #         if heading < 30:
    #             dist = d(
    #                 (data['y'], data['x']),
    #                 (current_position[1], current_position[0])
    #             ).m
    #
    #             if dist < best_node_dist:
    #                 best_node = node
    #                 best_node_dist = dist
    #
    #     return

    def _find_next_node(self, flightplan: Flightplan) -> Tuple[float, float]:
        idx = bs.traf.id.index(flightplan.id)

        return (bs.traf.actwp.lon[idx], bs.traf.actwp.lat[idx])

    def reroute_other_drones(self, affected_layers: Set[int], to_deconflict: Set[str], new_geofences: List[Tuple[int, Geofence]]):
        for layer_id in affected_layers:
            rerouted_flightplans = []
            layer = self.layers[layer_id]
            layer_flightplans = [x for x in self.flightplans if x.layer == layer_id]

            self.route_planner.geofences = self.layers[layer_id].path_planner.geofences

            for flightplan in layer_flightplans:
                if self.flightplan_needs_rerouting(flightplan, to_deconflict, new_geofences, layer_id, rerouted_flightplans):
                    start_position = self._find_next_node(flightplan)

                    destination_position = flightplan.waypoints[-1].x, flightplan.waypoints[-1].y

                    request = Request(flightplan.id, start_position, destination_position, flightplan.time_uncertainty_s,
                                flightplan.speed_m_s, math.floor(bs.sim.simt+self.FLIGHTPLAN_DELAY_S), flightplan.uncertainty_radius_m,
                                GDP(1, 100, 4))

                    sn_request = layer.path_planner.convert_request_to_sn(request, True)

                    new_sn_flightplan = layer.path_planner.resolve_request(sn_request)
                    new_flightplan = Flightplan.from_sn_flightplan(
                        layer.path_planner.get_time_cost_enhanced_network(flightplan.speed_m_s), new_sn_flightplan, layer=layer_id
                    )
                    layer.path_planner.planned_flightplans.append(new_flightplan)
                    rerouted_flightplans.append(new_flightplan)

                    self.flightplans[self._get_flightplan_idx_by_id(flightplan.id)] = new_flightplan

                    stack.stack('DELRTE {}'.format(flightplan.id))

                    new_plan = self.route_planner.get_flightplan_M2_waypoints(new_flightplan, layer_id, no_time=True)

                    new_plan = new_plan.split('\n')

                    fp_starts = min([i for i, x in enumerate(new_plan) if not x.startswith('DEFWPT2')])

                    for i in range(fp_starts):
                        stack.stack(new_plan[i])

                    stack.stack('ADDWPT {id} FLYTURN'.format(id=flightplan.id))
                    stack.stack('ADDWPT {id} TURNSPEED 5'.format(id=flightplan.id))

                    stack.stack(new_plan[fp_starts])
                    stack.stack('ADDWPT {id} FLYBY'.format(id=flightplan.id))

                    for command in new_plan[fp_starts+1:]:
                        stack.stack(command)

                    stack.stack("LNAV {id} ON\n".format(id=flightplan.id))
                    stack.stack("VNAV {id} ON\n".format(id=flightplan.id))
                else:
                    layer.path_planner.planned_flightplans.append(flightplan)

    def flightplan_needs_rerouting(self, flightplan: Flightplan, to_deconflict: Set[str], new_geofences: List[Tuple[int, Geofence]], layer_id: int, rerouted_flightplans: List[Flightplan]) -> bool:
        if flightplan.id not in to_deconflict and flightplan.id not in self.slowing_down_drones_list:
            for geofence_layer, geofence in new_geofences:
                if geofence_layer == layer_id:
                    if self.route_planner._flightplan_intersects_geofence(flightplan, geofence):
                        return True

            for rerouted_flightplan in rerouted_flightplans:
                intersection_in_time = self.route_planner._intersection_in_time(flightplan, rerouted_flightplan)
                if intersection_in_time and self.route_planner._initial_intersection_check(flightplan, rerouted_flightplan):
                    if self.route_planner._find_intersection_by_simulation(flightplan, rerouted_flightplan, intersection_in_time):
                        return True

        return False

    def reroute_conflict_drones(self, affected_layers: Set[int], to_deconflict: Set[str]):
        for layer_id in affected_layers:
            layer = self.layers[layer_id]
            layer_flightplans = [x for x in self.flightplans if x.layer == layer_id]

            self.route_planner.geofences = self.layers[layer_id].path_planner.geofences

            for flightplan in layer_flightplans:
                pass
