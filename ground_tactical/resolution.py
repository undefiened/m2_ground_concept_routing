import itertools
import math
import pickle
from typing import List, Set, Tuple

import pyproj
from geopy import distance

import bluesky as bs
import numpy as np
from ground_routing.common import GDP, DiskGeofence, Geofence, Request, PathNotFoundException
from ground_routing.planner import Layer, Flightplan, PathPlanner
from ground_routing.street_network.path_planner import SNPathPlanner

from bluesky import stack
from bluesky.core import timed_function
from bluesky.traffic.asas import ConflictResolution

NUMBER_OF_DECONFLICTION_ATTEMPTS = 4


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
    FLIGHTPLAN_DELAY_S = 2
    HOVERING_TRESHOLD = 3
    TIME_UNCERTAINTY_AFTER_REPLANNING_S = 5
    TAS_SPEED_AFTER_SLOWING_DOWN = 1

    layers: List[Layer]
    flightplans: List[Flightplan]
    geofenced_conflicts_list: Set[Tuple[str, str]]
    resolved_conflicts: Set[Tuple[str, str]]
    slowed_down_drones_list: Set[str]
    route_planner: PathPlanner
    flightplans_waiting: Set[str]

    def __init__(self):
        super().__init__()
        with open('../routing_hex/cache.pickle', 'rb') as f:
            d = pickle.load(f)
            self.route_planner = d['route_planner']
            self.requests = d['requests']
            self.flightplans = d['flightplans']
            self.layers = self.route_planner.layers

        self.geofenced_conflicts_list = set()
        self.slowing_down_drones_list = set()
        self.slowed_down_drones_list = set()
        self.resolved_conflicts = set()
        self.flightplans_waiting = set()

        self.failed_to_reroute_conflicting_drones = set()

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
        new_geofence.involved_drones.add(idss[0])
        new_geofence.involved_drones.add(idss[1])

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

        affected_layers = set()
        to_deconflict = set()
        new_geofences = []

        newgscapped, new_geofences, affected_layers = self.slow_down_and_geofence_conflicting_drones(affected_layers, conf, new_geofences, newgscapped)

        self.reroute_other_drones(affected_layers, to_deconflict, new_geofences)

        return newtrack, newgscapped, newvs, alt

    def slow_down_and_geofence_conflicting_drones(self, affected_layers, conf, new_geofences, newgscapped):
        for idx in list(itertools.compress(range(len(bs.traf.cr.active)), bs.traf.cr.active)):
            # Find the pairs in which IDX is involved in a conflict
            ids_pairs = [x for x in conf.confpairs if x[0] == bs.traf.id[idx]]
            idx_pairs = [(bs.traf.id.index(x[0]), bs.traf.id.index(x[1])) for x in ids_pairs]

            if bs.traf.id[idx] not in self.slowed_down_drones_list and bs.traf.id[
                idx] not in self.slowing_down_drones_list:
                self.slowing_down_drones_list.add(bs.traf.id[idx])

            if bs.traf.id[idx] in self.slowing_down_drones_list:
                newgscapped[idx] = 0

            for i, pair in enumerate(ids_pairs):
                if pair not in self.geofenced_conflicts_list and (
                pair[1], pair[0]) not in self.geofenced_conflicts_list:
                    affected_layer, new_geofence = self._add_temporary_geofence(pair, idx_pairs[i])
                    affected_layers.add(affected_layer)
                    new_geofences.append((affected_layer, new_geofence))
                    self.geofenced_conflicts_list.add(pair)

        return newgscapped, new_geofences, affected_layers

    @timed_function()
    def slow_down_conflicting_drones(self):
        for flightplan_id in self.slowing_down_drones_list:
            idx = self._get_flightplan_idx_by_id(flightplan_id)
            self._slow_down_drone(idx)

    @timed_function()
    def resolve_slowed_down_drones(self):
        to_deconflict = set()
        affected_layers = set()

        for flightplan_id in self.slowing_down_drones_list:
            self.continue_slowing_down_until_stopped(flightplan_id)

        for geofenced_conflict in self.geofenced_conflicts_list:
            if self._both_drones_stopped(geofenced_conflict) and geofenced_conflict not in self.resolved_conflicts:
                for id in geofenced_conflict:
                    idx = self._get_flightplan_idx_by_id(id)
                    to_deconflict.add(bs.traf.id[idx])

                    layer_id, layer = self._get_drone_layer(idx)
                    affected_layers.add(layer_id)
                    self.slowing_down_drones_list.remove(id)

                    self.resolved_conflicts.add(geofenced_conflict)

        if to_deconflict and affected_layers:
            self.reroute_conflicting_drones(affected_layers, to_deconflict)

    def _both_drones_stopped(self, geofenced_conflict):
        return geofenced_conflict[0] in self.slowed_down_drones_list and geofenced_conflict[
            1] in self.slowed_down_drones_list

    def continue_slowing_down_until_stopped(self, flightplan_id):
        idx = self._get_flightplan_idx_by_id(flightplan_id)
        if flightplan_id not in self.slowed_down_drones_list:
            if bs.traf.tas[idx] < 0.1:  # the drone has stopped
                self.slowed_down_drones_list.add(bs.traf.id[idx])

    @timed_function()
    def slow_down_waiting_drones(self):
        flightplans_to_remove = []

        for flightplan_id in self.flightplans_waiting:
            flightplan = self._get_flightplan_by_id(flightplan_id)
            if flightplan.departure_time > bs.sim.simt - self.HOVERING_TRESHOLD:
                idx = self._get_flightplan_idx_by_id(flightplan_id)
                self._slow_down_drone(idx)
                self._point_drone_towards_next_waypoint(idx)
            else:
                flightplans_to_remove.append(flightplan_id)

        for flightplan_id in flightplans_to_remove:
            self.flightplans_waiting.remove(flightplan_id)
            self._return_drone_to_flying(flightplan_id)

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
                    request = self._create_replanning_request_for_other_drone(flightplan)

                    new_sn_flightplan = layer.path_planner.resolve_request(layer.path_planner.convert_request_to_sn(request, True))
                    new_flightplan = Flightplan.from_sn_flightplan(
                        layer.path_planner.get_time_cost_enhanced_network(flightplan.speed_m_s), new_sn_flightplan, layer=layer_id
                    )
                    if new_flightplan.departure_time > bs.sim.simt + self.HOVERING_TRESHOLD:
                        self.flightplans_waiting.add(new_flightplan.id)

                    layer.path_planner.planned_flightplans.append(new_flightplan)
                    rerouted_flightplans.append(new_flightplan)

                    self.flightplans[self._get_flightplan_idx_by_id(flightplan.id)] = new_flightplan

                    self.enter_new_flightplan_to_sim(flightplan, layer_id, new_flightplan)
                else:
                    if flightplan.id not in self.slowed_down_drones_list and flightplan.id not in self.slowing_down_drones_list:
                        layer.path_planner.planned_flightplans.append(flightplan)

    def _create_replanning_request_for_other_drone(self, flightplan):
        if flightplan.id in bs.traf.id:
            start_position = self._find_next_node(flightplan)

            destination_position = flightplan.waypoints[-1].x, flightplan.waypoints[-1].y

            request = Request(flightplan.id, start_position, destination_position,
                              self.TIME_UNCERTAINTY_AFTER_REPLANNING_S,
                              flightplan.speed_m_s, math.floor(bs.sim.simt + self.FLIGHTPLAN_DELAY_S),
                              flightplan.uncertainty_radius_m,
                              GDP(1, 100, 4))
        else:
            request = next(x for x in self.requests if x.id == flightplan.id)
        return request

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

    def _find_starting_positions_for_deconfliction(self, flightplans: List[Flightplan]) -> List[Tuple[float, float]]:
        # TODO: this is a quick draft
        # proper way: look at the network, find the closest node, say that it is the current position, look at neighbours instead
        d = distance.great_circle

        starting_positions = []

        network = self.route_planner.top_layer.path_planner.subdivided_network

        for flightplan in flightplans:
            idx = self._get_flightplan_idx_by_id(flightplan.id)

            nodes = [node for node, data in network.nodes(data=True) if (data['x'], data['y']) not in starting_positions]

            closest = min(nodes, key=lambda node: d(
                (network.nodes[node]['y'], network.nodes[node]['x']),
                (bs.traf.lat[idx], bs.traf.lon[idx])
            ).m)

            closest_position = (network.nodes[closest]['x'], network.nodes[closest]['y'])
            starting_positions.append(closest_position)

        return starting_positions

    def reroute_conflicting_drones(self, affected_layers: Set[int], to_deconflict: Set[str]):
        # TODO: run another round if failed to reroute some drone
        for layer_id in affected_layers:
            layer_flightplans = [x for x in self.flightplans if x.layer == layer_id and x.id in to_deconflict]
            flightplans_starting_positions = self._find_starting_positions_for_deconfliction(layer_flightplans)

            self.route_planner.geofences = self.layers[layer_id].path_planner.geofences

            rerouted_flightplans: Set[str] = set()

            for i in range(NUMBER_OF_DECONFLICTION_ATTEMPTS):
                for flightplan_i, flightplan in enumerate(layer_flightplans):
                    if flightplan.id not in rerouted_flightplans:
                        sn_request = self._compute_sn_request_for_rerouting(flightplan, flightplan_i,
                                                                            flightplans_starting_positions, layer_id)

                        try:
                            self._prepare_path_planner(flightplan, flightplans_starting_positions,
                                                                       layer_flightplans, layer_id,
                                                                       rerouted_flightplans)

                            new_flightplan = self._compute_new_flightplan(flightplan, layer_id, sn_request)
                            print(new_flightplan)

                            self._stop_rerouted_drone_if_needed(flightplan, new_flightplan)

                            self._return_old_geofences(layer_id, flightplan)
                            rerouted_flightplans.add(new_flightplan.id)

                            self.flightplans[self._get_flightplan_idx_by_id(flightplan.id)] = new_flightplan

                            self.enter_new_flightplan_to_sim(flightplan, layer_id, new_flightplan)
                        except PathNotFoundException:
                            print('Failed to reroute {} on pass {}'.format(flightplan.id, i))

            for flightplan in layer_flightplans:
                if flightplan.id not in rerouted_flightplans:
                    self.failed_to_reroute_conflicting_drones.add(flightplan.id)

    def _compute_sn_request_for_rerouting(self, flightplan, flightplan_i, flightplans_starting_positions, layer_id):
        start_position = flightplans_starting_positions[flightplan_i]
        destination_position = flightplan.waypoints[-1].x, flightplan.waypoints[-1].y
        request = Request(flightplan.id, start_position, destination_position, self.TIME_UNCERTAINTY_AFTER_REPLANNING_S,
                          flightplan.speed_m_s, math.floor(bs.sim.simt + self.FLIGHTPLAN_DELAY_S),
                          flightplan.uncertainty_radius_m,
                          GDP(1, math.floor(self.GEOFENCE_TIME_S), 1000))
        sn_request = self.layers[layer_id].path_planner.convert_request_to_sn(request, True)
        return sn_request

    def _stop_rerouted_drone_if_needed(self, flightplan, new_flightplan):
        if new_flightplan.departure_time > bs.sim.simt + self.HOVERING_TRESHOLD:
            self.flightplans_waiting.add(new_flightplan.id)
        else:
            self._return_drone_to_flying(flightplan.id)

    def _compute_new_flightplan(self, flightplan, layer_id, sn_request):
        new_sn_flightplan = self.layers[layer_id].path_planner.resolve_request(sn_request)
        new_flightplan = Flightplan.from_sn_flightplan(
            self.layers[layer_id].path_planner.get_time_cost_enhanced_network(flightplan.speed_m_s), new_sn_flightplan,
            layer=layer_id
        )
        self.layers[layer_id].path_planner.planned_flightplans.append(new_flightplan)
        return new_flightplan

    def _return_old_geofences(self, layer_id, flightplan):
        self.layers[layer_id].path_planner.enable_geofences_involving_drone(flightplan.id)
        self.layers[layer_id].path_planner.clear_temporary_geofences()

    def _prepare_path_planner(self, flightplan, flightplans_starting_positions, layer_flightplans, layer_id,
                              rerouted_flightplans):
        tmp_geofences = [DiskGeofence(
            time=(math.floor(bs.sim.simt), math.floor(bs.sim.simt + self.GEOFENCE_TIME_S)),
            radius_m=0.1,
            center=self.layers[layer_id].path_planner.street_network.coordinate_to_norm(
                flightplans_starting_positions[k][1],
                flightplans_starting_positions[k][0]
            )
        ) for k, x in enumerate(layer_flightplans) if
            x.id != flightplan.id and x.id not in rerouted_flightplans]
        self.layers[layer_id].path_planner.disable_geofences_involving_drone(flightplan.id)
        self.layers[layer_id].path_planner.add_temporary_geofences(tmp_geofences)

    def enter_new_flightplan_to_sim(self, flightplan, layer_id, new_flightplan):
        stack.stack('DELRTE {}'.format(flightplan.id))
        new_plan = self.route_planner.get_flightplan_M2_waypoints(new_flightplan, layer_id,
                                                                  no_time=True)
        new_plan = new_plan.split('\n')
        fp_starts = min([i for i, x in enumerate(new_plan) if not x.startswith('DEFWPT2')])
        for i in range(fp_starts):
            stack.stack(new_plan[i])
        stack.stack('ADDWPT {id} FLYTURN'.format(id=flightplan.id))
        stack.stack('ADDWPT {id} TURNSPEED 5'.format(id=flightplan.id))
        stack.stack(new_plan[fp_starts])
        stack.stack('ADDWPT {id} FLYBY'.format(id=flightplan.id))
        for command in new_plan[fp_starts + 1:]:
            stack.stack(command)
        stack.stack("LNAV {id} ON\n".format(id=flightplan.id))
        stack.stack("VNAV {id} ON\n".format(id=flightplan.id))

    def _point_drone_towards_next_waypoint(self, idx):
        iwpid = bs.traf.ap.route[idx].findact(idx)
        if iwpid != -1:  # To avoid problems if there are no waypoints
            bs.traf.ap.route[idx].direct(
                idx, bs.traf.ap.route[idx].wpname[iwpid])

    def _return_drone_to_flying(self, drone_id):
        idx = self._get_flightplan_idx_by_id(drone_id)

        self.active[idx] = False
        # self.tas[idx] = 999
        bs.traf.ap.tas[idx] = 999

        to_del = []
        print(self.resopairs)
        for pair in self.resopairs:
            if pair[0] == drone_id or pair[1] == drone_id:
                to_del.append(pair)

        for pair in to_del:
            self.resopairs.remove(pair)

    def _slow_down_drone(self, idx):
        # self.tas[idx] = 0
        bs.traf.ap.tas[idx] = 0
