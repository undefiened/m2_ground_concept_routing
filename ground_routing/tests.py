from unittest import TestCase

from ground_routing.common import Flightplan, Request, TurnParamsTable, TurnParams, NO_GDP, DiskGeofence
from planner import RoutePlanner, Layer
from ground_routing.street_network.path_planner import PathPlanner, StreetNetwork, SNRequest, SNFlightplan


class PlannerColoringTestCase(TestCase):
    def test_location_range_precise(self):
        fp = Flightplan(waypoints=[
            Flightplan.Waypoint(0, 0, 0, 0, 0),
            Flightplan.Waypoint(0, 1, 0, 1, 1),
            Flightplan.Waypoint(0, 2, 0, 2, 2),
            Flightplan.Waypoint(0, 3, 0, 3, 3),
            Flightplan.Waypoint(0, 4, 0, 4, 4),
        ], time_uncertainty_s=2, speed_m_s=1, uncertainty_radius_m=1)

        r = fp.position_range(0)
        self.assertEqual(r, [Flightplan.Waypoint(0, 0, 0, 0, 0), Flightplan.Waypoint(0, 0, 0, 0, 0),])

        r = fp.position_range(1)
        self.assertEqual(r, [Flightplan.Waypoint(0, 0, 0, 0, 1), Flightplan.Waypoint(0, 1, 0, 1, 1), ])

        r = fp.position_range(2)
        self.assertEqual(r, [Flightplan.Waypoint(0, 0, 0, 0, 2), Flightplan.Waypoint(0, 1, 0, 1, 2), Flightplan.Waypoint(0, 2, 0, 2, 2), ])

        r = fp.position_range(3)
        self.assertEqual(r, [Flightplan.Waypoint(0, 1, 0, 1, 3), Flightplan.Waypoint(0, 2, 0, 2, 3),
                             Flightplan.Waypoint(0, 3, 0, 3, 3), ])

        r = fp.position_range(4)
        self.assertEqual(r, [Flightplan.Waypoint(0, 2, 0, 2, 4), Flightplan.Waypoint(0, 3, 0, 3, 4),
                             Flightplan.Waypoint(0, 4, 0, 4, 4), ])

        r = fp.position_range(5)
        self.assertEqual(r, [Flightplan.Waypoint(0, 3, 0, 3, 5),
                             Flightplan.Waypoint(0, 4, 0, 4, 5), ])

        r = fp.position_range(6)
        self.assertEqual(r, [Flightplan.Waypoint(0, 4, 0, 4, 6), Flightplan.Waypoint(0, 4, 0, 4, 6)])

        self.assertRaises(Exception, fp.position_range, 7)
        self.assertRaises(Exception, fp.position_range, -1)

    def test_location_range_partial(self):
        fp = Flightplan(waypoints=[
            Flightplan.Waypoint(0, 0, 0, 0, 0),
            Flightplan.Waypoint(0, 2, 0, 2, 2),
            Flightplan.Waypoint(0, 4, 0, 4, 4),
        ], time_uncertainty_s=2, speed_m_s=1, uncertainty_radius_m=1)

        r = fp.position_range(0)
        self.assertEqual(r, [Flightplan.Waypoint(0, 0, 0, 0, 0), Flightplan.Waypoint(0, 0, 0, 0, 0), ])

        r = fp.position_range(1)
        self.assertEqual(r, [Flightplan.Waypoint(0, 0, 0, 0, 1), Flightplan.Waypoint(0, 1, 0, 1, 1), ])

        r = fp.position_range(2)
        self.assertEqual(r, [Flightplan.Waypoint(0, 0, 0, 0, 2),
                             Flightplan.Waypoint(0, 2, 0, 2, 2), ])

        r = fp.position_range(3)
        self.assertEqual(r, [Flightplan.Waypoint(0, 1, 0, 1, 3), Flightplan.Waypoint(0, 2, 0, 2, 3),
                             Flightplan.Waypoint(0, 3, 0, 3, 3), ])

        r = fp.position_range(4)
        self.assertEqual(r, [Flightplan.Waypoint(0, 2, 0, 2, 4),
                             Flightplan.Waypoint(0, 4, 0, 4, 4), ])

        r = fp.position_range(5)
        self.assertEqual(r, [Flightplan.Waypoint(0, 3, 0, 3, 5),
                             Flightplan.Waypoint(0, 4, 0, 4, 5), ])

        r = fp.position_range(6)
        self.assertEqual(r, [Flightplan.Waypoint(0, 4, 0, 4, 6), Flightplan.Waypoint(0, 4, 0, 4, 6),])

        self.assertRaises(Exception, fp.position_range, 7)
        self.assertRaises(Exception, fp.position_range, -1)

    def test_location_range_no_uncertainty(self):
        fp = Flightplan(waypoints=[
            Flightplan.Waypoint(0, 0, 0, 0, 0),
            Flightplan.Waypoint(0, 2, 0, 2, 2),
            Flightplan.Waypoint(0, 4, 0, 4, 4),
        ], time_uncertainty_s=0, speed_m_s=1, uncertainty_radius_m=1)

        r = fp.position_range(0)
        self.assertEqual(r, [Flightplan.Waypoint(0, 0, 0, 0, 0), Flightplan.Waypoint(0, 0, 0, 0, 0), ])

        r = fp.position_range(1)
        self.assertEqual(r, [Flightplan.Waypoint(0, 1, 0, 1, 1), Flightplan.Waypoint(0, 1, 0, 1, 1), ])

        r = fp.position_range(2)
        self.assertEqual(r, [Flightplan.Waypoint(0, 2, 0, 2, 2),
                             Flightplan.Waypoint(0, 2, 0, 2, 2), ])

        r = fp.position_range(3)
        self.assertEqual(r, [Flightplan.Waypoint(0, 3, 0, 3, 3), Flightplan.Waypoint(0, 3, 0, 3, 3) ])

        r = fp.position_range(4)
        self.assertEqual(r, [Flightplan.Waypoint(0, 4, 0, 4, 4),
                             Flightplan.Waypoint(0, 4, 0, 4, 4), ])

        self.assertRaises(Exception, fp.position_range, 5)
        self.assertRaises(Exception, fp.position_range, -1)

    def test_coloring(self):
        graphml_string = """<?xml version='1.0' encoding='utf-8'?>
                                                <graphml xmlns="http://graphml.graphdrawing.org/xmlns" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="http://graphml.graphdrawing.org/xmlns http://graphml.graphdrawing.org/xmlns/1.0/graphml.xsd">
                                                  <key id="d12" for="edge" attr.name="length" attr.type="string" />
                                                  <key id="d2" for="node" attr.name="x" attr.type="string" />
                                                  <key id="d1" for="node" attr.name="y" attr.type="string" />
                                                  <graph edgedefault="undirected">
                                                    <node id="1">
                                                      <data key="d1">48.2172288</data>
                                                      <data key="d2">16.3424207</data>
                                                    </node>
                                                    <node id="2">
                                                      <data key="d1">48.2183536</data>
                                                      <data key="d2">16.3429038</data>
                                                    </node>
                                                    <node id="3">
                                                      <data key="d1">48.2250679</data>
                                                      <data key="d2">16.3498014</data>
                                                    </node>
                                                    <edge source="1" target="2" id="1">
                                                      <data key="d12">130.12182509932728</data>
                                                    </edge>
                                                    <edge source="2" target="3" id="2">
                                                      <data key="d12">905.5861012274036</data>
                                                    </edge>
                                                  </graph>
                                                </graphml>
                                                """
        sn = StreetNetwork.from_graphml_string(graphml_string, recompute_lengths=True, turn_params_table=TurnParamsTable([TurnParams(0, 180, 30, 0)]))

        layers = [
            Layer(altitude_m=10, type=Layer.Type.NETWORK, path_planner=PathPlanner(sn, 1, 10, NO_GDP)),
            Layer(altitude_m=20, type=Layer.Type.NETWORK, path_planner=PathPlanner(sn, 1, 10, NO_GDP)),
        ]
        rp = RoutePlanner(layers=layers, turn_params_table=TurnParamsTable([TurnParams(0, 180, 30, 0)]))
        colors = rp._color_flightplans([RoutePlanner.FlightplanToColor([0, 1], [], None, None), RoutePlanner.FlightplanToColor([0, 1], [], None, None)], [(0, 1), ])
        self.assertEqual(colors, [0, 1])

        colors = rp._color_flightplans([RoutePlanner.FlightplanToColor([0, 1], [], None, None),
                                        RoutePlanner.FlightplanToColor([0, 1], [], None, None),
                                        RoutePlanner.FlightplanToColor([0, 1], [], None, None),], [(0, 1), (0, 2)])
        self.assertIn(colors, [[0, 1, 1], [1, 0, 0]])

        colors = rp._color_flightplans([RoutePlanner.FlightplanToColor([0, ], [], None, None)], [])
        self.assertEqual(colors, [0, ])

        colors = rp._color_flightplans([RoutePlanner.FlightplanToColor([1, ], [], None, None)], [])
        self.assertEqual(colors, [1, ])

        colors = rp._color_flightplans([RoutePlanner.FlightplanToColor([0, 1], [], None, None),
                                        RoutePlanner.FlightplanToColor([0, 1], [], None, None),
                                        RoutePlanner.FlightplanToColor([1, ], [], None, None), ], [(0, 1), (0, 2)])
        self.assertEqual(colors, [0, 1, 1])

        colors = rp._color_flightplans([RoutePlanner.FlightplanToColor([1], [], None, None),
                                        RoutePlanner.FlightplanToColor([1], [], None, None),
                                        RoutePlanner.FlightplanToColor([1, ], [], None, None), ], [(0, 1), (0, 2)])
        self.assertEqual(colors, None)

    def test_intersection_in_time(self):
        fp1 = RoutePlanner.FlightplanToColor(available_layers=[0, ], path=['1', '2', '3'],
                                             flightplan=Flightplan(waypoints=[
                                                 Flightplan.Waypoint(0, 0, 0, 0, 0),
                                                 Flightplan.Waypoint(0, 0, 0, 0, 10),
                                             ], time_uncertainty_s=0, speed_m_s=0, uncertainty_radius_m=1),
                                             sn_request=None)

        fp2 = RoutePlanner.FlightplanToColor(available_layers=[0, ], path=['1', '2', '3'],
                                             flightplan=Flightplan(waypoints=[
                                                 Flightplan.Waypoint(0, 0, 0, 0, 11),
                                                 Flightplan.Waypoint(0, 0, 0, 0, 20),
                                             ], time_uncertainty_s=0, speed_m_s=0, uncertainty_radius_m=1),
                                             sn_request=None)

        self.assertFalse(RoutePlanner._intersection_in_time(fp1, fp2))

        fp2 = RoutePlanner.FlightplanToColor(available_layers=[0, ], path=['1', '2', '3'],
                                             flightplan=Flightplan(waypoints=[
                                                 Flightplan.Waypoint(0, 0, 0, 0, 1),
                                                 Flightplan.Waypoint(0, 0, 0, 0, 5),
                                             ], time_uncertainty_s=0, speed_m_s=0, uncertainty_radius_m=1),
                                             sn_request=None)

        self.assertTrue(RoutePlanner._intersection_in_time(fp1, fp2))

        fp2 = RoutePlanner.FlightplanToColor(available_layers=[0, ], path=['1', '2', '3'],
                                             flightplan=Flightplan(waypoints=[
                                                 Flightplan.Waypoint(0, 0, 0, 0, -3),
                                                 Flightplan.Waypoint(0, 0, 0, 0, 3),
                                             ], time_uncertainty_s=0, speed_m_s=0, uncertainty_radius_m=1),
                                             sn_request=None)

        self.assertTrue(RoutePlanner._intersection_in_time(fp1, fp2))

        fp2 = RoutePlanner.FlightplanToColor(available_layers=[0, ], path=['1', '2', '3'],
                                             flightplan=Flightplan(waypoints=[
                                                 Flightplan.Waypoint(0, 0, 0, 0, 5),
                                                 Flightplan.Waypoint(0, 0, 0, 0, 15),
                                             ], time_uncertainty_s=0, speed_m_s=0, uncertainty_radius_m=1),
                                             sn_request=None)

        self.assertTrue(RoutePlanner._intersection_in_time(fp1, fp2))

        fp2 = RoutePlanner.FlightplanToColor(available_layers=[0, ], path=['1', '2', '3'],
                                             flightplan=Flightplan(waypoints=[
                                                 Flightplan.Waypoint(0, 0, 0, 0, -10),
                                                 Flightplan.Waypoint(0, 0, 0, 0, -5),
                                             ], time_uncertainty_s=0, speed_m_s=0, uncertainty_radius_m=1),
                                             sn_request=None)

        self.assertFalse(RoutePlanner._intersection_in_time(fp1, fp2))

    def test_intersection_in_time_with_uncertainty(self):
        fp1 = RoutePlanner.FlightplanToColor(available_layers=[0, ], path=['1', '2', '3'],
                                             flightplan=Flightplan(waypoints=[
                                                 Flightplan.Waypoint(0, 0, 0, 0, 10),
                                                 Flightplan.Waypoint(0, 0, 0, 0, 20),
                                             ], time_uncertainty_s=0, speed_m_s=0, uncertainty_radius_m=1),
                                             sn_request=None)

        fp2 = RoutePlanner.FlightplanToColor(available_layers=[0, ], path=['1', '2', '3'],
                                             flightplan=Flightplan(waypoints=[
                                                 Flightplan.Waypoint(0, 0, 0, 0, 0),
                                                 Flightplan.Waypoint(0, 0, 0, 0, 9),
                                             ], time_uncertainty_s=10, speed_m_s=0, uncertainty_radius_m=1),
                                             sn_request=None)

        self.assertTrue(RoutePlanner._intersection_in_time(fp1, fp2))

    def test_buffer_intersection(self):
        b1 = RoutePlanner._get_points_buffer([
            (0, 0),
            (10, 0),
        ], 10)

        b2 = RoutePlanner._get_points_buffer([
            (0, 2),
            (10, 2),
        ], 10)

        b3 = RoutePlanner._get_points_buffer([
            (0, 21),
            (10, 21),
        ], 10)

        self.assertTrue(RoutePlanner._do_two_buffers_intersect(b1, b2))
        self.assertFalse(RoutePlanner._do_two_buffers_intersect(b1, b3))
        self.assertTrue(RoutePlanner._do_two_buffers_intersect(b2, b3))

        b1 = RoutePlanner._get_points_buffer([
            (0, 0),
            (10, 0),
        ], 10)

        b2 = RoutePlanner._get_points_buffer([
            (5, 18),
            (5, 30),
        ], 10)

        self.assertTrue(RoutePlanner._do_two_buffers_intersect(b1, b2))

        b2 = RoutePlanner._get_points_buffer([
            (5, 20),
            (5, 30),
        ], 10)

        self.assertFalse(RoutePlanner._do_two_buffers_intersect(b1, b2))

    def test_flightplans_intersection(self):
        graphml_string = """<?xml version='1.0' encoding='utf-8'?>
                                                        <graphml xmlns="http://graphml.graphdrawing.org/xmlns" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="http://graphml.graphdrawing.org/xmlns http://graphml.graphdrawing.org/xmlns/1.0/graphml.xsd">
                                                          <key id="d12" for="edge" attr.name="length" attr.type="string" />
                                                          <key id="d2" for="node" attr.name="x" attr.type="string" />
                                                          <key id="d1" for="node" attr.name="y" attr.type="string" />
                                                          <graph edgedefault="undirected">
                                                            <node id="l">
                                                              <data key="d1">0</data>
                                                              <data key="d2">-1</data>
                                                            </node>
                                                            <node id="ll">
                                                              <data key="d1">0</data>
                                                              <data key="d2">-2</data>
                                                            </node>
                                                            <node id="r">
                                                              <data key="d1">0</data>
                                                              <data key="d2">1</data>
                                                            </node>
                                                            <node id="c">
                                                              <data key="d1">0</data>
                                                              <data key="d2">0</data>
                                                            </node>
                                                            <node id="t">
                                                              <data key="d1">1</data>
                                                              <data key="d2">0</data>
                                                            </node>
                                                            <node id="b">
                                                              <data key="d1">-1</data>
                                                              <data key="d2">0</data>
                                                            </node>
                                                            
                                                            
                                                            <edge source="ll" target="l" id="0">
                                                              <data key="d12">40</data>
                                                            </edge>
                                                            <edge source="l" target="c" id="1">
                                                              <data key="d12">40</data>
                                                            </edge>
                                                            <edge source="r" target="c" id="2">
                                                              <data key="d12">40</data>
                                                            </edge>
                                                            <edge source="t" target="c" id="3">
                                                              <data key="d12">40</data>
                                                            </edge>
                                                            <edge source="b" target="c" id="4">
                                                              <data key="d12">40</data>
                                                            </edge>
                                                          </graph>
                                                        </graphml>
                                                        """
        sn = StreetNetwork.from_graphml_string(graphml_string, recompute_lengths=False, turn_params_table=TurnParamsTable([TurnParams(0, 180, 30, 0)]))

        sn.original_network.nodes['c']['norm_x'] = 0
        sn.original_network.nodes['c']['norm_y'] = 0

        sn.original_network.nodes['l']['norm_x'] = -40
        sn.original_network.nodes['l']['norm_y'] = 0

        sn.original_network.nodes['r']['norm_x'] = 40
        sn.original_network.nodes['r']['norm_y'] = 0

        sn.original_network.nodes['t']['norm_x'] = 0
        sn.original_network.nodes['t']['norm_y'] = 40

        sn.original_network.nodes['b']['norm_x'] = 0
        sn.original_network.nodes['b']['norm_y'] = -40

        sn.original_network.nodes['ll']['norm_x'] = -80
        sn.original_network.nodes['ll']['norm_y'] = 0


        layers = [
            Layer(altitude_m=10, type=Layer.Type.NETWORK, path_planner=PathPlanner(sn, 1, 10, NO_GDP)),
            Layer(altitude_m=20, type=Layer.Type.NETWORK, path_planner=PathPlanner(sn, 1, 10, NO_GDP)),
        ]
        rp = RoutePlanner(layers=layers, turn_params_table=TurnParamsTable([TurnParams(0, 180, 30, 0)]))

        flightplans_to_color = rp._prepare_flightplans_to_color([
            Request((-1, 0), (1, 0), 0, 10, 0, 20, NO_GDP),
            Request((0, -1), (0, 1), 0, 10, 0, 20, NO_GDP),
            Request((-2, 0), (-1, 0), 0, 10, 0, 20, NO_GDP),
            Request((-2, 0), (1, 0), 0, 10, 0, 20, NO_GDP),
        ])

        self.assertTrue(rp._initial_intersection_check(flightplans_to_color[0], flightplans_to_color[1]))
        self.assertFalse(rp._initial_intersection_check(flightplans_to_color[1], flightplans_to_color[2]))
        self.assertTrue(rp._initial_intersection_check(flightplans_to_color[1], flightplans_to_color[3]))

    def test_simulation_intersection_without_time_uncertainty(self):
        graphml_string = """<?xml version='1.0' encoding='utf-8'?>
                                                        <graphml xmlns="http://graphml.graphdrawing.org/xmlns" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="http://graphml.graphdrawing.org/xmlns http://graphml.graphdrawing.org/xmlns/1.0/graphml.xsd">
                                                          <key id="d12" for="edge" attr.name="length" attr.type="string" />
                                                          <key id="d2" for="node" attr.name="x" attr.type="string" />
                                                          <key id="d1" for="node" attr.name="y" attr.type="string" />
                                                          <graph edgedefault="undirected">
                                                            <node id="l">
                                                              <data key="d1">0</data>
                                                              <data key="d2">-1</data>
                                                            </node>
                                                            <node id="ll">
                                                              <data key="d1">0</data>
                                                              <data key="d2">-2</data>
                                                            </node>
                                                            <node id="r">
                                                              <data key="d1">0</data>
                                                              <data key="d2">1</data>
                                                            </node>
                                                            <node id="c">
                                                              <data key="d1">0</data>
                                                              <data key="d2">0</data>
                                                            </node>
                                                            <node id="t">
                                                              <data key="d1">1</data>
                                                              <data key="d2">0</data>
                                                            </node>
                                                            <node id="b">
                                                              <data key="d1">-1</data>
                                                              <data key="d2">0</data>
                                                            </node>


                                                            <edge source="ll" target="l" id="0">
                                                              <data key="d12">40</data>
                                                            </edge>
                                                            <edge source="l" target="c" id="1">
                                                              <data key="d12">40</data>
                                                            </edge>
                                                            <edge source="r" target="c" id="2">
                                                              <data key="d12">40</data>
                                                            </edge>
                                                            <edge source="t" target="c" id="3">
                                                              <data key="d12">40</data>
                                                            </edge>
                                                            <edge source="b" target="c" id="4">
                                                              <data key="d12">40</data>
                                                            </edge>
                                                          </graph>
                                                        </graphml>
                                                        """
        sn = StreetNetwork.from_graphml_string(graphml_string, recompute_lengths=False, turn_params_table=TurnParamsTable([TurnParams(0, 180, 30, 0)]))

        sn.original_network.nodes['c']['norm_x'] = 0
        sn.original_network.nodes['c']['norm_y'] = 0

        sn.original_network.nodes['l']['norm_x'] = -40
        sn.original_network.nodes['l']['norm_y'] = 0

        sn.original_network.nodes['r']['norm_x'] = 40
        sn.original_network.nodes['r']['norm_y'] = 0

        sn.original_network.nodes['t']['norm_x'] = 0
        sn.original_network.nodes['t']['norm_y'] = 40

        sn.original_network.nodes['b']['norm_x'] = 0
        sn.original_network.nodes['b']['norm_y'] = -40

        sn.original_network.nodes['ll']['norm_x'] = -80
        sn.original_network.nodes['ll']['norm_y'] = 0

        layers = [
            Layer(altitude_m=10, type=Layer.Type.NETWORK, path_planner=PathPlanner(sn, 1, 10, NO_GDP)),
            Layer(altitude_m=20, type=Layer.Type.NETWORK, path_planner=PathPlanner(sn, 1, 10, NO_GDP)),
        ]
        rp = RoutePlanner(layers=layers, turn_params_table=TurnParamsTable([TurnParams(0, 180, 30, 0)]))

        flightplans_to_color = rp._prepare_flightplans_to_color([
            Request((-1, 0), (1, 0), 0, 10, 0, 10, NO_GDP),
            Request((0, -1), (0, 1), 0, 10, 0, 10, NO_GDP),
            Request((-2, 0), (-1, 0), 0, 10, 0, 10, NO_GDP),
            Request((-2, 0), (1, 0), 0, 10, 0, 10, NO_GDP),
            Request((-2, 0), (1, 0), 0, 10, -4, 10, NO_GDP),
        ])

        self.assertTrue(rp._find_intersection_by_simulation(flightplans_to_color[0], flightplans_to_color[1], rp._intersection_in_time(flightplans_to_color[0], flightplans_to_color[1])))
        self.assertFalse(rp._find_intersection_by_simulation(flightplans_to_color[1], flightplans_to_color[2], rp._intersection_in_time(flightplans_to_color[1], flightplans_to_color[2])))
        self.assertFalse(rp._find_intersection_by_simulation(flightplans_to_color[1], flightplans_to_color[3], rp._intersection_in_time(flightplans_to_color[1], flightplans_to_color[3])))
        self.assertTrue(rp._find_intersection_by_simulation(flightplans_to_color[1], flightplans_to_color[4], rp._intersection_in_time(flightplans_to_color[1], flightplans_to_color[4])))

    def test_simulation_intersection_with_time_uncertainty(self):
        graphml_string = """<?xml version='1.0' encoding='utf-8'?>
                                                        <graphml xmlns="http://graphml.graphdrawing.org/xmlns" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="http://graphml.graphdrawing.org/xmlns http://graphml.graphdrawing.org/xmlns/1.0/graphml.xsd">
                                                          <key id="d12" for="edge" attr.name="length" attr.type="string" />
                                                          <key id="d2" for="node" attr.name="x" attr.type="string" />
                                                          <key id="d1" for="node" attr.name="y" attr.type="string" />
                                                          <graph edgedefault="undirected">
                                                            <node id="l">
                                                              <data key="d1">0</data>
                                                              <data key="d2">-1</data>
                                                            </node>
                                                            <node id="ll">
                                                              <data key="d1">0</data>
                                                              <data key="d2">-2</data>
                                                            </node>
                                                            <node id="r">
                                                              <data key="d1">0</data>
                                                              <data key="d2">1</data>
                                                            </node>
                                                            <node id="c">
                                                              <data key="d1">0</data>
                                                              <data key="d2">0</data>
                                                            </node>
                                                            <node id="t">
                                                              <data key="d1">1</data>
                                                              <data key="d2">0</data>
                                                            </node>
                                                            <node id="b">
                                                              <data key="d1">-1</data>
                                                              <data key="d2">0</data>
                                                            </node>


                                                            <edge source="ll" target="l" id="0">
                                                              <data key="d12">40</data>
                                                            </edge>
                                                            <edge source="l" target="c" id="1">
                                                              <data key="d12">40</data>
                                                            </edge>
                                                            <edge source="r" target="c" id="2">
                                                              <data key="d12">40</data>
                                                            </edge>
                                                            <edge source="t" target="c" id="3">
                                                              <data key="d12">40</data>
                                                            </edge>
                                                            <edge source="b" target="c" id="4">
                                                              <data key="d12">40</data>
                                                            </edge>
                                                          </graph>
                                                        </graphml>
                                                        """
        sn = StreetNetwork.from_graphml_string(graphml_string, recompute_lengths=False, turn_params_table=TurnParamsTable([TurnParams(0, 180, 30, 0)]))

        sn.original_network.nodes['c']['norm_x'] = 0
        sn.original_network.nodes['c']['norm_y'] = 0

        sn.original_network.nodes['l']['norm_x'] = -40
        sn.original_network.nodes['l']['norm_y'] = 0

        sn.original_network.nodes['r']['norm_x'] = 40
        sn.original_network.nodes['r']['norm_y'] = 0

        sn.original_network.nodes['t']['norm_x'] = 0
        sn.original_network.nodes['t']['norm_y'] = 40

        sn.original_network.nodes['b']['norm_x'] = 0
        sn.original_network.nodes['b']['norm_y'] = -40

        sn.original_network.nodes['ll']['norm_x'] = -80
        sn.original_network.nodes['ll']['norm_y'] = 0

        layers = [
            Layer(altitude_m=10, type=Layer.Type.NETWORK, path_planner=PathPlanner(sn, 1, 10, NO_GDP)),
            Layer(altitude_m=20, type=Layer.Type.NETWORK, path_planner=PathPlanner(sn, 1, 10, NO_GDP)),
        ]
        rp = RoutePlanner(layers=layers, turn_params_table=TurnParamsTable([TurnParams(0, 180, 30, 0)]))

        flightplans_to_color = rp._prepare_flightplans_to_color([
            Request((0, -1), (0, 1), 10, 10, 0, 10, NO_GDP),
            Request((-2, 0), (1, 0), 0, 10, 0, 10, NO_GDP),
            Request((0, -1), (0, 1), 1, 10, 0, 10, NO_GDP),
        ])

        self.assertTrue(rp._find_intersection_by_simulation(flightplans_to_color[0], flightplans_to_color[1], rp._intersection_in_time(flightplans_to_color[0], flightplans_to_color[1])))
        self.assertFalse(rp._find_intersection_by_simulation(flightplans_to_color[2], flightplans_to_color[1], rp._intersection_in_time(flightplans_to_color[2], flightplans_to_color[1])))

    def test_attempt_coloring(self):
        graphml_string = """<?xml version='1.0' encoding='utf-8'?>
                                                        <graphml xmlns="http://graphml.graphdrawing.org/xmlns" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="http://graphml.graphdrawing.org/xmlns http://graphml.graphdrawing.org/xmlns/1.0/graphml.xsd">
                                                          <key id="d12" for="edge" attr.name="length" attr.type="string" />
                                                          <key id="d2" for="node" attr.name="x" attr.type="string" />
                                                          <key id="d1" for="node" attr.name="y" attr.type="string" />
                                                          <graph edgedefault="undirected">
                                                            <node id="l">
                                                              <data key="d1">0</data>
                                                              <data key="d2">-1</data>
                                                            </node>
                                                            <node id="ll">
                                                              <data key="d1">0</data>
                                                              <data key="d2">-2</data>
                                                            </node>
                                                            <node id="r">
                                                              <data key="d1">0</data>
                                                              <data key="d2">1</data>
                                                            </node>
                                                            <node id="c">
                                                              <data key="d1">0</data>
                                                              <data key="d2">0</data>
                                                            </node>
                                                            <node id="t">
                                                              <data key="d1">1</data>
                                                              <data key="d2">0</data>
                                                            </node>
                                                            <node id="b">
                                                              <data key="d1">-1</data>
                                                              <data key="d2">0</data>
                                                            </node>


                                                            <edge source="ll" target="l" id="0">
                                                              <data key="d12">40</data>
                                                            </edge>
                                                            <edge source="l" target="c" id="1">
                                                              <data key="d12">40</data>
                                                            </edge>
                                                            <edge source="r" target="c" id="2">
                                                              <data key="d12">40</data>
                                                            </edge>
                                                            <edge source="t" target="c" id="3">
                                                              <data key="d12">40</data>
                                                            </edge>
                                                            <edge source="b" target="c" id="4">
                                                              <data key="d12">40</data>
                                                            </edge>
                                                          </graph>
                                                        </graphml>
                                                        """
        sn = StreetNetwork.from_graphml_string(graphml_string, recompute_lengths=False, turn_params_table=TurnParamsTable([TurnParams(0, 180, 30, 0)]))

        sn.original_network.nodes['c']['norm_x'] = 0
        sn.original_network.nodes['c']['norm_y'] = 0

        sn.original_network.nodes['l']['norm_x'] = -40
        sn.original_network.nodes['l']['norm_y'] = 0

        sn.original_network.nodes['r']['norm_x'] = 40
        sn.original_network.nodes['r']['norm_y'] = 0

        sn.original_network.nodes['t']['norm_x'] = 0
        sn.original_network.nodes['t']['norm_y'] = 40

        sn.original_network.nodes['b']['norm_x'] = 0
        sn.original_network.nodes['b']['norm_y'] = -40

        sn.original_network.nodes['ll']['norm_x'] = -80
        sn.original_network.nodes['ll']['norm_y'] = 0

        layers = [
            Layer(altitude_m=10, type=Layer.Type.NETWORK, path_planner=PathPlanner(sn, 1, 10, NO_GDP)),
            Layer(altitude_m=20, type=Layer.Type.NETWORK, path_planner=PathPlanner(sn, 1, 10, NO_GDP)),
        ]
        rp = RoutePlanner(layers=layers, turn_params_table=TurnParamsTable([TurnParams(0, 180, 30, 0)]))
        requests = [Request((0, -1), (0, 1), 10, 10, 0, 10, NO_GDP),
                    Request((-2, 0), (1, 0), 0, 10, 0, 10, NO_GDP),]
        # res, flightplans = rp._attempt_coloring(requests)
        #
        # self.assertIn(res, [[0, 1], [1, 0]])

        requests = [
            Request((-2, 0), (1, 0), 0, 10, 0, 10, NO_GDP),
            Request((0, -1), (0, 1), 1, 10, 0, 10, NO_GDP),
        ]
        res, flightplans = rp._attempt_coloring(requests)

        self.assertIn(res, [[0, 0], [1, 1]]) # making sure that we are using as few layers as possible

        layers = [
            Layer(altitude_m=10, type=Layer.Type.NETWORK, path_planner=PathPlanner(sn, 1, 10, NO_GDP)),
        ]

        rp = RoutePlanner(layers=layers, turn_params_table=TurnParamsTable([TurnParams(0, 180, 30, 0)]))

        requests = [Request((0, -1), (0, 1), 10, 10, 0, 10, NO_GDP),
                    Request((-2, 0), (1, 0), 0, 10, 0, 10, NO_GDP), ]
        res, flightplans = rp._attempt_coloring(requests)

        self.assertIsNone(res)

    def test_turn_cost(self):
        graphml_string = """<?xml version='1.0' encoding='utf-8'?>
                                                                <graphml xmlns="http://graphml.graphdrawing.org/xmlns" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="http://graphml.graphdrawing.org/xmlns http://graphml.graphdrawing.org/xmlns/1.0/graphml.xsd">
                                                                  <key id="d12" for="edge" attr.name="length" attr.type="string" />
                                                                  <key id="d2" for="node" attr.name="x" attr.type="string" />
                                                                  <key id="d1" for="node" attr.name="y" attr.type="string" />
                                                                  <graph edgedefault="undirected">
                                                                    <node id="l">
                                                                      <data key="d1">0</data>
                                                                      <data key="d2">-1</data>
                                                                    </node>
                                                                    <node id="ll">
                                                                      <data key="d1">0</data>
                                                                      <data key="d2">-2</data>
                                                                    </node>
                                                                    <node id="r">
                                                                      <data key="d1">0</data>
                                                                      <data key="d2">1</data>
                                                                    </node>
                                                                    <node id="c">
                                                                      <data key="d1">0</data>
                                                                      <data key="d2">0</data>
                                                                    </node>
                                                                    <node id="t">
                                                                      <data key="d1">1</data>
                                                                      <data key="d2">0</data>
                                                                    </node>
                                                                    <node id="b">
                                                                      <data key="d1">-1</data>
                                                                      <data key="d2">0</data>
                                                                    </node>


                                                                    <edge source="ll" target="l" id="0">
                                                                      <data key="d12">40</data>
                                                                    </edge>
                                                                    <edge source="l" target="c" id="1">
                                                                      <data key="d12">40</data>
                                                                    </edge>
                                                                    <edge source="r" target="c" id="2">
                                                                      <data key="d12">40</data>
                                                                    </edge>
                                                                    <edge source="t" target="c" id="3">
                                                                      <data key="d12">40</data>
                                                                    </edge>
                                                                    <edge source="b" target="c" id="4">
                                                                      <data key="d12">40</data>
                                                                    </edge>
                                                                  </graph>
                                                                </graphml>
                                                                """
        turn_params_table = TurnParamsTable([TurnParams(0, 30, 30, 0), TurnParams(30, 180, 10, 7)])

        sn = StreetNetwork.from_graphml_string(graphml_string, recompute_lengths=False,
                                               turn_params_table=turn_params_table)

        sn.original_network.nodes['c']['norm_x'] = 0
        sn.original_network.nodes['c']['norm_y'] = 0

        sn.original_network.nodes['l']['norm_x'] = -40
        sn.original_network.nodes['l']['norm_y'] = 0

        sn.original_network.nodes['r']['norm_x'] = 40
        sn.original_network.nodes['r']['norm_y'] = 0

        sn.original_network.nodes['t']['norm_x'] = 0
        sn.original_network.nodes['t']['norm_y'] = 40

        sn.original_network.nodes['b']['norm_x'] = 0
        sn.original_network.nodes['b']['norm_y'] = -40

        sn.original_network.nodes['ll']['norm_x'] = -80
        sn.original_network.nodes['ll']['norm_y'] = 0

        layers = [
            Layer(altitude_m=10, type=Layer.Type.NETWORK, path_planner=PathPlanner(sn, 1, 10, NO_GDP)),
            Layer(altitude_m=20, type=Layer.Type.NETWORK, path_planner=PathPlanner(sn, 1, 10, NO_GDP)),
        ]
        rp = RoutePlanner(layers=layers, turn_params_table=turn_params_table)

        flightplans_to_color = rp._prepare_flightplans_to_color([
            Request((-1, 0), (0, -1), 0, 10, 0, 10, NO_GDP),
            Request((1, 0), (-1, 0), 0, 10, 0, 10, NO_GDP),
            Request((1, 0), (-1, 0), 0, 10, 3, 10, NO_GDP),
            Request((1, 0), (-1, 0), 0, 10, 10, 10, NO_GDP),
        ])

        self.assertTrue(rp._find_intersection_by_simulation(flightplans_to_color[0], flightplans_to_color[1],
                                                            rp._intersection_in_time(flightplans_to_color[0],
                                                                                     flightplans_to_color[1])))

        self.assertTrue(rp._find_intersection_by_simulation(flightplans_to_color[0], flightplans_to_color[2],
                                                            rp._intersection_in_time(flightplans_to_color[0],
                                                                                     flightplans_to_color[2])))

        self.assertFalse(rp._find_intersection_by_simulation(flightplans_to_color[0], flightplans_to_color[3],
                                                            rp._intersection_in_time(flightplans_to_color[0],
                                                                                     flightplans_to_color[3])))

    def test_converting_sn_flightplan_with_delay(self):
        graphml_string = """<?xml version='1.0' encoding='utf-8'?>
                                                                        <graphml xmlns="http://graphml.graphdrawing.org/xmlns" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="http://graphml.graphdrawing.org/xmlns http://graphml.graphdrawing.org/xmlns/1.0/graphml.xsd">
                                                                          <key id="d12" for="edge" attr.name="length" attr.type="string" />
                                                                          <key id="d2" for="node" attr.name="x" attr.type="string" />
                                                                          <key id="d1" for="node" attr.name="y" attr.type="string" />
                                                                          <graph edgedefault="undirected">
                                                                            <node id="l">
                                                                              <data key="d1">0</data>
                                                                              <data key="d2">-1</data>
                                                                            </node>
                                                                            <node id="ll">
                                                                              <data key="d1">0</data>
                                                                              <data key="d2">-2</data>
                                                                            </node>
                                                                            <node id="r">
                                                                              <data key="d1">0</data>
                                                                              <data key="d2">1</data>
                                                                            </node>
                                                                            <node id="c">
                                                                              <data key="d1">0</data>
                                                                              <data key="d2">0</data>
                                                                            </node>
                                                                            <node id="t">
                                                                              <data key="d1">1</data>
                                                                              <data key="d2">0</data>
                                                                            </node>
                                                                            <node id="b">
                                                                              <data key="d1">-1</data>
                                                                              <data key="d2">0</data>
                                                                            </node>


                                                                            <edge source="ll" target="l" id="0">
                                                                              <data key="d12">40</data>
                                                                            </edge>
                                                                            <edge source="l" target="c" id="1">
                                                                              <data key="d12">40</data>
                                                                            </edge>
                                                                            <edge source="r" target="c" id="2">
                                                                              <data key="d12">40</data>
                                                                            </edge>
                                                                            <edge source="t" target="c" id="3">
                                                                              <data key="d12">40</data>
                                                                            </edge>
                                                                            <edge source="b" target="c" id="4">
                                                                              <data key="d12">40</data>
                                                                            </edge>
                                                                          </graph>
                                                                        </graphml>
                                                                        """
        turn_params_table = TurnParamsTable([TurnParams(0, 180, 30, 0)])

        sn = StreetNetwork.from_graphml_string(graphml_string, recompute_lengths=False,
                                               turn_params_table=turn_params_table)

        sn_flightplan = SNFlightplan([(0, 'll'), (1, 'll'), (2, 'l'), (3, 'c'), (4, 'r')], 1, 0, 0, SNRequest('ll', 'r', 0, 0, 50, 0))
        pp = PathPlanner(sn, 1, 50, None)

        fp = Flightplan.from_sn_flightplan(pp.get_time_cost_enhanced_network(50), sn_flightplan)
        self.assertEqual(fp.waypoints[0].time, 1)
        self.assertEqual(fp.waypoints[3].time, 4)

        sn_flightplan = SNFlightplan([(0, 'll'), (1, 'l'), (2, 'c'), (3, 'r')], 1, 0, 0,
                                     SNRequest('ll', 'r', 0, 0, 50, 0))
        pp = PathPlanner(sn, 1, 50, None)

        fp = Flightplan.from_sn_flightplan(pp.get_time_cost_enhanced_network(50), sn_flightplan)
        self.assertEqual(fp.waypoints[0].time, 0)
        self.assertEqual(fp.waypoints[3].time, 3)

    def test_conflict_with_disk_geofence(self):
        graphml_string = """<?xml version='1.0' encoding='utf-8'?>
        <graphml xmlns="http://graphml.graphdrawing.org/xmlns" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="http://graphml.graphdrawing.org/xmlns http://graphml.graphdrawing.org/xmlns/1.0/graphml.xsd">
          <key id="d12" for="edge" attr.name="length" attr.type="string" />
          <key id="d2" for="node" attr.name="x" attr.type="string" />
          <key id="d1" for="node" attr.name="y" attr.type="string" />
          <graph edgedefault="undirected">
            <node id="l">
              <data key="d1">0</data>
              <data key="d2">-1</data>
            </node>
            <node id="ll">
              <data key="d1">0</data>
              <data key="d2">-2</data>
            </node>
            <node id="r">
              <data key="d1">0</data>
              <data key="d2">1</data>
            </node>
            <node id="c">
              <data key="d1">0</data>
              <data key="d2">0</data>
            </node>
            <node id="t">
              <data key="d1">1</data>
              <data key="d2">0</data>
            </node>
            <node id="b">
              <data key="d1">-1</data>
              <data key="d2">0</data>
            </node>


            <edge source="ll" target="l" id="0">
              <data key="d12">40</data>
            </edge>
            <edge source="l" target="c" id="1">
              <data key="d12">40</data>
            </edge>
            <edge source="r" target="c" id="2">
              <data key="d12">40</data>
            </edge>
            <edge source="t" target="c" id="3">
              <data key="d12">40</data>
            </edge>
            <edge source="b" target="c" id="4">
              <data key="d12">40</data>
            </edge>
          </graph>
        </graphml>
        """
        sn = StreetNetwork.from_graphml_string(graphml_string, recompute_lengths=False,
                                               turn_params_table=TurnParamsTable([TurnParams(0, 180, 30, 0)]))

        sn.original_network.nodes['c']['norm_x'] = 0
        sn.original_network.nodes['c']['norm_y'] = 0

        sn.original_network.nodes['l']['norm_x'] = -40
        sn.original_network.nodes['l']['norm_y'] = 0

        sn.original_network.nodes['r']['norm_x'] = 40
        sn.original_network.nodes['r']['norm_y'] = 0

        sn.original_network.nodes['t']['norm_x'] = 0
        sn.original_network.nodes['t']['norm_y'] = 40

        sn.original_network.nodes['b']['norm_x'] = 0
        sn.original_network.nodes['b']['norm_y'] = -40

        sn.original_network.nodes['ll']['norm_x'] = -80
        sn.original_network.nodes['ll']['norm_y'] = 0

        layers = [
            Layer(altitude_m=10, type=Layer.Type.NETWORK, path_planner=PathPlanner(sn, 1, 10, NO_GDP)),
            Layer(altitude_m=20, type=Layer.Type.NETWORK, path_planner=PathPlanner(sn, 1, 10, NO_GDP)),
        ]
        rp = RoutePlanner(layers=layers, turn_params_table=TurnParamsTable([TurnParams(0, 180, 30, 0)]))

        flightplans_to_color = rp._prepare_flightplans_to_color([
            Request((0, -1), (0, 1), 10, 10, 0, 10, NO_GDP),
        ])
        flightplan = flightplans_to_color[0].flightplan

        disk_geofence = DiskGeofence((1, 3), 10, (0, 0))

        self.assertTrue(rp._test_overall_intersection_with_geofence(flightplan, disk_geofence))
        self.assertTrue(rp._test_intersection_with_geofence_by_simulation(flightplan, disk_geofence))

        disk_geofence = DiskGeofence((10, 30), 10, (0, 0))

        self.assertFalse(rp._test_overall_intersection_with_geofence(flightplan, disk_geofence))
        self.assertFalse(rp._test_intersection_with_geofence_by_simulation(flightplan, disk_geofence))

        disk_geofence = DiskGeofence((0, 1), 10, (0, 0))

        self.assertTrue(rp._test_overall_intersection_with_geofence(flightplan, disk_geofence))
        self.assertFalse(rp._test_intersection_with_geofence_by_simulation(flightplan, disk_geofence))
