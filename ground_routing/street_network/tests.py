import math
from unittest import TestCase

import networkx as nx
from geopy import distance

from ground_routing.common import GDP, PathNotFoundException, TurnParamsTable, TurnParams, NO_GDP, DiskGeofence
from path_planner import StreetNetwork, SNPathPlanner, SNRequest, SNFlightplan


def plot_network(network):
    import matplotlib.pyplot as plt
    nx.draw(network, with_labels=True)
    plt.show()


class StreetNetworkTestCase(TestCase):
    def test_simple_subdivision(self):
        graphml_string = """<?xml version='1.0' encoding='utf-8'?>
        <graphml xmlns="http://graphml.graphdrawing.org/xmlns" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="http://graphml.graphdrawing.org/xmlns http://graphml.graphdrawing.org/xmlns/1.0/graphml.xsd">
          <key id="d12" for="edge" attr.name="length" attr.type="string" />
          <key id="d2" for="node" attr.name="x" attr.type="string" />
          <key id="d1" for="node" attr.name="y" attr.type="string" />
          <graph edgedefault="undirected">
            <node id="378699">
              <data key="d1">0.0</data>
              <data key="d2">0.0</data>
            </node>
            <node id="33144414">
              <data key="d1">1.0</data>
              <data key="d2">1.0</data>
            </node>
            <edge source="378699" target="33144414" id="0">
              <data key="d12">25.0</data>
            </edge>
          </graph>
        </graphml>
        """

        sn = StreetNetwork.from_graphml_string(graphml_string, turn_params_table=TurnParamsTable([TurnParams(0, 180, 30, 0)]))
        time_ext_network = sn.get_subdivided_network(10)


        self.assertEqual(len(time_ext_network.nodes), 4)
        self.assertEqual(len(time_ext_network.edges), 6)

        self.assertEqual(time_ext_network.nodes['378699']['x'], 0.0)
        self.assertEqual(time_ext_network.nodes['33144414']['x'], 1.0)
        self.assertEqual(time_ext_network.nodes['378699_33144414_0']['x'], 0.4)
        self.assertEqual(time_ext_network.nodes['378699_33144414_1']['x'], 0.8)

        self.assertEqual(time_ext_network['378699']['378699_33144414_0']['length'], 10)
        self.assertEqual(time_ext_network['378699_33144414_0']['378699']['length'], 10)
        self.assertEqual(time_ext_network['378699_33144414_0']['378699_33144414_1']['length'], 10)
        self.assertEqual(time_ext_network['378699_33144414_1']['33144414']['length'], 5)
        self.assertEqual(time_ext_network['33144414']['378699_33144414_1']['length'], 5)

    def test_precise_subdivision(self):
        graphml_string = """<?xml version='1.0' encoding='utf-8'?>
        <graphml xmlns="http://graphml.graphdrawing.org/xmlns" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="http://graphml.graphdrawing.org/xmlns http://graphml.graphdrawing.org/xmlns/1.0/graphml.xsd">
          <key id="d12" for="edge" attr.name="length" attr.type="string" />
          <key id="d2" for="node" attr.name="x" attr.type="string" />
          <key id="d1" for="node" attr.name="y" attr.type="string" />
          <graph edgedefault="undirected">
            <node id="378699">
              <data key="d1">0.0</data>
              <data key="d2">0.0</data>
            </node>
            <node id="33144414">
              <data key="d1">1.0</data>
              <data key="d2">1.0</data>
            </node>
            <edge source="378699" target="33144414" id="0">
              <data key="d12">20.0</data>
            </edge>
          </graph>
        </graphml>
        """

        sn = StreetNetwork.from_graphml_string(graphml_string, turn_params_table=TurnParamsTable([TurnParams(0, 180, 30, 0)]))
        time_ext_network = sn.get_subdivided_network(10)

        self.assertEqual(len(time_ext_network.nodes), 3)
        self.assertEqual(len(time_ext_network.edges), 4)

        self.assertEqual(time_ext_network.nodes['378699']['x'], 0.0)
        self.assertEqual(time_ext_network.nodes['33144414']['x'], 1.0)
        self.assertEqual(time_ext_network.nodes['378699_33144414_0']['x'], 0.5)

        self.assertEqual(time_ext_network['378699']['378699_33144414_0']['length'], 10)
        self.assertEqual(time_ext_network['378699_33144414_0']['378699']['length'], 10)
        self.assertEqual(time_ext_network['378699_33144414_0']['33144414']['length'], 10)
        self.assertEqual(time_ext_network['33144414']['378699_33144414_0']['length'], 10)

    def test_no_subdivision(self):
        graphml_string = """<?xml version='1.0' encoding='utf-8'?>
        <graphml xmlns="http://graphml.graphdrawing.org/xmlns" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="http://graphml.graphdrawing.org/xmlns http://graphml.graphdrawing.org/xmlns/1.0/graphml.xsd">
          <key id="d12" for="edge" attr.name="length" attr.type="string" />
          <key id="d2" for="node" attr.name="x" attr.type="string" />
          <key id="d1" for="node" attr.name="y" attr.type="string" />
          <graph edgedefault="undirected">
            <node id="378699">
              <data key="d1">0.0</data>
              <data key="d2">0.0</data>
            </node>
            <node id="33144414">
              <data key="d1">1.0</data>
              <data key="d2">1.0</data>
            </node>
            <edge source="378699" target="33144414" id="0">
              <data key="d12">9.0</data>
            </edge>
          </graph>
        </graphml>
        """

        sn = StreetNetwork.from_graphml_string(graphml_string, turn_params_table=TurnParamsTable([TurnParams(0, 180, 30, 0)]))
        time_ext_network = sn.get_subdivided_network(10)

        self.assertEqual(len(time_ext_network.nodes), 2)
        self.assertEqual(len(time_ext_network.edges), 2)

        self.assertEqual(time_ext_network.nodes['378699']['x'], 0.0)
        self.assertEqual(time_ext_network.nodes['33144414']['x'], 1.0)

        self.assertEqual(time_ext_network['378699']['33144414']['length'], 9)
        self.assertEqual(time_ext_network['33144414']['378699']['length'], 9)

    def test_small_last_edge_subdivision(self):
        graphml_string = """<?xml version='1.0' encoding='utf-8'?>
        <graphml xmlns="http://graphml.graphdrawing.org/xmlns" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="http://graphml.graphdrawing.org/xmlns http://graphml.graphdrawing.org/xmlns/1.0/graphml.xsd">
          <key id="d12" for="edge" attr.name="length" attr.type="string" />
          <key id="d2" for="node" attr.name="x" attr.type="string" />
          <key id="d1" for="node" attr.name="y" attr.type="string" />
          <graph edgedefault="undirected">
            <node id="378699">
              <data key="d1">0.0</data>
              <data key="d2">0.0</data>
            </node>
            <node id="33144414">
              <data key="d1">1.0</data>
              <data key="d2">1.0</data>
            </node>
            <edge source="378699" target="33144414" id="0">
              <data key="d12">20.5</data>
            </edge>
          </graph>
        </graphml>
        """

        sn = StreetNetwork.from_graphml_string(graphml_string, turn_params_table=TurnParamsTable([TurnParams(0, 180, 30, 0)]))
        time_ext_network = sn.get_subdivided_network(10)

        self.assertEqual(len(time_ext_network.nodes), 3)
        self.assertEqual(len(time_ext_network.edges), 4)

        self.assertEqual(time_ext_network.nodes['378699']['x'], 0.0)
        self.assertEqual(time_ext_network.nodes['33144414']['x'], 1.0)
        self.assertEqual(time_ext_network.nodes['378699_33144414_0']['x'], 0.4878048780487805)

        self.assertEqual(time_ext_network['378699']['378699_33144414_0']['length'], 10)
        self.assertEqual(time_ext_network['378699_33144414_0']['378699']['length'], 10)
        self.assertEqual(time_ext_network['378699_33144414_0']['33144414']['length'], 10.5)
        self.assertEqual(time_ext_network['33144414']['378699_33144414_0']['length'], 10.5)

    def test_bounding_box(self):
        graphml_string = """<?xml version='1.0' encoding='utf-8'?>
                <graphml xmlns="http://graphml.graphdrawing.org/xmlns" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="http://graphml.graphdrawing.org/xmlns http://graphml.graphdrawing.org/xmlns/1.0/graphml.xsd">
                  <key id="d12" for="edge" attr.name="length" attr.type="string" />
                  <key id="d2" for="node" attr.name="x" attr.type="string" />
                  <key id="d1" for="node" attr.name="y" attr.type="string" />
                  <graph edgedefault="undirected">
                    <node id="378699">
                      <data key="d1">-9.3</data>
                      <data key="d2">11.2</data>
                    </node>
                    <node id="33144414">
                      <data key="d1">3.0</data>
                      <data key="d2">4.2</data>
                    </node>
                    <edge source="378699" target="33144414" id="0">
                      <data key="d12">20.5</data>
                    </edge>
                  </graph>
                </graphml>
                """

        sn = StreetNetwork.from_graphml_string(graphml_string, turn_params_table=TurnParamsTable([TurnParams(0, 180, 30, 0)]))
        bb = sn._bounding_box_lrbt()
        self.assertEqual(bb.left, 4.2)
        self.assertEqual(bb.right, 11.2)
        self.assertEqual(bb.top, 3.0)
        self.assertEqual(bb.bottom, -9.3)

    def test_normalized_positions(self):
        graphml_string = """<?xml version='1.0' encoding='utf-8'?>
        <graphml xmlns="http://graphml.graphdrawing.org/xmlns" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="http://graphml.graphdrawing.org/xmlns http://graphml.graphdrawing.org/xmlns/1.0/graphml.xsd">
          <key id="d16" for="edge" attr.name="to" attr.type="string" />
          <key id="d15" for="edge" attr.name="from" attr.type="string" />
          <key id="d14" for="edge" attr.name="bearing" attr.type="string" />
          <key id="d13" for="edge" attr.name="geometry" attr.type="string" />
          <key id="d12" for="edge" attr.name="length" attr.type="string" />
          <key id="d11" for="edge" attr.name="maxspeed" attr.type="string" />
          <key id="d10" for="edge" attr.name="highway" attr.type="string" />
          <key id="d9" for="edge" attr.name="name" attr.type="string" />
          <key id="d8" for="edge" attr.name="ref" attr.type="string" />
          <key id="d7" for="edge" attr.name="lanes" attr.type="string" />
          <key id="d6" for="edge" attr.name="oneway" attr.type="string" />
          <key id="d5" for="edge" attr.name="osmid" attr.type="string" />
          <key id="d4" for="node" attr.name="highway" attr.type="string" />
          <key id="d3" for="node" attr.name="street_count" attr.type="string" />
          <key id="d2" for="node" attr.name="x" attr.type="string" />
          <key id="d1" for="node" attr.name="y" attr.type="string" />
          <key id="d0" for="graph" attr.name="crs" attr.type="string" />
          <graph edgedefault="undirected">
            <node id="378699">
              <data key="d1">48.2172288</data>
              <data key="d2">16.3424207</data>
              <data key="d3">4</data>
            </node>
            <node id="33144414">
              <data key="d1">48.2183536</data>
              <data key="d2">16.3429038</data>
              <data key="d3">4</data>
            </node>
            <node id="378727">
              <data key="d1">48.2250679</data>
              <data key="d2">16.3498014</data>
              <data key="d3">4</data>
            </node>
            <node id="26405238">
              <data key="d1">48.224925</data>
              <data key="d2">16.3507849</data>
              <data key="d3">4</data>
            </node>
            <edge source="378699" target="33144414" id="0">
              <data key="d5">23311963</data>
              <data key="d6">True</data>
              <data key="d7">3</data>
              <data key="d8">B221</data>
              <data key="d9">Währinger Gürtel</data>
              <data key="d10">primary</data>
              <data key="d11">50</data>
              <data key="d12">130.102</data>
              <data key="d13">LINESTRING (16.3424207 48.2172288, 16.3424441 48.2172833, 16.3424708 48.2173351, 16.3428515 48.2182304, 16.3428721 48.2182789, 16.3429038 48.2183536)</data>
              <data key="d14">16.0</data>
              <data key="d15">378699</data>
              <data key="d16">33144414</data>
            </edge>
          </graph>
        </graphml>
        """

        sn = StreetNetwork.from_graphml_string(graphml_string, turn_params_table=TurnParamsTable([TurnParams(0, 180, 30, 0)]))
        self.assertEqual(sn.original_network.nodes['378699']['norm_y'], 0)
        self.assertEqual(sn.original_network.nodes['26405238']['norm_x'], 621.5565137776072)
        x_diff = sn.original_network.nodes['378699']['norm_x'] - sn.original_network.nodes['33144414']['norm_x']
        y_diff = sn.original_network.nodes['378699']['norm_y'] - sn.original_network.nodes['33144414']['norm_y']
        print(distance.distance(
            (sn.original_network.nodes['378699']['y'], sn.original_network.nodes['378699']['x']),
            (sn.original_network.nodes['33144414']['y'], sn.original_network.nodes['33144414']['x']),
        ).m)
        self.assertAlmostEqual(math.sqrt(x_diff**2 + y_diff**2), 130.102, delta=0.1)

    def test_geometry(self):
        pass


class StreetNetworkTurnCostTestCase(TestCase):
    def test_simple_turn_cost(self):
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
        sn = StreetNetwork.from_graphml_string(graphml_string, recompute_lengths=False, turn_params_table=TurnParamsTable([
            TurnParams(0, 30, 30, 0),
            TurnParams(30, 180, 10, 7)
        ]))

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


        subdivided_network = sn.get_subdivided_network(10)

        self.assertEqual(len(subdivided_network.nodes), 5+4+3*5)
        # print([x for x in subdivided_network.edges if subdivided_network.edges[x]['time_cost'] > 0])
        self.assertEqual(8, len([x for x in subdivided_network.edges if subdivided_network.edges[x]['time_cost'] > 0]))

    def test_all_turn_cost(self):
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
        sn = StreetNetwork.from_graphml_string(graphml_string, recompute_lengths=False, turn_params_table=TurnParamsTable([
            TurnParams(0, 180, 10, 7)
        ]))

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


        subdivided_network = sn.get_subdivided_network(60)

        self.assertEqual(6+4, len(subdivided_network.nodes))

        subdivided_network = sn.get_subdivided_network(10)

        self.assertEqual(6 + 4 + 5*3, len(subdivided_network.nodes))


class ShortestPathInNetworkTestCase(TestCase):
    def test_simple_shortest_path(self):
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
                    <node id="4">
                      <data key="d1">48.224925</data>
                      <data key="d2">16.3507849</data>
                    </node>
                    <edge source="1" target="2" id="0">
                      <data key="d12">30</data>
                    </edge>
                    <edge source="1" target="3" id="1">
                      <data key="d12">10</data>
                    </edge>
                    <edge source="2" target="4" id="2">
                      <data key="d12">10</data>
                    </edge>
                    <edge source="3" target="4" id="3">
                      <data key="d12">10</data>
                    </edge>
                  </graph>
                </graphml>
                """
        sn = StreetNetwork.from_graphml_string(graphml_string, turn_params_table=TurnParamsTable([TurnParams(0, 180, 30, 0)]))
        pp = SNPathPlanner(street_network=sn, timestep_s=1, edge_length_m=10, default_gdp=NO_GDP)

        flightplans = pp.resolve_requests([SNRequest('D0', '1', '4', 0, 1, 10, 0)])
        self.assertEqual(len(flightplans), 1)
        self.assertEqual(flightplans[0].end_time, 2)
        self.assertEqual(flightplans[0].nodes[0], '1')
        self.assertEqual(flightplans[0].nodes[1], '3')
        self.assertEqual(flightplans[0].nodes[2], '4')

    def test_no_path(self):
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
                            <node id="4">
                              <data key="d1">48.224925</data>
                              <data key="d2">16.3507849</data>
                            </node>
                            <edge source="1" target="2" id="0">
                              <data key="d12">30</data>
                            </edge>
                            <edge source="1" target="3" id="1">
                              <data key="d12">10</data>
                            </edge>
                            <edge source="2" target="3" id="2">
                              <data key="d12">10</data>
                            </edge>
                          </graph>
                        </graphml>
                        """
        sn = StreetNetwork.from_graphml_string(graphml_string, turn_params_table=TurnParamsTable([TurnParams(0, 180, 30, 0)]))
        pp = SNPathPlanner(street_network=sn, timestep_s=1, edge_length_m=10, default_gdp=NO_GDP)

        self.assertRaises(PathNotFoundException, pp.resolve_request, SNRequest('D0', '1', '4', 0, 1, 10, 0))

    def test_towards_each_other(self):
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
                                    <node id="4">
                                      <data key="d1">48.224925</data>
                                      <data key="d2">16.3507849</data>
                                    </node>
                                    <edge source="1" target="2" id="0">
                                      <data key="d12">30</data>
                                    </edge>
                                    <edge source="1" target="3" id="1">
                                      <data key="d12">10</data>
                                    </edge>
                                    <edge source="2" target="4" id="2">
                                      <data key="d12">10</data>
                                    </edge>
                                    <edge source="3" target="4" id="3">
                                      <data key="d12">10</data>
                                    </edge>
                                  </graph>
                                </graphml>
                                """
        sn = StreetNetwork.from_graphml_string(graphml_string, turn_params_table=TurnParamsTable([TurnParams(0, 180, 30, 0)]))
        pp = SNPathPlanner(street_network=sn, timestep_s=1, edge_length_m=10, default_gdp=NO_GDP)

        pp.resolve_requests([SNRequest('D0', '1', '4', 0, 1, 10, 0), SNRequest('D0', '4', '1', 0, 1, 10, 0)])

        self.assertEqual(len(pp.flightplans), 2)
        self.assertEqual(pp.flightplans[0].end_time, 2)
        self.assertEqual(pp.flightplans[1].end_time, 4)


    def test_gdp(self):
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
                                    <node id="4">
                                      <data key="d1">48.224925</data>
                                      <data key="d2">16.3507849</data>
                                    </node>
                                    <edge source="1" target="2" id="0">
                                      <data key="d12">30</data>
                                    </edge>
                                    <edge source="1" target="3" id="1">
                                      <data key="d12">10</data>
                                    </edge>
                                    <edge source="2" target="4" id="2">
                                      <data key="d12">10</data>
                                    </edge>
                                    <edge source="3" target="4" id="3">
                                      <data key="d12">10</data>
                                    </edge>
                                  </graph>
                                </graphml>
                                """
        sn = StreetNetwork.from_graphml_string(graphml_string, turn_params_table=TurnParamsTable([TurnParams(0, 180, 30, 0)]))
        pp = SNPathPlanner(street_network=sn, timestep_s=1, edge_length_m=10, default_gdp=GDP(max_time=4, penalty=0.5, time_step=1))

        pp.resolve_requests([SNRequest('D0', '1', '4', 0, 1, 10, 0), SNRequest('D0', '4', '1', 0, 1, 10, 0)])

        self.assertEqual(len(pp.flightplans), 2)
        self.assertEqual(pp.flightplans[0].end_time, 2)
        self.assertEqual(list(pp.flightplans[1].nodes.values()), ['4', '4', '4', '4', '3', '1'])
        self.assertEqual(pp.flightplans[1].end_time, 5)

    def test_no_jumping(self):
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
                                            <node id="3">
                                              <data key="d1">48.2250679</data>
                                              <data key="d2">16.3498014</data>
                                            </node>
                                            <node id="4">
                                              <data key="d1">48.224925</data>
                                              <data key="d2">16.3507849</data>
                                            </node>
                                            <edge source="1" target="3" id="1">
                                              <data key="d12">10</data>
                                            </edge>
                                            <edge source="3" target="4" id="3">
                                              <data key="d12">10</data>
                                            </edge>
                                          </graph>
                                        </graphml>
                                        """
        sn = StreetNetwork.from_graphml_string(graphml_string, turn_params_table=TurnParamsTable([TurnParams(0, 180, 30, 0)]))
        pp = SNPathPlanner(street_network=sn, timestep_s=1, edge_length_m=10, default_gdp=GDP(max_time=10, penalty=1000, time_step=1))

        pp.resolve_requests([SNRequest('D0', '1', '4', 0, 1, 10, 0), SNRequest('D0', '4', '1', 0, 1, 10, 0)])

        self.assertEqual(len(pp.flightplans), 2)
        self.assertEqual(pp.flightplans[0].end_time, 2)
        self.assertEqual(pp.flightplans[1].end_time, 5)

    def test_time_uncertainty_on_finished_drone(self):
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
                                                    <node id="4">
                                                      <data key="d1">48.224925</data>
                                                      <data key="d2">16.3507849</data>
                                                    </node>
                                                    <edge source="1" target="2" id="0">
                                                      <data key="d12">30</data>
                                                    </edge>
                                                    <edge source="1" target="3" id="1">
                                                      <data key="d12">10</data>
                                                    </edge>
                                                    <edge source="2" target="4" id="2">
                                                      <data key="d12">10</data>
                                                    </edge>
                                                    <edge source="3" target="4" id="3">
                                                      <data key="d12">10</data>
                                                    </edge>
                                                  </graph>
                                                </graphml>
                                                """
        sn = StreetNetwork.from_graphml_string(graphml_string, turn_params_table=TurnParamsTable([TurnParams(0, 180, 30, 0)]))
        pp = SNPathPlanner(street_network=sn, timestep_s=1, edge_length_m=10, default_gdp=GDP(max_time=30, penalty=1, time_step=1))

        pp.add_flightplan(SNFlightplan(id='D0', nodes=[(0, '1'), (1, '3'), (2, '4')], speed_node=1, time_uncertainty=5, radius_m=1, request=None))
        request = SNRequest('D0', '1', '4', 1, 1, 10, 0)
        self.assertEqual(len(pp._list_of_occupied_nodes_for_request(3, request.uncertainty_radius_m, pp._request_time_uncertainty_to_ticks(request))), 3)
        self.assertEqual(len(pp._list_of_occupied_nodes_for_request(4, request.uncertainty_radius_m, pp._request_time_uncertainty_to_ticks(request))), 3)
        self.assertEqual(len(pp._list_of_occupied_nodes_for_request(5, request.uncertainty_radius_m, pp._request_time_uncertainty_to_ticks(request))), 3)
        self.assertEqual(len(pp._list_of_occupied_nodes_for_request(6, request.uncertainty_radius_m, pp._request_time_uncertainty_to_ticks(request))), 2)
        self.assertEqual(len(pp._list_of_occupied_nodes_for_request(7, request.uncertainty_radius_m, pp._request_time_uncertainty_to_ticks(request))), 1)

        pp.resolve_requests([request, ])

        self.assertEqual(len(pp.flightplans), 2)
        self.assertEqual(pp.flightplans[0].end_time, 2)
        self.assertEqual(pp.flightplans[1].end_time, 8)

    def test_time_uncertainty_on_new_drone(self):
        graphml_string = """<?xml version='1.0' encoding='utf-8'?>
                                                <graphml xmlns="http://graphml.graphdrawing.org/xmlns" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="http://graphml.graphdrawing.org/xmlns http://graphml.graphdrawing.org/xmlns/1.0/graphml.xsd">
                                                  <key id="d12" for="edge" attr.name="length" attr.type="string" />
                                                  <key id="d2" for="node" attr.name="x" attr.type="string" />
                                                  <key id="d1" for="node" attr.name="y" attr.type="string" />
                                                  <graph edgedefault="undirected">
                                                    <node id="l">
                                                      <data key="d1">1</data>
                                                      <data key="d2">1</data>
                                                    </node>
                                                    <node id="r">
                                                      <data key="d1">2</data>
                                                      <data key="d2">2</data>
                                                    </node>
                                                    <node id="t">
                                                      <data key="d1">3</data>
                                                      <data key="d2">3</data>
                                                    </node>
                                                    <node id="b">
                                                      <data key="d1">4</data>
                                                      <data key="d2">4</data>
                                                    </node>
                                                    <node id="c">
                                                      <data key="d1">5</data>
                                                      <data key="d2">5</data>
                                                    </node>
                                                    <edge source="l" target="c" id="0">
                                                      <data key="d12">10</data>
                                                    </edge>
                                                    <edge source="r" target="c" id="1">
                                                      <data key="d12">10</data>
                                                    </edge>
                                                    <edge source="t" target="c" id="2">
                                                      <data key="d12">10</data>
                                                    </edge>
                                                    <edge source="b" target="c" id="3">
                                                      <data key="d12">10</data>
                                                    </edge>
                                                  </graph>
                                                </graphml>
                                                """
        sn = StreetNetwork.from_graphml_string(graphml_string, turn_params_table=TurnParamsTable([TurnParams(0, 180, 30, 0)]))
        pp = SNPathPlanner(street_network=sn, timestep_s=1, edge_length_m=10, default_gdp=GDP(max_time=30, penalty=1, time_step=1))

        pp.add_flightplan(SNFlightplan(id='D0', nodes=[(1, 'l'), (2, 'c'), (3, 'r')], speed_node=1, time_uncertainty=0, radius_m=1, request=None))
        request = SNRequest('D0', 't', 'b', 0, 1, 10, 5)
        self.assertEqual(len(pp._list_of_occupied_nodes_for_request(0, request.uncertainty_radius_m, pp._request_time_uncertainty_to_ticks(request))), 3)
        self.assertEqual(len(pp._list_of_occupied_nodes_for_request(1, request.uncertainty_radius_m, pp._request_time_uncertainty_to_ticks(request))), 3)
        self.assertEqual(len(pp._list_of_occupied_nodes_for_request(2, request.uncertainty_radius_m, pp._request_time_uncertainty_to_ticks(request))), 2)
        self.assertEqual(len(pp._list_of_occupied_nodes_for_request(3, request.uncertainty_radius_m, pp._request_time_uncertainty_to_ticks(request))), 1)
        self.assertEqual(len(pp._list_of_occupied_nodes_for_request(4, request.uncertainty_radius_m, pp._request_time_uncertainty_to_ticks(request))), 0)

        pp.resolve_requests([request, ])

        self.assertEqual(len(pp.flightplans), 2)
        self.assertEqual(pp.flightplans[0].end_time, 3)
        self.assertEqual(pp.flightplans[1].end_time, 4)

    def test_time_uncertainty_on_both_drones(self):
        graphml_string = """<?xml version='1.0' encoding='utf-8'?>
                                                <graphml xmlns="http://graphml.graphdrawing.org/xmlns" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="http://graphml.graphdrawing.org/xmlns http://graphml.graphdrawing.org/xmlns/1.0/graphml.xsd">
                                                  <key id="d12" for="edge" attr.name="length" attr.type="string" />
                                                  <key id="d2" for="node" attr.name="x" attr.type="string" />
                                                  <key id="d1" for="node" attr.name="y" attr.type="string" />
                                                  <graph edgedefault="undirected">
                                                    <node id="l">
                                                      <data key="d1">1</data>
                                                      <data key="d2">1</data>
                                                    </node>
                                                    <node id="r">
                                                      <data key="d1">2</data>
                                                      <data key="d2">2</data>
                                                    </node>
                                                    <node id="t">
                                                      <data key="d1">3</data>
                                                      <data key="d2">3</data>
                                                    </node>
                                                    <node id="b">
                                                      <data key="d1">4</data>
                                                      <data key="d2">4</data>
                                                    </node>
                                                    <node id="c">
                                                      <data key="d1">5</data>
                                                      <data key="d2">5</data>
                                                    </node>
                                                    <edge source="l" target="c" id="0">
                                                      <data key="d12">10</data>
                                                    </edge>
                                                    <edge source="r" target="c" id="1">
                                                      <data key="d12">10</data>
                                                    </edge>
                                                    <edge source="t" target="c" id="2">
                                                      <data key="d12">10</data>
                                                    </edge>
                                                    <edge source="b" target="c" id="3">
                                                      <data key="d12">10</data>
                                                    </edge>
                                                  </graph>
                                                </graphml>
                                                """
        sn = StreetNetwork.from_graphml_string(graphml_string, turn_params_table=TurnParamsTable([TurnParams(0, 180, 30, 0)]))
        pp = SNPathPlanner(street_network=sn, timestep_s=1, edge_length_m=10, default_gdp=GDP(max_time=30, penalty=1, time_step=1))

        pp.add_flightplan(SNFlightplan(id='D0', nodes=[(1, 'l'), (2, 'c'), (3, 'r')], speed_node=1, time_uncertainty=5, radius_m=1, request=None))
        request = SNRequest('D0', 't', 'b', 0, 1, 10, 5)
        self.assertEqual(len(pp._list_of_occupied_nodes_for_request(0, request.uncertainty_radius_m, pp._request_time_uncertainty_to_ticks(request))), 3)
        self.assertEqual(len(pp._list_of_occupied_nodes_for_request(1, request.uncertainty_radius_m, pp._request_time_uncertainty_to_ticks(request))), 3)
        self.assertEqual(len(pp._list_of_occupied_nodes_for_request(2, request.uncertainty_radius_m, pp._request_time_uncertainty_to_ticks(request))), 3)
        self.assertEqual(len(pp._list_of_occupied_nodes_for_request(3, request.uncertainty_radius_m, pp._request_time_uncertainty_to_ticks(request))), 3)
        self.assertEqual(len(pp._list_of_occupied_nodes_for_request(4, request.uncertainty_radius_m, pp._request_time_uncertainty_to_ticks(request))), 3)
        self.assertEqual(len(pp._list_of_occupied_nodes_for_request(5, request.uncertainty_radius_m, pp._request_time_uncertainty_to_ticks(request))), 3)
        self.assertEqual(len(pp._list_of_occupied_nodes_for_request(6, request.uncertainty_radius_m, pp._request_time_uncertainty_to_ticks(request))), 3)
        self.assertEqual(len(pp._list_of_occupied_nodes_for_request(7, request.uncertainty_radius_m, pp._request_time_uncertainty_to_ticks(request))), 2)
        self.assertEqual(len(pp._list_of_occupied_nodes_for_request(8, request.uncertainty_radius_m, pp._request_time_uncertainty_to_ticks(request))), 1)
        self.assertEqual(len(pp._list_of_occupied_nodes_for_request(9, request.uncertainty_radius_m, pp._request_time_uncertainty_to_ticks(request))), 0)

        pp.resolve_requests([request, ])

        self.assertEqual(len(pp.flightplans), 2)
        self.assertEqual(pp.flightplans[0].end_time, 3)
        self.assertEqual(pp.flightplans[1].end_time, 9)

    def test_drones_radius(self):
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
        pp = SNPathPlanner(street_network=sn, timestep_s=1, edge_length_m=1000, default_gdp=NO_GDP)
        pp.add_flightplan(SNFlightplan(id='D0', nodes=[(0, '1'), (1, '2'), (2, '3')], speed_node=1, time_uncertainty=0, radius_m=50, request=None))
        self.assertEqual(pp._list_of_occupied_nodes_for_request(0, 50, 0), {'1'})
        self.assertEqual(pp._list_of_occupied_nodes_for_request(0, 85, 0), {'1', '2'})
        self.assertEqual(pp._list_of_occupied_nodes_for_request(1, 700, 0), {'1', '2'})
        self.assertEqual(pp._list_of_occupied_nodes_for_request(1, 900, 0), {'1', '2', '3'})

        pp = SNPathPlanner(street_network=sn, timestep_s=1, edge_length_m=10, default_gdp=NO_GDP)
        pp.add_flightplan(
            SNFlightplan(id='D0', nodes=[(0, '1'), (1, '2'), (2, '3')], speed_node=1, time_uncertainty=0, radius_m=50, request=None))
        self.assertEqual(pp._list_of_occupied_nodes_for_request(0, 50, 0), {'1',
                                                                                                      '1_2_0', '1_2_1', '1_2_2',
                                                                                                      '1_2_3', '1_2_4', '1_2_5',
                                                                                                      '1_2_6', '1_2_7', '1_2_8',
                                                                                                        })

        pp = SNPathPlanner(street_network=sn, timestep_s=1, edge_length_m=50, default_gdp=NO_GDP)
        pp.add_flightplan(
            SNFlightplan(id='D0', nodes=[(0, '1'), (1, '2'), (2, '3')], speed_node=1, time_uncertainty=0, radius_m=50, request=None))
        self.assertEqual(pp._list_of_occupied_nodes_for_request(0, 250, 0), {'1_2_0', '2_3_2', '2', '2_3_0', '2_3_1', '1_2_1', '1'})


class ShortestPathWithTurnNodesTestCase(TestCase):
    def test_simple(self):
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
                                               turn_params_table=TurnParamsTable([
                                                   TurnParams(0, 30, 30, 0),
                                                   TurnParams(30, 180, 10, 7),
                                               ]))

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

        pp = SNPathPlanner(street_network=sn, timestep_s=1, edge_length_m=10, default_gdp=GDP(max_time=30, penalty=1, time_step=1))

        # pp.add_flightplan(
        #     SNFlightplan(nodes=[(1, 'l'), (2, 'c'), (3, 'r')], speed_node=1, time_uncertainty=5, radius_m=1,
        #                  request=None))
        request = SNRequest('D0', 'l', 'b', 0, 0.1, 10, 0)

        fp = pp.resolve_request(request)
        pp.add_flightplan(fp)
        self.assertEqual(15, fp.end_time)

        self.assertEqual(pp._list_of_occupied_nodes_for_request(0, 0.1, 0), {'l'})
        self.assertEqual(pp._list_of_occupied_nodes_for_request(4, 0.1, 0), {'r@c', 'l@c', 't@c', 'b@c'})
        self.assertEqual(pp._list_of_occupied_nodes_for_request(5, 0.1, 0), {'r@c', 'l@c', 't@c', 'b@c'})
        self.assertEqual(pp._list_of_occupied_nodes_for_request(6, 0.1, 0), {'r@c', 'l@c', 't@c', 'b@c'})

    def test_time_delay(self):
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
                                               turn_params_table=TurnParamsTable([
                                                   TurnParams(0, 30, 30, 0),
                                                   TurnParams(30, 180, 10, 7),
                                               ]))

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

        pp = SNPathPlanner(street_network=sn, timestep_s=1, edge_length_m=10, default_gdp=GDP(max_time=30, penalty=1, time_step=1))

        # pp.add_flightplan(
        #     SNFlightplan(nodes=[(1, 'l'), (2, 'c'), (3, 'r')], speed_node=1, time_uncertainty=5, radius_m=1,
        #                  request=None))
        request = SNRequest('D0', 'l', 'b', 0, 0.1, 10, 3)

        fp = pp.resolve_request(request)
        pp.add_flightplan(fp)
        self.assertEqual(15, fp.end_time)

        self.assertEqual(pp._list_of_occupied_nodes_for_request(0, 0.1, 0), {'l'})
        self.assertEqual(pp._list_of_occupied_nodes_for_request(4, 0.1, 0), {'l@c', 'b@c', 't@c', 'l_l@c_1', 'r@c', 'l_l@c_0', 'l_l@c_2'})
        self.assertEqual(pp._list_of_occupied_nodes_for_request(5, 0.1, 0), {'l@c', 'b@c', 't@c', 'l_l@c_1', 'r@c', 'l_l@c_2'})
        self.assertEqual(pp._list_of_occupied_nodes_for_request(6, 0.1, 0), {'l@c', 'b@c', 't@c', 'r@c', 'l_l@c_2'})
        self.assertEqual(pp._list_of_occupied_nodes_for_request(7, 0.1, 0), {'l@c', 'b@c', 't@c', 'r@c'})
        self.assertEqual(pp._list_of_occupied_nodes_for_request(8, 0.1, 0), {'l@c', 'b@c', 't@c', 'r@c'})

    def test_two_drones(self):
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
                                               turn_params_table=TurnParamsTable([
                                                   TurnParams(0, 30, 30, 0),
                                                   TurnParams(30, 180, 10, 7),
                                               ]))

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

        pp = SNPathPlanner(street_network=sn, timestep_s=1, edge_length_m=10, default_gdp=GDP(max_time=30, penalty=1, time_step=1))

        # pp.add_flightplan(
        #     SNFlightplan(nodes=[(1, 'l'), (2, 'c'), (3, 'r')], speed_node=1, time_uncertainty=5, radius_m=1,
        #                  request=None))
        request1 = SNRequest('D0', 'l', 'b', 0, 0.1, 10, 3)
        request2 = SNRequest('D0', 'r', 't', 0, 0.1, 10, 3)

        fps = pp.resolve_requests([request1, request2])
        self.assertEqual(15, fps[0].end_time)
        self.assertEqual(26, fps[1].end_time)


class GeofencesTestCase(TestCase):
    def test_circle(self):
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
                                               turn_params_table=TurnParamsTable([
                                                   TurnParams(0, 30, 30, 0),
                                                   TurnParams(30, 180, 10, 7),
                                               ]))

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

        pp = SNPathPlanner(street_network=sn, timestep_s=1, edge_length_m=40,
                           default_gdp=NO_GDP, geofences=[DiskGeofence((1, 4), 10, (0, 0)), DiskGeofence((10, 14), 10, (-35, 0))])

        res = pp._list_of_occupied_nodes_for_request(0, 0, 0)
        self.assertEqual(len(res), 0)

        res = pp._list_of_occupied_nodes_for_request(1, 0, 0)
        self.assertEqual(len(res), 4)

        res = pp._list_of_occupied_nodes_for_request(2, 0, 0)
        self.assertEqual(len(res), 4)

        res = pp._list_of_occupied_nodes_for_request(3, 0, 0)
        self.assertEqual(len(res), 4)

        res = pp._list_of_occupied_nodes_for_request(4, 0, 0)
        self.assertEqual(len(res), 4)

        res = pp._list_of_occupied_nodes_for_request(5, 0, 0)
        self.assertEqual(len(res), 0)

        res = pp._list_of_occupied_nodes_for_request(10, 0, 0)
        self.assertEqual(len(res), 1)
