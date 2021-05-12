from typing import List
from unittest import TestCase

import simplejson
import numpy as np

from main import HexMap, HexCoordinate, PathPlanner, Request, GDP, CityMap, HexHelper, Flightplan


class HexCoordinateTestCase(TestCase):
    def test_hex_coordinates(self):
        coord = HexCoordinate(x=10, y=10)
        self.assertEqual(coord.x, 10)
        self.assertEqual(coord.y, 10)

        coord = HexCoordinate(x=1, y=3)
        self.assertEqual(coord.x, 1)
        self.assertEqual(coord.y, 3)

        with self.assertRaises(Exception):
            HexCoordinate(x=10, y=5)

    def test_hex_coordinates_ordering(self):
        coords = [
            HexCoordinate(x=12, y=10),
            HexCoordinate(x=10, y=10),
            HexCoordinate(x=3, y=3),
            HexCoordinate(x=1, y=5),
            HexCoordinate(x=1, y=3),
        ]

        sorted_coords = sorted(coords)

        self.assertEqual(sorted_coords, [
            HexCoordinate(x=1, y=3),
            HexCoordinate(x=3, y=3),
            HexCoordinate(x=1, y=5),
            HexCoordinate(x=10, y=10),
            HexCoordinate(x=12, y=10),
        ])


class HexMapTestCase(TestCase):
    def test_coordinates_conversion(self):
        hexmap = HexMap(width=10, height=10)

        a = hexmap.coord_to_int(HexCoordinate(0, 0))
        self.assertEqual(a, 0)

        a = hexmap.coord_to_int(HexCoordinate(6, 0))
        self.assertEqual(a, 3)

        a = hexmap.coord_to_int(HexCoordinate(5, 1))
        self.assertEqual(a, 12)

        b = hexmap.int_to_coord(15)
        a = hexmap.coord_to_int(b)
        self.assertEqual(a, 15)

        b = hexmap.int_to_coord(20)
        a = hexmap.coord_to_int(b)
        self.assertEqual(a, 20)

        self.assertEqual(hexmap.int_to_coord(1), hexmap.int_to_coord(101))
        self.assertEqual(hexmap.int_to_coord(0), hexmap.int_to_coord(100))
        self.assertEqual(hexmap.int_to_coord(202), hexmap.int_to_coord(302))

    def test_is_feasible(self):
        hexmap = HexMap(width=10, height=10)

        self.assertTrue(hexmap._is_feasible_coordinate(y=0, x=0))
        self.assertTrue(hexmap._is_feasible_coordinate(y=0, x=2))
        self.assertTrue(hexmap._is_feasible_coordinate(y=0, x=18))
        self.assertFalse(hexmap._is_feasible_coordinate(y=0, x=20))
        self.assertFalse(hexmap._is_feasible_coordinate(y=1, x=0))
        self.assertFalse(hexmap._is_feasible_coordinate(y=1, x=21))
        self.assertFalse(hexmap._is_feasible_coordinate(y=1, x=20))
        self.assertTrue(hexmap._is_feasible_coordinate(y=1, x=19))
        self.assertTrue(hexmap._is_feasible_coordinate(y=9, x=19))
        self.assertFalse(hexmap._is_feasible_coordinate(y=10, x=18))
        self.assertFalse(hexmap._is_feasible_coordinate(y=-1, x=1))
        self.assertFalse(hexmap._is_feasible_coordinate(y=-1, x=1))
        self.assertFalse(hexmap._is_feasible_coordinate(y=1, x=0))

    def test_find_neighbours(self):
        hexmap = HexMap(width=7, height=7)

        neighbours = hexmap.get_neighbours(HexCoordinate(0, 0))
        expected_neighbours = [
            HexCoordinate(2, 0),
            HexCoordinate(1, 1),
        ]

        self.assertEqual(sorted(neighbours), sorted(expected_neighbours))

        neighbours = hexmap.get_neighbours(HexCoordinate(6, 2))
        expected_neighbours = [
            HexCoordinate(5, 1),
            HexCoordinate(7, 1),
            HexCoordinate(4, 2),
            HexCoordinate(8, 2),
            HexCoordinate(5, 3),
            HexCoordinate(7, 3),
        ]

        self.assertEqual(sorted(neighbours), sorted(expected_neighbours))

        neighbours = hexmap.get_neighbours(HexCoordinate(0, 6))
        expected_neighbours = [
            HexCoordinate(1, 5),
            HexCoordinate(2, 6),
        ]

        self.assertEqual(sorted(neighbours), sorted(expected_neighbours))


        neighbours = hexmap.get_neighbours(HexCoordinate(12, 6))
        expected_neighbours = [
            HexCoordinate(10, 6),
            HexCoordinate(11, 5),
            HexCoordinate(13, 5),
        ]

        self.assertEqual(sorted(neighbours), sorted(expected_neighbours))


class FlightsTestCase(TestCase):
    def test_jumping_2x2(self):
        width = 3
        height = 3

        requests_data = [
            {'from': [0, 0], 'to': [4, 0], 'start_time': 0},
            {'from': [4, 0], 'to': [0, 0], 'start_time': 0},
        ]
        planner = PathPlanner(obstacles=[], requests=[Request(x['from'], x['to'], x['start_time']) for x in requests_data], map_width=width, map_height=height, hex_radius_m=1, drone_radius_m=1, gdp=GDP(1000, 1))
        flightplans = planner.resolve_all()

        data = planner.get_data_as_dict()

        self.assertEqual(3, len(flightplans[0].points))
        self.assertEqual(4, len(flightplans[1].points))

        with open('./results/test_results.json', 'w') as wf:
            simplejson.dump(data, wf)

    def test_flying_after_each_other_3x3(self):
        width = 3
        height = 3

        requests_data = [
            {'from': [0, 0], 'to': [4, 0], 'start_time': 0},
            {'from': [0, 0], 'to': [4, 0], 'start_time': 1},
            {'from': [0, 0], 'to': [4, 0], 'start_time': 2},
            {'from': [0, 0], 'to': [4, 0], 'start_time': 3},
        ]
        planner = PathPlanner(obstacles=[],
                              requests=[Request(x['from'], x['to'], x['start_time']) for x in requests_data],
                              map_width=width, map_height=height,
                              drone_radius_m=1, hex_radius_m=1,
                              gdp=GDP(max_time=300, penalty=1))

        flightplans = planner.resolve_all()

        self.assertEqual(3, len(flightplans[0].points))
        self.assertEqual(3, len(flightplans[1].points))
        self.assertEqual(3, len(flightplans[2].points))
        self.assertEqual(3, len(flightplans[3].points))

        data = planner.get_data_as_dict()

        with open('./results/test_results.json', 'w') as wf:
            simplejson.dump(data, wf)


class ResolutionTestCase(TestCase):
    def test_single_straight_flight(self):
        width = 4
        height = 4

        requests_data = [
            {'from': [0, 0], 'to': [6, 0], 'start_time': 0},
        ]
        planner = PathPlanner(obstacles=[],
                              requests=[Request(x['from'], x['to'], x['start_time']) for x in requests_data],
                              map_width=width, map_height=height, hex_radius_m=1, drone_radius_m=1, gdp=GDP(10, 1))

        flightplans = planner.resolve_all()

        data = planner.get_data_as_dict()

        with open('./results/test_results.json', 'w') as wf:
            simplejson.dump(data, wf)

        self.assertEqual(4, len(flightplans[0].points))

    def test_single_straight_flight_time_offset(self):
        width = 4
        height = 4

        requests_data = [
            {'from': [0, 0], 'to': [6, 0], 'start_time': 3},
        ]
        planner = PathPlanner(obstacles=[],
                              requests=[Request(x['from'], x['to'], x['start_time']) for x in requests_data],
                              map_width=width, map_height=height, hex_radius_m=1, drone_radius_m=1, gdp=GDP(10, 1))

        flightplans = planner.resolve_all()

        data = planner.get_data_as_dict()

        with open('./results/test_results.json', 'w') as wf:
            simplejson.dump(data, wf)

        self.assertEqual(4, len(flightplans[0].points))

    def test_single_flight_around_obstacle(self):
        width = 4
        height = 4

        requests_data = [
            {'from': [0, 0], 'to': [6, 0], 'start_time': 0},
        ]
        planner = PathPlanner(obstacles=[HexCoordinate(4, 0), ],
                              requests=[Request(x['from'], x['to'], x['start_time']) for x in requests_data],
                              map_width=width, map_height=height, hex_radius_m=1, drone_radius_m=1, gdp=GDP(1000, 1))

        flightplans = planner.resolve_all()

        data = planner.get_data_as_dict()

        with open('./results/test_results.json', 'w') as wf:
            simplejson.dump(data, wf)

        self.assertEqual(5, len(flightplans[0].points))

    def test_two_flights_towards_each_other_without_obstacles(self):
        width = 4
        height = 4

        requests_data = [
            {'from': [0, 0], 'to': [6, 0], 'start_time': 0},
            {'from': [6, 0], 'to': [0, 0], 'start_time': 0},
        ]
        planner = PathPlanner(obstacles=[],
                              requests=[Request(x['from'], x['to'], x['start_time']) for x in requests_data],
                              map_width=width, map_height=height, hex_radius_m=1, drone_radius_m=1, gdp=GDP(1000, 1))

        flightplans = planner.resolve_all()

        data = planner.get_data_as_dict()

        with open('./results/test_results.json', 'w') as wf:
            simplejson.dump(data, wf)

        self.assertEqual(4, len(flightplans[0].points))
        self.assertEqual(5, len(flightplans[1].points))

    def test_two_flights_towards_each_other_with_obstacles(self):
        width = 4
        height = 4

        requests_data = [
            {'from': [0, 0], 'to': [6, 0], 'start_time': 0},
            {'from': [6, 0], 'to': [0, 0], 'start_time': 0},
        ]
        planner = PathPlanner(obstacles=[HexCoordinate(4, 0)],
                              requests=[Request(x['from'], x['to'], x['start_time']) for x in requests_data],
                              map_width=width, map_height=height, hex_radius_m=1, drone_radius_m=1, gdp=GDP(1000, 1))

        flightplans = planner.resolve_all()

        data = planner.get_data_as_dict()

        with open('./results/test_results.json', 'w') as wf:
            simplejson.dump(data, wf)

        self.assertEqual(5, len(flightplans[0].points))
        self.assertEqual(6, len(flightplans[1].points))


class FlightPlannerTestCase(TestCase):
    def test_get_all_neighbours(self):
        width = 4
        height = 4

        requests_data = [
            {'from': [0, 0], 'to': [6, 0], 'start_time': 3},
        ]
        planner = PathPlanner(obstacles=[],
                              requests=[Request(x['from'], x['to'], x['start_time']) for x in requests_data],
                              map_width=width, map_height=height, hex_radius_m=1, drone_radius_m=1, gdp=GDP(10, 1))

        neighbours = planner._get_all_neighbours(HexCoordinate(0, 0), 5, Request((0, 0), (6, 0), 3))

        self.assertEqual(3, len(neighbours))
        self.assertEqual(6, planner.map.int_time(neighbours[0][0]))
        self.assertEqual(6, planner.map.int_time(neighbours[1][0]))
        self.assertEqual(6, planner.map.int_time(neighbours[2][0]))

class CityMapTestCase(TestCase):
    def test_map_size(self):
        heights = np.zeros((40, 40))
        map = CityMap(heights=heights, pixel_size_m=1, hex_radius_m=1, flight_height=50)
        self.assertEqual(24, map.hexagonal_map_size()[0])
        self.assertEqual(27, map.hexagonal_map_size()[1])

        heights = np.zeros((320, 320))
        heights[15, 20] = 1000
        heights[16, 20] = 1000

        map = CityMap(heights=heights, pixel_size_m=1, hex_radius_m=16, flight_height=50)
        obstacles = map.obstacles()

        self.assertEqual(12, map.hexagonal_map_size()[0])
        self.assertEqual(14, map.hexagonal_map_size()[1])

        heights = np.zeros((340, 340))
        heights[15, 20] = 1000
        heights[16, 20] = 1000

        map = CityMap(heights=heights, pixel_size_m=1, hex_radius_m=16, flight_height=50)
        obstacles = map.obstacles()

        self.assertEqual(13, map.hexagonal_map_size()[0])
        self.assertEqual(15, map.hexagonal_map_size()[1])

    def test_map_obstacles(self):
        heights = np.zeros((335, 335))
        heights[15, 20] = 1000
        heights[16, 20] = 1000
        heights[50, 325] = 1000

        map = CityMap(heights=heights, pixel_size_m=1, hex_radius_m=16, flight_height=50)
        obstacles = map.obstacles()

        data = {
            'obstacles': [(x.x, x.y) for x in obstacles],
            'heights': heights.tolist(),
            'size': map.hexagonal_map_size(),
            'hex_r_px': map.hex_radius_in_px
        }

        # self.assertEqual(13, map.hexagonal_map_size()[0])
        # self.assertEqual(15, map.hexagonal_map_size()[1])

        with open('./results/test_map.json', 'w') as f:
            simplejson.dump(data, f)


class LineDrawingTestCase(TestCase):
    def test_simple_line_draw_1(self):
        line = HexHelper.line_drawing(HexCoordinate(0, 0), HexCoordinate(6, 6))
        self.assertEqual(7, len(line))
        self.assertEqual([
            HexCoordinate(0, 0),
            HexCoordinate(1, 1),
            HexCoordinate(2, 2),
            HexCoordinate(3, 3),
            HexCoordinate(4, 4),
            HexCoordinate(5, 5),
            HexCoordinate(6, 6),], line)

    def test_simple_line_draw_2(self):
        line = HexHelper.line_drawing(HexCoordinate(0, 0), HexCoordinate(4, 2))
        self.assertEqual(4, len(line))
        self.assertEqual([
            HexCoordinate(0, 0),
            HexCoordinate(1, 1),
            HexCoordinate(3, 1),
            HexCoordinate(4, 2),
        ], line)

    def test_rounding_consistent(self):
        line = HexHelper.line_drawing(HexCoordinate(4, 0), HexCoordinate(4, 2))
        self.assertEqual(3, len(line))
        self.assertEqual([
            HexCoordinate(4, 0),
            HexCoordinate(3, 1),
            HexCoordinate(4, 2),
        ], line)

        line = HexHelper.line_drawing(HexCoordinate(4, 0), HexCoordinate(4, 4))
        self.assertEqual(5, len(line))
        self.assertEqual([
            HexCoordinate(4, 0),
            HexCoordinate(3, 1),
            HexCoordinate(4, 2),
            HexCoordinate(3, 3),
            HexCoordinate(4, 4),
        ], line)

        line = HexHelper.line_drawing(HexCoordinate(8, 0), HexCoordinate(8, 2))
        self.assertEqual(3, len(line))
        self.assertEqual([
            HexCoordinate(8, 0),
            HexCoordinate(7, 1),
            HexCoordinate(8, 2),
        ], line)

        # line = HexHelper.line_drawing(HexCoordinate(0, 0), HexCoordinate(3, 1))
        # self.assertEqual(3, len(line))
        # self.assertEqual([
        #     HexCoordinate(0, 0),
        #     HexCoordinate(1, 1),
        #     HexCoordinate(3, 1),
        # ], line)


class PathSmoothingTest(TestCase):
    def _get_points(self, points) -> List[HexCoordinate]:
        return [x[1] for x in points]

    def test_simple_smoothing(self):
        flightplan = Flightplan([
            (0, HexCoordinate(0, 0)),
            (1, HexCoordinate(1, 1)),
            (2, HexCoordinate(3, 1)),
            (3, HexCoordinate(4, 2)),
        ])

        smoothed = self._get_points(flightplan.smoothed())

        self.assertEqual(len(smoothed), 2)
        self.assertEqual(smoothed, [HexCoordinate(0, 0), HexCoordinate(4, 2)])

    def test_simple_smoothing_in_middle(self):
        flightplan = Flightplan([
            (-1, HexCoordinate(2, 0)),
            (0, HexCoordinate(0, 0)),
            (1, HexCoordinate(1, 1)),
            (2, HexCoordinate(3, 1)),
            (3, HexCoordinate(4, 2)),
            (4, HexCoordinate(2, 2)),
        ])

        smoothed = self._get_points(flightplan.smoothed())
        self.assertEqual(len(smoothed), 4)
        self.assertEqual(smoothed, [
            HexCoordinate(2, 0),
            HexCoordinate(0, 0),
            HexCoordinate(4, 2),
            HexCoordinate(2, 2),
        ])

    def test_simple_smoothing_in_end(self):
        flightplan = Flightplan([
            (-2, HexCoordinate(1, 1)),
            (-1, HexCoordinate(2, 0)),
            (0, HexCoordinate(0, 0)),
            (1, HexCoordinate(1, 1)),
            (2, HexCoordinate(3, 1)),
            (3, HexCoordinate(4, 2)),
        ])

        smoothed = self._get_points(flightplan.smoothed())
        self.assertEqual(len(smoothed), 4)
        self.assertEqual(smoothed, [
            HexCoordinate(1, 1),
            HexCoordinate(2, 0),
            HexCoordinate(0, 0),
            HexCoordinate(4, 2),
        ])

    def test_simple_smoothing_in_beginning(self):
        flightplan = Flightplan([
            (0, HexCoordinate(0, 0)),
            (1, HexCoordinate(1, 1)),
            (2, HexCoordinate(3, 1)),
            (3, HexCoordinate(4, 2)),
            (4, HexCoordinate(2, 2)),
            (5, HexCoordinate(3, 3)),
        ])

        smoothed = self._get_points(flightplan.smoothed())
        self.assertEqual(len(smoothed), 4)
        self.assertEqual(smoothed, [
            HexCoordinate(0, 0),
            HexCoordinate(4, 2),
            HexCoordinate(2, 2),
            HexCoordinate(3, 3),
        ])


class DeviationPenaltyTestCase(TestCase):
    def _get_flightplan_points(self, flightplan):
        return [x for (t, x) in sorted(flightplan.points.items(), key=lambda x : x[0])]

    def test_simple_route(self):
        line = HexHelper.line_drawing(HexCoordinate(0, 0), HexCoordinate(6, 4))

        width = 20
        height = 20

        requests_data = [
            {'from': [0, 0], 'to': [6, 4], 'start_time': 0},
        ]
        planner = PathPlanner(obstacles=[],
                              requests=[Request(x['from'], x['to'], x['start_time']) for x in requests_data],
                              map_width=width, map_height=height, hex_radius_m=1, drone_radius_m=1, gdp=GDP(1000, 1),
                              punish_deviation=True)
        flightplans = planner.resolve_all()

        self.assertEqual(6, len(line))
        self.assertEqual(6, len(flightplans[0].points))
        self.assertEqual(line, self._get_flightplan_points(flightplans[0]))

    def test_route_around_obstacle(self):
        line = HexHelper.line_drawing(HexCoordinate(0, 2), HexCoordinate(10, 2))

        width = 20
        height = 20

        requests_data = [
            {'from': [0, 2], 'to': [10, 2], 'start_time': 0},
        ]
        planner = PathPlanner(obstacles=[HexCoordinate(2, 2), ],
                              requests=[Request(x['from'], x['to'], x['start_time']) for x in requests_data],
                              map_width=width, map_height=height, hex_radius_m=1, drone_radius_m=1, gdp=GDP(1000, 1),
                              punish_deviation=True)
        flightplans = planner.resolve_all()

        self.assertEqual(6, len(line))
        self.assertEqual(7, len(flightplans[0].points))

        self.assertEqual([
            HexCoordinate(0, 2),
            HexCoordinate(1, 1),
            HexCoordinate(3, 1),
            HexCoordinate(4, 2),
            HexCoordinate(6, 2),
            HexCoordinate(8, 2),
            HexCoordinate(10, 2),
        ], self._get_flightplan_points(flightplans[0]))
