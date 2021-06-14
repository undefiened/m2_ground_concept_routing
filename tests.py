from typing import List
from unittest import TestCase

import pytest
import simplejson
import numpy as np

from misc_functions import _is_feasible_coordinate, HexCoordinate
from path_planning import HexMap, Request, CityMap, HexHelper, Flightplan, GDP, PathPlanner, PathNotFoundException


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

    def test_is_feasible_2(self):
        height = 61
        width = 53
        self.assertFalse(_is_feasible_coordinate(15, 4, height, width))


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
        planner = PathPlanner(obstacles=[], map_width=width, map_height=height, hex_radius_m=1, default_drone_radius_m=1, gdp=GDP(1000, 1))
        flightplans = planner.resolve_requests([Request.from_dict(x) for x in requests_data])

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
                              map_width=width, map_height=height,
                              default_drone_radius_m=1, hex_radius_m=1,
                              gdp=GDP(max_time=300, penalty=1))

        flightplans = planner.resolve_requests([Request.from_dict(x) for x in requests_data])

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
                              map_width=width, map_height=height, hex_radius_m=1, default_drone_radius_m=1, gdp=GDP(10, 1))

        flightplans = planner.resolve_requests([Request.from_dict(x) for x in requests_data])

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
                              map_width=width, map_height=height, hex_radius_m=1, default_drone_radius_m=1, gdp=GDP(10, 1))

        flightplans = planner.resolve_requests([Request.from_dict(x) for x in requests_data])

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
                              map_width=width, map_height=height, hex_radius_m=1, default_drone_radius_m=1, gdp=GDP(1000, 1))

        flightplans = planner.resolve_requests([Request.from_dict(x) for x in requests_data])

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
                              map_width=width, map_height=height, hex_radius_m=1, default_drone_radius_m=1, gdp=GDP(1000, 1))

        flightplans = planner.resolve_requests([Request.from_dict(x) for x in requests_data])

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
                              map_width=width, map_height=height, hex_radius_m=1, default_drone_radius_m=1, gdp=GDP(1000, 1))

        flightplans = planner.resolve_requests([Request.from_dict(x) for x in requests_data])

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
                              # requests=[Request.from_dict(x) for x in requests_data],
                              map_width=width, map_height=height, hex_radius_m=1, default_drone_radius_m=1, gdp=GDP(10, 1))

        neighbours = planner._get_all_neighbours(HexCoordinate(0, 0), 5, Request((0, 0), (6, 0), 3))

        self.assertEqual(3, len(neighbours))
        self.assertEqual(6, planner._map.int_time(neighbours[0][0]))
        self.assertEqual(6, planner._map.int_time(neighbours[1][0]))
        self.assertEqual(6, planner._map.int_time(neighbours[2][0]))

class CityMapTestCase(TestCase):
    def test_map_size(self):
        heights = np.zeros((40, 40))
        map = CityMap(heights=heights, pixel_size_m=1, hex_radius_m=1, flight_height=50)
        self.assertEqual(25, map.hexagonal_map_size()[0])
        self.assertEqual(28, map.hexagonal_map_size()[1])

        heights = np.zeros((320, 320))
        heights[15, 20] = 1000
        heights[16, 20] = 1000

        map = CityMap(heights=heights, pixel_size_m=1, hex_radius_m=16, flight_height=50)
        obstacles = map.obstacles()

        self.assertEqual(13, map.hexagonal_map_size()[0])
        self.assertEqual(15, map.hexagonal_map_size()[1])

        heights = np.zeros((340, 340))
        heights[15, 20] = 1000
        heights[16, 20] = 1000

        map = CityMap(heights=heights, pixel_size_m=1, hex_radius_m=16, flight_height=50)
        obstacles = map.obstacles()

        self.assertEqual(14, map.hexagonal_map_size()[0])
        self.assertEqual(16, map.hexagonal_map_size()[1])

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
        ], 1, 0, 1)

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
        ], 1, 0, 1)

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
        ], 1, 0, 1)

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
        ], 1, 0, 1)

        smoothed = self._get_points(flightplan.smoothed())
        self.assertEqual(len(smoothed), 3)
        self.assertEqual(smoothed, [
            HexCoordinate(0, 0),
            HexCoordinate(3, 1),
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
                              map_width=width, map_height=height, hex_radius_m=1, default_drone_radius_m=1, gdp=GDP(1000, 1),
                              punish_deviation=True)
        flightplans = planner.resolve_requests([Request.from_dict(x) for x in requests_data])

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
                              map_width=width, map_height=height, hex_radius_m=1, default_drone_radius_m=1, gdp=GDP(1000, 1),
                              punish_deviation=True)
        flightplans = planner.resolve_requests([Request.from_dict(x) for x in requests_data])

        self.assertEqual(6, len(line))
        print(flightplans[0].points)
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


class DifferentRadiiTestCase(TestCase):
    def _get_flightplan_points(self, flightplan):
        return [x for (t, x) in sorted(flightplan.points.items(), key=lambda x : x[0])]

    def test_flying_towards_each_other_no_obstacles(self):
        requests_data = [
            {'from': [10, 0], 'to': [0, 0], 'start_time': 0, 'radius': 2},
            {'from': [0, 0], 'to': [10, 0], 'start_time': 0, 'radius': 1},
        ]
        planner = PathPlanner(obstacles=[],
                              map_width=20, map_height=20, hex_radius_m=1, default_drone_radius_m=1,
                              gdp=GDP(100, 1),
                              punish_deviation=True)

        flightplans = planner.resolve_requests([Request.from_dict(x) for x in requests_data])

        self.assertEqual(6, len(flightplans[0].points))
        self.assertEqual(8, len(flightplans[1].points))

    def test_flying_towards_each_other_no_obstacles2(self):
        requests_data = [
            {'from': [10, 0], 'to': [0, 0], 'start_time': 0, 'radius': 3},
            {'from': [0, 0], 'to': [10, 0], 'start_time': 0, 'radius': 1},
        ]
        planner = PathPlanner(obstacles=[],
                              map_width=20, map_height=20, hex_radius_m=1, default_drone_radius_m=1,
                              gdp=GDP(100, 1),
                              punish_deviation=True)

        flightplans = planner.resolve_requests([Request.from_dict(x) for x in requests_data])

        self.assertEqual(6, len(flightplans[0].points))
        self.assertEqual(9, len(flightplans[1].points))

    def test_flying_around_obstacle_with_radius(self):
        requests_data = [
            {'from': [0, 0], 'to': [10, 0], 'start_time': 0, 'radius': 2},
        ]
        planner = PathPlanner(obstacles=[HexCoordinate(4, 0)],
                              map_width=20, map_height=20, hex_radius_m=1, default_drone_radius_m=1,
                              gdp=GDP(100, 1),
                              punish_deviation=True)

        flightplans = planner.resolve_requests([Request.from_dict(x) for x in requests_data])

        self.assertEqual(8, len(flightplans[0].points))
        self.assertEqual([
            HexCoordinate(0, 0),
            HexCoordinate(1, 1),
            HexCoordinate(2, 2),
            HexCoordinate(4, 2),
            HexCoordinate(6, 2),
            HexCoordinate(7, 1),
            HexCoordinate(8, 0),
            HexCoordinate(10, 0),
        ], self._get_flightplan_points(flightplans[0]))


class NoViablePathDetectionTestCase(TestCase):
    @pytest.mark.timeout(5)
    def test_simple(self):
        requests_data = [
            {'from': [0, 0], 'to': [8, 0], 'start_time': 0, 'radius': 1},
        ]
        obstacles = [
            HexCoordinate(2, 0),
            HexCoordinate(3, 1),
            HexCoordinate(2, 2),
            HexCoordinate(3, 3),
            HexCoordinate(2, 4),
            HexCoordinate(3, 5),
        ]
        planner = PathPlanner(obstacles=obstacles,
                              map_width=5, map_height=5, hex_radius_m=1, default_drone_radius_m=1,
                              gdp=GDP(100, 1),
                              punish_deviation=True)

        self.assertRaises(PathNotFoundException, planner.resolve_requests, [Request.from_dict(x) for x in requests_data])

    @pytest.mark.timeout(5)
    def test_radius(self):
        requests_data = [
            {'from': [0, 0], 'to': [8, 0], 'start_time': 0, 'radius': 2},
        ]
        obstacles = [
            HexCoordinate(3, 1),
            HexCoordinate(2, 2),
            HexCoordinate(3, 3),
            HexCoordinate(2, 4),
            HexCoordinate(3, 5),
        ]
        planner = PathPlanner(obstacles=obstacles,
                              map_width=5, map_height=5, hex_radius_m=1, default_drone_radius_m=1,
                              gdp=GDP(100, 1),
                              punish_deviation=True)

        self.assertRaises(PathNotFoundException, planner.resolve_requests, [Request.from_dict(x) for x in requests_data])

    def test_surrounded(self):
        requests_data = [
            {'from': [0, 0], 'to': [8, 0], 'start_time': 0, 'radius': 2},
        ]
        obstacles = [
            HexCoordinate(6, 0),
            HexCoordinate(7, 1),
            HexCoordinate(9, 1),
            HexCoordinate(10, 0),
        ]
        planner = PathPlanner(obstacles=obstacles,
                              map_width=10, map_height=10, hex_radius_m=1, default_drone_radius_m=1,
                              gdp=GDP(100, 1),
                              punish_deviation=True)

        self.assertRaises(PathNotFoundException, planner.resolve_requests, [Request.from_dict(x) for x in requests_data])

    def test_surrounded_from_all_sides(self):
        requests_data = [
            {'from': [0, 0], 'to': [7, 3], 'start_time': 0, 'radius': 1},
        ]
        obstacles = [
            HexCoordinate(6, 2),
            HexCoordinate(8, 2),
            HexCoordinate(9, 3),
            HexCoordinate(8, 4),
            HexCoordinate(6, 4),
            HexCoordinate(5, 3),
        ]
        planner = PathPlanner(obstacles=obstacles,
                              map_width=10, map_height=10, hex_radius_m=1, default_drone_radius_m=1,
                              gdp=GDP(100, 1),
                              punish_deviation=True)

        self.assertRaises(PathNotFoundException, planner.resolve_requests, [Request.from_dict(x) for x in requests_data])

    def test_destination_blocked(self):
        requests_data = [
            {'from': [0, 0], 'to': [7, 3], 'start_time': 0, 'radius': 1},
        ]
        obstacles = [
            HexCoordinate(7, 3),
        ]
        planner = PathPlanner(obstacles=obstacles,
                              map_width=10, map_height=10, hex_radius_m=1, default_drone_radius_m=1,
                              gdp=GDP(100, 1),
                              punish_deviation=True)

        self.assertRaises(PathNotFoundException, planner.resolve_requests, [Request.from_dict(x) for x in requests_data])


class TimeUncertaintyTestCase(TestCase):
    def test_flighplan_is_present_at_time(self):
        f = Flightplan(points=[(0, HexCoordinate(0, 0)),
                               (1, HexCoordinate(0, 0)),
                               (2, HexCoordinate(0, 0)),
                               (3, HexCoordinate(0, 0)),
                               (4, HexCoordinate(0, 0)),
        ], radius_hex=1, time_uncertainty=3, speed_hex=1)

        self.assertTrue(f.is_present_at_time_with_uncertainty(0, 0))
        self.assertTrue(f.is_present_at_time_with_uncertainty(1, 0))
        self.assertTrue(f.is_present_at_time_with_uncertainty(2, 0))
        self.assertTrue(f.is_present_at_time_with_uncertainty(3, 0))
        self.assertTrue(f.is_present_at_time_with_uncertainty(4, 0))
        self.assertTrue(f.is_present_at_time_with_uncertainty(5, 0))
        self.assertTrue(f.is_present_at_time_with_uncertainty(6, 0))
        self.assertTrue(f.is_present_at_time_with_uncertainty(7, 0))
        self.assertFalse(f.is_present_at_time_with_uncertainty(8, 0))

    def test_flighplan_is_not_present_at_time(self):
        f = Flightplan(points=[(0, HexCoordinate(0, 0)),
                               (1, HexCoordinate(0, 0)),
                               (2, HexCoordinate(0, 0)),
                               (3, HexCoordinate(0, 0)),
                               (4, HexCoordinate(0, 0)),
        ], radius_hex=1, time_uncertainty=0, speed_hex=1)

        self.assertTrue(f.is_present_at_time_with_uncertainty(0, 0))
        self.assertTrue(f.is_present_at_time_with_uncertainty(1, 0))
        self.assertTrue(f.is_present_at_time_with_uncertainty(2, 0))
        self.assertTrue(f.is_present_at_time_with_uncertainty(3, 0))
        self.assertTrue(f.is_present_at_time_with_uncertainty(4, 0))
        self.assertFalse(f.is_present_at_time_with_uncertainty(5, 0))
        self.assertFalse(f.is_present_at_time_with_uncertainty(6, 0))
        self.assertFalse(f.is_present_at_time_with_uncertainty(7, 0))
        self.assertFalse(f.is_present_at_time_with_uncertainty(8, 0))

    def test_flightplan_is_present_at_time_with_both_uncertainties(self):
        f = Flightplan(points=[(0, HexCoordinate(0, 0)),
                               (1, HexCoordinate(0, 0)),
                               (2, HexCoordinate(0, 0)),
                               (3, HexCoordinate(0, 0)),
                               (4, HexCoordinate(0, 0)),
                               ], radius_hex=1, time_uncertainty=3, speed_hex=1)

        self.assertFalse(f.is_present_at_time_with_uncertainty(-3, 2))
        self.assertTrue(f.is_present_at_time_with_uncertainty(-2, 2))
        self.assertTrue(f.is_present_at_time_with_uncertainty(-1, 2))
        self.assertTrue(f.is_present_at_time_with_uncertainty(0, 2))
        self.assertTrue(f.is_present_at_time_with_uncertainty(1, 2))
        self.assertTrue(f.is_present_at_time_with_uncertainty(2, 2))
        self.assertTrue(f.is_present_at_time_with_uncertainty(3, 2))
        self.assertTrue(f.is_present_at_time_with_uncertainty(4, 2))
        self.assertTrue(f.is_present_at_time_with_uncertainty(5, 2))
        self.assertTrue(f.is_present_at_time_with_uncertainty(6, 2))
        self.assertTrue(f.is_present_at_time_with_uncertainty(7, 2))
        self.assertFalse(f.is_present_at_time_with_uncertainty(8, 2))

    def _get_flightplan_points(self, flightplan):
        return [x for (t, x) in sorted(flightplan.points.items(), key=lambda x : x[0])]

    def test_first_drone_time_uncertainty(self):
        requests_data = [
            {'from': [10, 0], 'to': [0, 0], 'start_time': 0, 'radius': 1, 'time_uncertainty': 2},
            {'from': [0, 0], 'to': [10, 0], 'start_time': 0, 'radius': 1, 'time_uncertainty': 0},
        ]
        planner = PathPlanner(obstacles=[],
                              map_width=20, map_height=20, hex_radius_m=1, default_drone_radius_m=1,
                              gdp=GDP(100, 1),
                              punish_deviation=True)

        flightplans = planner.resolve_requests([Request.from_dict(x) for x in requests_data])

        self.assertEqual(6, len(flightplans[0].points))
        self.assertEqual(7, len(flightplans[1].points))

        self.assertEqual(self._get_flightplan_points(flightplans[0]), [
            HexCoordinate(10, 0),
            HexCoordinate(8, 0),
            HexCoordinate(6, 0),
            HexCoordinate(4, 0),
            HexCoordinate(2, 0),
            HexCoordinate(0, 0),
        ])

        self.assertEqual(self._get_flightplan_points(flightplans[1]), [
            HexCoordinate(0, 0),
            HexCoordinate(2, 0),
            HexCoordinate(4, 0),
            HexCoordinate(5, 1),
            HexCoordinate(7, 1),
            HexCoordinate(8, 0),
            HexCoordinate(10, 0),
        ])

    def test_second_drone_time_uncertainty(self):
        requests_data = [
            {'from': [10, 0], 'to': [0, 0], 'start_time': 0, 'radius': 1, 'time_uncertainty': 0},
            {'from': [0, 0], 'to': [10, 0], 'start_time': 0, 'radius': 1, 'time_uncertainty': 2},
        ]
        planner = PathPlanner(obstacles=[],
                              map_width=20, map_height=20, hex_radius_m=1, default_drone_radius_m=1,
                              gdp=GDP(100, 1),
                              punish_deviation=True)

        flightplans = planner.resolve_requests([Request.from_dict(x) for x in requests_data])

        self.assertEqual(6, len(flightplans[0].points))
        self.assertEqual(7, len(flightplans[1].points))

        self.assertEqual(self._get_flightplan_points(flightplans[0]), [
            HexCoordinate(10, 0),
            HexCoordinate(8, 0),
            HexCoordinate(6, 0),
            HexCoordinate(4, 0),
            HexCoordinate(2, 0),
            HexCoordinate(0, 0),
        ])

        self.assertEqual(self._get_flightplan_points(flightplans[1]), [
            HexCoordinate(0, 0),
            HexCoordinate(2, 0),
            HexCoordinate(3, 1),
            HexCoordinate(5, 1),
            HexCoordinate(6, 0),
            HexCoordinate(8, 0),
            HexCoordinate(10, 0),
        ])

    def test_both_drones_time_uncertainty(self):
        requests_data = [
            {'from': [10, 0], 'to': [0, 0], 'start_time': 0, 'radius': 1, 'time_uncertainty': 2},
            {'from': [0, 0], 'to': [10, 0], 'start_time': 0, 'radius': 1, 'time_uncertainty': 2},
        ]
        planner = PathPlanner(obstacles=[],
                              map_width=10, map_height=10, hex_radius_m=1, default_drone_radius_m=1,
                              gdp=GDP(0, 1),
                              punish_deviation=True)

        flightplans = planner.resolve_requests([Request.from_dict(x) for x in requests_data])

        self.assertEqual(6, len(flightplans[0].points))
        self.assertEqual(7, len(flightplans[1].points))

        self.assertEqual(self._get_flightplan_points(flightplans[0]), [
            HexCoordinate(10, 0),
            HexCoordinate(8, 0),
            HexCoordinate(6, 0),
            HexCoordinate(4, 0),
            HexCoordinate(2, 0),
            HexCoordinate(0, 0),
        ])

        self.assertEqual(self._get_flightplan_points(flightplans[1]), [
            HexCoordinate(0, 0),
            HexCoordinate(2, 0),
            HexCoordinate(3, 1),
            HexCoordinate(5, 1),
            HexCoordinate(7, 1),
            HexCoordinate(8, 0),
            HexCoordinate(10, 0),
        ])

        data = planner.get_data_as_dict()

        with open('./results/test_results.json', 'w') as wf:
            simplejson.dump(data, wf)

    def test_both_drones_long_time_uncertainty(self):
        requests_data = [
            {'from': [10, 0], 'to': [0, 0], 'start_time': 0, 'radius': 1, 'time_uncertainty': 4},
            {'from': [0, 0], 'to': [10, 0], 'start_time': 0, 'radius': 1, 'time_uncertainty': 4},
        ]
        planner = PathPlanner(obstacles=[],
                              map_width=10, map_height=10, hex_radius_m=1, default_drone_radius_m=1,
                              gdp=GDP(0, 1),
                              punish_deviation=True)

        flightplans = planner.resolve_requests([Request.from_dict(x) for x in requests_data])

        self.assertEqual(6, len(flightplans[0].points))
        self.assertEqual(7, len(flightplans[1].points))

        self.assertEqual(self._get_flightplan_points(flightplans[0]), [
            HexCoordinate(10, 0),
            HexCoordinate(8, 0),
            HexCoordinate(6, 0),
            HexCoordinate(4, 0),
            HexCoordinate(2, 0),
            HexCoordinate(0, 0),
        ])

        self.assertEqual(self._get_flightplan_points(flightplans[1]), [
            HexCoordinate(0, 0),
            HexCoordinate(1, 1),
            HexCoordinate(3, 1),
            HexCoordinate(5, 1),
            HexCoordinate(7, 1),
            HexCoordinate(9, 1),
            HexCoordinate(10, 0),
        ])

        data = planner.get_data_as_dict()

        with open('./results/test_results.json', 'w') as wf:
            simplejson.dump(data, wf)

    def test_time_uncertainty_with_radius(self):
        requests_data = [
            {'from': [20, 0], 'to': [0, 0], 'start_time': 0, 'radius': 2, 'time_uncertainty': 2},
            {'from': [0, 0], 'to': [20, 0], 'start_time': 0, 'radius': 2, 'time_uncertainty': 2},
        ]
        planner = PathPlanner(obstacles=[],
                              map_width=30, map_height=30, hex_radius_m=1, default_drone_radius_m=1,
                              gdp=GDP(0, 1),
                              punish_deviation=True)

        flightplans = planner.resolve_requests([Request.from_dict(x) for x in requests_data])

        self.assertEqual(11, len(flightplans[0].points))
        self.assertEqual(14, len(flightplans[1].points))

        self.assertEqual(self._get_flightplan_points(flightplans[0]), [
            HexCoordinate(20, 0),
            HexCoordinate(18, 0),
            HexCoordinate(16, 0),
            HexCoordinate(14, 0),
            HexCoordinate(12, 0),
            HexCoordinate(10, 0),
            HexCoordinate(8, 0),
            HexCoordinate(6, 0),
            HexCoordinate(4, 0),
            HexCoordinate(2, 0),
            HexCoordinate(0, 0),
        ])

        self.assertEqual(self._get_flightplan_points(flightplans[1]), [
            HexCoordinate(x=0, y=0),
            HexCoordinate(x=2, y=0),
            HexCoordinate(x=3, y=1),
            HexCoordinate(x=4, y=2),
            HexCoordinate(x=5, y=3),
            HexCoordinate(x=7, y=3),
            HexCoordinate(x=9, y=3),
            HexCoordinate(x=11, y=3),
            HexCoordinate(x=12, y=2),
            HexCoordinate(x=13, y=1),
            HexCoordinate(x=14, y=0),
            HexCoordinate(x=16, y=0),
            HexCoordinate(x=18, y=0),
            HexCoordinate(x=20, y=0),
        ])

        data = planner.get_data_as_dict()

        with open('./results/test_results.json', 'w') as wf:
            simplejson.dump(data, wf)

    def test_impossible_time_uncertainty_with_radius(self):
        requests_data = [
            {'from': [10, 0], 'to': [0, 0], 'start_time': 0, 'radius': 2, 'time_uncertainty': 2},
            {'from': [0, 0], 'to': [10, 0], 'start_time': 0, 'radius': 2, 'time_uncertainty': 2},
        ]
        planner = PathPlanner(obstacles=[],
                              map_width=30, map_height=30, hex_radius_m=1, default_drone_radius_m=1,
                              gdp=GDP(0, 1),
                              punish_deviation=True)

        self.assertRaises(PathNotFoundException, planner.resolve_requests, [Request.from_dict(x) for x in requests_data])


class DifferentSpeedTestCase(TestCase):
    def _get_flightplan_points(self, flightplan):
        return [x for (t, x) in sorted(flightplan.points.items(), key=lambda x : x[0])]

    def test_single_slower_drone(self):
        requests_data = [
            {'from': [20, 0], 'to': [0, 0], 'start_time': 0, 'radius': 1, 'time_uncertainty': 0, 'speed_hex': 0.5},
        ]

        planner = PathPlanner(obstacles=[],
                              map_width=30, map_height=30, hex_radius_m=1, default_drone_radius_m=1,
                              gdp=GDP(0, 1),
                              punish_deviation=True)

        flightplans = planner.resolve_requests([Request.from_dict(x) for x in requests_data])

        self.assertEqual(11, len(flightplans[0].points))

        self.assertEqual(self._get_flightplan_points(flightplans[0]), [
            HexCoordinate(x=20, y=0),
            HexCoordinate(x=18, y=0),
            HexCoordinate(x=16, y=0),
            HexCoordinate(x=14, y=0),
            HexCoordinate(x=12, y=0),
            HexCoordinate(x=10, y=0),
            HexCoordinate(x=8, y=0),
            HexCoordinate(x=6, y=0),
            HexCoordinate(x=4, y=0),
            HexCoordinate(x=2, y=0),
            HexCoordinate(x=0, y=0),
        ])

        self.assertEqual(flightplans[0].points[0], HexCoordinate(x=20, y=0))
        self.assertEqual(flightplans[0].points[2], HexCoordinate(x=18, y=0))
        self.assertEqual(flightplans[0].points[20], HexCoordinate(x=0, y=0))

        self.assertEqual(len(planner.list_of_occupied_hexes(0)), 1)
        self.assertEqual(len(planner.list_of_occupied_hexes(1)), 2)
        self.assertEqual(len(planner.list_of_occupied_hexes(2)), 1)
        self.assertEqual(len(planner.list_of_occupied_hexes(3)), 2)


        data = planner.get_data_as_dict()

        with open('./results/test_results.json', 'w') as wf:
            simplejson.dump(data, wf)


    def test_slow_drone_occupies_two_hexes_simultaneously(self):
        requests_data = [
            {'from': [4, 6], 'to': [4, 0], 'start_time': 0, 'radius': 1, 'time_uncertainty': 0, 'speed_hex': 0.01},
            {'from': [0, 6], 'to': [6, 6], 'start_time': 0, 'radius': 1, 'time_uncertainty': 0, 'speed_hex': 1},
        ]

        planner = PathPlanner(obstacles=[],
                              map_width=30, map_height=30, hex_radius_m=1, default_drone_radius_m=1,
                              gdp=GDP(0, 1),
                              punish_deviation=True)

        flightplans = planner.resolve_requests([Request.from_dict(x) for x in requests_data])

        self.assertEqual(7, len(flightplans[0].points))

        self.assertEqual(self._get_flightplan_points(flightplans[0]), [
            HexCoordinate(x=4, y=6),
            HexCoordinate(x=3, y=5),
            HexCoordinate(x=4, y=4),
            HexCoordinate(x=3, y=3),
            HexCoordinate(x=4, y=2),
            HexCoordinate(x=3, y=1),
            HexCoordinate(x=4, y=0),
        ])

        self.assertEqual(self._get_flightplan_points(flightplans[1]), [
            HexCoordinate(x=0, y=6),
            HexCoordinate(x=2, y=6),
            HexCoordinate(x=3, y=7),
            HexCoordinate(x=5, y=7),
            HexCoordinate(x=6, y=6),
        ])

        data = planner.get_data_as_dict()

        with open('./results/test_results.json', 'w') as wf:
            simplejson.dump(data, wf)


class DouglasPeckerSmoothingTestCase(TestCase):
    def test_simple_smoothing(self):
        requests_data = [
            {'from': [6, 6], 'to': [0, 0], 'start_time': 0, 'radius': 1},
        ]

        planner = PathPlanner(obstacles=[],
                              map_width=30, map_height=30, hex_radius_m=1, default_drone_radius_m=1,
                              gdp=GDP(0, 1),
                              punish_deviation=True)

        flightplans = planner.resolve_requests([Request.from_dict(x) for x in requests_data])

        smoothed_points = flightplans[0].smoothed(1)

        self.assertEqual(len(smoothed_points), 2)

        data = planner.get_data_as_dict()

        with open('./results/test_results.json', 'w') as wf:
            simplejson.dump(data, wf)

    def test_two_point_smoothing(self):
        points = [
            (0, HexCoordinate(0, 0)),
            (1, HexCoordinate(2, 0)),
            (2, HexCoordinate(4, 0)),
            (3, HexCoordinate(6, 0)),
            (4, HexCoordinate(8, 0)),
            (5, HexCoordinate(9, 1)),
            (6, HexCoordinate(8, 2)),
            (7, HexCoordinate(9, 3)),
            (8, HexCoordinate(8, 4)),
            (9, HexCoordinate(9, 5)),
        ]
        fp = Flightplan(points=points, radius_hex=1, time_uncertainty=0, speed_hex=1)
        smoothed_points = fp.smoothed()

        self.assertEqual(len(smoothed_points), 3)

    def test_obstacle_avoidance_smoothing(self):
        requests_data = [
            {'from': [0, 0], 'to': [0, 2], 'start_time': 0, 'radius': 1},
        ]

        planner = PathPlanner(obstacles=[
                                HexCoordinate(1, 1),
                                HexCoordinate(2, 2),
                                HexCoordinate(3, 3),
                                HexCoordinate(4, 4),
                                HexCoordinate(5, 5),
                                         ],
                              map_width=8, map_height=8, hex_radius_m=1, default_drone_radius_m=1,
                              gdp=GDP(0, 1),
                              punish_deviation=True)

        flightplans = planner.resolve_requests([Request.from_dict(x) for x in requests_data])

        smoothed_points = flightplans[0].smoothed(1)

        self.assertEqual(len(smoothed_points), 5)

        data = planner.get_data_as_dict()

        with open('./results/test_results.json', 'w') as wf:
            simplejson.dump(data, wf)


class CacheErasureTestCase(TestCase):
    def test_jumping_can_occur_cache_erasure_resolve_request(self):
        planner = PathPlanner(obstacles=[],
            map_width=8, map_height=8, hex_radius_m=1, default_drone_radius_m=1,
            gdp=GDP(0, 1),
            punish_deviation=True)

        self.assertFalse(planner.jumping_can_occur())
        self.assertFalse(planner.jumping_can_occur())

        planner.resolve_request(Request(HexCoordinate(0, 0), HexCoordinate(2, 2), 0, drone_radius_m=2))
        self.assertFalse(planner.jumping_can_occur())
        self.assertFalse(planner.jumping_can_occur())

        planner.resolve_request(Request(HexCoordinate(0, 0), HexCoordinate(2, 2), 10, drone_radius_m=2))
        self.assertFalse(planner.jumping_can_occur())
        self.assertFalse(planner.jumping_can_occur())

        planner.resolve_request(Request(HexCoordinate(0, 0), HexCoordinate(2, 2), 20, drone_radius_m=1))
        self.assertFalse(planner.jumping_can_occur())
        self.assertFalse(planner.jumping_can_occur())

        planner.resolve_request(Request(HexCoordinate(0, 0), HexCoordinate(2, 2), 30, drone_radius_m=1))
        self.assertTrue(planner.jumping_can_occur())
        self.assertTrue(planner.jumping_can_occur())

    def test_jumping_can_occur_cache_erasure_resolve_requests(self):
        planner = PathPlanner(obstacles=[],
            map_width=8, map_height=8, hex_radius_m=1, default_drone_radius_m=1,
            gdp=GDP(0, 1),
            punish_deviation=True)

        requests = [
            Request(HexCoordinate(0, 0), HexCoordinate(2, 2), 0, drone_radius_m=2),
            Request(HexCoordinate(0, 0), HexCoordinate(2, 2), 10, drone_radius_m=1),
            Request(HexCoordinate(0, 0), HexCoordinate(2, 2), 20, drone_radius_m=1),
        ]
        self.assertFalse(planner.jumping_can_occur())
        self.assertFalse(planner.jumping_can_occur())

        planner.resolve_requests(requests)
        self.assertTrue(planner.jumping_can_occur())
        self.assertTrue(planner.jumping_can_occur())

    def test_jumping_can_occur_cache_erasure_resolve_requests_2(self):
        planner = PathPlanner(obstacles=[],
                              map_width=8, map_height=8, hex_radius_m=1, default_drone_radius_m=1,
                              gdp=GDP(0, 1),
                              punish_deviation=True)

        requests = [
            Request(HexCoordinate(0, 0), HexCoordinate(2, 2), 0, drone_radius_m=2),
            Request(HexCoordinate(0, 0), HexCoordinate(2, 2), 10, drone_radius_m=2),
            Request(HexCoordinate(0, 0), HexCoordinate(2, 2), 20, drone_radius_m=1),
        ]
        self.assertFalse(planner.jumping_can_occur())
        self.assertFalse(planner.jumping_can_occur())

        planner.resolve_requests(requests)
        self.assertFalse(planner.jumping_can_occur())
        self.assertFalse(planner.jumping_can_occur())

    def test_two_instances(self):
        planner = PathPlanner(obstacles=[],
                              map_width=8, map_height=8, hex_radius_m=1, default_drone_radius_m=1,
                              gdp=GDP(0, 1),
                              punish_deviation=True)

        requests = [
            Request(HexCoordinate(0, 0), HexCoordinate(2, 2), 0, drone_radius_m=2),
            Request(HexCoordinate(0, 0), HexCoordinate(2, 2), 10, drone_radius_m=1),
            Request(HexCoordinate(0, 0), HexCoordinate(2, 2), 20, drone_radius_m=1),
        ]
        self.assertFalse(planner.jumping_can_occur())
        self.assertFalse(planner.jumping_can_occur())

        planner.resolve_requests(requests)
        self.assertTrue(planner.jumping_can_occur())
        self.assertTrue(planner.jumping_can_occur())

        planner2 = PathPlanner(obstacles=[],
                              map_width=8, map_height=8, hex_radius_m=1, default_drone_radius_m=1,
                              gdp=GDP(0, 1),
                              punish_deviation=True)

        self.assertFalse(planner2.jumping_can_occur())
        planner2.resolve_request(Request(HexCoordinate(0, 0), HexCoordinate(2, 2), 0, drone_radius_m=2))

        self.assertTrue(planner.jumping_can_occur())
        self.assertTrue(planner.jumping_can_occur())
        self.assertFalse(planner2.jumping_can_occur())
        self.assertFalse(planner2.jumping_can_occur())
