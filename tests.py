from unittest import TestCase

import simplejson

from main import HexMap, HexCoordinate, PathPlanner, Request, GDP


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
        planner = PathPlanner(obstacles=[], requests=[Request(x['from'], x['to'], x['start_time']) for x in requests_data], map_width=width, map_height=height)
        flightplans = planner.resolve_all()

        data = {
            'map': {
                'width': width,
                'height': height
            },
            'flightplans': [],
            'obstacles': []
        }

        for flightplan in flightplans:
            data['flightplans'].append([(time, point.x, point.y) for (time, point) in flightplan.points.items()])

        with open('./results/results.json', 'w') as wf:
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

        data = planner.get_data_as_dict()

        with open('./results/results.json', 'w') as wf:
            simplejson.dump(data, wf)
