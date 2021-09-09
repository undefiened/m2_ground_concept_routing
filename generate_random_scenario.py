import simplejson
import numpy as np

from main import CityMap, SNPathPlanner, GDP, Request, HexHelper
from ground_routing.hexagonal import _is_feasible_coordinate


def generate_random_requests(number_of_requests_to_generate, time_span, gdp, radius, new_requests_fname, heights_fname, map_details_fname):
    with open('./data/{}.json'.format(new_requests_fname), 'w') as requests_f, open('./data/{}.json'.format(heights_fname), 'r') as heights_f, open('./data/{}.json'.format(map_details_fname), 'r') as map_f:
        map_data = simplejson.load(map_f)
        heights_raw = np.array(simplejson.load(heights_f))
        city_map = CityMap(heights=heights_raw, pixel_size_m=map_data["pixel_size_m"],
                           hex_radius_m=map_data["hex_radius_m"], flight_height=map_data["flight_height"])

        width = city_map.hexagonal_map_size()[0]
        height = city_map.hexagonal_map_size()[1]

        data = {
            'requests': [],
            'gdp': gdp,
            'radius': radius
        }

        obstacles = city_map.obstacles()
        planner = SNPathPlanner(obstacles=obstacles, map_width=width, map_height=height,
                                default_drone_radius_m=radius, hex_radius_m=1,
                                gdp=GDP(max_time=data['gdp']['max_time'], penalty=data['gdp']['penalty']),
                                city_map=city_map)

        for i in range(number_of_requests_to_generate):
            good_request = False

            while not good_request:
                good_request = True
                start_p = np.array([np.random.randint(0, city_map.heights_map.shape[1]), np.random.randint(0, city_map.heights_map.shape[0])])
                end_p = np.array([np.random.randint(0, city_map.heights_map.shape[1]), np.random.randint(0, city_map.heights_map.shape[0])])

                start = city_map.coord_to_hex(start_p[0], start_p[1])
                end = city_map.coord_to_hex(end_p[0], end_p[1])

                start_time = np.random.randint(time_span[0], time_span[1])

                if not _is_feasible_coordinate(start.x, start.y, height, width) or not _is_feasible_coordinate(end.x, end.y, height, width):
                    good_request = False
                    continue

                if HexHelper.hex_distance(start, end) < 1:
                    good_request = False
                    continue

                print(start)
                print(end)
                print('----------------------')
                new_request = Request(start=start, end=end, start_time=start_time)
                is_connected = planner._is_destination_reachable(new_request)

                if not is_connected:
                    good_request = False
                    continue

            data['requests'].append({'from': start_p.tolist(), 'to': end_p.tolist(), 'start_time': start_time})

        simplejson.dump(data, requests_f)

if __name__ == "__main__":
    number_of_requests_to_generate = 20
    time_span = [0, 100]
    gdp = {'max_time': 50, 'penalty': 1}
    radius = 2

    new_requests_fname = 'ny_1_requests_random_2'
    heights_fname = 'ny_1_heights'
    map_details_fname = 'ny_1_details'

    generate_random_requests(number_of_requests_to_generate, time_span, gdp, radius, new_requests_fname, heights_fname, map_details_fname)
