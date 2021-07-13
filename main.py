import numpy as np
import simplejson

# from hexagonal.misc_functions import HexCoordinate
# from hexagonal.path_planner import CityMap, Request, PathPlanner
from common import Request, GDP
from planner import Layer, RoutePlanner
from street_network.path_planner import StreetNetwork, PathPlanner


# def run_ny_map():
#     requests_fname = 'ny_1_requests_random_2'
#     heights_fname = 'ny_1_heights'
#     map_details_fname = 'ny_1_details'
#
#     with open('./data/{}.json'.format(requests_fname), 'r') as requests_f, open('./data/{}.json'.format(heights_fname), 'r') as heights_f, open('./data/{}.json'.format(map_details_fname), 'r') as map_f:
#         map_data = simplejson.load(map_f)
#         heights_raw = np.array(simplejson.load(heights_f))
#         city_map = CityMap(heights=heights_raw, pixel_size_m=map_data["pixel_size_m"], hex_radius_m=map_data["hex_radius_m"], flight_height=map_data["flight_height"])
#
#         width = city_map.hexagonal_map_size()[0]
#         height = city_map.hexagonal_map_size()[1]
#
#         data = simplejson.load(requests_f)
#         requests_data = data['requests']
#
#         requests = city_map.requests_to_hex_grid(requests_data)
#
#         obstacles = city_map.obstacles()
#         planner = PathPlanner(obstacles=obstacles, map_width=width, map_height=height,
#                               default_drone_radius_m=2, hex_radius_m=1,
#                               gdp=GDP(max_time=data['gdp']['max_time'], penalty=data['gdp']['penalty']), city_map=city_map, punish_deviation=True)
#         planner.resolve_requests(requests)
#
#         data = planner.get_data_as_dict()
#
#         with open('./results/ny_results.json', 'w') as wf:
#             simplejson.dump(data, wf)
#
#
# def run_artificial():
#     fname = 'requests'
#     fname = 'requests_towards_each_other'
#     fname = 'requests_towards_each_other_radius'
#     # fname = 'requests_towards_radius_gdp'
#     # fname = 'requests_towards_radius_gdp_longer'
#     # fname = 'requests_worst_case'
#
#     with open('./data/{}.json'.format(fname), 'r') as f:
#         width = 7
#         height = 7
#
#         data = simplejson.load(f)
#         requests_data = data['requests']
#
#         requests = [Request(x['from'], x['to'], x['start_time']) for x in requests_data]
#         # requests = []
#         # for i in range(20):
#         #     requests.append(Request((0, 4), (12, 4), i))
#         #
#         # for i in range(20):
#         #     requests.append(Request((4, 0), (4, 6), i))
#         #     requests.append(Request((6, 0), (6, 6), i))
#         #     # requests.append(Request((8, 0), (8, 6), i))
#         #     # requests.append(Request((2, 0), (2, 6), i))
#         #
#         # # random.shuffle(requests)
#
#         obstacles_data = data['obstacles']
#         obstacles = [HexCoordinate(x=x[0], y=x[1]) for x in obstacles_data]
#         planner = PathPlanner(obstacles=obstacles, map_width=width, map_height=height,
#                               default_drone_radius_m=data['radius'], hex_radius_m=1,
#                               gdp=GDP(max_time=data['gdp']['max_time'], penalty=data['gdp']['penalty']), punish_deviation=True)
#         planner.resolve_requests(requests)
#
#         data = planner.get_data_as_dict()
#
#         with open('./results/results.json', 'w') as wf:
#             simplejson.dump(data, wf)


def run_street_network_vienn():
    sn = StreetNetwork.from_graphml_file('../Test_Scenario/Test_Scenario/data/street_data/graph_files/processed_graphM2.graphml')
    fr = (16.3285369, 48.2210692)
    to = (16.3424207, 48.2172288)

    request = Request(fr, to, 0, 15, 0, 10, GDP(0, 1000))

    layers = [
        Layer(50, Layer.Type.NETWORK, PathPlanner(sn, 1, 15, GDP(0, 1000)))
    ]

    rp = RoutePlanner(layers)
    res = rp.resolve_requests([request, ])

    with open('new_scenario.scn', 'w') as f:
        f.write(res[0])


if __name__ == '__main__':
    run_street_network_vienn()
