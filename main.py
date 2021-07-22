import csv
import os
import pickle
from typing import Tuple

# from hexagonal.misc_functions import HexCoordinate
# from hexagonal.path_planner import CityMap, Request, PathPlanner
from ground_routing.common import Request, GDP, TurnParamsTable, TurnParams
from ground_routing.street_network.path_planner import PathPlanner, StreetNetwork
from ground_routing.planner import Layer, RoutePlanner


def read_M2_flight_intents_file(filename):
    def parse_M2_time(time: str) -> int:
        h, m, s = time.split(':')
        return int(h) * 3600 + int(m) * 60 + int(s)

    def parse_M2_tuple(tuple: str) -> Tuple[float, float]:
        c1, c2 = tuple.replace('(', '').replace(')', '').split(',')
        return float(c2), float(c1)

    with open(filename, 'r') as f:
        res = list(csv.reader(f, delimiter='\t'))

        intents = []

        for line in res:
            intents.append({
                'intent_arrival_time': parse_M2_time(line[0]),
                'id': line[1],
                'model': line[2],
                'time_start': parse_M2_time(line[3]),
                'origin': parse_M2_tuple(line[4]),
                'destination': parse_M2_tuple(line[5]),
                'priority': int(line[6])
            })

        return sorted(intents, key=lambda x: x['priority'])


def run_street_network_vienn():
    CACHE = True
    intents = read_M2_flight_intents_file('../Test_Scenario/M2_Test_FlightPlan.txt')

    turn_costs = TurnParamsTable([
        TurnParams(0, 30, 30, 0),
        TurnParams(30, 180, 10, 4)
    ])
    spd_m_s = 14

    sn = StreetNetwork.from_graphml_file('../Test_Scenario/Test_Scenario/data/street_data/graph_files/processed_graphM2.graphml', turn_costs)

    requests = []

    interesting_intents = range(len(intents))
    # interesting_intents = range()
    # interesting_intents = [9, ]
    gdp = GDP(10, 60, 1)

    for intent in [intents[x] for x in interesting_intents]:
        request = Request(intent['origin'], intent['destination'], 15, spd_m_s, intent['time_start'], 60, gdp)
        requests.append(request)

    layers = [
        Layer(0, Layer.Type.NETWORK, PathPlanner(sn, 1, spd_m_s, gdp)),
        Layer(50, Layer.Type.NETWORK, PathPlanner(sn, 1, spd_m_s, gdp)),
        Layer(100, Layer.Type.NETWORK, PathPlanner(sn, 1, spd_m_s, gdp)),
    ]

    rp = RoutePlanner(layers, turn_costs)

    if CACHE:
        if not os.path.isfile('cache.pickle'):
            with open('cache.pickle', 'wb') as f:
                flightplans, layers = rp.resolve_requests(requests, skip_coloring=True)
                pickle.dump({'flightplans': flightplans, 'layers': layers, 'requests': requests}, f)
        else:
            with open('cache.pickle', 'rb') as f:
                d = pickle.load(f)
                flightplans = d['flightplans']
                layers = d['layers']
    else:
        flightplans, layers = rp.resolve_requests(requests, skip_coloring=True)

    res = rp.convert_flightplans_to_M2_scenarios(flightplans, layers)

    with open('new_scenario.scn', 'w') as f:
        f.write(''.join(res))


if __name__ == '__main__':
    run_street_network_vienn()
