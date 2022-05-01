import json
import googlemaps

with open('config.json') as f:
    key = json.load(f)['google_maps_key']

maps_client = googlemaps.Client(key=key)


def generate_distance_matrix(addresses):
    distance_matrix_response = maps_client.distance_matrix(
        origins=addresses, destinations=addresses, mode='driving', language='json', units='metric')
    distance_matrix = [[row['elements'][j]['distance']['value'] for j in range(
        len(row['elements']))] for row in distance_matrix_response['rows']]
    duration_matrix = [[row['elements'][j]['duration']['value'] for j in range(
        len(row['elements']))] for row in distance_matrix_response['rows']]
    return distance_matrix, duration_matrix


if __name__ == '__main__':
    # Test
    locations = [
        [40.008992173966995, -105.28547778746469],
        (40.00320668011718, -105.24633899351937),
        (40.034495095289536, -105.27758136412484),
        (40.04343200726432, -105.2645350994764),
        (40.02897465276533, -105.26762500426156),
        (40.02792308924472, -105.2312327923475),
        (40.0176694953995, -105.23054614683969),
        (40.02581991357193, -105.25423541685922),
        (39.99527080014614, -105.23872375488281),
        (40.001891726772676, -105.27449145933969),
    ]
    distance, duration = generate_distance_matrix(locations)
    print(distance)
