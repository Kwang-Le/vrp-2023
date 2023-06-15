"""Simple Pickup Delivery Problem (PDP)."""

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import math
from time import time


def create_data_model():
    """Stores the data for the problem."""
    data = {}
    data['distance_matrix'] = [[0, 739, 175, 563, 810, 956, 689, 53, 793, 1241, 393, 399, 431, 758, 480, 661, 387, 832, 157, 749, 309, 853, 1023, 384, 734, 771, 329, 210, 552, 1125], [739, 0, 914, 175, 537, 683, 416, 793, 123, 502, 645, 503, 1171, 484, 353, 571, 1127, 629, 582, 171, 964, 149, 283, 934, 282, 498, 602, 722, 186, 386], [175, 914, 0, 738, 683, 829, 809, 130, 968, 1416, 568, 410, 256, 879, 561, 836, 227, 705, 332, 924, 182, 1028, 1198, 257, 909, 756, 456, 337, 727, 1300], [563, 175, 738, 0, 583, 729, 462, 617, 229, 677, 599, 328, 995, 531, 253, 525, 951, 605, 406, 218, 789, 289, 459, 758, 236, 544, 556, 546, 15, 561], [810, 537, 683, 583, 0, 191, 292, 814, 452, 899, 1182, 411, 773, 362, 329, 1109, 729, 231, 739, 408, 566, 512, 681, 536, 820, 240, 1140, 1021, 599, 784], [956, 683, 829, 729, 191, 0, 484, 960, 643, 1091, 1328, 556, 663, 554, 475, 1254, 602, 123, 885, 599, 647, 703, 873, 572, 966, 431, 1285, 1167, 745, 975], [689, 416, 809, 462, 292, 484, 0, 693, 292, 607, 1061, 398, 1066, 69, 248, 987, 1022, 524, 618, 244, 859, 266, 528, 829, 698, 81, 1018, 899, 478, 491], [53, 793, 130, 617, 814, 960, 693, 0, 847, 1295, 447, 403, 377, 761, 484, 714, 357, 836, 210, 803, 313, 907, 1077, 388, 788, 774, 325, 206, 606, 1179], [793, 123, 968, 229, 452, 643, 292, 847, 0, 447, 768, 558, 1225, 361, 407, 695, 1181, 683, 636, 48, 1018, 60, 235, 988, 406, 374, 726, 776, 240, 331], [1241, 502, 1416, 677, 899, 1091, 607, 1295, 447, 0, 848, 1006, 1673, 537, 855, 580, 1629, 1131, 1084, 491, 1466, 387, 218, 1436, 507, 659, 1006, 1224, 688, 402], [393, 645, 568, 599, 1182, 1328, 1061, 447, 768, 848, 0, 771, 825, 
1130, 852, 267, 781, 1204, 443, 817, 681, 794, 629, 756, 362, 1143, 158, 375, 583, 1007], [399, 503, 410, 328, 411, 556, 398, 403, 558, 1006, 771, 0, 667, 468, 150, 698, 623, 432, 328, 514, 460, 618, 787, 430, 498, 371, 729, 610, 317, 890], [431, 1171, 256, 995, 773, 663, 1066, 377, 1225, 1673, 825, 667, 0, 1135, 817, 1092, 61, 541, 588, 1181, 206, 1285, 1455, 236, 1165, 1013, 666, 503, 984, 1557], [758, 484, 879, 531, 362, 554, 69, 761, 361, 537, 1130, 468, 1135, 0, 318, 1056, 1092, 594, 686, 312, 929, 335, 596, 899, 767, 122, 
1087, 968, 546, 421], [480, 353, 561, 253, 329, 475, 248, 484, 407, 855, 852, 150, 817, 318, 0, 779, 774, 351, 409, 363, 611, 467, 637, 581, 490, 290, 810, 691, 269, 739], [661, 
571, 836, 525, 1109, 1254, 987, 714, 695, 580, 267, 698, 1092, 1056, 779, 0, 1048, 1131, 503, 743, 886, 721, 459, 856, 288, 1069, 425, 643, 509, 933], [387, 1127, 227, 951, 729, 
602, 1022, 357, 1181, 1629, 781, 623, 61, 1092, 774, 1048, 0, 497, 545, 1137, 162, 1241, 1411, 192, 1122, 969, 683, 564, 940, 1513], [832, 629, 705, 605, 231, 123, 524, 836, 683, 1131, 1204, 432, 541, 594, 351, 1131, 497, 0, 761, 640, 523, 743, 913, 448, 842, 471, 1162, 1043, 621, 1015], [157, 582, 332, 406, 739, 885, 618, 210, 636, 1084, 443, 328, 588, 
686, 409, 503, 545, 761, 0, 592, 382, 696, 866, 352, 577, 700, 400, 281, 395, 968], [749, 171, 924, 218, 408, 599, 244, 803, 48, 491, 817, 514, 1181, 312, 363, 743, 1137, 640, 592, 0, 975, 103, 283, 945, 454, 326, 774, 732, 233, 375], [309, 964, 182, 789, 566, 647, 859, 313, 1018, 1466, 681, 460, 206, 929, 611, 886, 162, 523, 382, 975, 0, 1078, 1248, 75, 959, 807, 638, 520, 778, 1350], [853, 149, 1028, 289, 512, 703, 266, 907, 60, 387, 794, 618, 1285, 335, 467, 721, 1241, 743, 696, 103, 1078, 0, 261, 1048, 432, 348, 752, 836, 300, 271], [1023, 283, 1198, 459, 681, 873, 528, 1077, 235, 218, 629, 787, 1455, 596, 637, 459, 1411, 913, 866, 283, 1248, 261, 0, 1218, 289, 609, 788, 1005, 470, 473], [384, 934, 
257, 758, 536, 572, 829, 388, 988, 1436, 756, 430, 236, 899, 581, 856, 192, 448, 352, 945, 75, 1048, 1218, 0, 929, 776, 713, 595, 747, 1320], [734, 282, 909, 236, 820, 966, 698, 
788, 406, 507, 362, 498, 1165, 767, 490, 288, 1122, 842, 577, 454, 959, 432, 289, 929, 0, 780, 499, 716, 220, 644], [771, 498, 756, 544, 240, 431, 81, 774, 374, 659, 1143, 371, 1013, 122, 290, 1069, 969, 471, 700, 326, 807, 348, 609, 776, 780, 0, 1100, 981, 559, 543], [329, 602, 456, 556, 1140, 1285, 1018, 325, 726, 1006, 158, 729, 666, 1087, 810, 425, 683, 1162, 400, 774, 638, 752, 788, 713, 499, 1100, 0, 217, 540, 964], [210, 722, 337, 546, 1021, 1167, 899, 206, 776, 1224, 375, 610, 503, 968, 691, 643, 564, 1043, 281, 732, 520, 836, 1005, 595, 716, 981, 217, 0, 535, 1108], [552, 186, 727, 15, 599, 745, 478, 606, 240, 688, 583, 317, 984, 546, 269, 509, 940, 621, 395, 233, 778, 300, 470, 747, 220, 559, 
540, 535, 0, 572], [1125, 386, 1300, 561, 784, 975, 491, 1179, 331, 402, 1007, 890, 1557, 421, 739, 933, 1513, 1015, 968, 375, 1350, 271, 473, 1320, 644, 543, 964, 1108, 572, 0]]
    
    for rows in data['distance_matrix']:
        for number in rows:
            number = math.floor(number)
    data['pickups_deliveries'] = [
        [29, 28], [15, 8], [6, 7], [19, 5], [2, 14], [1, 18], [12, 16], [27, 17], [20, 23], [11, 9], [4, 21], [10, 13], [26, 22], [3, 25]]
    data['num_vehicles'] = 4
    data['depot'] = 0
    return data


def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f'Objective: {solution.ObjectiveValue()}')
    total_distance = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        while not routing.IsEnd(index):
            plan_output += ' {} -> '.format(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
            # print("route distance: ", route_distance)
        plan_output += '{}\n'.format(manager.IndexToNode(index))
        plan_output += 'Distance of the route: {}m\n'.format(route_distance)
        print(plan_output)
        total_distance += route_distance
    print('Total Distance of all routes: {}m'.format(total_distance))


def main():
    """Entry point of the program."""
    # Instantiate the data problem.
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)
    # print(data['distance_matrix'])

    # Define cost of each arc.
    def distance_callback(from_index, to_index):
        """Returns the manhattan distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        # print(data['distance_matrix'][0][1])
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance constraint.
    dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        999999999999,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name)
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Define Transportation Requests.
    for request in data['pickups_deliveries']:
        pickup_index = manager.NodeToIndex(request[0])
        delivery_index = manager.NodeToIndex(request[1])
        routing.AddPickupAndDelivery(pickup_index, delivery_index)
        routing.solver().Add(
            routing.VehicleVar(pickup_index) == routing.VehicleVar(
                delivery_index))
        routing.solver().Add(
            distance_dimension.CumulVar(pickup_index) <=
            distance_dimension.CumulVar(delivery_index))

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution)


if __name__ == '__main__':
    t0 = time()
    main()
    t1 = time()
    print('Google OR takes %f \n' %(t1-t0))
