import math
class Reward:

    def __init__(self, verbose=True):
        self.first_racingpoint_index = 0
        self.verbose = verbose

    def reward_function(self, params):
        # TODO based on closest_2_racing_points_index
        def get_target_point(racing_coords,car_coords):
            # Calculate all distances to racing points
            distances = []
            for i in range(len(racing_coords)):
                distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                            y1=racing_coords[i][1], y2=car_coords[1])
                distances.append(distance)

            # Get index of the closest racing point
            # TODO look forward - rcoef
            closest_index = distances.index(min(distances))
            return racing_coords[(closest_index+1)%len(racing_coords)]

        def dist_falktan(point1, point2):
            return ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2) ** 0.5
        
        def polar(x, y):
            """
            returns r, theta(degrees)
            """

            r = (x ** 2 + y ** 2) ** .5
            theta = math.degrees(math.atan2(y,x))
            return r, theta

        def angle_mod_360(angle):
            """
            Maps an angle to the interval -180, +180.

            Examples:
            angle_mod_360(362) == 2
            angle_mod_360(270) == -90

            :param angle: angle in degree
            :return: angle in degree. Between -180 and +180
            """

            n = math.floor(angle/360.0)

            angle_between_0_and_360 = angle - n*360.0

            if angle_between_0_and_360 <= 180.0:
                return angle_between_0_and_360
            else:
                return angle_between_0_and_360 - 360
        
        def get_waypoints_ordered_in_driving_direction(params):
            # waypoints are always provided in counter clock wise order
            if params['is_reversed']: # driving clock wise.
                return list(reversed(params['waypoints']))
            else: # driving counter clock wise.
                return params['waypoints']


        def up_sample(waypoints, factor):
            """
            Adds extra waypoints in between provided waypoints

            :param waypoints:
            :param factor: integer. E.g. 3 means that the resulting list has 3 times as many points.
            :return:
            """
            p = waypoints
            n = len(p)

            return [[i / factor * p[(j+1) % n][0] + (1 - i / factor) * p[j][0],
                    i / factor * p[(j+1) % n][1] + (1 - i / factor) * p[j][1]] for j in range(n) for i in range(factor)]

        def get_target_point_falktan(params,coef):
            waypoints = up_sample(get_waypoints_ordered_in_driving_direction(params), 20)

            car = [params['x'], params['y']]

            distances = [dist_falktan(p, car) for p in waypoints]
            min_dist = min(distances)
            i_closest = distances.index(min_dist)

            n = len(waypoints)

            waypoints_starting_with_closest = [waypoints[(i+i_closest) % n] for i in range(n)]

            r = params['track_width'] * coef

            is_inside = [dist_falktan(p, car) < r for p in waypoints_starting_with_closest]
            i_first_outside = is_inside.index(False)

            if i_first_outside < 0:  # this can only happen if we choose r as big as the entire track
                return waypoints[i_closest]

            return waypoints_starting_with_closest[i_first_outside]


        def get_target_steering_degree_falktan(params,coef):
            tx, ty = get_target_point_falktan(params,coef)
            car_x = params['x']
            car_y = params['y']
            dx = tx-car_x
            dy = ty-car_y
            heading = params['heading']

            _, target_angle = polar(dx, dy)

            steering_angle = target_angle - heading

            return angle_mod_360(steering_angle)


        def score_steer_to_point_ahead_falktan(params,coef):
            best_stearing_angle = get_target_steering_degree_falktan(params,coef)
            steering_angle = params['steering_angle']

            error = (steering_angle - best_stearing_angle) / 60.0  # 60 degree is already really bad

            score = 1.0 - abs(error)

            return max(score, 0.01)  # optimizer is rumored to struggle with negative numbers and numbers too close to zero

        # TODO based on generic
        def get_target_steering_degree(params,racing_coords):
            car_x = params['x']
            car_y = params['y']
            tx, ty = get_target_point(racing_coords,[car_x,car_y])
            dx = tx-car_x
            dy = ty-car_y
            heading = params['heading']

            _, target_angle = polar(dx, dy)

            best_steering_angle = angle_mod_360((target_angle - heading))

            if self.verbose == True:
                print("car_x={:.4f} car_y={:.4f}".format(car_x,car_y))
                print("target_x={:.4f} target_y={:.4f}".format(tx,ty))
                print("heading={:.4f} target_angle={:.4f} best_steering_angle={:.4f}".format(heading,target_angle,best_steering_angle))

            return best_steering_angle

        def score_steer_to_point_ahead(params,racing_coords):
            best_steering_angle = get_target_steering_degree(params,racing_coords)
            steering_angle = params['steering_angle']

            MAX_DIFF=30.0
            steer2optimal_diff=abs(steering_angle - best_steering_angle)
            # error = (steer2optimal_diff / MAX_DIFF) if steer2optimal_diff<MAX_DIFF else 1  # 30 degree is already really bad
            if steer2optimal_diff>15:
                score=0
            elif steer2optimal_diff>10:
                score=0.5
            else:
                score=1

            # score = (1.0 - error)**2
            # if steer2optimal_diff<5:
            #     score += 0.3

            if self.verbose == True:
                print("actual_steering={:.2f} score={:.4f}".format(steering_angle,score))
            heading2optimal_diff = best_steering_angle
            return heading2optimal_diff,steer2optimal_diff,max(score, 0.01)  # optimizer is rumored to struggle with negative numbers and numbers too close to zero


        ################## HELPER FUNCTIONS ###################

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5

        def closest_2_racing_points_index(racing_coords, car_coords):

            # Calculate all distances to racing points
            distances = []
            for i in range(len(racing_coords)):
                distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                         y1=racing_coords[i][1], y2=car_coords[1])
                distances.append(distance)

            # Get index of the closest racing point
            closest_index = distances.index(min(distances))

            # Get index of the second closest racing point
            distances_no_closest = distances.copy()
            distances_no_closest[closest_index] = 999
            second_closest_index = distances_no_closest.index(
                min(distances_no_closest))

            return [closest_index, second_closest_index]

        def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):

            # Calculate the distances between 2 closest racing points
            a = abs(dist_2_points(x1=closest_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=closest_coords[1],
                                  y2=second_closest_coords[1]))

            # Distances between car and closest and second closest racing point
            b = abs(dist_2_points(x1=car_coords[0],
                                  x2=closest_coords[0],
                                  y1=car_coords[1],
                                  y2=closest_coords[1]))
            c = abs(dist_2_points(x1=car_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=car_coords[1],
                                  y2=second_closest_coords[1]))

            # Calculate distance between car and racing line (goes through 2 closest racing points)
            # try-except in case a=0 (rare bug in DeepRacer)
            try:
                distance = abs(-(a**4) + 2*(a**2)*(b**2) + 2*(a**2)*(c**2) -
                               (b**4) + 2*(b**2)*(c**2) - (c**4))**0.5 / (2*a)
            except:
                distance = b

            return distance

        # Calculate which one of the closest racing points is the next one and which one the previous one
        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):

            # Virtually set the car more into the heading direction
            heading_vector = [math.cos(math.radians(
                heading)), math.sin(math.radians(heading))]
            new_car_coords = [car_coords[0]+heading_vector[0],
                              car_coords[1]+heading_vector[1]]

            # Calculate distance from new car coords to 2 closest racing points
            distance_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                        x2=closest_coords[0],
                                                        y1=new_car_coords[1],
                                                        y2=closest_coords[1])
            distance_second_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                               x2=second_closest_coords[0],
                                                               y1=new_car_coords[1],
                                                               y2=second_closest_coords[1])

            if distance_closest_coords_new <= distance_second_closest_coords_new:
                next_point_coords = closest_coords
                prev_point_coords = second_closest_coords
            else:
                next_point_coords = second_closest_coords
                prev_point_coords = closest_coords

            return [next_point_coords, prev_point_coords]

        def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):

            # Calculate the direction of the center line based on the closest waypoints
            next_point, prev_point = next_prev_racing_point(closest_coords,
                                                            second_closest_coords,
                                                            car_coords,
                                                            heading)

            # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
            track_direction = math.atan2(
                next_point[1] - prev_point[1], next_point[0] - prev_point[0])

            # Convert to degree
            track_direction = math.degrees(track_direction)

            # Calculate the difference between the track direction and the heading direction of the car
            direction_diff = abs(track_direction - heading)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff

            return direction_diff

        # Gives back indexes that lie between start and end index of a cyclical list
        # (start index is included, end index is not)
        def indexes_cyclical(start, end, array_len):

            if end < start:
                end += array_len

            return [index % array_len for index in range(start, end)]

        # Calculate how long car would take for entire lap, if it continued like it did until now
        def projected_time(first_index, closest_index, step_count, times_list):

            # Calculate how much time has passed since start
            current_actual_time = (step_count-1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum([times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (current_actual_time/current_expected_time) * total_expected_time
            except:
                projected_time = 9999

            return projected_time

        #################### RACING LINE ######################

        # Optimal racing line for the Spain track
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [[0.9067, -0.332],
 [1.109, -0.3312],
 [1.3285, -0.3407],
 [1.5418, -0.3601],
 [1.7467, -0.3889],
 [1.9613, -0.4175],
 [2.1843, -0.4366],
 [2.4058, -0.4536],
 [2.639, -0.4753],
 [2.8714, -0.5013],
 [3.0846, -0.5299],
 [3.2997, -0.5604],
 [3.5289, -0.5906],
 [3.7549, -0.6153],
 [3.9871, -0.6367],
 [4.2151, -0.6552],
 [4.4402, -0.6708],
 [4.6695, -0.6801],
 [4.8891, -0.6811],
 [5.1144, -0.6732],
 [5.3365, -0.6535],
 [5.5379, -0.6207],
 [5.7216, -0.5784],
 [5.8924, -0.525],
 [6.0496, -0.4631],
 [6.1994, -0.3935],
 [6.3375, -0.3165],
 [6.4605, -0.2334],
 [6.5665, -0.1476],
 [6.6694, -0.0533],
 [6.784, 0.071],
 [6.883, 0.2004],
 [6.9727, 0.3355],
 [7.0565, 0.4872],
 [7.1318, 0.6482],
 [7.197, 0.8096],
 [7.2523, 0.9747],
 [7.2963, 1.1361],
 [7.3291, 1.2966],
 [7.352, 1.455],
 [7.3655, 1.6157],
 [7.3675, 1.7934],
 [7.3589, 1.9559],
 [7.3364, 2.1353],
 [7.2953, 2.33],
 [7.2523, 2.5168],
 [7.2141, 2.7066],
 [7.1846, 2.915],
 [7.1682, 3.1117],
 [7.1639, 3.303],
 [7.1559, 3.5037],
 [7.1343, 3.7096],
 [7.1035, 3.9035],
 [7.0607, 4.1161],
 [7.0033, 4.3396],
 [6.9324, 4.5603],
 [6.8528, 4.7678],
 [6.76, 4.9745],
 [6.6671, 5.1563],
 [6.5739, 5.3173],
 [6.4831, 5.4543],
 [6.382, 5.5879],
 [6.2897, 5.6943],
 [6.2017, 5.7832],
 [6.1135, 5.8579],
 [6.0212, 5.9256],
 [5.9244, 5.9845],
 [5.8251, 6.0325],
 [5.7203, 6.0683],
 [5.5821, 6.0976],
 [5.4418, 6.1112],
 [5.2963, 6.1175],
 [5.1281, 6.1185],
 [4.9582, 6.1144],
 [4.8014, 6.1104],
 [4.6625, 6.1142],
 [4.5303, 6.1271],
 [4.3997, 6.1436],
 [4.2613, 6.1684],
 [4.1142, 6.2048],
 [3.9611, 6.2546],
 [3.8099, 6.2982],
 [3.6345, 6.3341],
 [3.4666, 6.3565],
 [3.3021, 6.3679],
 [3.1575, 6.3671],
 [3.0263, 6.3555],
 [2.9024, 6.3349],
 [2.7888, 6.3039],
 [2.6984, 6.2719],
 [2.6064, 6.2293],
 [2.5232, 6.1765],
 [2.4453, 6.1124],
 [2.3754, 6.0437],
 [2.31, 5.9717],
 [2.2543, 5.8963],
 [2.2061, 5.8182],
 [2.1688, 5.7323],
 [2.1399, 5.6319],
 [2.1229, 5.5353],
 [2.1146, 5.4307],
 [2.1146, 5.33],
 [2.1261, 5.2213],
 [2.1445, 5.1238],
 [2.1752, 5.0175],
 [2.2148, 4.9138],
 [2.2662, 4.8099],
 [2.3164, 4.7255],
 [2.3801, 4.6387],
 [2.4581, 4.5539],
 [2.5567, 4.4698],
 [2.6742, 4.3911],
 [2.7998, 4.3193],
 [2.9415, 4.2343],
 [3.066, 4.1442],
 [3.1775, 4.0509],
 [3.2638, 3.9663],
 [3.3426, 3.8797],
 [3.4115, 3.7911],
 [3.4705, 3.7023],
 [3.523, 3.6059],
 [3.5571, 3.5224],
 [3.585, 3.4315],
 [3.6079, 3.3301],
 [3.6216, 3.2435],
 [3.6245, 3.1518],
 [3.6134, 3.0539],
 [3.5929, 2.9603],
 [3.5592, 2.8555],
 [3.5196, 2.761],
 [3.4724, 2.6721],
 [3.4086, 2.5727],
 [3.3337, 2.4812],
 [3.2398, 2.3885],
 [3.1282, 2.2975],
 [3.0016, 2.2125],
 [2.8636, 2.1307],
 [2.7174, 2.056],
 [2.5675, 1.9934],
 [2.3967, 1.935],
 [2.2376, 1.8924],
 [2.0614, 1.8559],
 [1.8767, 1.8297],
 [1.6596, 1.813],
 [1.4519, 1.8079],
 [1.2405, 1.8117],
 [1.0125, 1.8242],
 [0.782, 1.8439],
 [0.5543, 1.8694],
 [0.3319, 1.902],
 [0.1133, 1.9453],
 [-0.0853, 1.9966],
 [-0.2982, 2.0659],
 [-0.4585, 2.1303],
 [-0.6151, 2.206],
 [-0.7489, 2.2847],
 [-0.8664, 2.3634],
 [-0.9616, 2.4373],
 [-1.0445, 2.5094],
 [-1.1152, 2.5798],
 [-1.1726, 2.6491],
 [-1.2259, 2.7249],
 [-1.2735, 2.8051],
 [-1.3102, 2.8853],
 [-1.3411, 2.97],
 [-1.3614, 3.0637],
 [-1.3726, 3.1523],
 [-1.3779, 3.2387],
 [-1.3735, 3.3288],
 [-1.3626, 3.414],
 [-1.3405, 3.505],
 [-1.3076, 3.591],
 [-1.2598, 3.6821],
 [-1.1992, 3.7743],
 [-1.1308, 3.8773],
 [-1.0641, 3.9953],
 [-1.0059, 4.1281],
 [-0.9606, 4.2645],
 [-0.9321, 4.3808],
 [-0.9098, 4.5182],
 [-0.8991, 4.6391],
 [-0.8983, 4.7609],
 [-0.9072, 4.8802],
 [-0.9242, 4.9876],
 [-0.951, 5.0907],
 [-0.983, 5.185],
 [-1.02, 5.2688],
 [-1.0652, 5.351],
 [-1.1186, 5.4262],
 [-1.1901, 5.4981],
 [-1.2738, 5.5624],
 [-1.3647, 5.6163],
 [-1.4669, 5.6638],
 [-1.5735, 5.6969],
 [-1.7008, 5.7222],
 [-1.8352, 5.7321],
 [-1.9738, 5.7278],
 [-2.1087, 5.7104],
 [-2.2467, 5.6931],
 [-2.3996, 5.6886],
 [-2.5757, 5.6859],
 [-2.7407, 5.6688],
 [-2.9032, 5.6411],
 [-3.0564, 5.6036],
 [-3.2007, 5.5559],
 [-3.341, 5.4963],
 [-3.4777, 5.4235],
 [-3.6211, 5.3328],
 [-3.7348, 5.2444],
 [-3.8727, 5.1279],
 [-3.9979, 5.0113],
 [-4.1435, 4.8667],
 [-4.2825, 4.7201],
 [-4.4292, 4.554],
 [-4.5721, 4.3819],
 [-4.7223, 4.1915],
 [-4.8627, 4.0051],
 [-5.0011, 3.8217],
 [-5.1399, 3.6414],
 [-5.2828, 3.4653],
 [-5.4386, 3.2891],
 [-5.5959, 3.1151],
 [-5.7639, 2.9276],
 [-5.9101, 2.7626],
 [-6.0776, 2.5724],
 [-6.2304, 2.3943],
 [-6.3703, 2.2198],
 [-6.5089, 2.0291],
 [-6.6275, 1.8482],
 [-6.7418, 1.6575],
 [-6.8421, 1.4709],
 [-6.9398, 1.2685],
 [-7.0257, 1.0642],
 [-7.1017, 0.8589],
 [-7.1728, 0.6378],
 [-7.2296, 0.4213],
 [-7.2811, 0.1987],
 [-7.3288, -0.0293],
 [-7.3696, -0.2529],
 [-7.4033, -0.4806],
 [-7.4322, -0.7173],
 [-7.4574, -0.9582],
 [-7.4773, -1.1977],
 [-7.4854, -1.4225],
 [-7.4813, -1.6478],
 [-7.466, -1.8737],
 [-7.4394, -2.0786],
 [-7.4046, -2.2703],
 [-7.3599, -2.4657],
 [-7.3116, -2.6298],
 [-7.2536, -2.7862],
 [-7.1903, -2.9228],
 [-7.1115, -3.0619],
 [-7.0222, -3.1959],
 [-6.9165, -3.3309],
 [-6.7904, -3.4681],
 [-6.6524, -3.6033],
 [-6.5124, -3.7466],
 [-6.3927, -3.8858],
 [-6.2827, -4.0293],
 [-6.1748, -4.191],
 [-6.0669, -4.3551],
 [-5.9398, -4.5279],
 [-5.8191, -4.6792],
 [-5.6796, -4.834],
 [-5.5348, -4.9762],
 [-5.3965, -5.097],
 [-5.2461, -5.2134],
 [-5.0959, -5.3163],
 [-4.9341, -5.4136],
 [-4.7626, -5.503],
 [-4.5815, -5.5843],
 [-4.3827, -5.6596],
 [-4.1994, -5.7206],
 [-4.0196, -5.7711],
 [-3.8273, -5.8139],
 [-3.6279, -5.8475],
 [-3.4198, -5.8797],
 [-3.2088, -5.9129],
 [-2.9805, -5.9477],
 [-2.7533, -5.9806],
 [-2.5327, -6.0102],
 [-2.2914, -6.0447],
 [-2.0613, -6.074],
 [-1.8376, -6.0948],
 [-1.6004, -6.113],
 [-1.3729, -6.127],
 [-1.1429, -6.1427],
 [-0.8995, -6.1628],
 [-0.686, -6.1834],
 [-0.4604, -6.2124],
 [-0.2304, -6.2472],
 [0.0067, -6.2852],
 [0.2407, -6.3212],
 [0.4775, -6.3556],
 [0.698, -6.3835],
 [0.9266, -6.3995],
 [1.138, -6.4025],
 [1.3317, -6.3938],
 [1.5195, -6.3733],
 [1.6737, -6.3451],
 [1.8208, -6.3089],
 [1.9537, -6.2654],
 [2.0834, -6.2144],
 [2.2061, -6.1523],
 [2.339, -6.0688],
 [2.4732, -5.9764],
 [2.6306, -5.8688],
 [2.7903, -5.7641],
 [2.9594, -5.6612],
 [3.1337, -5.5617],
 [3.3072, -5.461],
 [3.4819, -5.3503],
 [3.6321, -5.2422],
 [3.7708, -5.127],
 [3.8916, -5.0113],
 [3.9897, -4.9031],
 [4.0724, -4.7994],
 [4.1394, -4.7045],
 [4.1962, -4.6087],
 [4.2417, -4.515],
 [4.2803, -4.4176],
 [4.308, -4.3186],
 [4.3292, -4.2168],
 [4.3423, -4.1191],
 [4.3492, -4.0203],
 [4.348, -3.9265],
 [4.3379, -3.827],
 [4.3225, -3.7393],
 [4.2967, -3.646],
 [4.2639, -3.5571],
 [4.2243, -3.4729],
 [4.1762, -3.3912],
 [4.1244, -3.3177],
 [4.0538, -3.241],
 [3.9826, -3.1817],
 [3.8918, -3.121],
 [3.8015, -3.0716],
 [3.7031, -3.0273],
 [3.6055, -2.9937],
 [3.5127, -2.9677],
 [3.4203, -2.9504],
 [3.3265, -2.9421],
 [3.2319, -2.9396],
 [3.1273, -2.951],
 [3.0168, -2.9711],
 [2.8878, -2.9937],
 [2.7449, -3.0114],
 [2.5927, -3.0301],
 [2.4303, -3.048],
 [2.2377, -3.0635],
 [2.0486, -3.0781],
 [1.8461, -3.0904],
 [1.6309, -3.1128],
 [1.404, -3.1407],
 [1.1958, -3.1593],
 [0.985, -3.1722],
 [0.7406, -3.1789],
 [0.5132, -3.1783],
 [0.2745, -3.1716],
 [0.0558, -3.1629],
 [-0.1833, -3.1513],
 [-0.4064, -3.1443],
 [-0.6372, -3.1391],
 [-0.8652, -3.1322],
 [-1.107, -3.1239],
 [-1.3447, -3.1165],
 [-1.5921, -3.1065],
 [-1.8183, -3.0915],
 [-2.0533, -3.0726],
 [-2.2946, -3.0526],
 [-2.529, -3.0333],
 [-2.7602, -3.0074],
 [-2.9931, -2.9735],
 [-3.2184, -2.932],
 [-3.4337, -2.88],
 [-3.6227, -2.8215],
 [-3.8053, -2.7535],
 [-3.9686, -2.6809],
 [-4.1128, -2.6045],
 [-4.2367, -2.5292],
 [-4.3374, -2.4578],
 [-4.4365, -2.379],
 [-4.5199, -2.3041],
 [-4.585, -2.2334],
 [-4.648, -2.1589],
 [-4.7005, -2.0856],
 [-4.7474, -1.9978],
 [-4.7856, -1.9047],
 [-4.8153, -1.8092],
 [-4.8393, -1.7176],
 [-4.8548, -1.6282],
 [-4.8651, -1.5343],
 [-4.8669, -1.4465],
 [-4.8578, -1.3586],
 [-4.8443, -1.2699],
 [-4.8218, -1.1831],
 [-4.7926, -1.1049],
 [-4.7551, -1.0231],
 [-4.7102, -0.9466],
 [-4.661, -0.8758],
 [-4.6018, -0.8087],
 [-4.5229, -0.7405],
 [-4.4301, -0.6809],
 [-4.3138, -0.6266],
 [-4.1862, -0.5838],
 [-4.0506, -0.5364],
 [-3.8789, -0.4796],
 [-3.719, -0.4382],
 [-3.5414, -0.4004],
 [-3.3453, -0.3666],
 [-3.143, -0.3368],
 [-2.9272, -0.3073],
 [-2.7135, -0.2824],
 [-2.4964, -0.2553],
 [-2.2683, -0.2269],
 [-2.053, -0.2084],
 [-1.8427, -0.2008],
 [-1.6481, -0.2041],
 [-1.4472, -0.2164],
 [-1.2577, -0.2259],
 [-1.0776, -0.2246],
 [-0.8841, -0.2294],
 [-0.6847, -0.2432],
 [-0.5008, -0.263],
 [-0.2877, -0.2954],
 [-0.0839, -0.3329],
 [0.1423, -0.3781],
 [0.3577, -0.4216],
 [0.5863, -0.4652],
 [0.8203, -0.5104]]

        ################## INPUT PARAMETERS ###################

        # Read all input parameters
        all_wheels_on_track = params['all_wheels_on_track']
        x = params['x']
        y = params['y']
        distance_from_center = params['distance_from_center']
        is_left_of_center = params['is_left_of_center']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        steering_angle = params['steering_angle']
        track_width = params['track_width']
        waypoints = params['waypoints']
        closest_waypoints = params['closest_waypoints']
        is_offtrack = params['is_offtrack']

        ############### OPTIMAL X,Y,SPEED,TIME ################

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        # Save first racingpoint of episode for later
        if self.verbose == True:
            self.first_racingpoint_index = 0 # this is just for testing purposes
        if steps == 1:
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1

        STEER_MULTIPLE=2
        heading2optimal_diff,steer2optimal_diff,steer_reward = score_steer_to_point_ahead(params,racing_track)
        heading2optimal_diff=abs(heading2optimal_diff)
        if heading2optimal_diff>40:
            steer_reward = 1e-3
        elif heading2optimal_diff>10:
            steer_reward = (1-(heading2optimal_diff/90))**2
        else:
            # steer_reward = 1
            steer_reward = (1-(heading2optimal_diff/90))**2
        steer_reward*=STEER_MULTIPLE
        # reward += (steer_reward)

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 2
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist/(track_width*0.5))) * DISTANCE_MULTIPLE
        reward += distance_reward
        '''
        ABS_STEERING_THRESHOLD = 15 
        abs_steering = abs(steering_angle)
        # Penalize reward if the car is steering too much
        if abs_steering > ABS_STEERING_THRESHOLD:
            reward *= (ABS_STEERING_THRESHOLD/abs_steering)
        ## Reward if speed is close to optimal speed ##
        '''
        # SPEED_DIFF_NO_REWARD = 1
        # SPEED_MULTIPLE = 1
        # speed_diff = abs(optimals[2]-speed)
        # if speed_diff <= SPEED_DIFF_NO_REWARD:
        #     # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
        #     # so, we do not punish small deviations from optimal speed
        #     speed_reward = (1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2)**2
        # else:
        #     speed_reward = 0
        # speed_reward = speed_reward * SPEED_MULTIPLE
        # reward += speed_reward
        
        # Reward if less steps
        # REWARD_PER_STEP_FOR_FASTEST_TIME = 1
        # STANDARD_TIME = 32
        # FASTEST_TIME = 24
        # times_list = [row[3] for row in racing_track]
        # projected_time = projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
        # try:
        #     steps_prediction = projected_time * 15 + 1
        #     reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME*(FASTEST_TIME) / (STANDARD_TIME-FASTEST_TIME))
        #                                         *(steps_prediction-(STANDARD_TIME*15+1)))
        #     steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
        # except:
        #     steps_reward = 0
        # reward += steps_reward
        
        # Zero reward if obviously wrong direction (e.g. spin)
        # direction_diff = racing_direction_diff(
        #     optimals[0:2], optimals_second[0:2], [x, y], heading)
        # if direction_diff > 30:
        #     reward = 1e-3
        
        # Zero reward of obviously too slow
        # speed_diff_zero = optimals[2]-speed
        # if speed_diff_zero > 0.5:
        #     reward = 1e-3

        ## Incentive for finishing the lap in less steps ##
        REWARD_FOR_FASTEST_TIME = 600 # should be adapted to track length and other rewards
        STANDARD_TIME = 35  # seconds (time that is easily done by model)
        FASTEST_TIME = 25  # seconds (best time of 1st place on the track)
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                      (15*(STANDARD_TIME-FASTEST_TIME)))*(steps-STANDARD_TIME*15))
        else:
            finish_reward = 0
        # reward += finish_reward
        
        # coef=1.2
        # reward=float(score_steer_to_point_ahead_falktan(params,coef))
        # print("dist_reward={:.3f} steer_reward={:.3f}".format(distance_reward,steer_reward))
        # print("speed_reward={:.3f} tot_reward={:.3f}".format(speed_reward,reward))steps = params['steps']
        # 
        steps = params['steps']
        progress = params['progress']
        STANDARD_STEPS = 450 # 30s
        STEP_MULTIPLE = 1.0
        # DAVID steps
        # STANDARD_STEPS * freq (15/s) = TARGET_FINISH_TIME
        # if (steps % 10) == 0 and progress > (steps / STANDARD_STEPS) * 100 :
        #     steps_reward_new += 10.0
        # reward = float(steps_reward_new)
        # VINCENT steps
        expected_tot_steps = ((steps+0.1) / ((progress+0.1)/100))
        steps_reward_new = ((STANDARD_STEPS / expected_tot_steps)**2) * STEP_MULTIPLE
        # reward = steps_reward_new
        
        ## Zero reward if off track ##
        if is_offtrack:
            reward = 1e-3
        # if all_wheels_on_track == False:
        #     reward = 1e-3
        print("=== Distance reward: %f ===" % (distance_reward))
        # print("=== Steps reward: %f ===" % steps_reward)
        print("=== New Steps reward: %f ===" % steps_reward_new)
        print("=== Finish reward: %f ===" % finish_reward)
        print("=== Total reward: %f ===" % reward)
        # print("=== Speed reward: %f ===" % speed_reward)
        print("=== Steer reward: %f ===" % steer_reward)
        print("Distance to racing line: %f" % dist)
        # print("Optimal speed: %f" % optimals[2])
        # print("Speed difference: %f" % speed_diff)
        # print("Direction difference: %f" % direction_diff)
        # print("Predicted time: %f" % projected_time)
        print("Closest index: %i" % closest_index)
        ####################### VERBOSE #######################
        '''
        if self.verbose == True:
            print("all_wheels_on_track=" + str(all_wheels_on_track) + " x=" + str(x) + " y=" + str(y) + " distance_from_center=" + str(distance_from_center)
              + " heading=" + str(heading) + " progress=" + str(progress) + " steps=" + str(steps) + " speed=" + str(speed) + " steering_angle=" + str(steering_angle)
              + " is_offtrack=" + str(is_offtrack) + " distance_reward=" + str(distance_reward) + " closest_index=" + str(closest_index) + " distance_racingline=" + str(dist)
              + " optimal_speed=" + str(optimals[2]) + " speed_diff=" + str(speed_diff) + " speed_reward=" + str(speed_reward) + " direction_diff=" + str(direction_diff)
              + " steps_reward=" + str(steps_reward) + " total_reward=" + str(reward))
        '''
        #################### RETURN REWARD ####################

        # Always return a float value
        return float(reward)


reward_object = Reward() # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)
