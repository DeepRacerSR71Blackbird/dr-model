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
        racing_track = [[0.8642, -0.3719],
 [1.0899, -0.3722],
 [1.3061, -0.3838],
 [1.5055, -0.4067],
 [1.709, -0.4289],
 [1.9263, -0.4451],
 [2.142, -0.4599],
 [2.3721, -0.4768],
 [2.5956, -0.4991],
 [2.8185, -0.5273],
 [3.064, -0.5637],
 [3.2951, -0.5922],
 [3.5142, -0.6162],
 [3.745, -0.6412],
 [3.971, -0.6687],
 [4.2057, -0.6922],
 [4.4574, -0.7094],
 [4.6883, -0.7117],
 [4.9075, -0.7024],
 [5.1318, -0.6821],
 [5.3608, -0.6499],
 [5.5658, -0.6116],
 [5.775, -0.5583],
 [5.9489, -0.5021],
 [6.1181, -0.4353],
 [6.269, -0.3638],
 [6.3958, -0.2913],
 [6.508, -0.2143],
 [6.615, -0.1279],
 [6.7162, -0.0316],
 [6.8028, 0.0642],
 [6.8869, 0.1733],
 [6.9688, 0.3007],
 [7.0421, 0.4358],
 [7.1095, 0.5854],
 [7.1696, 0.7452],
 [7.2199, 0.9094],
 [7.264, 1.1018],
 [7.291, 1.2801],
 [7.3082, 1.4596],
 [7.3138, 1.632],
 [7.3071, 1.8173],
 [7.2884, 2.0117],
 [7.2567, 2.2117],
 [7.2167, 2.4056],
 [7.1707, 2.621],
 [7.1322, 2.8236],
 [7.1102, 3.0248],
 [7.1005, 3.2222],
 [7.1015, 3.4381],
 [7.1014, 3.6299],
 [7.0914, 3.8231],
 [7.0683, 4.0381],
 [7.0326, 4.2493],
 [6.9892, 4.444],
 [6.9346, 4.6357],
 [6.8687, 4.8189],
 [6.7964, 4.9825],
 [6.7153, 5.1327],
 [6.638, 5.2573],
 [6.5521, 5.3753],
 [6.4438, 5.4978],
 [6.3418, 5.596],
 [6.2319, 5.688],
 [6.117, 5.7673],
 [5.9938, 5.8374],
 [5.8536, 5.9026],
 [5.7001, 5.9596],
 [5.5339, 6.0151],
 [5.3546, 6.0708],
 [5.1487, 6.1251],
 [4.9433, 6.1716],
 [4.7281, 6.2129],
 [4.5103, 6.2568],
 [4.2864, 6.3029],
 [4.0677, 6.3425],
 [3.8544, 6.37],
 [3.6509, 6.3815],
 [3.4555, 6.3802],
 [3.2747, 6.3652],
 [3.1075, 6.3383],
 [2.9609, 6.3036],
 [2.8297, 6.2611],
 [2.7102, 6.2118],
 [2.6056, 6.1563],
 [2.5218, 6.1017],
 [2.4432, 6.0362],
 [2.3697, 5.9614],
 [2.3024, 5.8824],
 [2.2396, 5.795],
 [2.1849, 5.7034],
 [2.1449, 5.6188],
 [2.1115, 5.5284],
 [2.0853, 5.4351],
 [2.0681, 5.3416],
 [2.0583, 5.2499],
 [2.0576, 5.1622],
 [2.0655, 5.0665],
 [2.0838, 4.9786],
 [2.1183, 4.8825],
 [2.1566, 4.797],
 [2.1987, 4.7187],
 [2.2496, 4.6409],
 [2.3162, 4.567],
 [2.3935, 4.5003],
 [2.479, 4.4354],
 [2.5876, 4.3732],
 [2.7146, 4.3088],
 [2.8378, 4.2349],
 [2.9722, 4.1396],
 [3.0817, 4.0483],
 [3.1959, 3.9346],
 [3.2869, 3.8255],
 [3.3678, 3.7131],
 [3.4334, 3.6083],
 [3.4907, 3.5008],
 [3.5343, 3.4001],
 [3.5658, 3.3082],
 [3.5919, 3.2063],
 [3.6098, 3.1041],
 [3.6202, 3.0044],
 [3.6195, 2.9008],
 [3.6075, 2.7978],
 [3.5864, 2.7063],
 [3.5591, 2.6196],
 [3.5215, 2.5345],
 [3.4736, 2.4512],
 [3.4247, 2.3871],
 [3.3621, 2.3226],
 [3.2869, 2.2633],
 [3.191, 2.2058],
 [3.0873, 2.1563],
 [2.9684, 2.1097],
 [2.8325, 2.0636],
 [2.675, 2.0202],
 [2.5079, 1.9771],
 [2.3152, 1.9342],
 [2.1366, 1.9027],
 [1.9362, 1.877],
 [1.7177, 1.8562],
 [1.4964, 1.8403],
 [1.2764, 1.8245],
 [1.0525, 1.8121],
 [0.8333, 1.8105],
 [0.6122, 1.8192],
 [0.3946, 1.8391],
 [0.1832, 1.8697],
 [-0.0136, 1.9122],
 [-0.1993, 1.9653],
 [-0.3841, 2.0269],
 [-0.548, 2.0923],
 [-0.7078, 2.1678],
 [-0.8388, 2.2411],
 [-0.9547, 2.3152],
 [-1.0556, 2.39],
 [-1.1395, 2.4635],
 [-1.2078, 2.5343],
 [-1.2693, 2.6073],
 [-1.3221, 2.6892],
 [-1.3686, 2.781],
 [-1.4038, 2.8657],
 [-1.4346, 2.9582],
 [-1.4558, 3.0452],
 [-1.4654, 3.152],
 [-1.4619, 3.2673],
 [-1.4467, 3.3716],
 [-1.4207, 3.4835],
 [-1.3911, 3.5855],
 [-1.3546, 3.6885],
 [-1.3062, 3.7936],
 [-1.242, 3.9031],
 [-1.1637, 4.0278],
 [-1.0963, 4.1552],
 [-1.0353, 4.2936],
 [-0.9931, 4.4166],
 [-0.9582, 4.5496],
 [-0.9372, 4.6726],
 [-0.9277, 4.7812],
 [-0.9282, 4.8801],
 [-0.9368, 4.9721],
 [-0.9536, 5.0624],
 [-0.9801, 5.1492],
 [-1.0179, 5.2402],
 [-1.0658, 5.3182],
 [-1.1316, 5.4],
 [-1.2009, 5.4721],
 [-1.2825, 5.5429],
 [-1.3672, 5.604],
 [-1.471, 5.662],
 [-1.5793, 5.7088],
 [-1.6835, 5.7445],
 [-1.7942, 5.7707],
 [-1.9125, 5.7849],
 [-2.0433, 5.7875],
 [-2.1723, 5.7759],
 [-2.3071, 5.7514],
 [-2.4568, 5.7149],
 [-2.6148, 5.6637],
 [-2.7774, 5.5954],
 [-2.9595, 5.5058],
 [-3.1405, 5.4007],
 [-3.3199, 5.2833],
 [-3.504, 5.1536],
 [-3.6769, 5.0301],
 [-3.8585, 4.894],
 [-4.0353, 4.7497],
 [-4.2094, 4.5908],
 [-4.3677, 4.4335],
 [-4.5344, 4.265],
 [-4.6902, 4.1034],
 [-4.852, 3.9326],
 [-5.0222, 3.7509],
 [-5.1779, 3.5787],
 [-5.3325, 3.4036],
 [-5.4905, 3.2209],
 [-5.6419, 3.0407],
 [-5.7931, 2.8589],
 [-5.942, 2.6752],
 [-6.0837, 2.4967],
 [-6.2241, 2.3175],
 [-6.3683, 2.1307],
 [-6.5179, 1.9265],
 [-6.6418, 1.7425],
 [-6.7667, 1.544],
 [-6.8853, 1.3399],
 [-6.9899, 1.1401],
 [-7.0844, 0.9359],
 [-7.1689, 0.7218],
 [-7.2418, 0.5216],
 [-7.3122, 0.3051],
 [-7.3737, 0.0783],
 [-7.4211, -0.1256],
 [-7.4635, -0.3477],
 [-7.4953, -0.5676],
 [-7.5167, -0.798],
 [-7.5253, -1.0452],
 [-7.5195, -1.2691],
 [-7.5056, -1.4921],
 [-7.4778, -1.7296],
 [-7.4408, -1.9557],
 [-7.3955, -2.1739],
 [-7.3338, -2.4063],
 [-7.2661, -2.6159],
 [-7.1937, -2.8063],
 [-7.1099, -2.9911],
 [-7.0301, -3.1457],
 [-6.928, -3.3199],
 [-6.8152, -3.4884],
 [-6.6932, -3.656],
 [-6.5553, -3.8294],
 [-6.4199, -3.9933],
 [-6.27, -4.1768],
 [-6.1324, -4.3486],
 [-5.9746, -4.5412],
 [-5.8246, -4.7085],
 [-5.6636, -4.874],
 [-5.5122, -5.015],
 [-5.3407, -5.1563],
 [-5.1873, -5.27],
 [-5.0143, -5.385],
 [-4.8516, -5.4813],
 [-4.6749, -5.5747],
 [-4.4981, -5.6551],
 [-4.3255, -5.7207],
 [-4.154, -5.776],
 [-3.9674, -5.8256],
 [-3.8005, -5.8579],
 [-3.6011, -5.881],
 [-3.402, -5.8915],
 [-3.1859, -5.892],
 [-2.9842, -5.8909],
 [-2.7758, -5.9001],
 [-2.5931, -5.92],
 [-2.4059, -5.9522],
 [-2.2329, -5.983],
 [-2.0385, -6.0138],
 [-1.8286, -6.0487],
 [-1.6221, -6.0809],
 [-1.4027, -6.1133],
 [-1.1889, -6.1401],
 [-0.9477, -6.1665],
 [-0.711, -6.1948],
 [-0.4799, -6.2284],
 [-0.2438, -6.2617],
 [-0.003, -6.2974],
 [0.2266, -6.3351],
 [0.4615, -6.365],
 [0.6888, -6.385],
 [0.9109, -6.3926],
 [1.1257, -6.3904],
 [1.3247, -6.378],
 [1.5143, -6.3569],
 [1.6819, -6.3277],
 [1.8381, -6.2905],
 [1.9755, -6.248],
 [2.1021, -6.1992],
 [2.2292, -6.138],
 [2.3585, -6.0603],
 [2.4865, -5.9733],
 [2.6307, -5.8773],
 [2.7862, -5.7756],
 [2.9513, -5.676],
 [3.1132, -5.5848],
 [3.2713, -5.4886],
 [3.432, -5.378],
 [3.5704, -5.2699],
 [3.6909, -5.165],
 [3.8124, -5.046],
 [3.9117, -4.9343],
 [3.9903, -4.8337],
 [4.0611, -4.7282],
 [4.1204, -4.6285],
 [4.1645, -4.5398],
 [4.2067, -4.4387],
 [4.2371, -4.3425],
 [4.2589, -4.2467],
 [4.2709, -4.1495],
 [4.2718, -4.0455],
 [4.2646, -3.9475],
 [4.2495, -3.8532],
 [4.2276, -3.7602],
 [4.2001, -3.6723],
 [4.1583, -3.578],
 [4.1041, -3.4909],
 [4.0464, -3.4143],
 [3.987, -3.3435],
 [3.9175, -3.276],
 [3.844, -3.2168],
 [3.764, -3.1619],
 [3.6804, -3.1164],
 [3.5979, -3.0791],
 [3.5137, -3.0487],
 [3.4132, -3.0238],
 [3.3196, -3.0098],
 [3.2051, -3.0076],
 [3.0822, -3.0207],
 [2.957, -3.0483],
 [2.8084, -3.0936],
 [2.6568, -3.1429],
 [2.5145, -3.1852],
 [2.368, -3.2176],
 [2.2382, -3.2341],
 [2.1253, -3.2383],
 [2.0073, -3.2345],
 [1.8835, -3.2195],
 [1.7439, -3.202],
 [1.5852, -3.1835],
 [1.4082, -3.1646],
 [1.2407, -3.1513],
 [1.0443, -3.1473],
 [0.8513, -3.1541],
 [0.653, -3.1619],
 [0.4258, -3.1615],
 [0.2104, -3.1564],
 [-0.0118, -3.1513],
 [-0.2429, -3.1447],
 [-0.4932, -3.1414],
 [-0.7244, -3.1352],
 [-0.944, -3.1172],
 [-1.1678, -3.0902],
 [-1.388, -3.0599],
 [-1.6187, -3.0308],
 [-1.8433, -3.0083],
 [-2.0687, -2.9921],
 [-2.2869, -2.9833],
 [-2.5085, -2.983],
 [-2.729, -2.983],
 [-2.9504, -2.9675],
 [-3.1473, -2.9416],
 [-3.3368, -2.9031],
 [-3.5029, -2.8584],
 [-3.6736, -2.8023],
 [-3.8244, -2.7435],
 [-3.9636, -2.6775],
 [-4.0889, -2.6087],
 [-4.1958, -2.5397],
 [-4.2946, -2.4667],
 [-4.3798, -2.3918],
 [-4.4483, -2.3204],
 [-4.5138, -2.2441],
 [-4.567, -2.1683],
 [-4.6103, -2.098],
 [-4.6517, -2.0154],
 [-4.6827, -1.9315],
 [-4.7069, -1.8504],
 [-4.7237, -1.7623],
 [-4.7331, -1.6706],
 [-4.7384, -1.5843],
 [-4.7321, -1.4941],
 [-4.7158, -1.4],
 [-4.6852, -1.301],
 [-4.643, -1.2051],
 [-4.5908, -1.1046],
 [-4.5315, -1.0099],
 [-4.4618, -0.9192],
 [-4.3796, -0.8308],
 [-4.2849, -0.7474],
 [-4.1731, -0.664],
 [-4.0619, -0.5926],
 [-3.9428, -0.527],
 [-3.7985, -0.4607],
 [-3.6515, -0.4093],
 [-3.4908, -0.3686],
 [-3.3207, -0.3394],
 [-3.1369, -0.3121],
 [-2.9374, -0.2885],
 [-2.7427, -0.273],
 [-2.5145, -0.259],
 [-2.2953, -0.2468],
 [-2.0807, -0.2373],
 [-1.8603, -0.2376],
 [-1.6298, -0.2414],
 [-1.4012, -0.2476],
 [-1.1671, -0.2542],
 [-0.9336, -0.2674],
 [-0.7004, -0.2891],
 [-0.4712, -0.3151],
 [-0.2538, -0.3415],
 [-0.012, -0.3645],
 [0.2156, -0.3707],
 [0.4368, -0.3644],
 [0.6822, -0.347]]

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
        FASTEST_STEPS = 390 # 26s
        STEP_MULTIPLE = 2.0
        # DAVID steps
        # STANDARD_STEPS * freq (15/s) = TARGET_FINISH_TIME
        # if (steps % 10) == 0 and progress > (steps / STANDARD_STEPS) * 100 :
        #     steps_reward_new += 10.0
        # reward = float(steps_reward_new)
        # VINCENT steps
        expected_tot_steps = ((steps+0.1) / ((progress+0.1)/100))
        steps_reward_new = ((STANDARD_STEPS-expected_tot_steps) / (STANDARD_STEPS-FASTEST_STEPS)) * STEP_MULTIPLE
        steps_reward_new = max(1e-3,steps_reward_new)
        reward += steps_reward_new
        
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
