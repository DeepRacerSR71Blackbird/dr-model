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
        racing_track = [[-5.7892, 2.9656],
 [-5.789, 2.9657],
 [-5.7963, 2.9606],
 [-5.8125, 2.9447],
 [-5.8354, 2.9155],
 [-5.8689, 2.8718],
 [-5.9035, 2.8258],
 [-5.9566, 2.7503],
 [-6.0106, 2.6715],
 [-6.0693, 2.5856],
 [-6.1331, 2.4945],
 [-6.2107, 2.3822],
 [-6.2966, 2.2551],
 [-6.3885, 2.1122],
 [-6.4808, 1.9645],
 [-6.5748, 1.8138],
 [-6.6868, 1.6295],
 [-6.7694, 1.4799],
 [-6.8474, 1.3124],
 [-6.9119, 1.152],
 [-6.9641, 1.006],
 [-7.021, 0.8482],
 [-7.0878, 0.6778],
 [-7.1549, 0.5025],
 [-7.2242, 0.3155],
 [-7.2738, 0.162],
 [-7.321, -0.0139],
 [-7.3494, -0.1522],
 [-7.3748, -0.3147],
 [-7.3987, -0.4717],
 [-7.4331, -0.6535],
 [-7.4634, -0.8521],
 [-7.4803, -1.0038],
 [-7.492, -1.1531],
 [-7.4904, -1.337],
 [-7.4711, -1.5257],
 [-7.4389, -1.7055],
 [-7.3958, -1.8784],
 [-7.3451, -2.0438],
 [-7.2773, -2.2198],
 [-7.1925, -2.4096],
 [-7.1277, -2.5634],
 [-7.0647, -2.7501],
 [-7.0077, -2.9177],
 [-6.9315, -3.0893],
 [-6.8564, -3.2309],
 [-6.7641, -3.3833],
 [-6.6916, -3.496],
 [-6.5919, -3.6505],
 [-6.4929, -3.8079],
 [-6.3841, -3.9742],
 [-6.2867, -4.1016],
 [-6.1787, -4.2279],
 [-6.0635, -4.3469],
 [-5.9568, -4.4477],
 [-5.8449, -4.5509],
 [-5.7329, -4.6538],
 [-5.5934, -4.7701],
 [-5.4517, -4.8802],
 [-5.2979, -4.9921],
 [-5.1574, -5.0876],
 [-5.0149, -5.1751],
 [-4.8712, -5.2595],
 [-4.7198, -5.3356],
 [-4.5797, -5.3975],
 [-4.4403, -5.4545],
 [-4.2762, -5.5214],
 [-4.1163, -5.5894],
 [-3.9743, -5.6472],
 [-3.8392, -5.6952],
 [-3.7011, -5.74],
 [-3.5709, -5.7771],
 [-3.4406, -5.8079],
 [-3.3153, -5.8356],
 [-3.1782, -5.8638],
 [-3.0358, -5.8973],
 [-2.8759, -5.9352],
 [-2.7057, -5.9683],
 [-2.5735, -5.9925],
 [-2.3759, -6.0232],
 [-2.1784, -6.049],
 [-1.9927, -6.0773],
 [-1.8233, -6.1078],
 [-1.5925, -6.1556],
 [-1.3899, -6.1975],
 [-1.1987, -6.2329],
 [-1.0028, -6.2693],
 [-0.831, -6.2937],
 [-0.6427, -6.3092],
 [-0.4618, -6.3207],
 [-0.2936, -6.3326],
 [-0.1211, -6.3428],
 [0.0885, -6.3487],
 [0.2592, -6.3537],
 [0.4509, -6.3553],
 [0.6523, -6.3455],
 [0.8088, -6.33],
 [0.9585, -6.3059],
 [1.1038, -6.2746],
 [1.2707, -6.2347],
 [1.4066, -6.2037],
 [1.5576, -6.1706],
 [1.7228, -6.131],
 [1.8937, -6.0763],
 [2.0349, -6.0207],
 [2.1418, -5.9719],
 [2.2914, -5.897],
 [2.4352, -5.8201],
 [2.5845, -5.7392],
 [2.7574, -5.6402],
 [2.8936, -5.5548],
 [3.0485, -5.4495],
 [3.201, -5.3315],
 [3.3448, -5.2082],
 [3.4791, -5.0828],
 [3.6045, -4.9515],
 [3.7295, -4.8041],
 [3.8226, -4.6788],
 [3.9049, -4.5491],
 [3.9642, -4.4429],
 [4.0139, -4.3358],
 [4.0516, -4.2359],
 [4.0782, -4.1408],
 [4.0993, -4.0395],
 [4.108, -3.9498],
 [4.1076, -3.8607],
 [4.0967, -3.7669],
 [4.0777, -3.6848],
 [4.0435, -3.5919],
 [3.9987, -3.507],
 [3.9521, -3.4383],
 [3.8989, -3.3748],
 [3.8284, -3.3111],
 [3.7544, -3.258],
 [3.6821, -3.2178],
 [3.5861, -3.1771],
 [3.5016, -3.1518],
 [3.4098, -3.1364],
 [3.3145, -3.1325],
 [3.1947, -3.1356],
 [3.0733, -3.1363],
 [2.9381, -3.139],
 [2.7842, -3.1398],
 [2.6564, -3.1403],
 [2.545, -3.1383],
 [2.375, -3.1352],
 [2.1953, -3.1351],
 [2.05, -3.137],
 [1.8398, -3.1412],
 [1.6079, -3.1443],
 [1.395, -3.1527],
 [1.1987, -3.1679],
 [1.0031, -3.181],
 [0.787, -3.1967],
 [0.5599, -3.206],
 [0.3818, -3.2051],
 [0.2167, -3.1966],
 [0.0135, -3.1854],
 [-0.1941, -3.1728],
 [-0.4032, -3.1647],
 [-0.6066, -3.1632],
 [-0.8151, -3.1619],
 [-1.03, -3.1572],
 [-1.2644, -3.1407],
 [-1.4643, -3.1194],
 [-1.6827, -3.0942],
 [-1.8895, -3.075],
 [-2.1092, -3.0597],
 [-2.3017, -3.0484],
 [-2.5027, -3.0279],
 [-2.6856, -3.0048],
 [-2.8634, -2.9746],
 [-3.0184, -2.9422],
 [-3.1684, -2.9073],
 [-3.3044, -2.8698],
 [-3.4607, -2.8109],
 [-3.596, -2.7477],
 [-3.7177, -2.6787],
 [-3.8349, -2.6012],
 [-3.962, -2.5026],
 [-4.0553, -2.42],
 [-4.1562, -2.3196],
 [-4.2524, -2.2072],
 [-4.3382, -2.0884],
 [-4.4133, -1.9712],
 [-4.4779, -1.8543],
 [-4.5229, -1.7493],
 [-4.5545, -1.6546],
 [-4.5832, -1.5583],
 [-4.6032, -1.462],
 [-4.6163, -1.3692],
 [-4.6203, -1.2772],
 [-4.6132, -1.1635],
 [-4.6015, -1.0725],
 [-4.583, -0.9935],
 [-4.5512, -0.8949],
 [-4.519, -0.8209],
 [-4.4763, -0.7422],
 [-4.4288, -0.667],
 [-4.3664, -0.5905],
 [-4.2929, -0.5169],
 [-4.2301, -0.4651],
 [-4.1496, -0.4119],
 [-4.0665, -0.3666],
 [-3.9912, -0.3367],
 [-3.9068, -0.3103],
 [-3.8108, -0.288],
 [-3.6872, -0.2738],
 [-3.5721, -0.2717],
 [-3.436, -0.2811],
 [-3.317, -0.2939],
 [-3.1853, -0.3028],
 [-3.042, -0.3035],
 [-2.8771, -0.2955],
 [-2.6847, -0.2737],
 [-2.5521, -0.2509],
 [-2.3728, -0.2176],
 [-2.1923, -0.1923],
 [-2.023, -0.1731],
 [-1.8616, -0.1628],
 [-1.6974, -0.1652],
 [-1.5231, -0.1766],
 [-1.3655, -0.192],
 [-1.2131, -0.2093],
 [-1.0308, -0.2273],
 [-0.8783, -0.2401],
 [-0.7082, -0.2576],
 [-0.5334, -0.2767],
 [-0.3625, -0.2907],
 [-0.1944, -0.2974],
 [0.0094, -0.2992],
 [0.2007, -0.2986],
 [0.4052, -0.3062],
 [0.5952, -0.32],
 [0.7345, -0.3384],
 [0.923, -0.3705],
 [1.0934, -0.4008],
 [1.288, -0.4297],
 [1.4346, -0.4471],
 [1.65, -0.4672],
 [1.8357, -0.4802],
 [2.0455, -0.4966],
 [2.1997, -0.5107],
 [2.4001, -0.5378],
 [2.5972, -0.5661],
 [2.8199, -0.5939],
 [2.9825, -0.6107],
 [3.1798, -0.6306],
 [3.3717, -0.6473],
 [3.55, -0.6619],
 [3.7415, -0.6744],
 [3.9405, -0.6802],
 [4.1569, -0.6813],
 [4.3441, -0.6809],
 [4.5674, -0.6747],
 [4.734, -0.6672],
 [4.9466, -0.6462],
 [5.1093, -0.6222],
 [5.2814, -0.5863],
 [5.4118, -0.5532],
 [5.5615, -0.5082],
 [5.6875, -0.4628],
 [5.8031, -0.4174],
 [5.9247, -0.3612],
 [6.0491, -0.3016],
 [6.1708, -0.2325],
 [6.2965, -0.1429],
 [6.4052, -0.0504],
 [6.5272, 0.0731],
 [6.6429, 0.2056],
 [6.7446, 0.3362],
 [6.8614, 0.5038],
 [6.948, 0.6483],
 [7.0335, 0.8155],
 [7.0919, 0.9572],
 [7.1423, 1.1054],
 [7.1732, 1.2217],
 [7.2109, 1.3786],
 [7.2422, 1.5288],
 [7.2614, 1.6772],
 [7.2725, 1.8106],
 [7.2772, 1.9236],
 [7.2772, 2.0579],
 [7.2715, 2.183],
 [7.2636, 2.3174],
 [7.2502, 2.4729],
 [7.2303, 2.6503],
 [7.2176, 2.7832],
 [7.2026, 2.976],
 [7.1937, 3.1578],
 [7.1811, 3.3348],
 [7.1597, 3.527],
 [7.1274, 3.733],
 [7.0889, 3.9269],
 [7.0427, 4.1076],
 [6.9811, 4.2929],
 [6.9234, 4.4354],
 [6.8532, 4.5821],
 [6.7629, 4.7635],
 [6.6926, 4.9237],
 [6.6174, 5.1032],
 [6.5348, 5.2748],
 [6.4384, 5.4448],
 [6.3594, 5.5689],
 [6.2642, 5.7003],
 [6.1872, 5.7946],
 [6.0996, 5.8854],
 [6.0276, 5.9483],
 [5.9457, 6.0081],
 [5.8669, 6.0577],
 [5.785, 6.0984],
 [5.6737, 6.1376],
 [5.571, 6.1589],
 [5.4481, 6.1697],
 [5.317, 6.1724],
 [5.1862, 6.1705],
 [5.0496, 6.1679],
 [4.9063, 6.1709],
 [4.7434, 6.1806],
 [4.5661, 6.1988],
 [4.4101, 6.2227],
 [4.2163, 6.2489],
 [4.0092, 6.2612],
 [3.8557, 6.2591],
 [3.6729, 6.2443],
 [3.4874, 6.2211],
 [3.3422, 6.1946],
 [3.1954, 6.1566],
 [3.0454, 6.1048],
 [2.8952, 6.0346],
 [2.7781, 5.9692],
 [2.6551, 5.8895],
 [2.5574, 5.8154],
 [2.4687, 5.7408],
 [2.3904, 5.6627],
 [2.3262, 5.5881],
 [2.2714, 5.5107],
 [2.223, 5.4285],
 [2.1772, 5.3298],
 [2.1536, 5.2578],
 [2.1313, 5.1651],
 [2.1197, 5.0525],
 [2.1193, 4.972],
 [2.1268, 4.8814],
 [2.146, 4.7917],
 [2.172, 4.704],
 [2.2134, 4.6095],
 [2.2502, 4.5445],
 [2.3059, 4.4625],
 [2.3698, 4.3885],
 [2.4376, 4.3246],
 [2.5302, 4.258],
 [2.6398, 4.1911],
 [2.7106, 4.1484],
 [2.8463, 4.0683],
 [2.9391, 4.0034],
 [3.0588, 3.9036],
 [3.1402, 3.8243],
 [3.2166, 3.739],
 [3.2866, 3.6476],
 [3.3348, 3.5762],
 [3.3921, 3.4723],
 [3.4265, 3.3931],
 [3.4529, 3.3097],
 [3.4721, 3.221],
 [3.4816, 3.144],
 [3.4828, 3.0314],
 [3.4752, 2.9491],
 [3.4589, 2.8679],
 [3.4296, 2.7795],
 [3.3947, 2.7051],
 [3.3518, 2.6336],
 [3.2987, 2.563],
 [3.2201, 2.479],
 [3.1396, 2.4103],
 [3.038, 2.341],
 [2.9166, 2.2679],
 [2.8016, 2.198],
 [2.6712, 2.1225],
 [2.5272, 2.0395],
 [2.4055, 1.9708],
 [2.2578, 1.8948],
 [2.0986, 1.8236],
 [1.9566, 1.7711],
 [1.7845, 1.7197],
 [1.6357, 1.6847],
 [1.4762, 1.6582],
 [1.3264, 1.6427],
 [1.1642, 1.6399],
 [0.98, 1.6513],
 [0.8452, 1.6715],
 [0.6818, 1.7106],
 [0.5173, 1.7607],
 [0.3607, 1.8189],
 [0.1871, 1.8916],
 [0.0331, 1.9628],
 [-0.1188, 2.0453],
 [-0.3174, 2.1564],
 [-0.4724, 2.2498],
 [-0.6409, 2.3692],
 [-0.7895, 2.4857],
 [-0.9316, 2.6121],
 [-1.0659, 2.7427],
 [-1.1645, 2.845],
 [-1.2549, 2.95],
 [-1.3302, 3.045],
 [-1.3901, 3.1317],
 [-1.4392, 3.2099],
 [-1.486, 3.2993],
 [-1.5226, 3.3915],
 [-1.5503, 3.4881],
 [-1.5662, 3.5737],
 [-1.5745, 3.6623],
 [-1.5722, 3.753],
 [-1.5585, 3.8449],
 [-1.5373, 3.9239],
 [-1.4997, 4.0122],
 [-1.4527, 4.0908],
 [-1.4001, 4.1757],
 [-1.3558, 4.2657],
 [-1.3089, 4.3773],
 [-1.2665, 4.5225],
 [-1.2432, 4.651],
 [-1.2292, 4.7817],
 [-1.2225, 4.9218],
 [-1.2256, 5.0393],
 [-1.2388, 5.154],
 [-1.2593, 5.2587],
 [-1.2881, 5.3561],
 [-1.3178, 5.4397],
 [-1.365, 5.5353],
 [-1.4113, 5.6109],
 [-1.4631, 5.6817],
 [-1.5275, 5.7503],
 [-1.5955, 5.8118],
 [-1.6734, 5.8674],
 [-1.7543, 5.9138],
 [-1.8416, 5.952],
 [-1.9162, 5.9753],
 [-2.0153, 5.9975],
 [-2.1113, 6.007],
 [-2.1935, 6.0069],
 [-2.2853, 5.9978],
 [-2.3826, 5.9774],
 [-2.4784, 5.9448],
 [-2.5855, 5.8955],
 [-2.6939, 5.8329],
 [-2.8243, 5.7482],
 [-2.934, 5.6712],
 [-3.0556, 5.5811],
 [-3.1775, 5.4871],
 [-3.3002, 5.3969],
 [-3.4341, 5.3066],
 [-3.5838, 5.2142],
 [-3.7016, 5.1466],
 [-3.8541, 5.0574],
 [-3.9873, 4.9685],
 [-4.141, 4.8571],
 [-4.2802, 4.7376],
 [-4.4137, 4.6144],
 [-4.5226, 4.5085],
 [-4.629, 4.4001],
 [-4.732, 4.2863],
 [-4.8283, 4.1716],
 [-4.9309, 4.0411],
 [-5.0191, 3.9281],
 [-5.1043, 3.8154],
 [-5.2171, 3.667],
 [-5.3498, 3.5026],
 [-5.4749, 3.3441],
 [-5.5985, 3.1771],
 [-5.6899, 3.051]]

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
