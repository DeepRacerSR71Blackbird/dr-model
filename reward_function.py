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

        # TODO based on generic
        def get_target_steering_degree(params,racing_coords):
            car_x = params['x']
            car_y = params['y']
            tx, ty, _ = get_target_point(racing_coords,[car_x,car_y])
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
        racing_track = [[0.64052, -0.37381, 4.0, 0.0753],
                        [0.94034, -0.40275, 4.0, 0.0753],
                        [1.24023, -0.43111, 4.0, 0.07531],
                        [1.54017, -0.45899, 3.81841, 0.07889],
                        [1.84011, -0.4873, 3.4779, 0.08662],
                        [2.14001, -0.51602, 3.15151, 0.0956],
                        [2.43988, -0.54511, 2.88704, 0.10435],
                        [2.73972, -0.57454, 2.67748, 0.11252],
                        [3.03952, -0.60432, 2.53534, 0.11883],
                        [3.33929, -0.6345, 2.44579, 0.12318],
                        [3.63896, -0.66386, 2.3907, 0.12595],
                        [3.93852, -0.68909, 2.35361, 0.12773],
                        [4.23768, -0.7068, 2.32305, 0.129],
                        [4.53569, -0.71306, 2.29848, 0.12968],
                        [4.83146, -0.70481, 2.29457, 0.12895],
                        [5.12368, -0.67951, 2.29457, 0.12783],
                        [5.41055, -0.63412, 2.29457, 0.12658],
                        [5.68981, -0.56571, 2.29457, 0.1253],
                        [5.95872, -0.47177, 2.29457, 0.12414],
                        [6.21425, -0.3508, 2.29457, 0.12321],
                        [6.45319, -0.20253, 2.29457, 0.12255],
                        [6.6722, -0.02781, 2.29457, 0.1221],
                        [6.86787, 0.17152, 2.29457, 0.12173],
                        [7.03686, 0.39293, 2.29457, 0.12139],
                        [7.17622, 0.63332, 2.29457, 0.1211],
                        [7.28401, 0.88923, 2.33381, 0.11898],
                        [7.35988, 1.15707, 2.43334, 0.1144],
                        [7.40525, 1.43348, 2.61761, 0.10701],
                        [7.42335, 1.7156, 2.8572, 0.09894],
                        [7.41892, 2.00121, 2.57884, 0.11076],
                        [7.39767, 2.28868, 2.38682, 0.12077],
                        [7.36489, 2.57697, 2.28426, 0.12702],
                        [7.32502, 2.86542, 2.27069, 0.12824],
                        [7.28131, 3.15378, 2.27069, 0.12844],
                        [7.23877, 3.44226, 2.27069, 0.12842],
                        [7.19402, 3.73133, 2.27069, 0.12882],
                        [7.14148, 4.01904, 2.27069, 0.1288],
                        [7.07606, 4.30249, 2.27069, 0.12811],
                        [6.99305, 4.57812, 2.27069, 0.12677],
                        [6.88858, 4.84227, 2.27069, 0.1251],
                        [6.75952, 5.09105, 2.27069, 0.12343],
                        [6.60384, 5.32052, 2.27069, 0.12212],
                        [6.42116, 5.52734, 2.27069, 0.12153],
                        [6.2129, 5.70947, 2.34534, 0.11796],
                        [5.98197, 5.86649, 2.34768, 0.11895],
                        [5.73223, 5.99978, 2.1209, 0.13347],
                        [5.46811, 6.1124, 1.92686, 0.14902],
                        [5.19257, 6.20633, 1.76208, 0.16521],
                        [4.90829, 6.28365, 1.62853, 0.1809],
                        [4.61738, 6.34589, 1.53814, 0.19341],
                        [4.32145, 6.39305, 1.50446, 0.19918],
                        [4.02259, 6.42393, 1.50446, 0.1997],
                        [3.72549, 6.43451, 1.50446, 0.19761],
                        [3.4358, 6.42065, 1.50446, 0.19278],
                        [3.15852, 6.37866, 1.50446, 0.18641],
                        [2.89854, 6.30664, 1.50446, 0.17932],
                        [2.66148, 6.20322, 1.50446, 0.17191],
                        [2.45358, 6.06839, 1.50446, 0.1647],
                        [2.28158, 5.90378, 1.50446, 0.15825],
                        [2.15242, 5.71288, 1.50446, 0.15321],
                        [2.07219, 5.50141, 1.50446, 0.15034],
                        [2.04465, 5.27709, 1.53244, 0.14748],
                        [2.07025, 5.04845, 1.56101, 0.14739],
                        [2.1459, 4.8232, 1.51496, 0.15685],
                        [2.2647, 4.60667, 1.51496, 0.16303],
                        [2.42058, 4.40222, 1.51496, 0.1697],
                        [2.60677, 4.21118, 1.51496, 0.17609],
                        [2.81595, 4.03263, 1.51496, 0.18153],
                        [3.03892, 3.87024, 1.51496, 0.18208],
                        [3.23339, 3.69064, 1.51496, 0.17474],
                        [3.39324, 3.49418, 1.51496, 0.16718],
                        [3.512, 3.28263, 1.51496, 0.16014],
                        [3.58355, 3.05973, 1.51496, 0.15453],
                        [3.60229, 2.83121, 1.51496, 0.15135],
                        [3.56607, 2.60481, 1.53792, 0.14908],
                        [3.47636, 2.3891, 1.61291, 0.14484],
                        [3.33762, 2.19209, 1.72563, 0.13964],
                        [3.15639, 2.02017, 1.86832, 0.13371],
                        [2.94026, 1.87738, 2.04103, 0.12691],
                        [2.69697, 1.76527, 2.25049, 0.11903],
                        [2.43366, 1.68299, 2.39229, 0.11532],
                        [2.15661, 1.62746, 2.12931, 0.1327],
                        [1.87014, 1.59614, 1.91812, 0.15024],
                        [1.57779, 1.58563, 1.75106, 0.16706],
                        [1.28216, 1.59227, 1.64442, 0.17983],
                        [0.98488, 1.61403, 1.63956, 0.1818],
                        [0.68732, 1.64964, 1.63956, 0.18278],
                        [0.39076, 1.69822, 1.63956, 0.18329],
                        [0.09788, 1.76152, 1.63956, 0.18276],
                        [-0.18338, 1.84367, 1.63956, 0.17871],
                        [-0.44484, 1.94783, 1.63956, 0.17166],
                        [-0.68082, 2.07505, 1.63956, 0.16352],
                        [-0.88613, 2.22568, 1.63956, 0.15531],
                        [-1.05603, 2.39903, 1.63956, 0.14804],
                        [-1.1859, 2.59363, 1.63956, 0.14269],
                        [-1.27169, 2.80717, 1.63956, 0.14036],
                        [-1.31218, 3.03599, 1.62993, 0.14257],
                        [-1.30992, 3.27569, 1.62068, 0.14791],
                        [-1.27163, 3.52202, 1.62068, 0.15382],
                        [-1.2041, 3.77199, 1.62068, 0.15977],
                        [-1.11666, 4.02353, 1.62068, 0.16431],
                        [-1.03833, 4.28244, 1.62068, 0.16691],
                        [-0.98207, 4.54023, 1.62068, 0.16281],
                        [-0.95621, 4.79466, 1.62068, 0.1578],
                        [-0.96689, 5.04215, 1.62068, 0.15285],
                        [-1.01977, 5.27723, 1.62068, 0.14867],
                        [-1.1169, 5.4931, 1.62068, 0.14606],
                        [-1.25686, 5.68242, 1.62068, 0.14527],
                        [-1.43516, 5.83816, 1.64512, 0.14391],
                        [-1.64517, 5.95455, 1.69575, 0.14159],
                        [-1.87908, 6.02771, 1.77203, 0.1383],
                        [-2.12878, 6.05606, 1.87723, 0.13387],
                        [-2.38675, 6.04036, 2.01531, 0.12824],
                        [-2.64656, 5.98336, 2.18921, 0.1215],
                        [-2.90327, 5.88931, 2.4005, 0.11389],
                        [-3.15346, 5.7633, 2.64986, 0.10571],
                        [-3.39512, 5.61065, 2.93816, 0.09728],
                        [-3.6274, 5.43644, 3.26846, 0.08883],
                        [-3.8503, 5.24516, 3.65011, 0.08047],
                        [-4.06446, 5.04067, 4.0, 0.07403],
                        [-4.2709, 4.82614, 4.0, 0.07443],
                        [-4.47092, 4.60413, 4.0, 0.07471],
                        [-4.66587, 4.37669, 4.0, 0.07489],
                        [-4.85701, 4.14535, 4.0, 0.07502],
                        [-5.04553, 3.91133, 4.0, 0.07513],
                        [-5.2325, 3.67562, 3.94945, 0.07618],
                        [-5.41879, 3.43905, 3.91262, 0.07696],
                        [-5.60491, 3.2022, 3.91262, 0.07699],
                        [-5.78968, 2.96459, 3.91262, 0.07693],
                        [-5.97183, 2.72539, 3.91262, 0.07684],
                        [-6.14999, 2.48379, 3.86431, 0.07768],
                        [-6.32271, 2.23896, 3.73845, 0.08015],
                        [-6.48839, 1.99007, 3.56123, 0.08396],
                        [-6.64537, 1.73633, 3.35941, 0.08882],
                        [-6.79206, 1.47712, 3.1628, 0.09417],
                        [-6.92724, 1.21208, 2.99665, 0.09928],
                        [-7.0501, 0.94122, 2.8802, 0.10326],
                        [-7.16023, 0.66483, 2.82895, 0.10517],
                        [-7.25745, 0.38343, 2.82895, 0.10524],
                        [-7.34163, 0.09767, 2.82895, 0.10531],
                        [-7.41248, -0.19174, 2.82895, 0.10532],
                        [-7.46936, -0.48399, 2.82895, 0.10525],
                        [-7.51114, -0.77819, 2.82895, 0.10504],
                        [-7.53626, -1.07326, 2.82895, 0.10468],
                        [-7.54276, -1.36786, 2.82895, 0.10416],
                        [-7.52857, -1.66042, 2.82895, 0.10354],
                        [-7.49183, -1.9492, 2.82895, 0.1029],
                        [-7.43127, -2.23245, 2.82895, 0.10239],
                        [-7.34659, -2.50868, 2.85949, 0.10104],
                        [-7.23871, -2.77696, 2.998, 0.09645],
                        [-7.1099, -3.0371, 2.9635, 0.09796],
                        [-6.96398, -3.28998, 2.93229, 0.09957],
                        [-6.80402, -3.53647, 2.93229, 0.10021],
                        [-6.63278, -3.77756, 2.93229, 0.10085],
                        [-6.45217, -4.01404, 2.93229, 0.10148],
                        [-6.26256, -4.24601, 2.93229, 0.10217],
                        [-6.06755, -4.46708, 2.93229, 0.10053],
                        [-5.86515, -4.67719, 2.93229, 0.09949],
                        [-5.65374, -4.87423, 2.93229, 0.09856],
                        [-5.43247, -5.05671, 2.93229, 0.09781],
                        [-5.2004, -5.22281, 2.93229, 0.09733],
                        [-4.95641, -5.37014, 2.93229, 0.0972],
                        [-4.70062, -5.49797, 3.02726, 0.09446],
                        [-4.43407, -5.607, 3.2447, 0.08876],
                        [-4.15837, -5.69906, 3.60162, 0.0807],
                        [-3.87543, -5.77682, 4.0, 0.07336],
                        [-3.58708, -5.84333, 4.0, 0.07398],
                        [-3.29489, -5.90145, 4.0, 0.07448],
                        [-3.00008, -5.95374, 4.0, 0.07485],
                        [-2.70359, -6.00235, 3.97248, 0.07563],
                        [-2.40619, -6.04913, 3.60772, 0.08345],
                        [-2.10852, -6.0956, 3.32306, 0.09066],
                        [-1.81091, -6.14238, 3.12292, 0.09647],
                        [-1.51338, -6.1898, 3.05163, 0.09873],
                        [-1.2157, -6.23625, 3.05163, 0.09873],
                        [-0.9178, -6.28108, 3.05163, 0.09872],
                        [-0.61948, -6.32222, 3.05163, 0.09868],
                        [-0.32084, -6.35739, 3.05163, 0.09854],
                        [-0.02236, -6.38407, 3.05163, 0.0982],
                        [0.27514, -6.39915, 3.05163, 0.09761],
                        [0.57077, -6.40076, 3.05163, 0.09688],
                        [0.86346, -6.38645, 2.6652, 0.10995],
                        [1.15202, -6.35392, 2.35788, 0.12316],
                        [1.43529, -6.30134, 2.11972, 0.13592],
                        [1.71248, -6.22831, 1.89864, 0.15098],
                        [1.98346, -6.13609, 1.71985, 0.16643],
                        [2.24881, -6.02766, 1.58697, 0.18063],
                        [2.50943, -5.90614, 1.5157, 0.18972],
                        [2.76552, -5.77251, 1.5, 0.19257],
                        [3.01737, -5.62763, 1.5, 0.1937],
                        [3.26378, -5.47215, 1.5, 0.19424],
                        [3.4918, -5.30604, 1.5, 0.18807],
                        [3.69614, -5.1256, 1.5, 0.18173],
                        [3.87153, -4.93033, 1.5, 0.17499],
                        [4.01276, -4.72129, 1.5, 0.16818],
                        [4.11367, -4.50083, 1.5, 0.16164],
                        [4.16827, -4.27337, 1.5, 0.15595],
                        [4.17169, -4.04561, 1.5, 0.15186],
                        [4.12207, -3.82599, 1.5, 0.1501],
                        [4.02073, -3.62338, 1.53766, 0.14733],
                        [3.87183, -3.44556, 1.63142, 0.14216],
                        [3.68174, -3.298, 1.78724, 0.13465],
                        [3.45815, -3.18303, 2.01089, 0.12503],
                        [3.20905, -3.09988, 2.28536, 0.11491],
                        [2.94136, -3.04591, 2.61302, 0.10451],
                        [2.66067, -3.01738, 3.01817, 0.09348],
                        [2.37135, -3.00987, 3.50267, 0.08263],
                        [2.07655, -3.01908, 4.0, 0.07374],
                        [1.7787, -3.04001, 4.0, 0.07465],
                        [1.47888, -3.06843, 4.0, 0.07529],
                        [1.17884, -3.09303, 4.0, 0.07526],
                        [0.87872, -3.11365, 4.0, 0.07521],
                        [0.57856, -3.13001, 4.0, 0.07515],
                        [0.27839, -3.14178, 4.0, 0.0751],
                        [-0.02178, -3.14857, 3.42657, 0.08762],
                        [-0.32193, -3.14993, 2.98699, 0.10049],
                        [-0.62204, -3.14587, 2.60067, 0.11541],
                        [-0.92214, -3.13703, 2.36821, 0.12677],
                        [-1.22222, -3.12423, 2.18476, 0.13748],
                        [-1.52232, -3.10827, 1.99062, 0.15097],
                        [-1.82246, -3.0896, 1.83671, 0.16373],
                        [-2.12274, -3.06847, 1.73712, 0.17328],
                        [-2.42258, -3.04417, 1.68305, 0.17874],
                        [-2.72044, -3.01192, 1.6639, 0.18006],
                        [-3.01429, -2.96714, 1.6639, 0.17864],
                        [-3.30132, -2.90548, 1.6639, 0.17644],
                        [-3.57779, -2.82299, 1.6639, 0.1734],
                        [-3.83834, -2.7155, 1.6639, 0.16939],
                        [-4.07763, -2.58154, 1.6639, 0.16482],
                        [-4.2903, -2.42106, 1.6639, 0.16012],
                        [-4.4695, -2.23442, 1.6639, 0.1555],
                        [-4.60827, -2.02402, 1.6639, 0.15147],
                        [-4.70088, -1.7949, 1.6639, 0.14853],
                        [-4.74354, -1.55431, 1.6639, 0.14685],
                        [-4.73467, -1.31105, 1.6687, 0.14588],
                        [-4.67506, -1.07455, 1.68922, 0.14438],
                        [-4.56761, -0.8539, 1.72425, 0.14234],
                        [-4.41697, -0.65687, 1.78153, 0.13922],
                        [-4.22898, -0.48909, 1.87564, 0.13434],
                        [-4.01008, -0.3535, 2.02362, 0.12724],
                        [-3.76673, -0.2502, 2.24274, 0.11788],
                        [-3.50488, -0.17678, 2.56661, 0.10596],
                        [-3.22979, -0.12875, 2.89121, 0.09659],
                        [-2.94487, -0.10257, 3.33684, 0.08575],
                        [-2.65302, -0.09402, 3.90268, 0.07481],
                        [-2.35652, -0.09909, 4.0, 0.07414],
                        [-2.05738, -0.11389, 4.0, 0.07488],
                        [-1.75724, -0.1356, 4.0, 0.07523],
                        [-1.45719, -0.16159, 4.0, 0.07529],
                        [-1.15734, -0.18985, 4.0, 0.0753],
                        [-0.85763, -0.21977, 4.0, 0.0753],
                        [-0.55802, -0.25085, 4.0, 0.0753],
                        [-0.2585, -0.28275, 4.0, 0.0753],
                        [0.0411, -0.31384, 4.0, 0.0753],
                        [0.34078, -0.34418, 4.0, 0.0753]]

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
        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 3
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist/(track_width*0.2))) * DISTANCE_MULTIPLE
        reward += distance_reward
        '''
        ABS_STEERING_THRESHOLD = 15 
        abs_steering = abs(steering_angle)
        # Penalize reward if the car is steering too much
        if abs_steering > ABS_STEERING_THRESHOLD:
            reward *= (ABS_STEERING_THRESHOLD/abs_steering)
        ## Reward if speed is close to optimal speed ##
        '''
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 1
        speed_diff = abs(optimals[2]-speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = 1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2
        else:
            speed_reward = 0
        speed_reward = speed_reward * SPEED_MULTIPLE
        reward += speed_reward
        
        # Reward if less steps 
        REWARD_PER_STEP_FOR_FASTEST_TIME = 4
        STANDARD_TIME = 32
        FASTEST_TIME = 24
        times_list = [row[3] for row in racing_track]
        projected_time = projected_time(self.first_racingpoint_index, closest_index, steps, times_list) 
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(1e-3, (- REWARD_PER_STEP_FOR_FASTEST_TIME * (FASTEST_TIME) /
                (STANDARD_TIME - FASTEST_TIME)) * ( steps_prediction - (STANDARD_TIME * 15 + 1)))
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
        except:
            steps_reward = 0 
        reward += steps_reward
        
        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 30:
            reward = 1e-3
        
        # Zero reward of obviously too slow
        speed_diff_zero = optimals[2]-speed
        if speed_diff_zero > 0.5:
            reward = 1e-3

        ## Incentive for finishing the lap in less steps ##
        REWARD_FOR_FASTEST_TIME = 1000 # should be adapted to track length and other rewards
        STANDARD_TIME = 32  # seconds (time that is easily done by model)
        FASTEST_TIME = 24  # seconds (best time of 1st place on the track)
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                        (15*(STANDARD_TIME-FASTEST_TIME)))*(steps-STANDARD_TIME*15))
        else:
            finish_reward = 0
        reward += finish_reward
        
        ## Zero reward if off track ##
        if all_wheels_on_track == False:
            reward = 1e-3

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
