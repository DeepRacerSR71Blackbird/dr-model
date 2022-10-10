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
            tx, ty, _, _ = get_target_point(racing_coords,[car_x,car_y])
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
                        [1.24023, -0.43111, 3.47301, 0.08673],
                        [1.54017, -0.45899, 3.05472, 0.09861],
                        [1.84011, -0.4873, 2.78232, 0.10828],
                        [2.14001, -0.51602, 2.52121, 0.1195],
                        [2.43988, -0.54511, 2.30963, 0.13044],
                        [2.73972, -0.57454, 2.14198, 0.14065],
                        [3.03952, -0.60432, 2.02827, 0.14854],
                        [3.33929, -0.6345, 1.95664, 0.15398],
                        [3.63896, -0.66386, 1.91256, 0.15744],
                        [3.93852, -0.68909, 1.88289, 0.15966],
                        [4.23768, -0.7068, 1.85844, 0.16125],
                        [4.53569, -0.71306, 1.83879, 0.1621],
                        [4.83146, -0.70481, 1.83566, 0.16119],
                        [5.12368, -0.67951, 1.83566, 0.15979],
                        [5.41055, -0.63412, 1.83566, 0.15822],
                        [5.68981, -0.56571, 1.83566, 0.15663],
                        [5.95872, -0.47177, 1.83566, 0.15517],
                        [6.21425, -0.3508, 1.83566, 0.15401],
                        [6.45319, -0.20253, 1.83566, 0.15319],
                        [6.6722, -0.02781, 1.83566, 0.15262],
                        [6.86787, 0.17152, 1.83566, 0.15216],
                        [7.03686, 0.39293, 1.83566, 0.15173],
                        [7.17622, 0.63332, 1.83566, 0.15137],
                        [7.28401, 0.88923, 1.86705, 0.14873],
                        [7.35988, 1.15707, 1.94668, 0.143],
                        [7.40525, 1.43348, 2.09409, 0.13376],
                        [7.42335, 1.7156, 2.28576, 0.12368],
                        [7.41892, 2.00121, 2.06307, 0.13845],
                        [7.39767, 2.28868, 1.90946, 0.15096],
                        [7.36489, 2.57697, 1.82741, 0.15877],
                        [7.32502, 2.86542, 1.81655, 0.1603],
                        [7.28131, 3.15378, 1.81655, 0.16055],
                        [7.23877, 3.44226, 1.81655, 0.16052],
                        [7.19402, 3.73133, 1.81655, 0.16102],
                        [7.14148, 4.01904, 1.81655, 0.161],
                        [7.07606, 4.30249, 1.81655, 0.16014],
                        [6.99305, 4.57812, 1.81655, 0.15847],
                        [6.88858, 4.84227, 1.81655, 0.15638],
                        [6.75952, 5.09105, 1.81655, 0.15428],
                        [6.60384, 5.32052, 1.81655, 0.15265],
                        [6.42116, 5.52734, 1.81655, 0.15191],
                        [6.2129, 5.70947, 1.87628, 0.14745],
                        [5.98197, 5.86649, 1.87814, 0.14869],
                        [5.73223, 5.99978, 1.69672, 0.16683],
                        [5.46811, 6.1124, 1.54149, 0.18627],
                        [5.19257, 6.20633, 1.40966, 0.20651],
                        [4.90829, 6.28365, 1.30283, 0.22613],
                        [4.61738, 6.34589, 1.23051, 0.24176],
                        [4.32145, 6.39305, 1.20357, 0.24898],
                        [4.02259, 6.42393, 1.20357, 0.24963],
                        [3.72549, 6.43451, 1.20357, 0.24701],
                        [3.4358, 6.42065, 1.20357, 0.24097],
                        [3.15852, 6.37866, 1.20357, 0.23301],
                        [2.89854, 6.30664, 1.20357, 0.22415],
                        [2.66148, 6.20322, 1.20357, 0.21489],
                        [2.45358, 6.06839, 1.20357, 0.20588],
                        [2.28158, 5.90378, 1.20357, 0.19781],
                        [2.15242, 5.71288, 1.20357, 0.19151],
                        [2.07219, 5.50141, 1.20357, 0.18792],
                        [2.04465, 5.27709, 1.22595, 0.18435],
                        [2.07025, 5.04845, 1.24881, 0.18423],
                        [2.1459, 4.8232, 1.21197, 0.19606],
                        [2.2647, 4.60667, 1.21197, 0.20378],
                        [2.42058, 4.40222, 1.21197, 0.21213],
                        [2.60677, 4.21118, 1.21197, 0.22011],
                        [2.81595, 4.03263, 1.21197, 0.22692],
                        [3.03892, 3.87024, 1.21197, 0.22759],
                        [3.23339, 3.69064, 1.21197, 0.21842],
                        [3.39324, 3.49418, 1.21197, 0.20898],
                        [3.512, 3.28263, 1.21197, 0.20017],
                        [3.58355, 3.05973, 1.21197, 0.19316],
                        [3.60229, 2.83121, 1.21197, 0.18919],
                        [3.56607, 2.60481, 1.23033, 0.18635],
                        [3.47636, 2.3891, 1.29033, 0.18106],
                        [3.33762, 2.19209, 1.3805, 0.17455],
                        [3.15639, 2.02017, 1.49466, 0.16713],
                        [2.94026, 1.87738, 1.63282, 0.15864],
                        [2.69697, 1.76527, 1.80039, 0.14879],
                        [2.43366, 1.68299, 1.91383, 0.14414],
                        [2.15661, 1.62746, 1.70345, 0.16588],
                        [1.87014, 1.59614, 1.5345, 0.1878],
                        [1.57779, 1.58563, 1.40085, 0.20883],
                        [1.28216, 1.59227, 1.31554, 0.22478],
                        [0.98488, 1.61403, 1.31165, 0.22725],
                        [0.68732, 1.64964, 1.31165, 0.22848],
                        [0.39076, 1.69822, 1.31165, 0.22911],
                        [0.09788, 1.76152, 1.31165, 0.22845],
                        [-0.18338, 1.84367, 1.31165, 0.22339],
                        [-0.44484, 1.94783, 1.31165, 0.21457],
                        [-0.68082, 2.07505, 1.31165, 0.20439],
                        [-0.88613, 2.22568, 1.31165, 0.19414],
                        [-1.05603, 2.39903, 1.31165, 0.18505],
                        [-1.1859, 2.59363, 1.31165, 0.17837],
                        [-1.27169, 2.80717, 1.31165, 0.17545],
                        [-1.31218, 3.03599, 1.30395, 0.17821],
                        [-1.30992, 3.27569, 1.29654, 0.18489],
                        [-1.27163, 3.52202, 1.29654, 0.19227],
                        [-1.2041, 3.77199, 1.29654, 0.19971],
                        [-1.11666, 4.02353, 1.29654, 0.20539],
                        [-1.03833, 4.28244, 1.29654, 0.20864],
                        [-0.98207, 4.54023, 1.29654, 0.20351],
                        [-0.95621, 4.79466, 1.29654, 0.19724],
                        [-0.96689, 5.04215, 1.29654, 0.19107],
                        [-1.01977, 5.27723, 1.29654, 0.18584],
                        [-1.1169, 5.4931, 1.29654, 0.18258],
                        [-1.25686, 5.68242, 1.29654, 0.18158],
                        [-1.43516, 5.83816, 1.31609, 0.17988],
                        [-1.64517, 5.95455, 1.3566, 0.17699],
                        [-1.87908, 6.02771, 1.41762, 0.17288],
                        [-2.12878, 6.05606, 1.50178, 0.16734],
                        [-2.38675, 6.04036, 1.61225, 0.1603],
                        [-2.64656, 5.98336, 1.75137, 0.15188],
                        [-2.90327, 5.88931, 1.9204, 0.14236],
                        [-3.15346, 5.7633, 2.11989, 0.13214],
                        [-3.39512, 5.61065, 2.35053, 0.1216],
                        [-3.6274, 5.43644, 2.61477, 0.11104],
                        [-3.8503, 5.24516, 2.92008, 0.10059],
                        [-4.06446, 5.04067, 3.28415, 0.09016],
                        [-4.2709, 4.82614, 3.7374, 0.07966],
                        [-4.47092, 4.60413, 4.0, 0.07471],
                        [-4.66587, 4.37669, 3.71445, 0.08065],
                        [-4.85701, 4.14535, 3.4241, 0.08764],
                        [-5.04553, 3.91133, 3.24873, 0.0925],
                        [-5.2325, 3.67562, 3.15956, 0.09522],
                        [-5.41879, 3.43905, 3.1301, 0.0962],
                        [-5.60491, 3.2022, 3.1301, 0.09624],
                        [-5.78968, 2.96459, 3.1301, 0.09616],
                        [-5.97183, 2.72539, 3.1301, 0.09605],
                        [-6.14999, 2.48379, 3.09144, 0.0971],
                        [-6.32271, 2.23896, 2.99076, 0.10018],
                        [-6.48839, 1.99007, 2.84899, 0.10495],
                        [-6.64537, 1.73633, 2.68753, 0.11102],
                        [-6.79206, 1.47712, 2.53024, 0.11772],
                        [-6.92724, 1.21208, 2.39732, 0.1241],
                        [-7.0501, 0.94122, 2.30416, 0.12908],
                        [-7.16023, 0.66483, 2.26316, 0.13146],
                        [-7.25745, 0.38343, 2.26316, 0.13155],
                        [-7.34163, 0.09767, 2.26316, 0.13163],
                        [-7.41248, -0.19174, 2.26316, 0.13165],
                        [-7.46936, -0.48399, 2.26316, 0.13156],
                        [-7.51114, -0.77819, 2.26316, 0.1313],
                        [-7.53626, -1.07326, 2.26316, 0.13085],
                        [-7.54276, -1.36786, 2.26316, 0.1302],
                        [-7.52857, -1.66042, 2.26316, 0.12942],
                        [-7.49183, -1.9492, 2.26316, 0.12863],
                        [-7.43127, -2.23245, 2.26316, 0.12798],
                        [-7.34659, -2.50868, 2.28759, 0.1263],
                        [-7.23871, -2.77696, 2.3984, 0.12056],
                        [-7.1099, -3.0371, 2.3708, 0.12244],
                        [-6.96398, -3.28998, 2.34583, 0.12446],
                        [-6.80402, -3.53647, 2.34583, 0.12526],
                        [-6.63278, -3.77756, 2.34583, 0.12606],
                        [-6.45217, -4.01404, 2.34583, 0.12684],
                        [-6.26256, -4.24601, 2.34583, 0.12772],
                        [-6.06755, -4.46708, 2.34583, 0.12567],
                        [-5.86515, -4.67719, 2.34583, 0.12437],
                        [-5.65374, -4.87423, 2.34583, 0.12319],
                        [-5.43247, -5.05671, 2.34583, 0.12227],
                        [-5.2004, -5.22281, 2.34583, 0.12166],
                        [-4.95641, -5.37014, 2.34583, 0.1215],
                        [-4.70062, -5.49797, 2.42181, 0.11807],
                        [-4.43407, -5.607, 2.59576, 0.11095],
                        [-4.15837, -5.69906, 2.8813, 0.10088],
                        [-3.87543, -5.77682, 3.30243, 0.08886],
                        [-3.58708, -5.84333, 3.87815, 0.0763],
                        [-3.29489, -5.90145, 4.0, 0.07448],
                        [-3.00008, -5.95374, 3.45168, 0.08674],
                        [-2.70359, -6.00235, 3.17799, 0.09454],
                        [-2.40619, -6.04913, 2.88617, 0.10431],
                        [-2.10852, -6.0956, 2.65845, 0.11333],
                        [-1.81091, -6.14238, 2.49833, 0.12059],
                        [-1.51338, -6.1898, 2.44131, 0.12341],
                        [-1.2157, -6.23625, 2.44131, 0.12341],
                        [-0.9178, -6.28108, 2.44131, 0.1234],
                        [-0.61948, -6.32222, 2.44131, 0.12336],
                        [-0.32084, -6.35739, 2.44131, 0.12317],
                        [-0.02236, -6.38407, 2.44131, 0.12275],
                        [0.27514, -6.39915, 2.44131, 0.12202],
                        [0.57077, -6.40076, 2.44131, 0.1211],
                        [0.86346, -6.38645, 2.13216, 0.13744],
                        [1.15202, -6.35392, 1.8863, 0.15394],
                        [1.43529, -6.30134, 1.69577, 0.1699],
                        [1.71248, -6.22831, 1.51891, 0.18873],
                        [1.98346, -6.13609, 1.37588, 0.20804],
                        [2.24881, -6.02766, 1.26958, 0.22579],
                        [2.50943, -5.90614, 1.21256, 0.23715],
                        [2.76552, -5.77251, 1.2, 0.24072],
                        [3.01737, -5.62763, 1.2, 0.24212],
                        [3.26378, -5.47215, 1.2, 0.2428],
                        [3.4918, -5.30604, 1.2, 0.23509],
                        [3.69614, -5.1256, 1.2, 0.22717],
                        [3.87153, -4.93033, 1.2, 0.21873],
                        [4.01276, -4.72129, 1.2, 0.21023],
                        [4.11367, -4.50083, 1.2, 0.20204],
                        [4.16827, -4.27337, 1.2, 0.19493],
                        [4.17169, -4.04561, 1.2, 0.18983],
                        [4.12207, -3.82599, 1.2, 0.18763],
                        [4.02073, -3.62338, 1.23013, 0.18416],
                        [3.87183, -3.44556, 1.30514, 0.1777],
                        [3.68174, -3.298, 1.42979, 0.16831],
                        [3.45815, -3.18303, 1.60871, 0.15628],
                        [3.20905, -3.09988, 1.82829, 0.14364],
                        [2.94136, -3.04591, 2.09041, 0.13063],
                        [2.66067, -3.01738, 2.41454, 0.11685],
                        [2.37135, -3.00987, 2.80214, 0.10328],
                        [2.07655, -3.01908, 3.42353, 0.08615],
                        [1.7787, -3.04001, 4.0, 0.07465],
                        [1.47888, -3.06843, 4.0, 0.07529],
                        [1.17884, -3.09303, 4.0, 0.07526],
                        [0.87872, -3.11365, 4.0, 0.07521],
                        [0.57856, -3.13001, 4.0, 0.07515],
                        [0.27839, -3.14178, 3.25243, 0.09236],
                        [-0.02178, -3.14857, 2.74126, 0.10953],
                        [-0.32193, -3.14993, 2.38959, 0.12561],
                        [-0.62204, -3.14587, 2.08054, 0.14426],
                        [-0.92214, -3.13703, 1.89457, 0.15847],
                        [-1.22222, -3.12423, 1.74781, 0.17185],
                        [-1.52232, -3.10827, 1.5925, 0.18871],
                        [-1.82246, -3.0896, 1.46936, 0.20466],
                        [-2.12274, -3.06847, 1.38969, 0.21661],
                        [-2.42258, -3.04417, 1.34644, 0.22343],
                        [-2.72044, -3.01192, 1.33112, 0.22507],
                        [-3.01429, -2.96714, 1.33112, 0.2233],
                        [-3.30132, -2.90548, 1.33112, 0.22055],
                        [-3.57779, -2.82299, 1.33112, 0.21675],
                        [-3.83834, -2.7155, 1.33112, 0.21173],
                        [-4.07763, -2.58154, 1.33112, 0.20603],
                        [-4.2903, -2.42106, 1.33112, 0.20015],
                        [-4.4695, -2.23442, 1.33112, 0.19438],
                        [-4.60827, -2.02402, 1.33112, 0.18934],
                        [-4.70088, -1.7949, 1.33112, 0.18566],
                        [-4.74354, -1.55431, 1.33112, 0.18356],
                        [-4.73467, -1.31105, 1.33496, 0.18234],
                        [-4.67506, -1.07455, 1.35138, 0.18048],
                        [-4.56761, -0.8539, 1.3794, 0.17792],
                        [-4.41697, -0.65687, 1.42523, 0.17402],
                        [-4.22898, -0.48909, 1.50051, 0.16792],
                        [-4.01008, -0.3535, 1.6189, 0.15905],
                        [-3.76673, -0.2502, 1.79419, 0.14735],
                        [-3.50488, -0.17678, 2.05329, 0.13244],
                        [-3.22979, -0.12875, 2.31297, 0.12073],
                        [-2.94487, -0.10257, 2.66948, 0.10718],
                        [-2.65302, -0.09402, 3.12215, 0.09352],
                        [-2.35652, -0.09909, 3.76486, 0.07877],
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
        reward += (steer_reward)

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
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 1
        speed_diff = abs(optimals[2]-speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2)**2
        else:
            speed_reward = 0
        speed_reward = speed_reward * SPEED_MULTIPLE
        # reward += speed_reward
        
        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1
        STANDARD_TIME = 35
        FASTEST_TIME = 25
        times_list = [row[3] for row in racing_track]
        projected_time = projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME*(FASTEST_TIME) /
                                           (STANDARD_TIME-FASTEST_TIME))*(steps_prediction-(STANDARD_TIME*15+1)))
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
        except:
            steps_reward = 0
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
        reward += finish_reward
        
        ## Zero reward if off track ##
        if all_wheels_on_track == False:
            reward = 1e-3
        # coef=1.2
        # reward=float(score_steer_to_point_ahead_falktan(params,coef))
        # print("dist_reward={:.3f} steer_reward={:.3f}".format(distance_reward,steer_reward))
        # print("speed_reward={:.3f} tot_reward={:.3f}".format(speed_reward,reward))
        print("=== Distance reward: %f ===" % (distance_reward))
        print("=== Steer reward: %f ===" % steer_reward)
        print("=== Finish reward: %f ===" % finish_reward)
        print("=== Total reward: %f ===" % reward)
        print("=== Speed reward: %f ===" % speed_reward)
        print("=== Steps reward: %f ===" % steps_reward)
        print("Distance to racing line: %f" % dist)
        print("Optimal speed: %f" % optimals[2])
        print("Speed difference: %f" % speed_diff)
        # print("Direction difference: %f" % direction_diff)
        print("Predicted time: %f" % projected_time)
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
