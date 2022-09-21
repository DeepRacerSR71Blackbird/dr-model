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
        racing_track = [[0.64079, -0.37133, 4.0, 0.0753],
                [0.94056, -0.4008, 4.0, 0.0753],
                [1.24036, -0.42999, 4.0, 0.0753],
                [1.54019, -0.45892, 4.0, 0.07531],
                [1.84008, -0.4876, 4.0, 0.07531],
                [2.13998, -0.51636, 4.0, 0.07532],
                [2.43985, -0.54539, 4.0, 0.07532],
                [2.7397, -0.57472, 4.0, 0.07532],
                [3.03952, -0.60432, 4.0, 0.07532],
                [3.33925, -0.63389, 4.0, 0.0753],
                [3.63875, -0.66028, 4.0, 0.07517],
                [3.93779, -0.68079, 4.0, 0.07494],
                [4.23585, -0.6926, 3.875, 0.07698],
                [4.53213, -0.69295, 3.49912, 0.08467],
                [4.82557, -0.67911, 3.19342, 0.09199],
                [5.11477, -0.64838, 2.94186, 0.09886],
                [5.39795, -0.59814, 2.74173, 0.1049],
                [5.6729, -0.52592, 2.59299, 0.10963],
                [5.93705, -0.42968, 2.48835, 0.11298],
                [6.1875, -0.30816, 2.41692, 0.11518],
                [6.42118, -0.16104, 2.36891, 0.11657],
                [6.63491, 0.01104, 2.33905, 0.11731],
                [6.82561, 0.20655, 2.32882, 0.11727],
                [6.99045, 0.42325, 2.32882, 0.11691],
                [7.12722, 0.65839, 2.34651, 0.11593],
                [7.23465, 0.90889, 2.40533, 0.11332],
                [7.31274, 1.17161, 2.52167, 0.10869],
                [7.3629, 1.44355, 2.71611, 0.10181],
                [7.38794, 1.72207, 3.01992, 0.0926],
                [7.39185, 2.00495, 3.49865, 0.08086],
                [7.37948, 2.29049, 4.0, 0.07145],
                [7.35482, 2.57758, 4.0, 0.07204],
                [7.32104, 2.86551, 4.0, 0.07248],
                [7.28131, 3.15378, 4.0, 0.07275],
                [7.23815, 3.4422, 4.0, 0.07291],
                [7.18785, 3.72962, 3.75643, 0.07768],
                [7.12702, 4.01341, 3.30856, 0.08772],
                [7.05266, 4.29125, 2.96178, 0.09711],
                [6.96172, 4.56049, 2.69166, 0.10558],
                [6.8513, 4.81816, 2.49572, 0.11232],
                [6.71882, 5.06098, 2.39379, 0.11555],
                [6.56243, 5.28565, 2.38196, 0.11493],
                [6.38164, 5.48955, 2.38196, 0.11441],
                [6.17749, 5.67117, 2.46162, 0.111],
                [5.95236, 5.83042, 2.6491, 0.1041],
                [5.70965, 5.96884, 2.88898, 0.09672],
                [5.4526, 6.08841, 3.07686, 0.09214],
                [5.1834, 6.19009, 3.31503, 0.08681],
                [4.90432, 6.2753, 3.34566, 0.08722],
                [4.61642, 6.34324, 3.26483, 0.0906],
                [4.32145, 6.39305, 3.02821, 0.09879],
                [4.02278, 6.42245, 2.74349, 0.10939],
                [3.72776, 6.42785, 2.5223, 0.11699],
                [3.44289, 6.40598, 2.29674, 0.1244],
                [3.17305, 6.3555, 2.09881, 0.1308],
                [2.92329, 6.27547, 1.92105, 0.13652],
                [2.69876, 6.16613, 1.77422, 0.14076],
                [2.50472, 6.02885, 1.65366, 0.14374],
                [2.34609, 5.86633, 1.56721, 0.14491],
                [2.2275, 5.68229, 1.51751, 0.14428],
                [2.15299, 5.48136, 1.51751, 0.14122],
                [2.12611, 5.2689, 1.5491, 0.13825],
                [2.14717, 5.05113, 1.6588, 0.13189],
                [2.21343, 4.83388, 1.84921, 0.12283],
                [2.31951, 4.62149, 2.10314, 0.11288],
                [2.45917, 4.41658, 2.41265, 0.10278],
                [2.6263, 4.22021, 2.74296, 0.09401],
                [2.81595, 4.03263, 2.5209, 0.10581],
                [3.01637, 3.85834, 2.19676, 0.12091],
                [3.19101, 3.67246, 1.94006, 0.13146],
                [3.33405, 3.47507, 1.73863, 0.14021],
                [3.43978, 3.26755, 1.59007, 0.14648],
                [3.50275, 3.05278, 1.52602, 0.14666],
                [3.51798, 2.83529, 1.52602, 0.14287],
                [3.48337, 2.62114, 1.53456, 0.14136],
                [3.39969, 2.41708, 1.60225, 0.13765],
                [3.27022, 2.22958, 1.71658, 0.13274],
                [3.1001, 2.06394, 1.86863, 0.12707],
                [2.8956, 1.92376, 2.0551, 0.12064],
                [2.66341, 1.81062, 2.30871, 0.11188],
                [2.41047, 1.7234, 2.52137, 0.10612],
                [2.1416, 1.66133, 2.80166, 0.09849],
                [1.86147, 1.62204, 3.09131, 0.09151],
                [1.57361, 1.60307, 3.3805, 0.08534],
                [1.28069, 1.60212, 3.58115, 0.0818],
                [0.98467, 1.6179, 3.62696, 0.08173],
                [0.68732, 1.64964, 3.18226, 0.09397],
                [0.39076, 1.69822, 2.89985, 0.10363],
                [0.102, 1.76733, 2.61717, 0.11345],
                [-0.17001, 1.85755, 2.35764, 0.12156],
                [-0.41975, 1.96934, 2.1435, 0.12765],
                [-0.64278, 2.1027, 1.96901, 0.13197],
                [-0.83555, 2.25689, 1.83122, 0.1348],
                [-0.99514, 2.43064, 1.72148, 0.13705],
                [-1.11879, 2.62248, 1.72148, 0.13258],
                [-1.20313, 2.83107, 1.72513, 0.13042],
                [-1.24762, 3.05381, 1.84974, 0.1228],
                [-1.25495, 3.28746, 2.12228, 0.11015],
                [-1.23107, 3.52862, 2.5193, 0.09619],
                [-1.18267, 3.77463, 3.1012, 0.08085],
                [-1.11666, 4.02353, 2.97533, 0.08655],
                [-1.05864, 4.28038, 2.4622, 0.10695],
                [-1.0203, 4.53417, 2.11242, 0.1215],
                [-1.00851, 4.78234, 1.89638, 0.13101],
                [-1.02938, 5.02115, 1.73052, 0.13853],
                [-1.08646, 5.24602, 1.6426, 0.14124],
                [-1.18224, 5.45103, 1.61412, 0.14019],
                [-1.31617, 5.63003, 1.61412, 0.1385],
                [-1.48504, 5.77719, 1.63089, 0.13734],
                [-1.68362, 5.88771, 1.68367, 0.13498],
                [-1.90548, 5.95828, 1.76772, 0.1317],
                [-2.14373, 5.98741, 1.88169, 0.12756],
                [-2.39174, 5.97547, 2.02604, 0.12255],
                [-2.64364, 5.92451, 2.20176, 0.11672],
                [-2.89465, 5.83791, 2.40988, 0.11019],
                [-3.14127, 5.71988, 2.65178, 0.1031],
                [-3.38117, 5.57502, 2.93011, 0.09564],
                [-3.61311, 5.40792, 3.25064, 0.08794],
                [-3.83673, 5.22285, 3.62843, 0.08],
                [-4.05236, 5.02362, 4.0, 0.0734],
                [-4.2609, 4.81363, 4.0, 0.07398],
                [-4.46339, 4.59554, 4.0, 0.0744],
                [-4.66072, 4.37116, 4.0, 0.0747],
                [-4.85399, 4.14218, 4.0, 0.07491],
                [-5.04419, 3.90989, 4.0, 0.07506],
                [-5.23222, 3.67529, 4.0, 0.07516],
                [-5.41879, 3.43905, 4.0, 0.07526],
                [-5.60428, 3.20184, 4.0, 0.07528],
                [-5.78752, 2.9633, 4.0, 0.0752],
                [-5.96743, 2.72277, 4.0, 0.07509],
                [-6.14262, 2.4795, 4.0, 0.07495],
                [-6.31221, 2.23301, 4.0, 0.0748],
                [-6.47491, 1.98269, 4.0, 0.07464],
                [-6.62934, 1.72792, 4.0, 0.07448],
                [-6.77406, 1.46815, 4.0, 0.07434],
                [-6.90786, 1.20305, 4.0, 0.07424],
                [-7.02987, 0.93257, 3.9744, 0.07466],
                [-7.13948, 0.65691, 3.94414, 0.07521],
                [-7.23625, 0.37649, 3.89679, 0.07613],
                [-7.31975, 0.09187, 3.81371, 0.07777],
                [-7.38945, -0.19622, 3.68935, 0.08034],
                [-7.44455, -0.48696, 3.53271, 0.08376],
                [-7.48394, -0.77937, 3.36281, 0.08774],
                [-7.50622, -1.07231, 3.20175, 0.09176],
                [-7.50977, -1.36449, 3.06993, 0.09518],
                [-7.49301, -1.65444, 2.98481, 0.09731],
                [-7.45462, -1.9407, 2.96227, 0.0975],
                [-7.39385, -2.22186, 2.96227, 0.09711],
                [-7.31076, -2.49681, 3.01806, 0.09517],
                [-7.20644, -2.76492, 3.16469, 0.09091],
                [-7.08288, -3.02609, 3.39491, 0.0851],
                [-6.9426, -3.28061, 3.66165, 0.07937],
                [-6.78791, -3.5289, 3.90948, 0.07483],
                [-6.62063, -3.77111, 4.0, 0.07359],
                [-6.44224, -4.00675, 4.0, 0.07389],
                [-6.25412, -4.23444, 3.88135, 0.0761],
                [-6.05742, -4.4522, 3.66929, 0.07997],
                [-5.85245, -4.65845, 3.45192, 0.08424],
                [-5.63879, -4.85184, 3.26621, 0.08823],
                [-5.41567, -5.03093, 3.14774, 0.09089],
                [-5.18237, -5.19418, 3.12188, 0.09121],
                [-4.93844, -5.34038, 3.12188, 0.0911],
                [-4.68396, -5.46903, 3.20052, 0.08909],
                [-4.41963, -5.58059, 3.39139, 0.0846],
                [-4.14665, -5.67647, 3.70646, 0.07806],
                [-3.8665, -5.75878, 4.0, 0.073],
                [-3.58073, -5.82996, 4.0, 0.07363],
                [-3.29071, -5.89243, 4.0, 0.07417],
                [-2.99762, -5.9484, 4.0, 0.0746],
                [-2.7024, -5.99985, 4.0, 0.07492],
                [-2.40584, -6.04847, 4.0, 0.07513],
                [-2.10852, -6.0956, 4.0, 0.07526],
                [-1.81091, -6.14244, 4.0, 0.07532],
                [-1.51338, -6.1898, 4.0, 0.07532],
                [-1.21565, -6.23577, 4.0, 0.07531],
                [-0.91763, -6.27862, 4.0, 0.07527],
                [-0.61942, -6.3163, 4.0, 0.07515],
                [-0.32135, -6.34692, 4.0, 0.07491],
                [-0.02395, -6.3684, 4.0, 0.07454],
                [0.27209, -6.37863, 3.72789, 0.07946],
                [0.56603, -6.37588, 3.46454, 0.08485],
                [0.85698, -6.35807, 3.31986, 0.0878],
                [1.14399, -6.32328, 3.30534, 0.08747],
                [1.4263, -6.27047, 3.30534, 0.08689],
                [1.70351, -6.19985, 3.45723, 0.08274],
                [1.97577, -6.11316, 3.67705, 0.07771],
                [2.24342, -6.01233, 3.72548, 0.07677],
                [2.50656, -5.89831, 3.60195, 0.07962],
                [2.76476, -5.77046, 3.32728, 0.08659],
                [3.01737, -5.62763, 2.88146, 0.10071],
                [3.25925, -5.46953, 2.57324, 0.1123],
                [3.47831, -5.2985, 2.32937, 0.11931],
                [3.67091, -5.11394, 2.10152, 0.12693],
                [3.83308, -4.9168, 1.87781, 0.13594],
                [3.96032, -4.70895, 1.74469, 0.13969],
                [4.0472, -4.49341, 1.63528, 0.14211],
                [4.09126, -4.27496, 1.5272, 0.14592],
                [4.09059, -4.05897, 1.5, 0.14399],
                [4.04219, -3.85186, 1.5, 0.14179],
                [3.94655, -3.6604, 1.53912, 0.13905],
                [3.80673, -3.49063, 1.64071, 0.13405],
                [3.62773, -3.34701, 1.80091, 0.12743],
                [3.41574, -3.23188, 2.01602, 0.11966],
                [3.1773, -3.14547, 2.28428, 0.11103],
                [2.91862, -3.08618, 2.61014, 0.10168],
                [2.64512, -3.05109, 3.01209, 0.09154],
                [2.36127, -3.0364, 3.54326, 0.08022],
                [2.07056, -3.03774, 4.0, 0.07268],
                [1.77576, -3.05004, 4.0, 0.07376],
                [1.47888, -3.06843, 4.0, 0.07436],
                [1.17878, -3.08787, 4.0, 0.07518],
                [0.87866, -3.10547, 4.0, 0.07516],
                [0.57853, -3.12026, 4.0, 0.07512],
                [0.27839, -3.1312, 4.0, 0.07508],
                [-0.02175, -3.13774, 4.0, 0.07505],
                [-0.32189, -3.1397, 4.0, 0.07504],
                [-0.62201, -3.13721, 4.0, 0.07503],
                [-0.92212, -3.13064, 4.0, 0.07505],
                [-1.22223, -3.12038, 4.0, 0.07507],
                [-1.52234, -3.10674, 4.0, 0.0751],
                [-1.82246, -3.0896, 4.0, 0.07515],
                [-2.12274, -3.06847, 4.0, 0.07525],
                [-2.42176, -3.04128, 3.81997, 0.0786],
                [-2.71748, -3.00374, 3.31448, 0.08994],
                [-3.00749, -2.95211, 2.93556, 0.10034],
                [-3.28864, -2.88284, 2.6305, 0.11008],
                [-3.55704, -2.7929, 2.38233, 0.11882],
                [-3.80808, -2.68003, 2.19729, 0.12527],
                [-4.03664, -2.54303, 2.0137, 0.13233],
                [-4.23794, -2.3824, 1.86261, 0.13827],
                [-4.40638, -2.19926, 1.74924, 0.14225],
                [-4.53642, -1.99628, 1.67897, 0.14358],
                [-4.62319, -1.77792, 1.64656, 0.1427],
                [-4.66332, -1.55037, 1.64399, 0.14055],
                [-4.65539, -1.32111, 1.64399, 0.13954],
                [-4.59991, -1.09813, 1.66578, 0.13794],
                [-4.49923, -0.8891, 1.7114, 0.13557],
                [-4.35724, -0.70061, 1.78576, 0.13215],
                [-4.17891, -0.53752, 1.89776, 0.12734],
                [-3.96981, -0.40258, 2.05809, 0.12092],
                [-3.73566, -0.29629, 2.2772, 0.11292],
                [-3.48189, -0.21722, 2.56098, 0.10379],
                [-3.21326, -0.16255, 2.90427, 0.09439],
                [-2.93364, -0.12891, 3.30808, 0.08514],
                [-2.64603, -0.11286, 3.78981, 0.07601],
                [-2.35283, -0.11112, 4.0, 0.0733],
                [-2.05595, -0.12057, 4.0, 0.07426],
                [-1.75699, -0.13842, 4.0, 0.07487],
                [-1.4572, -0.1622, 4.0, 0.07518],
                [-1.15734, -0.18992, 4.0, 0.07528],
                [-0.85764, -0.2199, 4.0, 0.0753],
                [-0.55802, -0.25085, 4.0, 0.0753],
                [-0.25836, -0.28141, 4.0, 0.0753],
                [0.04133, -0.31164, 4.0, 0.0753],
                [0.34105, -0.34158, 4.0, 0.0753]]

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
