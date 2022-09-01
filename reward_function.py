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
        racing_track = [[-1.8984, -5.965, 1.7],
            [-1.8986, -5.9647, 2.9],
            [-1.8919, -5.9693, 3.0],
            [-1.8712, -5.9762, 2.9],
            [-1.838, -5.9835, 3.0],
            [-1.7827, -5.9924, 2.9],
            [-1.7181, -6.0032, 2.9],
            [-1.6468, -6.0151, 2.9],
            [-1.5466, -6.0313, 3.5],
            [-1.4458, -6.0469, 3.5],
            [-1.3209, -6.0602, 2.9],
            [-1.165, -6.0727, 3.2],
            [-1.0248, -6.0876, 2.9],
            [-0.8497, -6.1137, 2.8],
            [-0.6854, -6.1389, 2.9],
            [-0.5257, -6.1672, 2.9],
            [-0.3058, -6.2148, 3.2],
            [-0.1312, -6.2606, 3.0],
            [0.0544, -6.317, 3.0],
            [0.2488, -6.3753, 2.9],
            [0.4483, -6.426, 2.9],
            [0.645, -6.4668, 2.9],
            [0.8651, -6.5006, 2.9],
            [1.0558, -6.522, 3.5],
            [1.275, -6.533, 1.7],
            [1.5059, -6.5262, 1.7],
            [1.668, -6.5108, 2.9],
            [1.8788, -6.4787, 2.9],
            [2.0512, -6.4413, 3.5],
            [2.2363, -6.3884, 3.5],
            [2.4057, -6.3267, 2.9],
            [2.6136, -6.233, 2.9],
            [2.7958, -6.138, 2.9],
            [2.9826, -6.0238, 2.9],
            [3.1503, -5.9075, 2.9],
            [3.3102, -5.7841, 3.6],
            [3.4771, -5.6395, 3.6],
            [3.631, -5.487, 2.9],
            [3.7807, -5.3181, 2.9],
            [3.9106, -5.1532, 2.9],
            [4.0303, -4.9787, 2.9],
            [4.1301, -4.8155, 2.9],
            [4.2347, -4.6235, 2.9],
            [4.3232, -4.4399, 2.9],
            [4.4031, -4.2473, 3.0],
            [4.4755, -4.0493, 3.0],
            [4.5389, -3.8467, 2.9],
            [4.5848, -3.6595, 3.5],
            [4.6241, -3.4461, 2.9],
            [4.6496, -3.222, 2.9],
            [4.6626, -2.9972, 2.9],
            [4.6642, -2.8045, 2.9],
            [4.6634, -2.6067, 2.9],
            [4.6618, -2.3908, 2.9],
            [4.6644, -2.185, 2.9],
            [4.6731, -1.9805, 2.9],
            [4.6878, -1.77, 2.9],
            [4.7116, -1.5574, 2.9],
            [4.7456, -1.3447, 1.4],
            [4.7831, -1.1773, 2.9],
            [4.8425, -0.9819, 2.9],
            [4.91, -0.8024, 2.9],
            [4.989, -0.6284, 2.9],
            [5.0842, -0.4491, 4.0],
            [5.1881, -0.2739, 2.9],
            [5.3128, -0.0914, 2.9],
            [5.4391, 0.0754, 2.9],
            [5.5717, 0.2354, 2.9],
            [5.7142, 0.3914, 2.9],
            [5.87, 0.5482, 3.6],
            [6.0179, 0.6827, 2.0],
            [6.1953, 0.8253, 2.9],
            [6.3559, 0.9426, 2.9],
            [6.5267, 1.0598, 2.9],
            [6.6861, 1.1716, 2.9],
            [6.8459, 1.2951, 2.9],
            [6.9925, 1.4199, 3.5],
            [7.1463, 1.5674, 2.9],
            [7.2941, 1.7326, 3.5],
            [7.4239, 1.8981, 3.5],
            [7.5446, 2.0744, 3.5],
            [7.6663, 2.2772, 2.0],
            [7.7604, 2.474, 2.9],
            [7.8389, 2.6649, 2.9],
            [7.9116, 2.8773, 1.7],
            [7.958, 3.0597, 1.7],
            [7.9927, 3.2443, 1.7],
            [8.0114, 3.4032, 3.0],
            [8.0205, 3.5732, 1.7],
            [8.0113, 3.7176, 1.7],
            [7.9891, 3.8629, 2.9],
            [7.9553, 4.006, 3.0],
            [7.8978, 4.1664, 2.9],
            [7.8355, 4.3036, 3.0],
            [7.7425, 4.4693, 2.9],
            [7.6321, 4.6339, 2.9],
            [7.5283, 4.7692, 2.9],
            [7.3874, 4.9341, 3.0],
            [7.2488, 5.08, 2.8],
            [7.1042, 5.2129, 2.8],
            [6.9493, 5.3404, 2.8],
            [6.7954, 5.4568, 2.9],
            [6.6291, 5.5692, 2.9],
            [6.4643, 5.6688, 3.0],
            [6.2879, 5.7668, 3.0],
            [6.1082, 5.8548, 2.8],
            [5.9133, 5.9377, 2.8],
            [5.7194, 6.0062, 3.6],
            [5.5286, 6.0664, 2.8],
            [5.3309, 6.1147, 2.8],
            [5.1477, 6.1483, 2.9],
            [4.9425, 6.1799, 3.0],
            [4.751, 6.2095, 2.9],
            [4.5531, 6.2361, 2.9],
            [4.3488, 6.261, 3.0],
            [4.1334, 6.2894, 2.9],
            [3.9414, 6.3094, 2.9],
            [3.7311, 6.3285, 3.6],
            [3.5353, 6.3466, 2.9],
            [3.3068, 6.3576, 2.9],
            [3.0977, 6.3572, 3.0],
            [2.9125, 6.3476, 2.9],
            [2.6855, 6.3241, 3.5],
            [2.461, 6.2888, 2.9],
            [2.2734, 6.2484, 2.9],
            [2.0533, 6.1872, 1.7],
            [1.865, 6.1203, 3.0],
            [1.6777, 6.042, 2.9],
            [1.5082, 5.9571, 2.9],
            [1.3149, 5.847, 3.0],
            [1.1684, 5.7532, 2.9],
            [0.9941, 5.6299, 2.8],
            [0.8392, 5.507, 3.0],
            [0.6856, 5.3768, 3.0],
            [0.5347, 5.2455, 2.9],
            [0.3724, 5.1091, 2.9],
            [0.1841, 4.9617, 2.9],
            [0.0457, 4.8573, 2.9],
            [-0.1211, 4.74, 3.2],
            [-0.2878, 4.6334, 3.0],
            [-0.5074, 4.5034, 2.9],
            [-0.6653, 4.4195, 2.9],
            [-0.843, 4.3338, 2.8],
            [-1.0541, 4.2422, 2.9],
            [-1.2473, 4.1647, 2.9],
            [-1.4168, 4.1, 2.9],
            [-1.5929, 4.0311, 2.9],
            [-1.7796, 3.954, 2.8],
            [-1.9751, 3.8751, 2.9],
            [-2.1526, 3.8088, 2.8],
            [-2.3443, 3.7413, 2.0],
            [-2.5369, 3.6815, 2.0],
            [-2.699, 3.6397, 2.9],
            [-2.8529, 3.6049, 2.9],
            [-3.0371, 3.5632, 2.9],
            [-3.2131, 3.524, 2.0],
            [-3.4096, 3.4874, 2.0],
            [-3.5971, 3.4628, 3.0],
            [-3.7399, 3.4477, 3.0],
            [-3.9338, 3.4298, 2.9],
            [-4.1132, 3.4097, 4.0],
            [-4.3157, 3.3775, 3.5],
            [-4.5267, 3.3283, 2.9],
            [-4.72, 3.2707, 3.0],
            [-4.9128, 3.2031, 2.9],
            [-5.1155, 3.1184, 2.9],
            [-5.3138, 3.0271, 2.9],
            [-5.4944, 2.9355, 3.0],
            [-5.6737, 2.845, 3.0],
            [-5.8624, 2.7504, 3.0],
            [-6.0643, 2.6531, 2.9],
            [-6.229, 2.5708, 1.7],
            [-6.4263, 2.4576, 2.9],
            [-6.5667, 2.3683, 2.9],
            [-6.7528, 2.2369, 1.7],
            [-6.8998, 2.1172, 2.9],
            [-7.0065, 2.0192, 2.9],
            [-7.1533, 1.8669, 2.9],
            [-7.2852, 1.715, 1.7],
            [-7.4017, 1.5632, 1.7],
            [-7.5008, 1.4099, 2.9],
            [-7.5785, 1.2681, 2.9],
            [-7.6471, 1.1089, 3.0],
            [-7.7151, 0.9115, 2.8],
            [-7.7642, 0.7286, 2.8],
            [-7.8031, 0.5553, 3.0],
            [-7.8376, 0.3541, 2.9],
            [-7.8593, 0.1564, 4.0],
            [-7.8718, -0.0288, 3.6],
            [-7.8727, -0.2526, 2.9],
            [-7.8631, -0.4712, 2.9],
            [-7.8474, -0.6768, 2.9],
            [-7.8247, -0.9006, 2.0],
            [-7.8003, -1.0862, 2.9],
            [-7.7668, -1.3033, 3.0],
            [-7.7377, -1.4787, 2.9],
            [-7.7082, -1.6837, 2.0],
            [-7.6848, -1.866, 2.9],
            [-7.6643, -2.0506, 2.9],
            [-7.6305, -2.2675, 2.9],
            [-7.5913, -2.4456, 2.8],
            [-7.5357, -2.6568, 2.8],
            [-7.4885, -2.821, 2.8],
            [-7.4348, -2.9925, 2.8],
            [-7.3664, -3.1874, 2.9],
            [-7.3, -3.3546, 2.9],
            [-7.2192, -3.5295, 3.5],
            [-7.115, -3.7227, 3.5],
            [-7.0001, -3.9004, 3.5],
            [-6.8613, -4.0866, 2.9],
            [-6.7441, -4.2288, 2.9],
            [-6.5849, -4.4037, 2.9],
            [-6.4228, -4.5608, 3.0],
            [-6.2644, -4.698, 3.0],
            [-6.1084, -4.8207, 3.5],
            [-5.9303, -4.9443, 2.8],
            [-5.7232, -5.0642, 2.9],
            [-5.5512, -5.1503, 2.0],
            [-5.3619, -5.2281, 2.0],
            [-5.1834, -5.2875, 2.9],
            [-5.0017, -5.3424, 3.0],
            [-4.8351, -5.3926, 3.0],
            [-4.6504, -5.4434, 3.0],
            [-4.4361, -5.4994, 3.0],
            [-4.2288, -5.5526, 3.0],
            [-4.0332, -5.6022, 2.9],
            [-3.8511, -5.6434, 3.0],
            [-3.6548, -5.6803, 2.8],
            [-3.4258, -5.7109, 2.8],
            [-3.2553, -5.7241, 2.9],
            [-3.0519, -5.7338, 2.9],
            [-2.8612, -5.7423, 2.9],
            [-2.6574, -5.752, 3.2],
            [-2.4415, -5.7689, 2.9],
            [-2.2445, -5.7928, 2.9],
            [-2.049, -5.8214, 3.2],
            [-1.8524, -5.8593, 2.9]]

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
        reward = 1e-3

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 2
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-2, 1 - (dist/(track_width*0.2))) * DISTANCE_MULTIPLE
        reward += distance_reward
        '''
        ABS_STEERING_THRESHOLD = 15 
        abs_steering = abs(steering_angle)
        # Penalize reward if the car is steering too much
        if abs_steering > ABS_STEERING_THRESHOLD:
            reward *= (ABS_STEERING_THRESHOLD/abs_steering)
        '''
        ## Reward if speed is close to optimal speed ##
        
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
        '''
        # Reward if less steps 
        REWARD_PER_STEP_FOR_FASTEST_TIME = 4
        STANDARD_TIME = 21
        FASTEST_TIME = 16
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
        '''
        # slowest = 2.0
        # avg_slow = 2.5
        # if steps<15:
        #     speed_reward = speed/2.0
        # else:
        #     if speed<slowest:
        #         speed_reward = 1e-3
        #     elif speed<avg_slow:
        #         speed_reward = 0.1
        #     else:
        #         speed_reward = 0.1+(speed - avg_slow)
        # MAX_SPEED = 4.0
        # speed_multiple = 0.8
        # speed_reward = (speed / MAX_SPEED) * speed_multiple
        # reward += speed_reward
        # Zero reward if obviously wrong direction (e.g. spin)
        # direction_diff = racing_direction_diff(
        #     optimals[0:2], optimals_second[0:2], [x, y], heading)
        # if direction_diff > 30:
        #     reward = 1e-3

        heading2optimal_diff,steer2optimal_diff,steer_reward = score_steer_to_point_ahead(params,racing_track)
        heading2optimal_diff=abs(heading2optimal_diff)
        if heading2optimal_diff>35:
            steer_reward = 1e-3
        elif heading2optimal_diff>15:
            steer_reward = (1-(heading2optimal_diff/90))**2
        else:
            steer_reward = 1
        reward += steer_reward

        def calc_step_reward(params):
            steps = params['steps']
            progress = params['progress']
            TOTAL_NUM_STEPS = 250
            reward = 0
            if progress > (steps / TOTAL_NUM_STEPS) * 100 :
                reward += 1.0
            return float(reward)
        
        step_reward_multiple=0.8
        step_reward=calc_step_reward(params)*step_reward_multiple
        reward+=step_reward
        '''
        # Zero reward of obviously too slow
        speed_diff_zero = optimals[2]-speed
        if speed_diff_zero > 0.5:
            reward = 1e-3

        ## Incentive for finishing the lap in less steps ##
        REWARD_FOR_FASTEST_TIME = 1000 # should be adapted to track length and other rewards
        STANDARD_TIME = 21  # seconds (time that is easily done by model)
        FASTEST_TIME = 16  # seconds (best time of 1st place on the track)
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                        (15*(STANDARD_TIME-FASTEST_TIME)))*(steps-STANDARD_TIME*15))
        else:
            finish_reward = 0
        reward += finish_reward
        '''
        ## Zero reward if off track ##
        if all_wheels_on_track == False:
            reward = 1e-3
        print("dist_reward={:.3f} steer_reward={:.3f} step_reward={:.3f}".format(distance_reward,steer_reward,step_reward))
        print("speed_reward={:.3f} tot_reward={:.3f}".format(speed_reward,reward))

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
