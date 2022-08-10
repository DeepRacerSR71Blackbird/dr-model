import math
class Reward:

    def __init__(self, verbose=True):
        self.first_racingpoint_index = 0
        self.verbose = verbose

    def reward_function(self, params):

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
            dir_diff=abs(steering_angle - best_steering_angle)
            # error = (dir_diff / MAX_DIFF) if dir_diff<MAX_DIFF else 1  # 30 degree is already really bad
            if dir_diff>15:
                score=0
            elif dir_diff>10:
                score=0.5
            else:
                score=1

            # score = (1.0 - error)**2
            # if dir_diff<5:
            #     score += 0.3

            if self.verbose == True:
                print("actual_steering={:.2f} score={:.4f}".format(steering_angle,score))

            return max(score, 0.01)  # optimizer is rumored to struggle with negative numbers and numbers too close to zero

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
        
        def racing_direction_diff_noabs(closest_coords, second_closest_coords, car_coords, heading):

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
            direction_diff = track_direction - heading

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
        racing_track = [[-2.03902, -5.95812, 4.0, 0.07546],
                [-1.74078, -6.00621, 4.0, 0.07552],
                [-1.44228, -6.05404, 4.0, 0.07558],
                [-1.14379, -6.10211, 4.0, 0.07558],
                [-0.84581, -6.14932, 4.0, 0.07542],
                [-0.54871, -6.19378, 4.0, 0.0751],
                [-0.25267, -6.234, 4.0, 0.07469],
                [0.04206, -6.26838, 3.5908, 0.08264],
                [0.33519, -6.2953, 3.25949, 0.09031],
                [0.62634, -6.31313, 2.98944, 0.09757],
                [0.91501, -6.32024, 2.76381, 0.10448],
                [1.20061, -6.315, 2.57361, 0.11099],
                [1.48241, -6.29577, 2.41629, 0.1169],
                [1.75954, -6.26098, 2.29198, 0.12186],
                [2.03099, -6.20919, 2.19926, 0.12565],
                [2.29562, -6.13918, 2.13472, 0.12823],
                [2.55222, -6.05013, 2.09377, 0.12972],
                [2.79949, -5.94161, 2.07143, 0.13036],
                [3.0361, -5.81362, 2.06293, 0.1304],
                [3.26071, -5.66657, 2.06293, 0.13014],
                [3.472, -5.50121, 2.06428, 0.12997],
                [3.66871, -5.31857, 2.07291, 0.12949],
                [3.8497, -5.11987, 2.08811, 0.12871],
                [4.01401, -4.90651, 2.11139, 0.12755],
                [4.16093, -4.67993, 2.14645, 0.12581],
                [4.29009, -4.44165, 2.19905, 0.12325],
                [4.40151, -4.19315, 2.27704, 0.1196],
                [4.49572, -3.93589, 2.39111, 0.11457],
                [4.57372, -3.67131, 2.55751, 0.10786],
                [4.63708, -3.40077, 2.80202, 0.09917],
                [4.68788, -3.12557, 3.02495, 0.09251],
                [4.72761, -2.84661, 3.17173, 0.08884],
                [4.75703, -2.56443, 3.35632, 0.08453],
                [4.77716, -2.27956, 3.04678, 0.09373],
                [4.78744, -2.00298, 2.81823, 0.09821],
                [4.80831, -1.73141, 2.65231, 0.10269],
                [4.84095, -1.46566, 2.47631, 0.10812],
                [4.88618, -1.20643, 2.32119, 0.11337],
                [4.94505, -0.95456, 2.18734, 0.11825],
                [5.01863, -0.71098, 2.11078, 0.12055],
                [5.10793, -0.47658, 2.09093, 0.11996],
                [5.21338, -0.25197, 2.09093, 0.11867],
                [5.33477, -0.03731, 2.12971, 0.11579],
                [5.47124, 0.16768, 2.23674, 0.1101],
                [5.62132, 0.36375, 2.4362, 0.10135],
                [5.78295, 0.55213, 2.79561, 0.08879],
                [5.95345, 0.73452, 3.24158, 0.07702],
                [6.13065, 0.91235, 3.99128, 0.0629],
                [6.31231, 1.08711, 3.8908, 0.06479],
                [6.50463, 1.27738, 3.23491, 0.08363],
                [6.69091, 1.47067, 2.76979, 0.09692],
                [6.86861, 1.66813, 2.45977, 0.108],
                [7.03489, 1.87093, 2.10073, 0.12484],
                [7.18706, 2.08, 1.86071, 0.13897],
                [7.32042, 2.29684, 1.6554, 0.15378],
                [7.43038, 2.52235, 1.5, 0.16726],
                [7.51119, 2.7569, 1.5, 0.16539],
                [7.55645, 2.99934, 1.63571, 0.15078],
                [7.57153, 3.24569, 1.73841, 0.14198],
                [7.55952, 3.4935, 1.81414, 0.13676],
                [7.52218, 3.74089, 1.82739, 0.13691],
                [7.45944, 3.98596, 1.89433, 0.13354],
                [7.3727, 4.22691, 1.95354, 0.13109],
                [7.26318, 4.46204, 1.9681, 0.1318],
                [7.13118, 4.68932, 2.01941, 0.13015],
                [6.97811, 4.90697, 2.07555, 0.1282],
                [6.80563, 5.1134, 2.12861, 0.12638],
                [6.61546, 5.30724, 2.17635, 0.12477],
                [6.40931, 5.48728, 2.21895, 0.12335],
                [6.18883, 5.65256, 2.25757, 0.12206],
                [5.95562, 5.80227, 2.29361, 0.12082],
                [5.71124, 5.93579, 2.32813, 0.11961],
                [5.45718, 6.05267, 2.36148, 0.11842],
                [5.19489, 6.15257, 2.39304, 0.11729],
                [4.92581, 6.23529, 2.42133, 0.11626],
                [4.65134, 6.3007, 2.44432, 0.11543],
                [4.37289, 6.34869, 2.46007, 0.11486],
                [4.0918, 6.37923, 2.46723, 0.1146],
                [3.80944, 6.39225, 2.46177, 0.11482],
                [3.52707, 6.38772, 2.45575, 0.115],
                [3.24592, 6.36565, 2.4555, 0.11485],
                [2.96709, 6.32611, 2.4555, 0.11469],
                [2.69154, 6.26929, 2.46865, 0.11397],
                [2.42005, 6.19558, 2.50419, 0.11234],
                [2.15318, 6.10561, 2.57318, 0.10945],
                [1.8912, 6.00032, 2.69103, 0.10492],
                [1.63413, 5.88105, 2.88307, 0.0983],
                [1.38163, 5.74955, 3.21624, 0.08852],
                [1.13301, 5.60817, 3.53052, 0.08101],
                [0.88777, 5.45855, 3.92839, 0.07313],
                [0.64534, 5.30222, 4.0, 0.07211],
                [0.40556, 5.1398, 4.0, 0.0724],
                [0.16248, 4.96764, 4.0, 0.07447],
                [-0.08359, 4.80119, 3.99635, 0.07434],
                [-0.33279, 4.6409, 3.80099, 0.07795],
                [-0.58525, 4.48723, 3.57162, 0.08275],
                [-0.8412, 4.34087, 3.40459, 0.0866],
                [-1.10097, 4.20279, 3.23951, 0.09081],
                [-1.3648, 4.07383, 3.18099, 0.09232],
                [-1.63296, 3.95497, 3.18099, 0.09221],
                [-1.90545, 3.84662, 3.19473, 0.09179],
                [-2.18205, 3.74878, 3.27403, 0.08962],
                [-2.46244, 3.66102, 3.42555, 0.08577],
                [-2.74616, 3.58255, 3.67283, 0.08015],
                [-3.03269, 3.51225, 4.0, 0.07376],
                [-3.32143, 3.44866, 4.0, 0.07392],
                [-3.61175, 3.38995, 4.0, 0.07405],
                [-3.9029, 3.33389, 4.0, 0.07412],
                [-4.18779, 3.27956, 3.77704, 0.07678],
                [-4.47072, 3.22126, 3.18041, 0.09083],
                [-4.75005, 3.15579, 2.82403, 0.10159],
                [-5.02426, 3.08049, 2.58399, 0.11005],
                [-5.29192, 2.99313, 2.41353, 0.11666],
                [-5.55162, 2.89192, 2.29139, 0.12164],
                [-5.80197, 2.77549, 2.20685, 0.12511],
                [-6.04164, 2.64286, 2.15395, 0.12717],
                [-6.26937, 2.4935, 2.12878, 0.12793],
                [-6.48402, 2.32729, 2.12811, 0.12757],
                [-6.68459, 2.1445, 2.12811, 0.12752],
                [-6.87028, 1.94576, 2.14881, 0.12657],
                [-7.04046, 1.73204, 2.18748, 0.12489],
                [-7.19472, 1.50453, 2.24045, 0.12269],
                [-7.33284, 1.26458, 2.30377, 0.12017],
                [-7.45473, 1.01367, 2.37328, 0.11754],
                [-7.56045, 0.7533, 2.44476, 0.11495],
                [-7.65015, 0.48494, 2.51402, 0.11255],
                [-7.72401, 0.21003, 2.57708, 0.11046],
                [-7.78225, -0.07005, 2.63041, 0.10875],
                [-7.82504, -0.35401, 2.67117, 0.10751],
                [-7.85254, -0.64064, 2.69741, 0.10675],
                [-7.86484, -0.92877, 2.70393, 0.10666],
                [-7.86196, -1.21728, 2.68557, 0.10744],
                [-7.84388, -1.50508, 2.6551, 0.10861],
                [-7.81049, -1.79108, 2.61492, 0.11011],
                [-7.76167, -2.07415, 2.56771, 0.11187],
                [-7.69725, -2.35315, 2.51626, 0.1138],
                [-7.61707, -2.62692, 2.46338, 0.1158],
                [-7.52098, -2.89422, 2.4119, 0.11777],
                [-7.40891, -3.15381, 2.36464, 0.11957],
                [-7.28083, -3.40442, 2.32445, 0.12108],
                [-7.13683, -3.64483, 2.29417, 0.12215],
                [-6.97713, -3.87385, 2.27664, 0.12264],
                [-6.80208, -4.09039, 2.27465, 0.12241],
                [-6.61219, -4.29355, 2.27465, 0.12225],
                [-6.40812, -4.48259, 2.29094, 0.12142],
                [-6.19069, -4.65706, 2.32823, 0.11974],
                [-5.96085, -4.81676, 2.38931, 0.11714],
                [-5.71966, -4.96182, 2.47721, 0.11362],
                [-5.46825, -5.09266, 2.59547, 0.1092],
                [-5.20782, -5.21001, 2.74857, 0.10393],
                [-4.93957, -5.31485, 2.94252, 0.09788],
                [-4.66467, -5.40836, 3.18576, 0.09115],
                [-4.38422, -5.49188, 3.4903, 0.08384],
                [-4.09925, -5.56683, 3.87226, 0.0761],
                [-3.81068, -5.63465, 4.0, 0.07411],
                [-3.5193, -5.6967, 4.0, 0.07448],
                [-3.22576, -5.75421, 4.0, 0.07478],
                [-2.93058, -5.80825, 4.0, 0.07502],
                [-2.63418, -5.85974, 4.0, 0.07521],
                [-2.33689, -5.90946, 4.0, 0.07535]]

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
        reward = 0.01

        ## Reward if car goes towards right direction
        DIRECTION_MULTIPLE = 1
        if list(x[0] for x in racing_track).index(optimals[0])>list(x[0] for x in racing_track).index(optimals_second[0]):
            tmp = optimals
            optimals = optimals_second
            optimals = tmp
        #racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading)
        dir_diff = racing_direction_diff_noabs(optimals[0:2], optimals_second[0:2], [x, y], heading)
        # direction_reward = (1e-3 if  (( dir_diff * steering_angle>0) | (abs(dir_diff-steering_angle)>30)) else (abs(dir_diff-steering_angle)/30)**0.5 )
        # reward += direction_reward
        steer_reward = score_steer_to_point_ahead(params,racing_track)
        reward += steer_reward
  
        ## Zero reward if off track ##
        # if is_offtrack:
        #     reward = 1e-3
        # if all_wheels_on_track == False:
        #     reward /= 2

        ####################### VERBOSE #######################
        if self.verbose == True:
            print(" total_reward=" + str(reward) + " steer_reward=" + str(steer_reward)+"all_wheels_on_track=" + str(all_wheels_on_track) + " x=" + str(x) + " y=" + str(y) + " distance_from_center=" + str(distance_from_center)
              + " heading=" + str(heading) + " progress=" + str(progress) + " steps=" + str(steps) + " speed=" + str(speed) + " steering_angle=" + str(steering_angle)
              + " is_offtrack=" + str(is_offtrack) + " closest_index=" + str(closest_index) + " distance_racingline="
              + " optimal_speed=" + str(optimals[2]) + " speed_diff=" + " speed_reward=") # + " distance_reward=" + str(distance_reward) + str(speed_diff)
            #+ str(speed_reward)+ " steps_reward="  + str(steps_reward) + str(direction_diff) + str(dist)
        #################### RETURN REWARD ####################

        # Always return a float value
        return float(reward)


reward_object = Reward() # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)
