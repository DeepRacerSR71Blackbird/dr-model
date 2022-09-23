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
            # tx, ty, _ = get_target_point(racing_coords,[car_x,car_y])
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
        racing_track = [[ 0.640522  , -0.37381337],
       [ 0.9403424 , -0.40274743],
       [ 1.24022733, -0.43110514],
       [ 1.54017036, -0.45898804],
       [ 1.8401079 , -0.48730168],
       [ 2.14001086, -0.51602259],
       [ 2.43988015, -0.54511362],
       [ 2.73971772, -0.57454481],
       [ 3.03952221, -0.60432471],
       [ 3.33928903, -0.63449543],
       [ 3.6389596 , -0.66385992],
       [ 3.93852384, -0.68909402],
       [ 4.23768257, -0.70679687],
       [ 4.53569092, -0.71306415],
       [ 4.83146347, -0.70481486],
       [ 5.1236823 , -0.67951298],
       [ 5.41055352, -0.63411667],
       [ 5.68981285, -0.56571405],
       [ 5.95871989, -0.47177054],
       [ 6.21425022, -0.35080028],
       [ 6.45319164, -0.20252804],
       [ 6.67220047, -0.02781422],
       [ 6.86786735,  0.1715236 ],
       [ 7.03685538,  0.39293004],
       [ 7.17622255,  0.63331616],
       [ 7.28401278,  0.8892278 ],
       [ 7.35987561,  1.15707011],
       [ 7.4052524 ,  1.43347929],
       [ 7.42335115,  1.71560255],
       [ 7.41892468,  2.00121068],
       [ 7.39767266,  2.28868481],
       [ 7.36489107,  2.57696888],
       [ 7.32502024,  2.86542233],
       [ 7.28131327,  3.15378334],
       [ 7.23877268,  3.44226478],
       [ 7.19401828,  3.73132823],
       [ 7.14147577,  4.01903991],
       [ 7.07605832,  4.30248512],
       [ 6.99305313,  4.57811995],
       [ 6.88857674,  4.84227241],
       [ 6.75951844,  5.09105104],
       [ 6.60384004,  5.32051599],
       [ 6.42116046,  5.52734207],
       [ 6.21289996,  5.70946828],
       [ 5.98196591,  5.86649377],
       [ 5.73223432,  5.99977542],
       [ 5.46810681,  6.11239547],
       [ 5.19256652,  6.20632654],
       [ 4.90828502,  6.28365035],
       [ 4.61737694,  6.34589272],
       [ 4.32144515,  6.39304648],
       [ 4.02259308,  6.42392708],
       [ 3.72549279,  6.43451091],
       [ 3.43579683,  6.42065359],
       [ 3.15851829,  6.37865537],
       [ 2.89853541,  6.30663555],
       [ 2.66148247,  6.20321519],
       [ 2.45358167,  6.06839326],
       [ 2.28157895,  5.90377884],
       [ 2.15241732,  5.71287673],
       [ 2.07219408,  5.50140816],
       [ 2.04465083,  5.27709069],
       [ 2.07025222,  5.04844802],
       [ 2.14590295,  4.82319818],
       [ 2.2647004 ,  4.60666861],
       [ 2.42057872,  4.4022194 ],
       [ 2.60677268,  4.2111791 ],
       [ 2.81594741,  4.03263044],
       [ 3.0389185 ,  3.87024145],
       [ 3.23339023,  3.69063822],
       [ 3.39324206,  3.494177  ],
       [ 3.51199615,  3.28262756],
       [ 3.58354873,  3.05972911],
       [ 3.60228851,  2.83120666],
       [ 3.5660734 ,  2.60481253],
       [ 3.47636426,  2.38910221],
       [ 3.3376228 ,  2.19209308],
       [ 3.15638985,  2.02016712],
       [ 2.94026449,  1.87738458],
       [ 2.69697298,  1.76526681],
       [ 2.43365933,  1.68299079],
       [ 2.15660813,  1.62745608],
       [ 1.87013656,  1.59613952],
       [ 1.57779106,  1.58562742],
       [ 1.28215676,  1.59227111],
       [ 0.98487932,  1.61403413],
       [ 0.6873184 ,  1.64964181],
       [ 0.39075617,  1.69822287],
       [ 0.09787946,  1.76152439],
       [-0.18338086,  1.84367406],
       [-0.44483933,  1.94783292],
       [-0.68082351,  2.07505473],
       [-0.88613218,  2.22568305],
       [-1.05603335,  2.39902882],
       [-1.18589622,  2.5936314 ],
       [-1.27169484,  2.80716598],
       [-1.31218493,  3.03598568],
       [-1.30991635,  3.27569155],
       [-1.27163002,  3.52201955],
       [-1.2041041 ,  3.77199408],
       [-1.11666039,  4.02352905],
       [-1.03833424,  4.28244499],
       [-0.98206846,  4.54023144],
       [-0.95621435,  4.79465699],
       [-0.96688587,  5.04215432],
       [-1.01976707,  5.27723085],
       [-1.11690466,  5.49310434],
       [-1.25685794,  5.68241772],
       [-1.43515869,  5.83815959],
       [-1.64517308,  5.95454609],
       [-1.8790759 ,  6.02770669],
       [-2.12878309,  6.05606375],
       [-2.38675032,  6.04035988],
       [-2.64656303,  5.98335852],
       [-2.90326983,  5.88930785],
       [-3.15345771,  5.76329879],
       [-3.39511937,  5.61065203],
       [-3.6273987 ,  5.43643588],
       [-3.85030438,  5.24516422],
       [-4.06445791,  5.04067325],
       [-4.27089938,  4.82613556],
       [-4.47092005,  4.60413076],
       [-4.6658745 ,  4.3766921 ],
       [-4.85701489,  4.14534934],
       [-5.04553421,  3.91132971],
       [-5.23249818,  3.67561542],
       [-5.41878692,  3.43905003],
       [-5.60490559,  3.20219883],
       [-5.78967803,  2.96458655],
       [-5.97183129,  2.72539336],
       [-6.14999212,  2.48378962],
       [-6.32270516,  2.23895528],
       [-6.48838604,  1.99006509],
       [-6.64536865,  1.73633251],
       [-6.79206284,  1.47711505],
       [-6.92723822,  1.21207631],
       [-7.05009841,  0.94121694],
       [-7.16022693,  0.6648322 ],
       [-7.25744812,  0.38343439],
       [-7.34163205,  0.09767032],
       [-7.41248286, -0.19173564],
       [-7.46935571, -0.48399087],
       [-7.51114295, -0.77819314],
       [-7.53626154, -1.07325914],
       [-7.54276339, -1.36786216],
       [-7.5285732 , -1.6604247 ],
       [-7.49182939, -1.94919947],
       [-7.43127076, -2.23244638],
       [-7.34659281, -2.50868195],
       [-7.23870966, -2.77695595],
       [-7.10989739, -3.0371022 ],
       [-6.96398216, -3.2899773 ],
       [-6.80402498, -3.53646926],
       [-6.63277759, -3.77756485],
       [-6.4521705 , -4.01404049],
       [-6.26255734, -4.24601235],
       [-6.06754571, -4.46708029],
       [-5.86514569, -4.67719261],
       [-5.65373737, -4.87422847],
       [-5.4324667 , -5.05671499],
       [-5.20039638, -5.22281448],
       [-4.95640713, -5.37013636],
       [-4.70062343, -5.49797203],
       [-4.43407139, -5.60700058],
       [-4.15837353, -5.6990602 ],
       [-3.87542624, -5.77682298],
       [-3.58708248, -5.84332978],
       [-3.29488993, -5.90145043],
       [-3.00007906, -5.95374245],
       [-2.70358625, -6.00234627],
       [-2.4061883 , -6.049126  ],
       [-2.10852342, -6.09559584],
       [-1.81090519, -6.14237928],
       [-1.51337559, -6.18979531],
       [-1.21570444, -6.23624963],
       [-0.91780181, -6.28108269],
       [-0.6194761 , -6.3222182 ],
       [-0.32083915, -6.35739275],
       [-0.02235895, -6.38407444],
       [ 0.27514302, -6.39914593],
       [ 0.57077322, -6.40076038],
       [ 0.86345899, -6.38645351],
       [ 1.15201571, -6.35391762],
       [ 1.43528603, -6.30134456],
       [ 1.71248445, -6.22831037],
       [ 1.98345605, -6.13608918],
       [ 2.24881265, -6.02765521],
       [ 2.50942974, -5.90613712],
       [ 2.76552286, -5.77250658],
       [ 3.01737473, -5.6276297 ],
       [ 3.26378391, -5.47214751],
       [ 3.4918041 , -5.30603948],
       [ 3.69613861, -5.12560249],
       [ 3.87153478, -4.93032733],
       [ 4.01276361, -4.72129195],
       [ 4.11366614, -4.50083195],
       [ 4.16826539, -4.27337232],
       [ 4.17169253, -4.04560773],
       [ 4.12207389, -3.82599121],
       [ 4.02073185, -3.62338207],
       [ 3.87183324, -3.44556298],
       [ 3.68174159, -3.29799543],
       [ 3.45815236, -3.18303264],
       [ 3.20905186, -3.09988484],
       [ 2.94136162, -3.04591287],
       [ 2.66067367, -3.01738312],
       [ 2.37135469, -3.00987291],
       [ 2.07655425, -3.01908432],
       [ 1.77869852, -3.04001237],
       [ 1.47888372, -3.0684348 ],
       [ 1.17883919, -3.09302532],
       [ 0.87872146, -3.11364628],
       [ 0.57856336, -3.13001044],
       [ 0.27838965, -3.14177771],
       [-0.02177934, -3.14857435],
       [-0.32192678, -3.14992989],
       [-0.62204455, -3.14586643],
       [-0.92213822, -3.13703231],
       [-1.22222329, -3.12423252],
       [-1.52232094, -3.10827475],
       [-1.82246155, -3.08960369],
       [-2.12273522, -3.06847117],
       [-2.4225829 , -3.04417008],
       [-2.7204407 , -3.01191938],
       [-3.01429415, -2.96714429],
       [-3.30131731, -2.90547907],
       [-3.57778948, -2.82298519],
       [-3.83833525, -2.71550499],
       [-4.07763453, -2.58153867],
       [-4.2903024 , -2.42105648],
       [-4.46950456, -2.23441773],
       [-4.60826758, -2.0240249 ],
       [-4.70088362, -1.79490138],
       [-4.74353796, -1.55431103],
       [-4.73467047, -1.31104976],
       [-4.67505652, -1.07455016],
       [-4.56760867, -0.85389751],
       [-4.41697018, -0.65686562],
       [-4.22898074, -0.48908688],
       [-4.01008253, -0.35349652],
       [-3.7667293 , -0.25019561],
       [-3.5048786 , -0.17678275],
       [-3.22978659, -0.12875378],
       [-2.94486821, -0.10257328],
       [-2.65301869, -0.09402233],
       [-2.35651919, -0.09909318],
       [-2.05737527, -0.11388648],
       [-1.75724128, -0.135599  ],
       [-1.45719237, -0.16158643],
       [-1.15733561, -0.18984557],
       [-0.85762539, -0.21977085],
       [-0.55801955, -0.25085463],
       [-0.25849524, -0.28275094],
       [ 0.04110391, -0.31383727],
       [ 0.34077605, -0.34418354],
       [ 0.640522  , -0.37381337]]

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
        
        # SPEED_DIFF_NO_REWARD = 1
        # SPEED_MULTIPLE = 1
        # speed_diff = abs(optimals[2]-speed)
        # if speed_diff <= SPEED_DIFF_NO_REWARD:
        #     # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
        #     # so, we do not punish small deviations from optimal speed
        #     speed_reward = 1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2
        # else:
        #     speed_reward = 0
        # speed_reward = speed_reward * SPEED_MULTIPLE
        # reward += speed_reward
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
        if heading2optimal_diff>40:
            steer_reward = 1e-3
        elif heading2optimal_diff>15:
            steer_reward = (1-(heading2optimal_diff/90))**2
        else:
            steer_reward = 1
        reward += steer_reward

        def calc_step_reward(params):
            steps = params['steps']
            progress = params['progress']
            TOTAL_NUM_STEPS = 270
            reward = 0
            if progress > (steps / TOTAL_NUM_STEPS) * 100 :
                reward += 1.0
            return float(reward)
        
        # step_reward_multiple=0.8
        # step_reward=calc_step_reward(params)*step_reward_multiple
        # reward+=step_reward
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
        print("dist_reward={:.3f} steer_reward={:.3f}".format(distance_reward,steer_reward))
        print("tot_reward={:.3f}".format(reward))
        # print("dist_reward={:.3f} steer_reward={:.3f} step_reward={:.3f}".format(distance_reward,steer_reward,step_reward))
        # print("speed_reward={:.3f} tot_reward={:.3f}".format(speed_reward,reward))

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