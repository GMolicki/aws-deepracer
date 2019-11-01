import math

def dotproduct(v1, v2):
  return sum((a*b) for a, b in zip(v1, v2))

def length(v):
  return math.sqrt(dotproduct(v, v))

def angle(v1, v2):
    v1l = length(v1)
    v2l = length(v2)
    if v1l == 0 or v2l == 0:
      return 180
    return math.acos(dotproduct(v1, v2) / (length(v1) * length(v2)))

def reward_function(params):
    '''
    Example of rewarding the agent to follow center line
    '''

    reward = 0
    track_width = params['track_width']
    distance_from_center = params['distance_from_center']
    steering = abs(params['steering_angle'])
    direction_stearing=params['steering_angle']
    speed = params['speed']
    steps = params['steps']
    progress = params['progress']
    all_wheels_on_track = params['all_wheels_on_track']
    ABS_STEERING_THRESHOLD = 15
    TOTAL_NUM_STEPS = 85
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    heading = params['heading']

    if not all_wheels_on_track:
        return float(1e-3);

    if progress == 100:
        reward = reward + 100

    Y_INDEX = 1;
    X_INDEX = 0;
    prev_point = waypoints[closest_waypoints[0]] #current point
    next_point = waypoints[closest_waypoints[1]] #next point
    track_direction = math.atan2((next_point[1] - prev_point[1]), (next_point[0] - prev_point[0])) #calculate the direction_steering in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
    track_direction = math.degrees(track_direction)
    direction_diff = abs(track_direction - heading)
    DIRECTION_THRESHOLD = 15.0
    SPEED_THRESHOLD = 2.25 
    reward_calculator=1
    if direction_diff > DIRECTION_THRESHOLD:
        reward_calculator=1-(direction_diff/50)
        if reward_calculator<0 or reward_calculator>1:
            reward_calculator = 0
        reward *= reward_calculator
# Penalize if the car goes off track        
    if not all_wheels_on_track:
       reward = reward - 1.0
    elif speed < SPEED_THRESHOLD:
# Penalize if the car goes too slow
        reward = reward - 2.0
    else:
# High reward if the car stays on track and goes fast
        reward = reward + 2.0
    return float(reward)


    waypoints = params['waypoints']
    closestWaypoint = waypoints[closest_waypoints[0]]
    secondClosestWaypoint = waypoints[min(closest_waypoints[1]+5, len(waypoints)-1)]
    yaw = float(params['heading'])
    yaw += steering
    carX = float(params['x'])
    carY = float(params['y'])
    carVector = [math.cos(math.radians(yaw)), math.sin(math.radians(yaw))]
    closestWaypointVector = [float(secondClosestWaypoint[0])-float(secondClosestWaypoint[0]),float(secondClosestWaypoint[1])-float(secondClosestWaypoint[1])]
    # Calculate 3 markers that are at varying distances away from the center line
#    marker_1 = 0.1 * track_width
#    marker_2 = 0.25 * track_width
#    marker_3 = 0.5 * track_width
    # Give higher reward if the car is closer to center line and vice versa
#    if distance_from_center <= marker_1:
#        reward = 1.0
#    elif distance_from_center <= marker_2:
#        reward = 0.7
#    elif distance_from_center <= marker_3:
#        reward = 1e-3
#    else:
#        reward = 1e-3  # likely crashed/ close to off track
    angleBetween = angle(carVector, closestWaypointVector)
    if angleBetween >= 45 and angleBetween <= 315:
        return 1e-3
    if angleBeetween >= 30 and angleBetween <= 330:
        return 0.7
    if angleBetween >= 15 and angleBetween <= 345:
        return 1.0
#    rewardMultiplier = math.radians(angleBetween)/(math.pi*2)
#    return float(reward*rewardMultiplier)
#import math
#def reward_function(params):
    '''
# Read all input parameters
    track_width = params['track_width']
    distance_from_center = params['distance_from_center']
    steering = abs(params['steering_angle'])
    direction_stearing=params['steering_angle']
    speed = params['speed']
    steps = params['steps']
    progress = params['progress']
    all_wheels_on_track = params['all_wheels_on_track']
    ABS_STEERING_THRESHOLD = 15
    TOTAL_NUM_STEPS = 85
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    heading = params['heading']
--------------------------------------------------------------------    #Set the reward reward_function
    reward = 1
 -------------------------------------------------------------------
#Reward if the car completes the track
   if progress == 100:
        reward = reward + 100
-------------------------------------------------------------------
#calculate the current & next waypoints
    prev_point = waypoints[closest_waypoints[0]]
    next_point = waypoints[closest_waypoints[1]]
--------------------------------------------------------------------    
#calculate the direction_steering in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
    track_direction = math.atan2((next_point[1] - prev_point[1]), (next_point[0] - prev_point[0])) 
    
#convert to degrees
    track_direction = math.degrees(track_direction)
-------------------------------------------------------------------    
# Calculate the difference between the track direction and the heading direction of the car
    direction_diff = abs(track_direction - heading)
    
# Penalize the reward if the difference is too large
    DIRECTION_THRESHOLD = 12.0
    SPEED_THRESHOLD = 2.25 
    
    reward_calculator=1
--------------------------------------------------------------------    
    if direction_diff > DIRECTION_THRESHOLD:
        reward_calculator=1-(direction_diff/50)
        if reward_calculator<0 or reward_calculator>1:
            reward_calculator = 0
        reward *= reward_calculator
--------------------------------------------------------------------        
# Penalize if the car goes off track        
    if not all_wheels_on_track:
       reward = reward - 1.0
    elif speed < SPEED_THRESHOLD:
# Penalize if the car goes too slow
        reward = reward - 2.0
    else:
# High reward if the car stays on track and goes fast
        reward = reward + 2.0
--------------------------------------------------------------------    
    return float(reward)


'''
