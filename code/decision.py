import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if clear_path_ahead(Rover):  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = target_direction(Rover)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            else:
                # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving then do something else
            else:
                # Now we're stopped and we have vision data to see if there's a path forward
                if not clear_path_ahead(Rover):
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15  # Go towards the right since we are stopped and want to follow left wall
                # If we're stopped but see sufficient navigable terrain in front then go!
                else:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = target_direction(Rover)
                    Rover.mode = 'forward'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover


def clear_path_ahead(Rover):
    # Verify if the path ahead is clear

    # limit to direct accessible ground (within angle and close)
    max_distance = 4
    max_angle = 10
    valid_idx = (-max_angle < (Rover.nav_angles * 180/np.pi)) & ((Rover.nav_angles * 180/np.pi)< max_angle) &\
                (Rover.nav_dists < max_distance * 10)
    valid_idx = np.sum(valid_idx)
    print('Clear path ahead:', valid_idx)

    # verify if path is clear based on Rover mode
    if Rover.mode == 'forward':
        return valid_idx >= Rover.stop_forward
    else:
        return valid_idx >= Rover.go_forward


def target_direction(Rover):
    # Define target direction for Rover

    # limit to direct accessible ground (within angle) and not too far
    low_angle, high_angle = -30, 40     # to bias the rover to go towards a side and not visit same places
    distance_inc = 1 * 10     # increment of distance intervals to calculate average direction
    intervals = 6
    target_directions = []

    # iterate over interval of distances so we correctly weigh each interval
    for i in range(intervals):
        dist_low, dist_high = distance_inc * i, distance_inc * (i + 1)        
        valid_idx = (low_angle < (Rover.nav_angles * 180/np.pi)) & ((Rover.nav_angles * 180/np.pi)<high_angle) &\
                        (Rover.nav_dists < dist_high) & (Rover.nav_dists >dist_low)
        if valid_idx.any():
            valid_nav_angles = Rover.nav_angles[valid_idx]
            target_directions.append(np.mean(valid_nav_angles * 180/np.pi))

    # Take the mean angle, multiply by a scaling factor (to increase effect of decision) and add bias
    scale = 1.5  # to increase effect of our decision
    target_angle = np.mean(target_directions) * scale
    print("Target angle: ", target_angle)
    return np.clip(target_angle, -15, 15)
