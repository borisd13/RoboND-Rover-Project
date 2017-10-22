import numpy as np
import cv2

# Identify pixels belonging to ground, rocks, and obstacles
def color_thresh(img, ground_thresh=(160, 160, 160), low_rock=(130, 100, 0), high_rock=(200, 170, 80)):

    # Detect ground based on a threshold
    ground = (img[:,:,0] > ground_thresh[0]) \
                & (img[:,:,1] > ground_thresh[1]) \
                & (img[:,:,2] > ground_thresh[2])

    # Detect rocks based on a range
    rock = (img[:,:,0] > low_rock[0]) \
                & (img[:,:,1] > low_rock[1]) \
                & (img[:,:,2] > low_rock[2]) \
                & (img[:,:,0] < high_rock[0]) \
                & (img[:,:,1] < high_rock[1]) \
                & (img[:,:,2] < high_rock[2])

    # Detect obstacles as points not detected previously and not completely black
    obstacle = ~(ground | rock) & (img[:,:,0] > 0)

    return ground, rock, obstacle

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # NOTE: camera image is accessible at Rover.img

    # Define used scale and word size
    scale = 10
    world_size = 200

    # 1) Define source and destination points for perspective transform
    dst_size = 5
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
                      [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
                      [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset], 
                      [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                      ])

    # 2) Apply perspective transform
    warped = perspect_transform(Rover.img, source, destination)

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    ground, rock, obstacle = color_thresh(warped)

    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    Rover.vision_image[:,:,0] = obstacle
    Rover.vision_image[:,:,1] = rock
    Rover.vision_image[:,:,2] = ground    
    Rover.vision_image *= 255

    # 5) Convert map image pixel values to rover-centric coords
    xpix_nav, ypix_nav = rover_coords(ground)
    xpix_rock, ypix_rock = rover_coords(rock)
    xpix_obs, ypix_obs = rover_coords(obstacle)

    # 6) Convert rover-centric pixel values to world coordinates
    xpos, ypos = Rover.pos
    x_world_nav, y_world_nav = pix_to_world(xpix_nav, ypix_nav, xpos, ypos, Rover.yaw, world_size, scale)
    x_world_rock, y_world_rock = pix_to_world(xpix_nav, ypix_nav, xpos, ypos, Rover.yaw, world_size, scale)
    x_world_obs, y_world_obs = pix_to_world(xpix_nav, ypix_nav, xpos, ypos, Rover.yaw, world_size, scale)

    # 7) Update Rover worldmap (to be displayed on right side of screen)
    # verify that roll and pitch are low to increase fidelity
    precision_pitch, precision_roll = 0.5, 0.8
    pitch = min(Rover.pitch, 360 - Rover.pitch)
    roll = min(Rover.roll, 360 - Rover.roll)
    if (pitch < precision_pitch) and (roll < precision_roll):
        Rover.worldmap[y_world_obs, x_world_obs, 0] += 1
        Rover.worldmap[y_world_rock, x_world_rock, 1] += 1
        Rover.worldmap[y_world_nav, x_world_nav, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(xpix_nav, ypix_nav)
    
    return Rover