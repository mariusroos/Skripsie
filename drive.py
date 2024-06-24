# Driving in my car...

import logging
from datetime import datetime
import time
from picamera2 import Picamera2
import sys
import logging
import cv2
import numpy as np
from PIL import Image
import yaml
import argparse
import libcamera
from numpy import ones, vstack
from numpy.linalg import lstsq
import movement

logging.basicConfig(format='{%(asctime)s, %(levelname)s, %(filename)s:%(lineno)d}: %(message)s', datefmt='%Y-%m-%d:%H:%M:%S')
logger = logging.getLogger('drive')
logger.setLevel(logging.DEBUG)

args_parser = argparse.ArgumentParser()
args_parser.add_argument("-v", action='store_true', help='Verbose logging')
args_parser.add_argument("-d", action='store_true', help='Debug output')
args_parser.add_argument('--size', help='Marker size')
args_parser.add_argument('--sleep', help='Sleep time in s')
args_parser.add_argument('--hres', help='hres')
args_parser.add_argument('--vres', help='vres')
args_parser.add_argument('--target', help='Target position to go to in (x,y)')
args_parser.add_argument('--world', help='YAML file containing the world')

# define some constants
# TODO: How did we get frame_scale?
frame_scale = 0.0002593         # multiply this scale with the distance to get the m/px offset
angle_scale=(67/90)
# px_offset = 0                   # this is the distance from the center of the aruco marker to the center of the image frame
# mpx = 0                         # multiply frame_scale with distance
# offset = 0                      # multiply mxp with px_offset to get actual offset in m form the centre of the frame
# offsetangle = 0                 # shift sin(offset/dist)

# this is where it's at
car_x = 0                       # x pos of car = marker_xy(0)+(dist*sin(90-yaw-offsetangle))
car_y = 0                       # y pos of car = marker_xy(1)-(dist*cos(90-yaw-offsetangle))

hres = 4605
vres = 2592
channels = 3
marker_size = 64
sleep_time = 10
deviation = 30                   # Deviating less than this when driving to our target we'll just go forward, anything more we'll turn
world = {}
obj_points = np.array([(-marker_size / 2,  marker_size / 2, 0),
                       ( marker_size / 2,  marker_size / 2, 0),
                       ( marker_size / 2, -marker_size / 2, 0),
                       (-marker_size / 2, -marker_size / 2, 0)])

# Parse arguments
args = args_parser.parse_args()
# pprint.pprint(args)
if args.v:
    verbose = True
    logger.setLevel(logging.INFO)
if args.d:
    verbose = True
    logger.setLevel(logging.DEBUG)
if args.size is not None:
    marker_size = int(args.size)
if args.sleep is not None:
    sleep_time = float(args.sleep)
if args.hres is not None:
    hres = int(args.hres)
if args.vres is not None:
    vres = int(args.vres)
if args.target is not None:
    target_pos = tuple(map(float, args.target.split(',')))
else:
    args_parser.print_help()
    sys.exit(-1)
if args.world is not None:
    world = yaml.load(open(args.world), Loader=yaml.FullLoader)
    print(world)
else:
    args_parser.print_help()
    sys.exit(-1)


logger.debug('marker_size changed to {}'.format(marker_size))
logger.debug('sleep_time changed to {}'.format(sleep_time))
obj_points = np.array([(-marker_size / 2,  marker_size / 2, 0),
                       ( marker_size / 2,  marker_size / 2, 0),
                       ( marker_size / 2, -marker_size / 2, 0),
                       (-marker_size / 2, -marker_size / 2, 0)])

aruco_dictionary = None
aruco_parameters =  None

cameras = {}
right = 0
left = 1

# get_camera()
# Parameters: camera id, hres and vres
# Returns: a camera object in JSON format
# Description:
# Create and configure a Picamera2 camera object, set config and load calibration
# ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def get_camera(camera_id, hres, vres):
    logger.info('Creating Picamera2 object for camera {}...'.format(camera_id))
    camera = Picamera2(camera_id)

    logger.info('Setting camera goodies...')
    config = camera.create_still_configuration(main={"size": (hres, vres)})
    config["transform"] = libcamera.Transform(vflip=False, hflip=False)
    camera.configure(config)
    camera.set_controls({"ExposureTime": 20000, "AfMode": libcamera.controls.AfModeEnum.Continuous})
    camera.resolution = (hres, vres)
    logger.debug('get_camera(): camera.camera_controls: {}'.format(camera.camera_controls))

    # Load camera calibration
    d = yaml.load(open('calibration-{}.yaml'.format(camera_id)), Loader=yaml.FullLoader)
    # logger.debug('get_camera(): d: {}'.format(d))
    intrinsic = np.array(d['camera_matrix'])
    distortion = np.array(d['dist_coeff'])

    camera.start()
    time.sleep(2)
    logger.info('Camera {} started'.format(camera_id))
    return {"ID": camera_id, "Camera": camera, "Intrinsic": intrinsic, "Distortion": distortion}

# snap()
# Parameters: camera object, hres and vres
# Returns: a grayscale, RGB image ready for cv2.aruco
# Description:
# Rhubarb
# ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def snap(camera, hres, vres):
    logger.debug('snap({})'.format(camera["ID"]))

    image = np.zeros((hres, vres, channels), dtype=np.uint8)
    req = camera["Camera"].capture_request()
    image = req.make_array("main")
    metadata = req.get_metadata()
    # logger.debug(metadata)
    req.release()

    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    return gray

# get_markers_and_pose()
# Parameters: camera object, a grayscale RGB image, dictionary of markers to add to
# Returns: more markers
# Description:
# Rhubarb
# ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def get_markers_and_pose(camera, image):
    logger.debug('get_markers_and_pose({})'.format(camera["ID"]))

    markers = []
    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(image, aruco_dictionary, parameters=aruco_parameters)
    if len(corners) > 0:
        logger.debug('get_markers_and_pose(): Camera {} has {} ids and {} corners and the corners are at {}'.format(camera["ID"], len(ids), len(corners), corners))

        logger.debug('get_markers_and_pose(): Camera {} ids: {}'.format(camera["ID"], ids))
        # Sort the markers by detected id
        _markers = {}
        for i in range(0, len(ids)):
            _markers[ids[i][0]] = i
        _sorted_markers = dict(sorted(_markers.items()))

        for marker_id, i in _sorted_markers.items():
            if world[marker_id]["Status"] == "Off":
                logger.debug('get_markers_and_pose(): i: {}, Marker_id: {} is switched off, ignoring...'.format(i, marker_id))
                continue
            logger.debug('get_markers_and_pose(): i: {}, Using marker_id: {}'.format(i, marker_id))
            success, rvec, tvec = cv2.solvePnP(obj_points, corners[i], camera["Intrinsic"], camera["Distortion"])
            tvec /= 1000.0
            logger.debug('get_markers_and_pose(): camera {} -> solvePnP(): marker {}: success: {}, tvec: {}, rvec: {}'.format(camera["ID"], marker_id, success, tvec, rvec))
            markers.append({
                'ID': marker_id,
                'Rotation': rvec,
                'Translation': tvec,
                'Corners': corners[i],
                'Source': camera["ID"]
            })
    else:
        logger.debug('get_markers_and_pose(): Camera {} detected no markers'.format(camera['ID']))

    return markers

# get_position()
# Parameter: a marker
# Returns: the x, y, position calculated from this marker
# Description:
# Rhubarb
# ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def get_position(marker):
    # logger.debug('get_position({}): {}'.format(marker['ID'], marker))
    logger.debug('get_position({})'.format(marker['ID']))

    x = 0
    y = 0
    id = marker['ID']
    tvec = marker['Translation']
    rvec = marker['Rotation']
    corners = marker['Corners']
    logger.debug("get_position(): tvec: {}, rvec: {}, corners: {}".format(tvec, rvec, corners))

    # distance from marker
    distance = np.sqrt(tvec[0][0] ** 2 + tvec[1][0] ** 2 + tvec[2][0] ** 2)
    logger.debug("get_position(): distance: {}".format(distance))

    # distance from the center of the aruco marker to the center of the image frame
    # TODO: explain 2200
    px_offset = corners[0][0][0] - 2200
    logger.debug("get_position(): px_offset: {}".format(px_offset))

    # multiply frame_scale with distance
    mpx = distance * frame_scale
    logger.debug("get_position(): mpx: {}".format(mpx))

    # multiply mpx with px_offset to get actual offset in m from the centre of the frame
    offset = mpx * px_offset
    logger.debug("get_position(): offset: {}".format(offset))

    # arcsin(offset/dist)
    offset_angle = np.arcsin(offset/distance)
    logger.debug("get_position(): offset_angle: {}".format(offset_angle))

    logger.debug("get_position(): Using marker {} x pos: {}, ypos: {}, orientation: {}".format(id, world[id]["x"], world[id]["y"], world[id]["Orientation"]))

    # TODO: angle_scale, WTH man
    x = world[id]["x"] + (distance * np.sin(np.deg2rad(world[id]["Orientation"]+90) - (rvec[2][0] * angle_scale) - offset_angle))
    y = world[id]["y"] - (distance * np.cos(np.deg2rad(world[id]["Orientation"]+90) - (rvec[2][0] * angle_scale) - offset_angle))
    # x = world[id]["x"] + (distance * np.sin(np.deg2rad(world[id]["Orientation"]) - rvec[2][0] - offset_angle))
    # y = world[id]["y"] - (distance * np.cos(np.deg2rad(world[id]["Orientation"]) - rvec[2][0] - offset_angle))

    orientation = np.mod(np.rad2deg(np.deg2rad(world[id]["Orientation"]) - (rvec[2][0] * angle_scale) + np.pi), 360)
    if orientation > 180:
        orientation -= 360

    logger.info('On camera {}, from marker {} I am at distance: {}, x: {}, y: {}, orientation: {}'.format(marker['Source'], id, distance, x, y, orientation))

    return (x, y, orientation)

# get_avg_position()
# Parameters: Dictionary of all detected makers
# Returns: Average x, y position from markers
# Description:
# Rhubarb
# ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def get_avg_position(markers):
    logger.debug('get_avg_position()')

    x = 0
    y = 0

    positions = []

    # get all positions
    for marker in markers:
        positions.append(get_position(marker))


    # get average position
    xs = []
    ys = []
    orientations = []
    for position in positions:
        xs.append(position[0])
        ys.append(position[1])
        orientations.append(position[2])

    logger.debug('get_avg_position(): xs: {}'.format(xs))
    logger.debug('get_avg_position(): ys: {}'.format(ys))
    logger.debug('get_avg_position(): orientations: {}'.format(orientations))

    x = np.average(xs)
    y = np.average(ys)
    orientation = np.average(orientations)

    logger.debug('get_avg_position(): x: {}'.format(x))
    logger.debug('get_avg_position(): y: {}'.format(y))
    logger.debug('get_avg_position(): orientation: {}'.format(orientation))

    return (x, y, orientation)

# where_to()
# Parameters: current location and target location
# Returns: tuple with movement for left and right wheel
# Description:
# Rhubarb
# ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def where_to(right, left, target):
    logger.debug('where_to(): from {}, {} to {}'.format(right, left, target))

    # Calculate the equation of the line through the average of right and 1

    if(right is not None and left is not None):
        x0 = np.average([right[0], left[0]])
        y0 = np.average([right[1], left[1]])
        if((left[2] < 0) and (right[2] > 0)): # car is facing left
            orientation = np.average([right[2], left[2] + 360])
            if orientation > 180:
                orientation -= 360
        else:
            orientation = np.average([right[2], left[2]])
    elif(right is not None and left is None):
        x0 = right[0]
        y0 = right[1]
        orientation = right[2] + 58
    elif(right is None and left is not None):
        x0 = left[0]
        y0 = left[1]
        orientation = left[2] - 58
    else:
        return False, None, None

    x1 = target[0]
    y1 = target[1]

    x_coords, y_coords = zip(*[(x0, y0), (x1, y1)])
    A = vstack([x_coords, ones(len(x_coords))]).T
    m, c = lstsq(A, y_coords, rcond=None)[0]

    distance = np.sqrt((x0 - x1) ** 2 + (y0 - y1) ** 2)
    direction = np.rad2deg(np.arctan(m))
    logger.debug('where_to(): direction before: {}'.format(direction))
    # If we're on the other side of the target add 180
    if(x0 >= x1):
        if(y0 <= y1):
            direction += 180
        else:
            direction -= 180

    logger.info('Driving from ({:.2f}, {:.2f}) to ({:.2f}, {:.2f}), a distance of {:.2f}m along {:.3f}x + {:.3f}. I am oriented {:.1f}deg and need to turn to {:.1f}deg'.format(x0, y0, x1, y1, distance, m, c, orientation, direction))

    # Now figure out which direction is the closest to turn
    if(np.abs(orientation - direction) < deviation):
        # Just go forward
        return True, distance, None
    else:
        # Turn
        return True, distance, (direction - orientation)


# drive()
# Parameters: driving in my car...
# Returns: boolean
# Description:
# Rhubarb
# ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def drive(chip, distance, direction, speed):
    logger.debug('move(chip, {}, {}, {})'.format(distance, direction, speed))

    if direction is not None:
        if direction < -180:
            direction += 360
        elif direction > 180:
            direction -= 360
        
        logger.debug('move(): {}'.format(direction))
        if(direction > 0): # left
            movement.step(chip, [1, 0, 0, 1], 100, np.abs(direction)/360, 100)
            # movement.step(chip, [1, 0, 0, 1], 100, .05, 100)
        else: # right
            movement.step(chip, [0, 1, 1, 0], 100, np.abs(direction)/360, 100)
            # movement.step(chip, [0, 1, 1, 0], 100, .05, 100)
    else:
        logger.debug('move(): straight')
        # movement.step(chip, [1, 0, 1, 0], 100, distance/.5*1.5, 100)
        movement.step(chip, [1, 0, 1, 0], 100, distance/.5*1.5, 100)

    return True

# speed()
# Parameters: distance now or steps
# Returns: speed now
# Description:
# Start slow, max out mid way and slow down again
# ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def speed(distance, step):
    logger.debug('speed({}, {})'.format(distance, step))
    if distance is not None and distance < .1:
        speed = distance/.1
    elif step < 10:
        speed = (step + 1)/ 10
    else:
        speed = 1

    return speed


# test_drive()
# Description:
# ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def test_drive(chip):

    time.sleep(.1)
    movement.step(chip, [1, 0, 0, 1], 100, .1, 50)
    time.sleep(.1)
    movement.step(chip, [0, 1, 1, 0], 100, .1, 50)
    time.sleep(.1)
    movement.step(chip, [1, 0, 0, 1], 100, .1, 50)
    time.sleep(.1)
    movement.step(chip, [0, 1, 1, 0], 100, .1, 50)

# main()
# Description:
# Main function
# ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    right_camera = get_camera(right, hres, vres)
    left_camera = get_camera(left, hres, vres)

    # ArUco goodies
    aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
    aruco_parameters =  cv2.aruco.DetectorParameters_create()

    chip = movement.init()
    test_drive(chip)

    step = 0
    stuck = 0
    driving = True
    while(driving): # step < 2):
        right_current_pos = None
        left_current_pos = None

        logger.debug('Capturing...')
        image = snap(right_camera, hres, vres)
        markers = get_markers_and_pose(right_camera, image)
        if(len(markers) > 0):
            right_current_pos = get_avg_position(markers)
            stuck = 0
        else:
            stuck += 1

        image = snap(left_camera, hres, vres)
        markers = get_markers_and_pose(left_camera, image)
        if(len(markers) > 0):
            left_current_pos = get_avg_position(markers)
            stuck = 0
        else:
            stuck += 1
        if stuck > 10:
            logger.info('I am not seeing any markers, so I am stuck')
            driving = False

        # Determine direction from here
        r, distance, direction = where_to(right_current_pos, left_current_pos, target_pos)
        if r == True:
            if distance > .05:
                # Drive...
                drive(chip, distance, direction, speed(distance, step))
            else:
                logger.info('You have arrived at your destination, please rate your driver')
                driving = False

        time.sleep(sleep_time)
        step += 1

    movement.close(chip)
    logger.info('Done')
