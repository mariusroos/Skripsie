#!/usr/bin/env python
import os
import numpy as np
import cv2
import os
import argparse
import yaml
import pickle
from glob import glob
import time
from picamera2 import Picamera2, Preview
from pathlib import Path

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Calibrate camera using a video of a chessboard or a sequence of images.') 
    parser.add_argument('input',nargs="?", help='input video file or glob mask')
    parser.add_argument('out',nargs="?",help='output calibration yaml file')
    parser.add_argument('--debug_dir',nargs="?", help='path to directory where images with detected chessboard will be written',
                        default='./pictures')
    parser.add_argument('--output_dir',nargs="?",help='path to directory where calibration files will be saved.',default='./calibrationFiles')
    parser.add_argument('-c', '--corners',nargs="?", help='output corners file', default=None)
    parser.add_argument('-fs', '--framestep',nargs="?", help='use every nth frame in the video', default=20, type=int)
    parser.add_argument('--height',nargs="?", help='Height in pixels of the image',default=480,type=int)
    parser.add_argument('--width',nargs="?", help='Width in pixels of the image',default=640,type=int)
    parser.add_argument('--mm',nargs="?",help='Size in mm of each square.',default=22,type=int)
    parser.add_argument('--camera',nargs="?",help='Camera you want to calibrate',default=0,type=int)
    # parser.add_argument('--figure', help='saved visualization name', default=None)
    args = parser.parse_args()

    pattern_size = (9, 6)
    pattern_points = np.zeros((np.prod(pattern_size), 3), np.float32)
    pattern_points[:, :2] = np.indices(pattern_size).T.reshape(-1, 2)
    # pattern_points *= square_size

    obj_points = []
    img_points = []
    h, w = args.height, args.width
    # source.set(cv2.CAP_PROP_FRAME_HEIGHT,h)
    # source.set(cv2.CAP_PROP_FRAME_WIDTH,w)
    
    i = -1
    image_count=0
    image_goal=136

    root = Path(__file__).parent.absolute()
    directory = root.joinpath("aruco_data/{}".format(args.camera))

    jpeg_data_list = []

    # Iterate over files in the directory
    for filename in os.listdir(directory):
        if filename.endswith(".jpg"):
            # Construct the full path to the file
            file_path = os.path.join(directory, filename)
            jpeg_data = cv2.imread(file_path)
            jpeg_data_list.append(jpeg_data)


    i = 0
    # while True:
    while i < len(jpeg_data_list):
        print('true: i {}'.format(i))
        # if False: # isinstance(source, list):
        img = jpeg_data_list[i]
        # print('imshow()...')
        # cv2.imshow('Image',img)
        time.sleep(0.1)
        # print('cool()...')

        # key = cv2.waitKey(1) & 0xFF
        # if key == ord('q'):
            # break
        print('Searching for chessboard in frame ' + str(i) + '...'),
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        h, w = img.shape[:2]
        found, corners = cv2.findChessboardCorners(img, pattern_size, flags=cv2.CALIB_CB_FILTER_QUADS)
        if found:
            term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, args.mm, 0.1)
            cv2.cornerSubPix(img, corners, (5, 5), (-1, -1), term)
            image_count=image_count+1
            print ('found')
            # if image_count==image_goal:
                # break
        if args.debug_dir:
            img_chess = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            cv2.drawChessboardCorners(img_chess, pattern_size, corners, found)
            cv2.imwrite(os.path.join(args.debug_dir, '%04d.png' % i), img_chess)
        if not found:
            print ('not found')
            # continue
        else:
            img_points.append(corners.reshape(1, -1, 2))
            obj_points.append(pattern_points.reshape(1, -1, 3))
        i += 1

        print ('ok')

    if args.corners:
        with open(args.corners, 'wb') as fw:
            pickle.dump(img_points, fw)
            pickle.dump(obj_points, fw)
            pickle.dump((w, h), fw)
        

    # print('\nPerforming calibration...')
    print('\nPerforming calibration..., len(obj_points): {}, len(img_points): {}'.format(len(obj_points), len(img_points)))
    rms, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, (w, h), None, None)
    print ("RMS:", rms)
    print ("camera matrix:\n", mtx)
    print ("distortion coefficients: ", dist.ravel())

    # # fisheye calibration
    # rms, camera_matrix, dist_coefs, rvecs, tvecs = cv2.fisheye.calibrate(
    #     obj_points, img_points,
    #     (w, h), camera_matrix, np.array([0., 0., 0., 0.]),
    #     None, None,
    #     cv2.fisheye.CALIB_USE_INTRINSIC_GUESS, (3, 1, 1e-6))
    # print "RMS:", rms
    # print "camera matrix:\n", camera_matrix
    # print "distortion coefficients: ", dist_coefs.ravel()

    # calibration = {'rms': rms, 'camera_matrix': camera_matrix.tolist(), 'dist_coefs': dist_coefs.tolist() }

    # ##OUTPUT DIRECTORIES
    # file1 = args.output_dir + "/cameraMatrix.txt"
    # np.savetxt(file1,camera_matrix,delimiter=',')
    # file2 = args.output_dir + "/cameraDistortion.txt"
    # np.savetxt(file2,dist_coefs,delimiter=',')

    print("Camera matrix is \n", mtx, "\n And is stored in calibration.yaml file along with distortion coefficients : \n", dist)
    data = {'camera_matrix': np.asarray(mtx).tolist(), 'dist_coeff': np.asarray(dist).tolist()}
    with open("calibration-{}.yaml".format(args.camera), "w") as f:
        yaml.dump(data, f)

