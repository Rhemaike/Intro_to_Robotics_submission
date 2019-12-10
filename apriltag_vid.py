#!/usr/bin/env python

# import packages
from __future__ import division
from __future__ import print_function
from argparse import ArgumentParser
import cv2
import apriltag
import time
import math
from math import degrees, atan2, sqrt
from multiprocessing import Process, Queue, Manager

scale_x =  1854 / 640  # 540 mm / 640 pixels
scale_y = 1384 / 480  # 540 mm / 489 pixels
def orientation(p1, p2,r,c):
    p1x, p1y = p1[0], p1[1]
    p2x, p2y = p2[0], p2[1]
    np1x = p1x - int(c/2)
    np1y = int(r/2) - p1y

    np2x = p2x - int(c/2)
    np2y = int(r/2) - p2y

    npy,npx = np2x - np1x, np2y - np1y
    theta = atan2(npy,npx)
    return theta

def change_grid(center, r,c):
    x = center[0]
    y = center[1]

    nx = x - int(c/2)
    ny = int(r/2) - y

    # print("y {}, rhalf {}".format(y, int(r/2)))

    # print("nx {} ny {}".format(nx,ny))

    return nx,ny



def tracker(qlist, ids, goals, loc, obs, ob_ids, debug = False):

    parser = ArgumentParser(
        description='test apriltag Python')

    parser.add_argument('device_or_movie', metavar='INPUT', nargs='?', default=1)

    apriltag.add_arguments(parser)

    options = parser.parse_args()

    cap = cv2.VideoCapture(2)

    # window = 'Camera'
    win2 = "bigger_image"
    cv2.namedWindow(win2)
    # cv2.namedWindow(window)

    dic = {}
    dic2 = {}
    detector = apriltag.Detector(options, searchpath=apriltag._get_demo_searchpath())
    got_detect  = {}
    got_detect2 = {}
    while True:
        # print("visual man loc", loc)
        success, frame = cap.read()
        if not success:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        r,c = gray.shape

        
        detections, dimg = detector.detect(gray, return_image=True)

        num_detections = len(detections)
        if debug:
            print(c, r)
            print('Detected {} tags.\n'.format(num_detections))

        for i in ids:
            got_detect[i] = False

        for i, detection in enumerate(detections):
            # print('Detection {} of {}:'.format(i+1, num_detections))
            # print()
            # print(detection.tostring(indent=2))
            # print(pose)

            p1 = detection.corners[0]
            p2 = detection.corners[1]
            location = detection.center
            tid = detection.tag_id
            # print("detection_id is", tid)
            got_detect[tid] = True
            orient = orientation(p1,p2,r,c)

            p = detection.center
            
            x,y = p[0], p[1]
            x,y = change_grid((x,y),r,c)
            x,y = x*scale_x, y*scale_y


            if debug:
                print(p1,p2)
                print(detection.corners)
                print("center is", p)
                print("x and y", x, y, "orientation:", degrees(orientation(p1,p2,r,c)), "tid is", tid)


            if tid in ids:
                # print("found id",tid)
                dic[tid] = (x, y, orient, 1)
                # loc.value[tid] = (x, y, orient,1)
                # print("x and y and orient", x,y,orient)
                got_detect[tid] = True
                # print("loc.value", loc.value, tid)
                if debug:
                    print("stored in", tid)

            if tid in ob_ids:
                # print("**************************")
                # print("Found obstacle {}".format(tid))
                # print("**************************")
                p =  detection.center
                x,y = p[0], p[1]
                x,y = change_grid((x,y),r,c)
                x,y = x*scale_x, y*scale_y
                dic2[tid] = (x,y)
                got_detect2[tid] = True


            # print(theta_x, theta_y,theta_z)

        for i in ids:
            if not got_detect[i]:
                dic[i] = (goals[i][0],goals[i][1],0,0)
                # loc.value[i] = ((goals[i][0],goals[i][1],0))
        default = {12:(200,-210), 19:(-320,10,)}
        for i in ob_ids:
            if not got_detect2[i]:
                dic2[i] = default[i]
                # loc.value[i] = ((goals[i][0],goals[i][1],0))

        loc.value = dic
        obs.value = dic2
        # print("loc.value", loc.value)

        overlay = frame // 2 + dimg[:, :, None] // 2

        # cv2.imshow(window, overlay)
        scale_percent_r = 160
        scale_percent_c = 150
        width = int(c * scale_percent_c / 100)
        height = int(r * scale_percent_r / 100)
        dim = (width, height) 
        resized = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)
        cv2.imshow(win2, resized)
        k = cv2.waitKey(1)

        if k == 27:
            break

if __name__ == '__main__':
    # ids = [7,6]
    ids = [1]
    # goals = {7:(-500,0,0,0), 6:(500,0,0,0)}
    goals = {1:(-500,0,0,0)}
    qlist = {}
    m = Manager()
    loc = m.Value('loc',{})
    o = Manager()
    obs = o.Value('obs',{12:(200,-210), 19:(-320,10)})
    for i in ids:
        q = Queue()
        qlist[i] = q

    ob_ids = [12,19]
    tracker(qlist, ids, goals, loc,obs, ob_ids, True)