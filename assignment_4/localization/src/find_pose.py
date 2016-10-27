#!/usr/bin/env python

import roslib; roslib.load_manifest('localization')
from localization.bag import get_dict
from assignment_3.geometry import *
from assignment_4.laser import *
from math import pi
from tf.transformations import euler_from_quaternion
import argparse

# Parse Args
parser = argparse.ArgumentParser(description='Brute Force Pose Finder')
parser.add_argument('mapbag')
parser.add_argument('databag')
parser.add_argument('-resolution',  default=1, type=int)
parser.add_argument('-angles',      default=8, type=int)

args = parser.parse_args()

# Get Data From Bag Files
the_map = get_dict( args.mapbag )['/map']
test_files = get_dict( args.databag )
scan = test_files['/base_scan']
truth = test_files['/base_pose_ground_truth']

pose = truth.pose.pose
true_pos = pose.position.x, pose.position.y, euler_from_quaternion((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))[2]
print "True Position:", true_pos

angles = [float(i)/args.angles * 2 * pi for i in range(args.angles)]

best = None
best_score = -1E6

for x in range(0, the_map.info.width, args.resolution):
    for y in range(0, the_map.info.height, args.resolution):
        for theta in angles:
            ex_scan = expected_scan(x, y, theta, scan.angle_min, scan.angle_increment, len(scan.ranges), scan.range_max, the_map)
            score = scan_similarity(scan.ranges, ex_scan, scan.range_max)
            if score > best_score:
                best_score = score
                world = to_world(x, y, the_map.info.origin.position.x, the_map.info.origin.position.y, the_map.info.width, the_map.info.height, the_map.info.resolution)
                best = (world[0],world[1],theta)
                print "Current best:", best_score, best
print                 
print "Final Estimate:", best, best_score
print "True Position:", true_pos
