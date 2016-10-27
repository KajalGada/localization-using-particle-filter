#!/usr/bin/env python

import roslib; roslib.load_manifest('localization')
from localization import *
from localization.bag import get_dict
from assignment_3.geometry import *
from assignment_4.laser import *
from assignment_4.particle import *
from math import pi
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import argparse
import copy

import rospy
from sensor_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *

# Parse Args
parser = argparse.ArgumentParser(description='Hill Climbing Pose Finder')
parser.add_argument('mapbag')
parser.add_argument('databag')
parser.add_argument('-particles',  default=100, type=int)
parser.add_argument('-iterations', default=5, type=int)
parser.add_argument('-ros', action='store_true')

args = parser.parse_args()

# Get Data From Bag Files
the_map = get_dict( args.mapbag )['/map']
test_files = get_dict( args.databag )
scan = test_files['/base_scan']
best_scan = copy.deepcopy(scan)
truth = test_files['/base_pose_ground_truth']

pose = truth.pose.pose
true_pos = pose.position.x, pose.position.y, euler_from_quaternion((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))[2]
print "True Position:", true_pos

if args.ros:
    rospy.init_node('hill_climb')
    mpub = rospy.Publisher('/map', OccupancyGrid, latch=True, queue_size=10)
    mpub.publish(the_map)
    papub = rospy.Publisher('/poses', PoseArray, queue_size=10)
    
    tposepub = rospy.Publisher('/truth', PoseStamped, latch=True, queue_size=10)
    truth = PoseStamped()
    truth.header.frame_id = '/map'
    truth.pose = apply(to_pose, true_pos)
    
    posepub = rospy.Publisher('/estimate', PoseStamped, queue_size=10)
    estimate = PoseStamped()
    estimate.header.frame_id = '/map'
    
    laserpub_true = rospy.Publisher('/base_scan', LaserScan, latch=True, queue_size=10)
    laserpub_expected = rospy.Publisher('/base_scan_expected', LaserScan, latch=True, queue_size=10)
    
    br = tf.TransformBroadcaster()

    rospy.sleep(1)
    tposepub.publish(truth)
    

pa = PoseArray()
pa.header.frame_id = '/map'

particles = []
for i in range(args.particles):
    particles.append( random_particle(the_map) )

# print particles

for iteration in range(args.iterations):
    print 'Iteration:' ,iteration
    if args.ros:
        pa.poses = []
        for x,y,theta in particles:
            pa.poses.append(to_pose(x,y,theta))
        pa.header.stamp = rospy.Time.now()    
        papub.publish(pa)     
    
    scores = []
    scans = [] 
    for x,y,theta in particles:
        result = to_grid(x,y, the_map.info.origin.position.x, the_map.info.origin.position.y, the_map.info.width, the_map.info.height, the_map.info.resolution)
        # print 'result: ' ,result
        if not result:
            continue
        else:
            mx, my = result    
        ex_scan = expected_scan(mx, my, theta, scan.angle_min, scan.angle_increment, len(scan.ranges), scan.range_max, the_map)
        score = scan_similarity(scan.ranges, ex_scan, scan.range_max)
        scores.append( (score, (x,y,theta) ))
        scans.append(ex_scan)
        
        if rospy.is_shutdown():
            break

    length = len(scores)
    print 'length' ,length
    best_index = sorted(range(len(scores)), key=lambda k: scores[k])[-1]
    best_score, best_estimate = scores[best_index]
    best_scan.ranges = copy.deepcopy(scans[best_index])
    print "Best estimate (%d):"%iteration, best_estimate, best_score

    if args.ros:    
        estimate.pose = apply(to_pose, best_estimate)
        posepub.publish(estimate)
    
        publish_update(laserpub_true, scan, br, best_estimate)
        publish_update(laserpub_expected, best_scan, br, best_estimate)

    # Run debug function
    debug_call(scores, the_map)

    particles = resample( scores, args.particles )

    if rospy.is_shutdown():
        break

print "True Position:", true_pos
if args.ros:
    rospy.spin()
