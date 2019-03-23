#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int32
import numpy as np
from copy import deepcopy

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 100 # 200 Default; Number of waypoints we will publish. You can change this number

class WaypointUpdater(object):

    def __init__(self):
        rospy.init_node('waypoint_updater',log_level = rospy.INFO)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = []
        self.final_waypoints = []
        self.index_closest_waypoint = []
        self.position = []
        self.traffic_waypoint_index=-1 # -1 means no red traffic light ahead; use -1 for reset of speed profile when traffic light turns green
        self.speed_adapted_base_waypoints=[]

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
            self.set_final_waypoints()
            lane = Lane()
            lane.waypoints=self.final_waypoints
            self.final_waypoints_pub.publish(lane)
        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        self.position=msg.pose.position
        self.get_closest_waypoint(msg)


    def waypoints_cb(self, msg):
        self.base_waypoints = msg.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic_waypoint_index=msg.data
        self.set_speed_adapted_base_waypoints()

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def get_closest_waypoint(self, current_pose):
        min_dist = 999999
        index_min_dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)
        for i in range(len(self.base_waypoints)):
            dist = dl(self.base_waypoints[i].pose.pose.position, current_pose.pose.position)
            if (dist < min_dist):
                min_dist = dist
                index_min_dist = i
        _, _, current_yaw = euler_from_quaternion(
            [current_pose.pose.orientation.x,current_pose.pose.orientation.y,
             current_pose.pose.orientation.z,current_pose.pose.orientation.w])

        heading = math.atan2(
            (self.base_waypoints[index_min_dist].pose.pose.position.x - current_pose.pose.position.y),
            (self.base_waypoints[index_min_dist].pose.pose.position.y - current_pose.pose.position.x)
        )
        if abs(current_yaw - heading) > math.pi / 4:
            self.index_closest_waypoint = index_min_dist + 1
        else:
            self.index_closest_waypoint = index_min_dist

        self.index_closest_waypoint = index_min_dist

    def set_final_waypoints(self):
        number_of_rear_waypoints=0
        if not self.index_closest_waypoint:
            self.final_waypoints = []
        else:
            if self.speed_adapted_base_waypoints:
                self.final_waypoints=self.speed_adapted_base_waypoints[(self.index_closest_waypoint-number_of_rear_waypoints):(self.index_closest_waypoint+LOOKAHEAD_WPS)]
            else:
                self.final_waypoints=self.base_waypoints[(self.index_closest_waypoint-number_of_rear_waypoints):(self.index_closest_waypoint+LOOKAHEAD_WPS)]

    def set_speed_adapted_base_waypoints(self):
        # init with base_waypoints
        if self.base_waypoints:
            if not self.speed_adapted_base_waypoints or self.traffic_waypoint_index ==-1:
                self.speed_adapted_base_waypoints=deepcopy(self.base_waypoints)
                if self.traffic_waypoint_index ==-1:
                    rospy.logwarn("green light detected: reset to base_waypoints speed profile")
                else:
                    rospy.logwarn("initalize with base_waypoints speed profile")

        # set speed profile as linear interpolattion before traffic light
        number_of_interpolation_points=80
        if self.traffic_waypoint_index >=0:
            start_index=max(0, self.traffic_waypoint_index-number_of_interpolation_points)
            start_speed=self.base_waypoints[start_index].twist.twist.linear.x
            end_speed=0
            speed_range=[start_speed,end_speed]
            index_range=[start_index,self.traffic_waypoint_index]
            rospy.logwarn("red light detected %s waypoints ahead. Set speed profile for waypoints with idx from %s to %s", self.traffic_waypoint_index - self.index_closest_waypoint, start_index,self.traffic_waypoint_index)
            # linear speed interpolation
            for i in range(start_index, self.traffic_waypoint_index):
                self.speed_adapted_base_waypoints[i].twist.twist.linear.x=np.interp(i,index_range,speed_range)
            # ensure standstill: if red light ahead, set speed for all waypoints behind the red light to zero
            if self.traffic_waypoint_index > self.index_closest_waypoint:
                for i in range(self.traffic_waypoint_index, len(self.speed_adapted_base_waypoints)):
                    self.speed_adapted_base_waypoints[i].twist.twist.linear.x=0


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
