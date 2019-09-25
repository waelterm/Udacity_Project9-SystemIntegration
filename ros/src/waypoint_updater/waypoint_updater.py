#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
import math
from std_msgs.msg import Int32

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

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number
MAX_DECEL = .5


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        

        # Variable Declarations
        #self.pose = None
        self.base_lane = None
        self.stopline_wp_idx = -1
        self.x = None
        self.y = None
        #self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.previous_index = None
        self.traffic_waypoint = None
        
        # Subscriber Declarations
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        # Variable declarations
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.loop()
        
    def loop(self):
        rate = rospy.Rate(10) #Running at 50Hz
        while not rospy.is_shutdown():
            #Check if vehicle position and list of waypoints have been received at least once
            if self.x and self.base_lane and self.waypoint_tree: #self.x and self.y and self.base_waypoints and self.waypoint_tree:
                self.publish_waypoints()
                #closest_waypoint_ahead_index = self.get_closest_waypoint_idx()
                #self.publish_waypoints(closest_waypoint_ahead_index)
            rate.sleep()
            
    def get_closest_waypoint_idx(self):
        reference_point = [self.x, self.y]
        number_of_nearest_neighbors = 1
        search_range = 50
        if self.previous_index is not None:
            self.waypoint_tree = KDTree(self.waypoints_2d[self.previous_index-search_range:self.previous_index+search_range])
            nn_distance, nn_index = self.waypoint_tree.query(reference_point, number_of_nearest_neighbors)
            nn_index = nn_index + self.previous_index - search_range
	else:
	    nn_distance, nn_index = self.waypoint_tree.query(reference_point, number_of_nearest_neighbors)

        self.previous_index = nn_index
        nearest_waypoint = self.waypoints_2d[nn_index]
        previous_waypoint = self.waypoints_2d[nn_index-1]
        road_direction_vector = np.array([nearest_waypoint[0] - previous_waypoint[0], nearest_waypoint[1] - previous_waypoint[1]])
        vehicle_waypoint_direction = np.array([nearest_waypoint[0] - self.x, nearest_waypoint[1]-self.y])
        # The dot product gives information about the angle of the two vectors.
        # If the dot product is 0, the angles are perpendiculat
        # If the value is positive , the angles generally point in a similar direction (-90deg to 90 deg)
        # If the value is negative, the angles generally point in the opposite direction( 90 to 270 deg)
        dot_product = np.dot(road_direction_vector, vehicle_waypoint_direction)
        # If the dot product is negative, it means that the vehicle to waypoint vector points in the opposite direction of the road. This means the point is behind the vehicle
        if dot_product < 0:
            closest_waypoint_ahead_index = nn_index +1
        else:
            closest_waypoint_ahead_index = nn_index
        return closest_waypoint_ahead_index
    
    def publish_waypoints(self ):
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)
        
    def generate_lane(self):
        lane = Lane()

        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_lane.waypoints[closest_idx:farthest_idx]

        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints = base_waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)
        return lane

    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose
            stop_idx = max(self.stopline_wp_idx - closest_idx -2, 0)
            dist = self.distance(waypoints, i, stop_idx)
            vel = math.sqrt(2* MAX_DECEL*dist)
            if vel <1.0:
                vel = 0.0
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)
        return temp

    def pose_cb(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        
        # TODO: Implement
        pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_lane = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y]for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        print(msg)
        self.stopline_wp_idx = msg.data
        

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


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

