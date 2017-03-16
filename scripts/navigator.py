#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
import numpy as np
import matplotlib.pyplot as plt
import tf
from std_msgs.msg import Float32MultiArray
from astar import AStar, StochOccupancyGrid2D, StochOccupancyGrid2D
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

#     Subscribes to controller: either too close to wall or has been to desired tag
#     Publishes to list of waypoint nodes for a trajectory between two tags

class Navigator:

    def __init__(self):
        rospy.init_node('navigator', anonymous=True)

        self.plan_resolution = 0.10
        self.plan_horizon = 15

        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0
        self.map_origin = [0,0]
        self.map_probs = []
        self.occupancy = None

        self.current_path = Path() #None

        self.nav_sp = None
        self.execution_mode = False

        self.trans_listener = tf.TransformListener()

        rospy.Subscriber("map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("map_metadata", MapMetaData, self.map_md_callback)

        rospy.Subscriber("mission_mode", Float32MultiArray, self.nav_sp_callback)

        self.nav_path_pub = rospy.Publisher('/turtlebot_mission_control/path_goal', Path, queue_size=10)
        self.wp_node_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

    def map_md_callback(self,msg):
        self.map_width = msg.width
        self.map_height = msg.height
        self.map_resolution = msg.resolution
        self.map_origin = (msg.origin.position.x,msg.origin.position.y)

    def map_callback(self,msg):
        self.map_probs = msg.data
        if self.map_width>0 and self.map_height>0 and len(self.map_probs)>0:
            self.occupancy = StochOccupancyGrid2D(self.map_resolution,
                                                  self.map_width,
                                                  self.map_height,
                                                  self.map_origin[0],
                                                  self.map_origin[1],
                                                  int(self.plan_resolution / self.map_resolution) * 2,
                                                  self.map_probs)

    def nav_sp_callback(self,msg):
        if msg.data[0] == 1.0:
            self.execution_mode = True
            if msg.data[1] != self.nav_sp:
                self.nav_sp = (msg.data[1][0],msg.data[1][1],msg.data[1][2])
                self.update_path()

        self.nav_path_pub.publish(self.current_path)


    def update_path(self):
        try:
            (robot_translation,robot_rotation) = self.trans_listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
            self.has_robot_location = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            robot_translation = (0,0,0)
            robot_rotation = (0,0,0,1)
            self.has_robot_location = False

        if self.occupancy and self.has_robot_location:
            state_min = (-int(round(self.plan_horizon)), -int(round(self.plan_horizon)))
            state_max = (int(round(self.plan_horizon)), int(round(self.plan_horizon)))
            x_init = (int(round(robot_translation[0])), int(round(robot_translation[1])))
            x_goal = (int(round(self.nav_sp[0])), int(round(self.nav_sp[1])))
            astar = AStar(state_min,state_max,x_init,x_goal,self.occupancy,self.plan_resolution)

            rospy.loginfo("Computing navigation plan")
            if astar.solve():
                path_msg = Path()
                path_msg.header.frame_id = 'map'
                for idx, state in enumerate(astar.path):
                    pose_st = PoseStamped()
                    pose_st.pose.position.x = state[0]
                    pose_st.pose.position.y = state[1]
                    pose_st.header.frame_id = 'map'
                    path_msg.poses.append(pose_st)

                    if idx == len(astar.path)-1:
                        self.wp_node_pub.publish(pose_st)

                self.current_path = path_msg
            else:
                rospy.logwarn("Could not find path")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    nav = Navigator()
    nav.run()
