#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray, Float32MultiArray, String
from geometry_msgs.msg import PoseStamped
import tf
import numpy as np

def pose_to_xyth(pose):
    th = tf.transformations.euler_from_quaternion((pose.orientation.x,
                                                   pose.orientation.y,
                                                   pose.orientation.z,
                                                   pose.orientation.w))[2]
    return [pose.position.x, pose.position.y, th]


def wrapToPi(a):
    b = a
    for i in range(len(a)):
        if a[i] < -np.pi or a[i] > np.pi:
            b[i] = ((a[i]+np.pi) % (2*np.pi)) - np.pi
    return b


class Supervisor:

    def __init__(self):
        rospy.init_node('turtlebot_supervisor', anonymous=True)
        self.trans_listener = tf.TransformListener()
        self.trans_broad = tf.TransformBroadcaster()
        self.bot_pose=np.array([0., 0., 0.])# x, y, th

        #relevant i execution phase
        self.been_at=[]
        self.next_goal=[]
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)    # rviz "2D Nav Goal"

        self.mission=[1,2,3]
        self.waypoint_locations = {}    # dictionary that caches the most updated locations of each mission waypoint
        self.waypoint_offset = PoseStamped()
        self.waypoint_offset.pose.position.z = .4    # waypoint is located 40cm in front of the AprilTag, facing it
        quat = tf.transformations.quaternion_from_euler(0., np.pi/2, np.pi/2)
        self.waypoint_offset.pose.orientation.x = quat[0]
        self.waypoint_offset.pose.orientation.y = quat[1]
        self.waypoint_offset.pose.orientation.z = quat[2]
        self.waypoint_offset.pose.orientation.w = quat[3]

        self.mode_pub = rospy.Publisher('/mission_mode', Float32MultiArray ,queue_size=10)
        self.flag=0
        #self.tag_pub = rospy.Publisher('/next_tag', Float32MultiArray ,queue_size=10)

    def rviz_goal_callback(self, msg):
        self.next_goal=pose_to_xyth(msg.pose)

    def get_mission(self):
        rospy.Subscriber('/mission', Int32MultiArray, self.mission_callback)

    def mission_callback(self, msg):
        self.mission=msg.data
        rospy.loginfo(msg.data)
		
    def update_waypoints(self):
        for tag_number in self.mission:
            try:
                self.waypoint_offset.header.frame_id = "/tag_{0}".format(tag_number)
                self.waypoint_locations[int(tag_number)] = pose_to_xyth(self.trans_listener.transformPose("/map", self.waypoint_offset))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

    def find_bot(self):
        try:
            (trans, rot)= self.trans_listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
            euler=tf.transformations.euler_from_quaternion(rot)
            self.bot_pose=np.array([trans[0], trans[1], wrapToPi(euler[2])])

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException): pass

    def check_mode(self):
        all_tags_seen=True
        if not (tag in self.waypoint_locations for tag in self.mission) : all_tags_seen=False
        return all_tags_seen

    def run(self, thresh):
        rate = rospy.Rate(1) # 1 Hz, change this to whatever you like
        while not rospy.is_shutdown():
            self.find_bot()
            dist=1#np.linalg.norm([np.sum([np.array(self.bot_pose[:2]), -np.array(self.next_goal[:2])], axis=0)) #euclidian distance
            if dist<thresh:
                rospy.logwarn('passed checkpoint')
                if all(self.next_goal==coord for tag_nbrs, coord in zip(self.waypoint_loctaions.keys(), self.waypoint_locations.values())):
                     tag=tag_nbrs
                self.been_at.append(int(tag))
            else:
                rospy.logwarn('on the way')
                
            msg=Float32MultiArray()
            tag=self.mission[len(self.been_at)-1]
            msg.data=[1, self.waypoint_locations[tag]]

            return msg
            rate.sleep()

    def explore(self):
        rate = rospy.Rate(1) # 1 Hz, change this to whatever you like
        while not rospy.is_shutdown():
            self.update_waypoints()
            if self.check_mode():
                self.flag=1
                loc= self.waypoint_locations[self.mission[0]] #first location to go to, ie first tag of mission execution
            else: 
                self.flag=0
                loc=[0,0,0]

            msg=Float32MultiArray()
            msg.data=[self.flag, loc]
            return msg

            rate.sleep()

if __name__ == '__main__':
    sup = Supervisor()
    sup.get_mission()
    flag=self.flag
    if flag==0: #exploration phase, no navigator
        msg=sup.explore()
        self.mode_pub.publish(msg)
        flag=msg.data[0]

    #now navigator kicks in:
    msg=sup.run(0.01)
    self.mode_pub.publish(msg)



