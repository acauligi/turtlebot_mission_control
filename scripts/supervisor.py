#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray, Float32MultiArray, String
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import tf
import numpy as np

#----------------------
#input: bot pose, mission {list of int}
#output: publish /mission_mode as a list of flaots (flag {0 or 1}, x, y, th)

def pose_to_xyth(pose):
    th = tf.transformations.euler_from_quaternion((pose.orientation.x,
                                                   pose.orientation.y,
                                                   pose.orientation.z,
                                                   pose.orientation.w))[2]
    return [pose.position.x, pose.position.y, th]
def wrapToPi(a):
	b = a
	if a< -np.pi or a> np.pi:
		b = ((a+np.pi) % (2*np.pi)) - np.pi
	return b


class Supervisor:

    def __init__(self):
        rospy.init_node('turtlebot_supervisor', anonymous=True)
        self.trans_listener = tf.TransformListener()
        self.trans_broad = tf.TransformBroadcaster()
        self.bot_pose=np.array([0.,0.,0.])

        #relevant i execution phase
        self.current_g=np.array([0.,0.,0.])
        self.step=0
        #self.next_goal=[]
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)    # rviz "2D Nav Goal"

        self.mission=[]
        rospy.Subscriber('/mission', Int32MultiArray, self.mission_callback)
        # rospy.spin()

        self.waypoint_locations = {}    # dictionary that caches the most updated locations of each mission waypoint
        self.waypoint_offset = PoseStamped()
        self.waypoint_offset.pose.position.z = .4    # waypoint is located 40cm in front of the AprilTag, facing it
        quat = tf.transformations.quaternion_from_euler(0., np.pi/2, np.pi/2)
        self.waypoint_offset.pose.orientation.x = quat[0]
        self.waypoint_offset.pose.orientation.y = quat[1]
        self.waypoint_offset.pose.orientation.z = quat[2]
        self.waypoint_offset.pose.orientation.w = quat[3]

        self.mode_pub = rospy.Publisher('/mission_mode', Float32MultiArray ,queue_size=10)
        # self.tag_pub = rospy.Publisher('/next_tag', Float32MultiArray ,queue_size=10)

        self.testing_pub = rospy.Publisher('/testingOnly', Float32MultiArray, queue_size=10)

        self.we_are_done = False
        self.mission_complete = rospy.Publisher('/success', Bool, queue_size=10)

        self.flag = 0. #exploration phase, no navigator
        self.loc = [0., 0., 0.]
        self.thresh=0.1

    def rviz_goal_callback(self, msg):
        self.next_goal=pose_to_xyth(msg.pose)
	    # rospy.logwarn("received")

    def mission_callback(self, msg):
        self.mission=msg.data
	    # rospy.logwarn("received")
	
    def update_waypoints(self):
        for tag_number in range(8):
            try:
                self.waypoint_offset.header.frame_id = "/tag_{0}".format(tag_number)

                (translation, rotation) = self.trans_listener.lookupTransform("/map", self.waypoint_offset.header.frame_id, rospy.Time(0))

                x = translation[0]
                y = translation[1]
                euler = tf.transformations.euler_from_quaternion(rotation)
                theta = euler[2]
                
                self.waypoint_locations[int(tag_number)] = [x, y, theta]

                rospy.logwarn(tag_number)
                self.waypoint_locations[int(tag_number)] = self.add_offset(x, y, theta)

                rospy.logwarn(self.waypoint_locations[int(tag_number)])

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

    def find_bot(self):
        try:
            (trans, rot)= self.trans_listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
            euler=tf.transformations.euler_from_quaternion(rot)
            self.bot_pose=np.array([trans[0], trans[1], wrapToPi(euler[2])])

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException): pass
        rospy.logwarn(self.bot_pose)

    def add_offset(self, x,y,th):
        x=self.trans_listener.transformPose("/map", self.waypoint_offset).pose.position#, rospy.Time(0))
        rot=self.trans_listener.transformPose("/map", self.waypoint_offset).pose.orientation
        euler_off = tf.transformations.euler_from_quaternion((rot.x, rot.y, rot.z, rot.w))
        return [x.x, x.y, euler_off[2]]

    def run(self):
        rate = rospy.Rate(1) # 1 Hz, change this to whatever you like
        while not rospy.is_shutdown():
            # rospy.logwarn(len(self.mission))
            self.update_waypoints()
            if self.flag==0.0:
                rospy.logwarn('in')
                if len(self.waypoint_locations.keys()) == 8: 
                    self.flag=1.
                    loc= self.waypoint_locations[self.mission[0]] #first location to go to, ie first tag of mission execution
                    self.current_g=np.array(self.waypoint_locations[self.mission[0]])
                else: 
                    loc=[0.,0.,0.]

                msg=Float32MultiArray()
                msg.data=[self.flag] + loc
            else: 
                rospy.logwarn('1')
                self.find_bot()

                dist=np.linalg.norm(np.array([self.bot_pose[0]-self.current_g[0], self.bot_pose[1]-self.current_g[1]]))

                if dist<self.thresh:
                    rospy.logwarn('passed checkpoint')
                    self.step+=1
                    self.current_g=np.array(self.waypoint_locations[self.mission[self.step]])
                    rospy.logwarn(self.current_g)

                    if self.step == 8:
						self.we_are_done = True

                else:
                    rospy.logwarn('on the way')
                
                msg=Float32MultiArray()
                msg.data = [self.flag, self.current_g[0], self.current_g[1], self.current_g[2]]

            if self.step==len(self.mission):
                msg.data=[0., 0., 0., 0.]

            self.mode_pub.publish(msg)

            msg = Bool()
            msg.data = self.we_are_done
            self.mission_complete.publish(msg)

            tagsSeen = Float32MultiArray()
            tagsSeen.data = self.waypoint_locations.keys()
            self.testing_pub.publish(tagsSeen)
            rate.sleep()

if __name__ == '__main__':
    sup = Supervisor()
    sup.run()
