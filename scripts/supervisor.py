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
        self.current_g=-np.ones((1,3))
        self.step=0
        #self.next_goal=[]
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)    # rviz "2D Nav Goal"

        self.mission=[]
        rospy.Subscriber('/mission', Int32MultiArray, self.mission_callback)
        rospy.spin()

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

        self.flag = 0 #exploration phase, no navigator
        self.loc = [0, 0, 0]

 	self.current_g=-np.ones((1,3))
        self.thresh=0.01
        self.step=0

    def rviz_goal_callback(self, msg):
        self.next_goal=pose_to_xyth(msg.pose)
	    # rospy.logwarn("received")

    def mission_callback(self, msg):
        self.mission=msg.data
	    # rospy.logwarn("received")
	
    def update_waypoints(self):
        for tag_number in range(8): #self.mission:
            try:
                self.waypoint_offset.header.frame_id = "/tag_{0}".format(tag_number)

                (translation, rotation) = self.trans_listener.lookupTransform("/map", \
                    self.waypoint_offset.header.frame_id, rospy.Time(0))

                x = translation[0]
                y = translation[1]
                euler = tf.transformations.euler_from_quaternion(rotation)
                theta = euler[2]

                
                # OFFSET = rospy.logwarn(self.trans_listener.transformPose("/map", self.waypoint_offset))


                # self.waypoint_locations[int(tag_number)] = [ x+OFFSET.pose.position.x, y+OFFSET.pose.position.y, \
                     # theta+tf.transformations.euler_from_quaternion(OFFSET.pose.orientation)[2] ]

                # rospy.logwarn([ x+OFFSET.pose.position.x, y+OFFSET.pose.position.y, \
                      # theta+tf.transformations.euler_from_quaternion(OFFSET.pose.orientation)[2] ])
               
                self.waypoint_locations[int(tag_number)] = [x, y, theta]

                rospy.logwarn(self.waypoint_locations[int(tag_number)])

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

    def find_bot(self):
        try:
            (trans, rot)= self.trans_listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
            euler=tf.transformations.euler_from_quaternion(rot)
            self.bot_pose=np.array([trans[0], trans[1], wrapToPi(euler[2])])

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException): pass

    def check_mode(self):
        all_tags_seen = True
        # if not (tag in self.waypoint_locations.keys() for tag in self.mission):
        if not any(self.mission):
            all_tags_seen = False
            return all_tags_seen

        for tag in self.mission:
            if not tag in self.waypoint_locations.keys():
                all_tags_seen = False
                return all_tags_seen

        return all_tags_seen

    def run(self):
        rate = rospy.Rate(1) # 1 Hz, change this to whatever you like
        while not rospy.is_shutdown():
            # rospy.logwarn(len(self.mission))
            if self.flag==0.0:
                rospy.logwarn('in')
                self.update_waypoints()
                if self.check_mode():
                    self.flag=1.
                    loc= self.waypoint_locations[self.mission[self.step]] #first location to go to, ie first tag of mission execution
                else: 
                    loc=[0.,0.,0.]

                msg=Float32MultiArray()
                msg.data=[self.flag] + loc

            else: 
                self.find_bot()
                dist=np.linalg.norm(np.sum([self.bot_pose[:2], -self.current_g[:2]], axis=0)) #euclidian distance
                if dist<self.thresh:
                    rospy.logwarn('passed checkpoint')
                    self.step+=1
                    self.current_g=np.array(self.waypoint_locations[self.mission[self.step]])
                else:
                    rospy.logwarn('on the way')
                
                msg=Float32MultiArray()
                #tag=self.mission[len(self.been_at)-1]
                msg.data=[self.flag] + self.current_g

            self.mode_pub.publish(msg)

            tagsSeen = Float32MultiArray()
            tagsSeen.data = self.waypoint_locations.keys()
            self.testing_pub.publish(tagsSeen)
            rate.sleep()

#    def explore(self):
#        rate = rospy.Rate(1) # 1 Hz, change this to whatever you like
#        while not rospy.is_shutdown():
#            self.update_waypoints()
#            if self.check_mode(): #first location to go to, ie first tag of mission execution
#                self.flag = 1.0
#                self.loc = self.waypoint_locations[int(self.mission[0])]
#            else: 
#                self.flag = 0.0
#                self.loc = [0, 0, 0]

#            msg=Float32MultiArray()
#            msg.data=[self.flag] + self.loc
#            rate.sleep()
#            return msg

if __name__ == '__main__':
    sup = Supervisor()
    sup.run()
