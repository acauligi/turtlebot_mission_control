#!/usr/bin/env python
#INPUTS:
#From /mission_mode expects [float32, float32, float32, float32]
#     where elements are [explore or mission(0 or 1), x location of current goal, y loc of current goal, theta of current goal]
#From /turtlebot_mission_control/path_goal expects list of floats. List dimensions of 2 rows and unknown columns depending on path length. Cannot be an empty list.
#     where elements look like [[x1,y1],[x2,y2],[x3,y3],...,[x_goal,y_goal]]
#
#OUTPUTS:
#To cmd_vel_mux/input/navi publishes a cmd = Twist() where cmd.linear.x is the velocity and cmd.angular.z is rotation
#goes to turtlebot, probably overriden when keyop is in use.
import rospy
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
import tf
import numpy as np

def wrapToPi(a):
    if isinstance(a, list):    # backwards compatibility for lists (distinct from np.array)
        return [(x + np.pi) % (2*np.pi) - np.pi for x in a]
    return (a + np.pi) % (2*np.pi) - np.pi

class Controller:

    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)
        self.trans_listener = tf.TransformListener()
        self.pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.x_g = 0.0
        self.y_g = 0.0
        self.th_g = 0.0

        # if being driven by user, use_controller = False
        self.use_controller = False 

        self.pose_goal = None
        self.pathlist = Path()

        rospy.Subscriber('/mission_mode', Float32MultiArray, self.poseCallback) #final pose to be acheived in front of apriltag because now care about orientation
        rospy.Subscriber('/turtlebot_mission_control/path_goal', Path, self.pathCallback) #a list of nodes computed using a*
		
    def poseCallback(self, msg):
        # robot autonomously drives
        if bool(round(msg.data[0])) == True: #if mission mode is in 1 (or True or Mission) then
            self.use_controller = True
        else:
            self.use_controller = False
        # i.e. still exploring so let user drive, will not publish a control

        self.pose_goal = [msg.data[1], msg.data[2], msg.data[3]] #for final pose in front of desired apriltag saved as a list [x,y,theta] this is goal right in front of apriltag

    def pathCallback(self, msg):
        self.pathlist = msg #hopefully a list from astar such that pathlist[i][0] is the x coord of ith point along path for example

    def get_ctrl_output(self):
        #Get current position and assign to self.x,y, and theta. If the lookupTransform isn't functioning self.x,y,theta gets zeros
        try:
            (translation,rotation) = self.trans_listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
        except:
            translation = (0.0,0.0,0.0)
            rotation = (0.0,0.0,0.0,1.0)
        euler = tf.transformations.euler_from_quaternion(rotation)
        self.theta = euler[2]
        self.x=translation[0]
        self.y=translation[1]
		
		#Code pose controller
		#move pose along path, find point we are closest to on path and make the target the next point on path
        #####warning, there is a chance robot may get stuck in loop if strays from path on a sharp turn might want to loop back
        #####might want to drop point from path after it has been visited
        #first check if path being posted to topic is the same
        #if different, update current total path
        #else if same target
        #refer to a path where we remove points that have been visited?
        
        #the difficult part is choosing the right point on path to target, then just use code from hw1
        #here we look what point on path we are currently at, and target the next point along path
        #the turtlebot will maintain a distance of about 1+0.5*resolution to 1-0.5*resolution from its goal point until the end.
        distances = [np.linalg.norm(np.array((self.pathlist[i][0],self.pathlist[i][1]))-np.array((self.x,self.y))) for i in range(len(self.pathlist.poses))] #get distances to all points on current path from astar
        closest=distances.index(min(distances)) #index of closest point on path provided by astar
        if closest != len(pathlist)-1: #if the closest point on path is not the final goal target a point further along the path
            self.x_g=pathlist[closest+1][0]
            self.y_g=pathlist[closest+1][1]
            self.th_g=np.arctan2(pathlist[closest+1][1]-pathlist[closest][1],pathlist[closest+1][0]-pathlist[closest][0]) #make it parallel to current path(line segment) we are on
            #we use separate gains based on our situation, less worried about approaching a final goal here so k3 is low
            k1=0.6
            k2=30.0
            k3=0.01
        else: #if the closest point is the final goal of the path just go there
            #pose position and orientations according to the topic publishing april_tag info directly.
            self.x_g=self.pose_goal[0]
            self.y_g=self.pose_goal[1]
            self.th_g=self.pose_goal[2]
            #actually want to park so concerned about orientation, less about speed or staying on line near our goal, use defaults from the homework
            k1=0.4
            k2=0.8
            k3=0.8
			
		#assume we are close enough that our pose controller from hw1 will function properly
        rho=np.sqrt((self.x-self.x_g)**2 + (self.y-self.y_g)**2)
        delta=np.arctan2(-(self.y-self.y_g), -(self.x-self.x_g))-self.th_g
        alpha=delta+self.th_g-self.theta
        delta=wrapToPi(delta)
        alpha=wrapToPi(alpha)
        
        #Define control inputs (V,om) - without saturation constraints
        V=k1*rho*np.cos(alpha)
        om=k2*alpha+k1*np.sinc(alpha/np.pi)*np.cos(alpha)*(alpha+k3*delta)
        
        # Divide by pi b/c sinc(x):=sin(pi*x)/(pi*x)
        # Apply saturation limits
        V = np.sign(V)*min(0.5, np.abs(V))
        om = np.sign(om)*min(1.0, np.abs(om))
        
        #Make final check, if at the goal, just stop
        currentpos=np.array((self.x,self.y,self.theta))
        goalpos=np.array((self.pose_goal[0],self.pose_goal[1],self.pose_goal[2]))
        
        reltol=0.000001 #very small percent relative tolerance since large numbers should not affect okay stopping area too much
        atol=0.02 #2cm absolute tolerance
        
        if np.allclose(currentpos,goalpos,reltol,atol): #if our current position is our goal, or close enough, stop moving until different path is provided
            V=0.0
            om=0.0
		
        #Package results into cmd
        cmd_x_dot = V
        cmd_theta_dot = om

        cmd = Twist()
        cmd.linear.x = cmd_x_dot
        cmd.angular.z = cmd_theta_dot
        return cmd

    def run(self):
        rate = rospy.Rate(10) # 10 Hz 
        while not rospy.is_shutdown():
            if self.use_controller and len(self.pathlist.poses) != 0: #if we want to use autonomous controller, (not human controlled), publish the autonomous control outputs
                ctrl_output = self.get_ctrl_output()
                self.pub.publish(ctrl_output)
            #if we are in user control mode nothing is published since keyop is in control
            rate.sleep()

if __name__ == '__main__':
    ctrl = Controller()
    ctrl.run()
