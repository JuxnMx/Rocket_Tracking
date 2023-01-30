#!/usr/bin/env python3
# Copyright Juan Manuel Moreno 2022
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import numpy as np
from my_functions import init_pose

MAT = np.array

class Rocket:
    def __init__(self,initPose,goalPose):
        rospy.init_node('ard_pub', anonymous=True)
        self.a = 0.0
        self.c = 0.0
        self.initPose = initPose
        self.goalPose = goalPose
        self.xd = self.initPose[0]
        self.yd = self.initPose[1]
        self.ref_RocketPath()
        self.dt = 0.25 # 4Hz
        self.sum_e_x = 0.0
        self.sum_e_y = 0.0
        self.e_x_prev = 0.0
        self.e_y_prev = 0.0
        rospy.loginfo('Start Sending Model Outputs')
        self.pub_u_ard = rospy.Publisher('/ard_msg', String, queue_size=10)
        self.sub_pose = rospy.Subscriber('/cam_msg', Pose, self.controller)
        self.pub_ref = rospy.Publisher('/ref_msg', Pose, queue_size=16)
        self.pub_u = rospy.Publisher('/u_msg', Pose, queue_size=16)

    def ref_RocketPath(self):
        '''
        ref_RocketPath calculating the parameters of the
        reference path equation
        Parameters
        ----------
        init
            _description_ initial x and y position of the rocket control
        goal
            _description_ the final position where the rocket is guided
        '''
        # Model used for the path is ax+c
        x0,y0 = self.initPose ; xG,yG = self.goalPose

        A = MAT([[x0, 1], [xG, 1]])
        B = MAT([y0, yG])
        C = np.linalg.solve(A,B)
        self.a,self.c = C

    def controller(self,rocket_pose,Kp= 1,Ki= 0.01,Kd= 0.1):
        '''
        controller PID controllers, for each motor.
        Parameters
        ----------
        rocket_pose
            position getting througth the camera
        '''
        dt = self.dt
        x = rocket_pose.position.x
        y = rocket_pose.position.y
        self.xd += dt
        self.yd = self.a*self.xd + self.c
        #Sending ref_trajectory
        ref_pose=Pose()
        ref_pose.position.x=self.xd
        ref_pose.position.y=self.yd
        self.pub_ref.publish(ref_pose)
        #Calculating error
        e_x = self.xd-x
        e_y = self.yd-y
        self.sum_e_x += e_x
        self.sum_e_y += e_y
        e_x_prev = self.e_x_prev
        e_y_prev = self.e_y_prev
        # Calculating input
        u = (Kp*MAT([e_x,e_y]).T)+(Ki*MAT([self.sum_e_x,self.sum_e_y]).T)+((Kd/dt)*MAT([e_x_prev,e_y_prev]).T)
        rocket_u = Pose()
        rocket_u.position.x = round(u[0]/100,4)
        rocket_u.position.y = round(u[1]/100,4)
        rocket_u_str = f'{round(u[0]/100,4)}|{round(u[1]/100,4)}'
        self.pub_u_ard.publish(rocket_u_str)
        self.pub_u.publish(rocket_u)
        # rospy.loginfo(rocket_u_str) 
        # Updating previous error
        self.e_x_prev = e_x
        self.e_y_prev = e_y
    
if __name__ == '__main__':
    x0,y0 = init_pose()
    initPose = np.array([x0, y0]) #cv2 function
    goalPose = np.array([450, 0]) #initial parameter 
    Rocket(initPose, goalPose)
    rospy.spin()
    rospy.loginfo('Stop Sending Model Outputs')