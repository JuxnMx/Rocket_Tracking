#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose
from matplotlib import pyplot as plt

class Visualizer:
    def __init__(self):
        rospy.init_node('position_viz', anonymous=True)
        self.counter = 0
        self.x=[]
        self.y=[]
        self.xd=[]
        self.yd=[]
        self.rocket_sub = rospy.Subscriber('/cam_msg', Pose, self.rocket_track)
        self.ref_sub = rospy.Subscriber('/ref_msg', Pose, self.ref_trajectory)
        
    def rocket_track(self, data):
        self.x.append(data.position.x)
        self.y.append(data.position.y)
        self.counter+=1
    
    def ref_trajectory(self, data):
        self.xd.append(data.position.x)
        self.yd.append(data.position.y)
        

    def plot(self):
        while not rospy.is_shutdown():
            if self.counter % 5 == 0:
                time = rospy.get_time()
                #print(self.x)
                #print(self.y)
                plt.title(f'Simulation Time = {time:.2f} seconds')
                plt.plot(self.x, self.y,'b',self.xd,self.yd,'--r')
                plt.axis("equal")
                plt.draw()
                plt.pause(0.05)
                plt.ion()
                plt.show()
                plt.clf()

if __name__ == '__main__':
    viz = Visualizer()
    viz.plot()