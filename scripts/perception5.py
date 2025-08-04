#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Polygon
import open3d as o3d
import numpy as np
from lecture2.msg import PosVel,PosVelArray

class Perception:
    def __init__(self):
        rospy.init_node('perception_master')
        rospy.Subscriber('robot_odometry',PosVel,self.process_callback,queue_size=10)
        self.pub_pts = rospy.Publisher('point_cloud',Polygon,queue_size=10)
        self.pcd = None
        self.pcd_tree = None
        self.addObstcale()
        rospy.spin()
        return   
    
    def process_callback(self,msg):
        robot_position = np.array((msg.x,msg.y,msg.z),float)
        [k, idx, _] = self.pcd_tree.search_radius_vector_3d(robot_position, 3)
        pts = Polygon()
        for id in idx:
            pt = Point()
            pt.x = self.pcd.points[id][0]
            pt.y = self.pcd.points[id][1]
            pt.z = self.pcd.points[id][2]
            pts.points.append(pt)
        self.pub_pts.publish(pts)
        return 
    
    def addObstcale(self):
        base_pt1 = np.array([3.05,1.05,0.0],float)
        base_pt2 = np.array([6.05,1.05,0.0],float)
        base_pt3 = np.array([3.15,1.05,0.0],float)
        base_pt4 = np.array([6.15,1.05,0.0],float)
        pts= np.zeros((120,3),float)
        for i in range(30):
            dh = 0.1*i*np.array((0.0,1.0,0.0),float)
            pts[i,:] = base_pt1 + dh
            pts[i+30,:] = base_pt2 + dh
            pts[i+60,:] = base_pt3 + dh
            pts[i+90,:] = base_pt4 + dh
        self.pcd = o3d.geometry.PointCloud()
        self.pcd.points = o3d.utility.Vector3dVector(pts)  
        self.pcd_tree = o3d.geometry.KDTreeFlann(self.pcd)
        
if __name__=='__main__':
    perception = Perception()
    
    