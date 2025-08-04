#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from lecture2.msg import PosVel,PosVelArray

class Action:
    def __init__(self):
        rospy.init_node('action_master')
        self.odom_pub = rospy.Publisher('robot_odometry',PosVel,queue_size=10)
        rospy.Subscriber('traj',PosVelArray,self.traj_callback,queue_size=10)
        rospy.Timer(rospy.Duration(0.1),self.action_callback)#位置采样间隔0.05s
        self.action_pub = rospy.Publisher('action_pos',Marker,queue_size=10)
        self.action_marker = Marker()        
        self.traj_pts = []
        self.cur_pos = PosVel(0.0,0.0,0.0,0.0,0.0,0.0)
        rospy.sleep(1.0)
        rospy.spin()
        return
            
    def traj_callback(self,msg):
        # 更新轨迹
        self.traj_pts = msg.traj
        return
    
    def action_callback(self,msg):
        # 周期发送新的轨迹点位置
        if len(self.traj_pts)>0:
            pt = self.traj_pts[0]
            self.traj_pts.pop(0)
            self.odom_pub.publish(pt)
            self.cur_pos = pt
            self.draw(pts = [Point(pt.x,pt.y,pt.z)])
            return
        else:
            self.draw(pts = [Point(self.cur_pos.x,self.cur_pos.y,self.cur_pos.z)])
            self.odom_pub.publish(self.cur_pos)
            return
        
    def draw(self,id=3,type=Marker.SPHERE_LIST,color=[1,0,0],scale=0.3,pts=[]):
        self.action_marker.header.frame_id = 'map'
        self.action_marker.type =type
        self.action_marker.action = Marker.ADD
        self.action_marker.id = id
        self.action_marker.color.r = color[0]
        self.action_marker.color.g = color[1]
        self.action_marker.color.b = color[2]
        self.action_marker.color.a = 1.0
        self.action_marker.scale.x = scale
        self.action_marker.scale.y = scale
        self.action_marker.scale.z = scale
        self.action_marker.points = pts
        self.action_pub.publish(self.action_marker)

if __name__ =='__main__':
        action = Action()
        
