#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point,Polygon
from visualization_msgs.msg import Marker
from kinodyamic_astar5 import Map,KinoAstar
import numpy as np
import random
from lecture2.msg import PosVelArray,PosVel


class Planning:
    def __init__(self):
        rospy.init_node('Planning')
        rospy.Subscriber('goal',Point,self.goal_callback,queue_size=10)
        rospy.Subscriber('point_cloud',Polygon,self.obstcal_callback,queue_size=10)
        rospy.Subscriber('robot_odometry',PosVel,self.odometry_callback,queue_size=10)
        self.traj_pub = rospy.Publisher('traj',PosVelArray,queue_size=10)
        #执行间隔缩短意味着速度提升，此时check周期必须缩短，但search规划时间如果过长，轨迹会直接穿过障碍物
        rospy.Timer(rospy.Duration(0.5),self.fsm_callback)   
        self.map = Map()
        self.kino = KinoAstar(self.map)
        self.robot_pos = None
        self.robot_vel = None
        self.end_pos = None
        self.robot_pos_mark = False
        self.traj_pts = None
        self.traj_vpts = None
        self.state = 0
        rospy.sleep(1.0)
        rospy.spin()
        return 
    # 外部触发
    def goal_callback(self,msg):
        # print('receive goal')
        if self.robot_pos_mark:
            self.end_pos = np.array((msg.x,msg.y),float)
            self.state = 1
        return
    
    def obstcal_callback(self,msg):
        # print('receive obstcal')
        if len(msg.points)>0:
            self.map.addObstcal(msg.points)
        return 
    
    def odometry_callback(self,msg):
        # print('receive odometry')
        self.robot_pos_mark = True
        self.robot_pos = np.array([msg.x,msg.y],float)
        self.robot_vel = np.array([msg.vx,msg.vy],float)
        return
    
    def fsm_callback(self,msg):
        # 0 等待目标  1正在规划  2正在执行 3到达终点
        if self.state==0: 
            return 
        if self.state==1:
            self.kino.search(start_pos=self.robot_pos,start_vel=self.robot_vel,end_pos=self.end_pos)
            self.traj_pts,self.traj_vpts = self.kino.getSamples()
            self.traj_pts.reverse()
            self.traj_vpts.reverse()
            pts = PosVelArray()
            for i in range(len(self.traj_pts)):
                pos = self.traj_pts[i]
                vel = self.traj_vpts[i]
                pts.traj.append(PosVel(pos[0],pos[1],0.0,vel[0],vel[1],0.0))
            self.traj_pub.publish(pts)
            self.state=2
            return
        if self.state==2:
            res = self.check(self.traj_pts,self.robot_pos)
            if res:
                self.state = 1
                print('replan')
            if np.linalg.norm(self.robot_pos-self.end_pos)<0.1:
                self.state = 3
            return
        if self.state==3:
            print('reach goal,next goal',self.end_pos)
            randy = random.uniform(1.0,4.0)
            if self.robot_pos[0]>6:
                self.end_pos = np.array((0.0,randy),float)
            else:
                self.end_pos = np.array((8.0,randy),float)
            self.state=1
            return
    
    def check(self,traj_pts,cur_pos):
        cur_i = 0
        end_i = len(traj_pts)
        for i,pt in enumerate(traj_pts):
            if np.linalg.norm(pt-cur_pos)<1e-5:
                cur_i = i
                break
        for i in range(cur_i,end_i):
            pt = traj_pts[i]
            id = self.map.pos2id(pt)
            if id in self.map.occ:
                return True
            dis = np.linalg.norm(pt-cur_pos)
            if dis>3:
                break
        return False
     
    
if __name__=='__main__':
    planning = Planning()