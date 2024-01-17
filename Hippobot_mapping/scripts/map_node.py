#!/usr/bin/env python3

from hashlib import algorithms_available
from pickletools import float8
from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray, Pose, Point
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
import rclpy
import numpy as np

class Map(Node):
    def __init__(self) :
        super().__init__('map')
        #Create publisher
        self.map_pub = self.create_publisher(OccupancyGrid, '/world_map', 10)
        #Create subsciption to enemie
        self.enemie_sub = self.create_subscription(Pose, '/enemie_pose', self.enemie_cb, 10)
        self.enemie_ = Point()
        self.enemieHistory_ = []
        self.enemieFlag_ = False
        #Create subsciption to allies and start the history
        self.allies_sub = []
        self.allies_sub = self.create_subscription(PoseArray, 'allies/pos/world_frame', self.allies_cb, 10) 
        self.allies_ = []
        self.alliesHistory_ = []
        self.alliesFlag = False
        #Create subsciption to pinger and start the history
        self.pinger_sub = self.create_subscription(Pose, 'buoy_pose', self.pinger_cb, 10) 
        self.pinger_ = Point()
        self.pingerHistory_ = []
        #Create subsciption to obstacles
        self.obstacles_sub = []
        self.obstacles_sub = self.create_subscription(PoseArray, 'obstacles', self.obstacles_cb, 10)
        self.obstacles_ = []
        
        #Do self.makeMap each 01. seconds
        self.timer = self.create_timer(0.1, self.makeMap)
        
        #Define the width and height 
        self.width_ = 600
        self.height_ = 600
        
        #Initialize the map
        self.map_ = OccupancyGrid()
        self.map_.header.frame_id = 'map'
        self.map_.info.width = self.width_
        self.map_.info.height = self.height_
        self.map_.info.resolution = 1.0
        self.map_.data = [-1]*(self.width_*self.height_)
        #Define the origin
        self.map_.info.origin.position.x = -self.width_ / 2.0
        self.map_.info.origin.position.y = -self.height_ / 2.0
        #Declare the matrix of the map
        self.map_matrix_ = np.zeros((self.width_, self.height_), dtype=np.int8)
        
    def enemie_cb(self, msg: Pose) :
        self.enemieFlag_ = True
        try :
            self.enemie_.x = msg.position.x
            self.enemie_.y = msg.position.y
        except IndexError as e:
            print(f"Error in enemies callback: {e}")

    def allies_cb(self, msg : PoseArray) :
        try:
            self.alliesFlag = True
            self.allies_ = []
            for pose in msg.poses :
                allie = Point()
                allie.x = pose.position.x
                allie.y = pose.position.y
                self.allies_.append(allie)
        except IndexError as e:
            print(f"Error in allies callback: {e}")

    def pinger_cb(self, msg : Pose) :
        try :
            self.pinger_.x = msg.position.x
            self.pinger_.y = msg.position.y
        except Exception as e:
            print(f"Error in pinger callback: {e}")

    def obstacles_cb(self, msg : PoseArray) :
        try :
            self.obstacles_ = []
            for pose in msg.poses :
                obstacle = Point()
                obstacle.x = pose.position.x
                obstacle.y = pose.position.y
                self.obstacles_.append(obstacle)
        except IndexError as e:
            print(f"Error in obstacles callback: {e}")


    def makeMap(self) :      
        #print('-------------------------------------------------------------------------------')
        #Clean last allies, pinger and enemie position
        self.cleanMap()


        #Complete the map
        #pinger
        x_idx = int(self.pinger_.x) + self.width_ // 2
        y_idx = int(self.pinger_.y) + self.height_ // 2
        if 0 <= x_idx < self.width_ and 0 <= y_idx < self.height_ :
                self.map_matrix_[x_idx, y_idx] = 100
                self.pingerHistory_.append([x_idx, y_idx])  
        
        #obstacles
        for point in self.obstacles_ :
            if (np.sqrt(np.power(point.x-self.enemie_.x,2)+np.power(point.x-self.enemie_.y,2))>3.) :
                x_idx = int(point.x) + self.width_ // 2
                y_idx = int(point.y) + self.height_ // 2
                if 0 <= x_idx < self.width_ and 0 <= y_idx < self.height_ :
                    self.map_matrix_[x_idx, y_idx] = 100

        #allies
        if(self.alliesFlag) :
            for point in self.allies_ :
                x_idx = int(point.x) + self.width_ // 2
                y_idx = int(point.y) + self.height_ // 2
                if 0 <= x_idx < self.width_ and 0 <= y_idx < self.height_ :
                    self.alliesHistory_.append([x_idx, y_idx])
                    self.map_matrix_[x_idx, y_idx] = 100
                    self.makeRadius(x_idx, y_idx, 60, 1)

        #enemie
        if(self.enemieFlag_) :
            x_idx = int(self.enemie_.x) + self.width_ // 2
            y_idx = int(self.enemie_.y) + self.height_ // 2
            if 0 <= x_idx <self.width_ and 0 <= y_idx < self.height_ :
                self.enemieHistory_.append([x_idx, y_idx])
                self.map_matrix_[x_idx,y_idx] = 100
                self.makeRadius(x_idx, y_idx, 20, 1)

        self.publishMap(self.map_matrix_)

    def publishMap(self, map_matrix) :
        self.map_.data = []
        #transform matrix to list
        self.map_.data = map_matrix.flatten().tolist()
        self.map_pub.publish(self.map_)

    def cleanMap(self) :
        #Delete last position of pinger
        #Check if there are 2 or more positions of pinger
        if(len(self.pingerHistory_)>1) :
            x_idx = int(self.pingerHistory_[-2][0])
            y_idx = int(self.pingerHistory_[-2][1])
            if 0 <= x_idx <self.width_ and 0 <= y_idx < self.height_ :
                #Make a free cell in the last position
                self.map_matrix_[x_idx,y_idx] = 0
                self.makeRadius(x_idx, y_idx, 20, 0)
        #Delete last position of enemie
        if(len(self.enemieHistory_)>1) :
            x_idx = int(self.enemieHistory_[-2][0]) 
            y_idx = int(self.enemieHistory_[-2][1])
            if 0 <= x_idx <self.width_ and 0 <= y_idx < self.height_ :
                #Make a free cell in the last position
                self.map_matrix_[x_idx,y_idx] = 0
                #Make a free cells area of the last position
                self.makeRadius(x_idx, y_idx, 20, 0)
        #Delete last positions of allies
        if(len(self.alliesHistory_)>=2*len(self.allies_) and len(self.alliesHistory_)>0) :
            for i in range(len(self.allies_)) :
                x_idx = int(self.alliesHistory_[-len(self.allies_)-1-i][0]) 
                y_idx = int(self.alliesHistory_[-len(self.allies_)-1-i][1])
                if 0 <= x_idx < self.width_ and 0 <= y_idx < self.height_ :
                    #Make a free cell in the last position
                    self.map_matrix_[x_idx, y_idx] = 0
                    #Make a free cells area of the last position
                    self.makeRadius(x_idx, y_idx, 60, 0)
                    

    def makeRadius(self,x : int, y : int, radius : int, add_supp : bool) :
        if(add_supp) :
            for i in range(-radius, radius+1) :
                for j in range(-radius,radius +1) :
                    x_idx = x+i
                    y_idx = y+j
                    if 0 <= x_idx <self.width_ and 0 <= y_idx < self.height_ :
                           self.map_matrix_[x_idx, y_idx] = 100
        else :
            for i in range(-radius, radius+1) :
                    for j in range(-radius,radius +1) :
                        x_idx = x+i
                        y_idx = y+j
                        if 0 <= x_idx <self.width_ and 0 <= y_idx < self.height_ :
                            self.map_matrix_[x_idx, y_idx] = 0
                

        



def main(args=None):
    rclpy.init(args=args)
    map = Map()
    try :
        rclpy.spin(map)
    finally:
        map.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



        

                
            

        


        





    
    


        

