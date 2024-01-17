#!/usr/bin/env python3

from hashlib import algorithms_available
from turtle import width
from geometry_msgs.msg import PoseArray, Pose, Point
from nav_msgs.msg import OccupancyGrid
import copy
from rclpy.node import Node
import rclpy
import math


class Radar(Node):
    def __init__(self) :
        super().__init__('radar')
        self.obst_pub = self.create_publisher(PoseArray, 'obstacles', 10)
        self.posiEne_pub = self.create_publisher(PoseArray, 'posiEne',10)

        self.allies_sub = []
        self.allies_sub = self.create_subscription(PoseArray, 'allies/pos/world_frame', self.allies_cb, 10) 
        self.allies_ = []

        self.pinger_sub = self.create_subscription(Pose, 'buoy_pose', self.pinger_cb, 10)
        self.pinger_ = Point()

        self.bateau_sub = self.create_subscription(Pose, 'boat_pose', self.bateau_cb, 10)
        self.bateau_ = Point()

        self.lidar_sub = self.create_subscription(PoseArray, 'cluster_centroids', self.lidar_cb, 10)
        self.lidar_ = []

        self.timer = self.create_timer(0.1, self.mapping)

        self.obstacles_ = PoseArray()
        
        self.history = []

    def allies_cb(self, msg : PoseArray) :
        try:
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

    def bateau_cb(self, msg : Pose) :
        try :
            self.bateau_.x = msg.position.x
            self.bateau_.y = msg.position.y
            
        except Exception as e:
            print(f"Error in bateau callback: {e}")

    def lidar_cb(self, msg : PoseArray) :
        
        try :
            self.obstacles_.poses = copy.copy(msg.poses)
            self.lidar_ = []
            for pose in msg.poses:
                centroid = Point()
                centroid.x = pose.position.x+self.bateau_.x
                centroid.y = pose.position.y+self.bateau_.y
                self.lidar_.append(centroid)
            
            
        except Exception as e:
            print(f"Error in lidar callback: {e}")


    def mapping(self) :
        index=[]
        
        #Comparer la position des allies et la bouey avec les centres geometriques du lidar
        for i in range(len(self.lidar_)) :
                if(math.sqrt(math.pow(self.pinger_.x-self.lidar_[i].x,2)+math.pow(self.pinger_.y-self.lidar_[i].y,2)) < 10.) :
                    index.append(i)
                    print('It''s the pinger')
                else :    
                    for j in range(len(self.allies_)) :
                        if(math.sqrt(math.pow(self.allies_[j].x-self.lidar_[i].x,2) + math.pow(self.allies_[j].y-self.lidar_[i].y,2)) < 10.) :
                            index.append(i)
                            print('It''s an allie')

        self.obstacles_.poses = [pose for i, pose in enumerate(self.obstacles_.poses) if i not in index]
            

        self.obst_pub.publish(self.obstacles_)
        #print("Tamaño de obstacles:", len(self.obstacles_.poses))
        self.history.append(copy.deepcopy(self.obstacles_))

        if len(self.history) >= 2:
            self.compare_maps(self.history[-1], self.history[-2])

    def compare_maps(self, map_1, map_2) :
        
        posible_enemies = copy.copy(map_1)
        index = []
        for i in range(len(map_1.poses)) :
            for j in range(len(map_2.poses)) :
                if (math.sqrt(math.pow(map_1.poses[i].position.x-map_2.poses[j].position.x,2)+math.pow(map_1.poses[i].position.y-map_2.poses[j].position.y,2))<1.5):
                    index.append(i)

        posible_enemies.poses = [pose for i, pose in enumerate(posible_enemies.poses) if i not in index]
        if(len(posible_enemies.poses)>0) :
            print("Tamaño de self.lidar:", len(self.lidar_))
            print("Tamaño de posible_enemies:", len(posible_enemies.poses))
            print('NEW MAP')
            for pose in map_1.poses :
                print('(',pose.position.x,',',pose.position.y,')')
            print('OLD MAP')
            for pose in map_2.poses :
                print('(',pose.position.x,',',pose.position.y,')')
            for pose in posible_enemies.poses :
                print('sus : (',pose.position.x,',',pose.position.y,')')
            print('------------------------------------------------------')
        self.posiEne_pub.publish(posible_enemies)



def main(args=None):
    rclpy.init(args=args)
    radar = Radar()
    try :
        rclpy.spin(radar)
    finally:
        radar.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



        

                
            

        


        





    
    


        

