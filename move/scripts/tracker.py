#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import TransformStamped, PoseArray, Pose
from rclpy.parameter import Parameter
from geometry_msgs.msg import Quaternion
from tf2_ros import TransformBroadcaster, TransformListener, TransformException, Buffer
from ros_gz_interfaces.msg import ParamVec
import utm
import numpy as np
import quaternion
from copy import deepcopy
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float64, Int32
import math


# THIS NODE CALCULATES ALL USEFULL POSITION IN THE WORLD FIXED FRAME USING TF2, PUBLISH THE ONES NEEDED IN OTHER NODES AND SENDS THE BOAT TO A WAYPOINT THROUGH PID CONTROL

me = 'wamv/wamv/base_link'

def clamp(val, high):
    if abs(val) > high:
        return val * high / abs(val)
    return val


class Target:

    distance_to_keep = 0.0

    def __init__(self, link):

        self.pose = TransformStamped()
        self.pose.header.frame_id = 'world'
        self.pose.child_frame_id = link

    def publish(self, now):

        self.pose.header.stamp = now.to_msg()
        self.br.sendTransform(self.pose) #move frame

class Tracker(Node):
    
    def __init__(self):

        super().__init__('tracker')
        param = Parameter('use_sim_time', Parameter.Type.BOOL, True)
        
        self.set_parameters([param])

        #waypoint
        self.target = Target('waypoint')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.angle_pub = self.create_publisher(Float64, '/wamv/thrusters/main/pos', 10)
        
        self.thrust_pub = self.create_publisher(Float64, '/wamv/thrusters/main/thrust', 10)

        self.angle_error_pub = self.create_publisher(Float64, 'angle/error', 10) #publisher for the angle that need to be controled to 0 in order ta align with waypoint, debug purposes

        self.boat_pos_pub = self.create_publisher(Pose, 'boat_pose', 10) #boat position in world frame, used in other nodes"""

        self.allies_pub = self.create_publisher(PoseArray, 'allies/pos/world_frame',10)
        self.allies_amount_sub = self.create_subscription(Int32, 'allies_amount', self.publish_allies_poses, 10)
        

        # get buoy position in world frame
        self.buoy_pose = Pose()
        self.buoy_pose_pub = self.create_publisher(Pose, 'buoy_pose', 10) #buoy position in world frame, used in other nodes"""

        # initialization of Pid internal variables
        self.old_error_angle = 0.0
        self.old_error_distance = 0.0
        self.error_angle_integer = 0.0
        self.error_angle_derivative = 0.0
        self.error_distance_integer = 0.0
        self.error_distance_derivative = 0.0
        self.distance_to_keep = 0.0 #desired distance to keep with target, would have been a subsriber ideally

        # control gains Aiming to work fine for waypoints at max pi/3 relative orientation to the boat 
        self.Kx = 75.
        self.Kw = 0.1 #0.5 for heavy turblences
        self.Kxi = 10.0
        self.Kwi = 0.00000  # 0.00005 for heavy turbulences 
        self.Kxd = 0.0 
        self.Kwd = 0.0 # 0.5 for heavy turbulences

        self.iterator_count =1
        self.buoy_pose_sum = Pose()
        self.timer = self.create_timer(0.1, self.publish_poses_and_control_loop)

    def publish_allies_poses(self, msg: Int32):
        msg_pose = PoseArray()
        try:
            for i in range(msg.data): #passes through all ally boats
                #self.get_logger().info("looping through ally"), DEBUG PURPOSES
                pose = Pose() # create a new Pose message
                ally_pose = self.tf_buffer.lookup_transform(
                        'world',
                        f'ally{i}',
                        rclpy.time.Time()).transform.translation
                pose.position.x, pose.position.y = ally_pose.x, ally_pose.y
                msg_pose.poses.append(pose)
                #self.get_logger().info("ally x in world = %f" %ally_pose.x), DEBUG PURPOSES
        except TransformException:
            self.get_logger().info(
                f'ally not found')
            return
        
        self.allies_pub.publish(msg_pose)


    def publish_poses_and_control_loop(self): # GET ALL NECESSARY POSES IN WORLD FRAMES PUBLISH SOME

        try:
            if self.target.pose.header.frame_id == 'world':
                    
                    world_boat = self.tf_buffer.lookup_transform(
                        'world',
                        me,
                        rclpy.time.Time()).transform.translation
                    
                    boat_target = self.tf_buffer.lookup_transform(
                        me,
                        self.target.pose.child_frame_id,
                        rclpy.time.Time()).transform.translation
                    
            
                    #publish boat position in world frame"
                    msg = Pose()
                    msg.position.x = world_boat.x
                    msg.position.y = world_boat.y
                    self.boat_pos_pub.publish(msg)
                
        except TransformException:
            self.get_logger().info(
                f'Could not transform {me} to {self.target.pose.header.frame_id}')
            self.thrust_pub.publish(Float64())
            self.angle_pub.publish(Float64())
            return
        except ValueError:
            self.get_logger().info(
                f'{self.target} are invalid coordinates')
            self.thrust_pub.publish(Float64())
            self.angle_pub.publish(Float64())
            return

        msg = Float64()
        error_angle = boat_to_target_yaw = -math.atan2(boat_target.y,boat_target.x)
        #debug purposes
        self.get_logger().info("error angle = %f" % error_angle) 
        msg.data = error_angle
        self.angle_error_pub.publish(msg)

        
        self.error_angle_derivative = (error_angle - self.old_error_angle)/0.01 #calculating differential term
        self.error_angle_integer = self.error_angle_integer+error_angle #incrementing integral term
        self.old_error_angle = error_angle #saving the error_angle for next iteration's differential term
        msg.data = angle = self.Kw*error_angle + self.Kwi*self.error_angle_integer + self.Kwd*self.error_angle_derivative #calculating control output value
        
        #self.angle_pub.publish(msg) #publish to the thruster angle

        error_distance = math.sqrt(boat_target.x**2+boat_target.y**2) - self.distance_to_keep #distance from boat to target in fixed world frame minus the desired distance to keep 

        #debug purposes
        #self.get_logger().info("distance = %f" % error_distance)
        self.get_logger().info("error x = %f" % boat_target.x)
        self.get_logger().info("error y = %f" % boat_target.y)

        # calculating buoy pose and attempting to work around noise with mobile average
        new_buoy_pos = self.tf_buffer.lookup_transform(
                        'world',
                        'buoy',
                        rclpy.time.Time()).transform.translation
        self.buoy_pose_sum.position.x += new_buoy_pos.x
        self.buoy_pose_sum.position.y += new_buoy_pos.y
        self.iterator_count +=1

        self.buoy_pose.position.x = self.buoy_pose_sum.position.x/self.iterator_count
        self.buoy_pose.position.y = self.buoy_pose_sum.position.y/self.iterator_count
        self.buoy_pose_pub.publish(self.buoy_pose)


        self.error_distance_integer = self.error_distance_integer+error_distance #incrementing integral term
        self.error_distance_derivative = (error_distance - self.old_error_distance)/0.01 #calculating differential term
        self.old_error_distance = error_distance; #saving the error_distance for next iteration's differential term
        msg.data = self.Kx *error_distance + self.Kxi*self.error_distance_integer + self.Kxd*self.error_distance_derivative #calculating control output value

	    # adapt thrust, not too much when rotating

        if abs(error_angle)>0.4 :
            msg.data = clamp(msg.data, 1000.0)
        elif abs(error_angle)>0.3 :
            msg.data = clamp(msg.data, 9000.0)
        elif abs(error_angle)>0.2 :
            msg.data = clamp(msg.data, 6000.0)
        else : 
            msg.data = clamp(msg.data, 12000.0)

        #in cause we went too far, stop the thruster and reset integral error
        if error_distance < 0 :
            msg.data = abs(msg.data)
            self.error_distance_integer = 0.0

        #self.thrust_pub.publish(msg)

def main():
    rclpy.init()

    try:
        tracker = Tracker()

        while rclpy.ok():
            rclpy.spin_once(tracker)

    finally:
        # Cleanup and shutdown when the loop is interrupted
        tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

