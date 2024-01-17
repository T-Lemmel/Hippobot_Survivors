#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import TransformStamped, PoseArray, Pose, Quaternion
from rclpy.parameter import Parameter
from tf2_ros import TransformBroadcaster, TransformListener, TransformException, Buffer
from ros_gz_interfaces.msg import ParamVec
import utm
from copy import deepcopy
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float64, Int32
import math

# THIS NODE DOES SUBSCRIBES TO VARIOUS TOPIC, PLACES FRAMES USING TF2

me = 'wamv/wamv/base_link'

def clamp(val, high):
    if abs(val) > high:
        return val * high / abs(val)
    return val


class Waypoint: #defines the waypoint class which will be used for actuals waypoints of the trajectory but also boats
    E = None
    N = None
    z = None
    Z = None
    br = None
    
    def __init__(self, link):

        self.pose = TransformStamped()
        self.pose.header.frame_id = 'world'
        self.pose.child_frame_id = link

    def update(self, now = None, latitude=None, longitude=None, orientation=None):

        if latitude is not None:
            E,N,_,_ = utm.from_latlon(latitude, longitude, self.z, self.Z)

            x = E - self.E
            y = N - self.N

            self.pose.transform.translation.x = x
            self.pose.transform.translation.y = y

        if orientation is not None:
            self.pose.transform.rotation = orientation
            
    def publish(self, now):

        self.pose.header.stamp = now.to_msg()
        self.br.sendTransform(self.pose) #move frame

class Place_frame(Node):

    def __init__(self):
        super().__init__("Place_frame")

        # force use_sim_time
        param = Parameter('use_sim_time', Parameter.Type.BOOL, True)
        self.set_parameters([param])

        Waypoint.br = TransformBroadcaster(self)

        # get Boat data
        self.Boat = Waypoint('wamv/wamv/base_link')
        self.gps_sub = self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_cb, 10)
        self.imu_sub = self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.imu_cb, 10)

        #waypoint
        self.Waypoint = Waypoint('waypoint')
        self.waypoint_pose = Pose()
        self.Waypoint_pose_sub= self.create_subscription(Pose, 'waypoint_pose', self.waypoint_frame_placing, 10)

        # allies data
        self.allies = []
        self.get_logger().info("subscribing to allies")
        self.allies_sub = self.create_subscription(PoseArray, '/wamv/ais_sensor/allies_positions', self.allies_cb, 10)
        self.allies_amount_pub = self.create_publisher(Int32, 'allies_amount', 10)


        # buoy
        self.buoy = Waypoint('buoy')
        self.buoy.pose.header.frame_id = 'wamv/wamv/base_link'
        self.buoy_sub = self.create_subscription(ParamVec, '/wamv/sensors/acoustics/receiver/range_bearing', self.buoy_cb, 10)

        # loop timer
        self.timer = self.create_timer(0.05, self.loop)
        
    def is_init(self):
        return self.Boat.Z is not None

    def gps_cb(self, msg: NavSatFix):

        if not self.is_init():
            self.Boat.E,self.Boat.N,self.Boat.z,self.Boat.Z = utm.from_latlon(msg.latitude, msg.longitude)
            return
        
        self.Boat.update(latitude = msg.latitude, longitude = msg.longitude)

    def imu_cb(self, msg: Imu):
        self.Boat.update(orientation = msg.orientation)
    
    def waypoint_frame_placing(self, msg: Pose):
        
        
        if (self.waypoint_pose.position.x != msg.position.x) :
            self.Waypoint.pose.transform.translation.x = msg.position.x
            self.waypoint_pose.position.x = msg.position.x
        if (self.waypoint_pose.position.y != msg.position.y) :
            self.Waypoint.pose.transform.translation.y = msg.position.y
            self.waypoint_pose.position.y = msg.position.y

    def allies_is_init(self):
        for i in range(len(self.allies)):
            if self.allies[i].Z is None :
                return False
        return True
                
    def allies_cb(self, msg: PoseArray):

        if not self.is_init():
            return

        self.get_logger().info("got past the init condition in ally")
        # adapt messages
        if len(self.allies) != len(msg.poses):
            #self.get_logger().info("creating waypoint object in self.allies")
            self.allies = [Waypoint(f'ally{i}') for i in range(len(msg.poses))]

        if not self.allies_is_init():
            for i,pose in enumerate(msg.poses):
                self.allies[i].E,self.allies[i].N,self.allies[i].z,self.allies[i].Z = utm.from_latlon(pose.position.x, pose.position.y)
            
        for i,pose in enumerate(msg.poses):

            self.allies[i].update(self.get_clock().now(), latitude = pose.position.x, longitude = pose.position.y, orientation= pose.orientation)
            msg = Int32()
            msg.data = len(self.allies)
            self.allies_amount_pub.publish(msg) # acts as a timer to keep publishing ally poses in tracker.py
            
    def buoy_cb(self, msg: ParamVec):

        rho = None
        theta = None

        for param in msg.params:
            if param.name == 'range':
                rho = param.value.double_value
            elif param.name == 'bearing':
                theta = param.value.double_value

        if rho is None or theta is None:
            return

        self.buoy.pose.transform.translation.x = rho * math.cos(theta)
        self.buoy.pose.transform.translation.y = rho * math.sin(theta)

    def loop(self):

        if not self.is_init():
            return

        now = self.get_clock().now()
        self.Boat.publish(now) #update the position of the boat tf2 frame, COMMENTED SO EKF CAN DO IT BETTER
            
        self.Waypoint.publish(now) #update the position of the waypoint tf2 frame

        for ally in self.allies: #update the position of the allies tf2 frame
            ally.publish(now)

        self.buoy.publish(now) #update the position of the buoy tf2 frame

def main():
    rclpy.init()

    try:
        place_frame = Place_frame()

        while rclpy.ok():
            rclpy.spin_once(place_frame)

    finally:
        # Cleanup and shutdown when the loop is interrupted
        place_frame.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

