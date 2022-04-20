#!/usr/bin/env python3

from time import time
import numpy as np
import rospy
from math import sin, cos, pi
from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import WheelEncoderStamped
from nav_msgs.msg import Odometry
import geometry_msgs.msg 
import tf


class dt_duckie_odometry_node(DTROS):
    
    def __init__(self, node_name):

        super(dt_duckie_odometry_node, self).__init__(node_name=node_name, node_type=NodeType.LOCALIZATION)
        self.veh_name = rospy.get_namespace().strip("/")
        
        self.delta_left = 0
        self.delta_right = 0
        
        self.distancePerCount = (pi * 0.0636) / 135
        self.distanceBetweenWheels = 0.1

        self.x = 0
        self.y = 0
        self.th = 0

        self.left_tick_prev = None
        self.right_tick_prev = None

        self.current_time = rospy.Time.now()
        
        #Subscribing to Data from ROS Topic which contain ticks from wheel encoder 
        left_encoder_topic = f"/{self.veh_name}/left_wheel_encoder_node/tick"
        self.sub_encoder_ticks_left = rospy.Subscriber(left_encoder_topic, WheelEncoderStamped, self.callback_encoder_left, queue_size=1)
        
        right_encoder_topic = f"/{self.veh_name}/right_wheel_encoder_node/tick"
        self.sub_encoder_ticks_right = rospy.Subscriber(right_encoder_topic, WheelEncoderStamped, self.callback_encoder_right, queue_size=1)
        

        #Publisher for odometry messages
        self.odomPub = rospy.Publisher(f"/{self.veh_name}/dt_duckie_odometry/odometry" ,Odometry, queue_size=10, dt_topic_type=TopicType.LOCALIZATION) 

        self.LEFT_RECIEVED = False
        self.RIGHT_RECIEVED = False

    #Call for retrieving data of left encoder
    def callback_encoder_left(self, encode_msg_l):

        if self.left_tick_prev is None:
            ticks = encode_msg_l.data
            self.left_tick_prev = ticks
        
        delta_left = (encode_msg_l.data - self.left_tick_prev)
        self.delta_left = delta_left
        
        #Benyttet for å se data fra venstre rotasjonskoder
        #print(f"left {delta_left}")
        
        self.LEFT_RECIEVED = True
        self.publishing_odom()   
        self.left_tick_prev = encode_msg_l.data

    #Call for retrieving data of right encoder 
    def callback_encoder_right(self, encode_msg_r):

        if self.right_tick_prev is None:
            ticks = encode_msg_r.data
            self.right_tick_prev = ticks
        
        delta_right = (encode_msg_r.data - self.right_tick_prev)
        self.delta_right = delta_right
        
        #Benyttet for å se data fra høyre rotasjonskoder
        #print(f"right {delta_right}")
        
        self.RIGHT_RECIEVED = True
        self.publishing_odom()
        self.right_tick_prev = encode_msg_r.data        

    #Publishing odometry data with encoder data
    def publishing_odom(self):
        
        #Sjekker om begge koderne har mottat ny data som skal brukes
        if not (self.LEFT_RECIEVED and self.RIGHT_RECIEVED):
            return None
        
        self.LEFT_RECIEVED = False
        self.RIGHT_RECIEVED = False
    
        self.current_time = rospy.Time.now()

        #Beregner siste rotasjonskoder med omkretsen av hjulet delt på antall rotasjoner for å rotere fullt
        handled_data_left = (self.delta_left * self.distancePerCount) 
        handled_data_right = (self.delta_right * self.distancePerCount) 

        vx = ((handled_data_right + handled_data_left) / 2)
        vy = 0
        vth = ((handled_data_right - handled_data_left) / self.distanceBetweenWheels)

        #Her beregnes distansen som er traversert
        data_x = (vx * cos(self.th)) 
        data_y = (vx * sin(self.th)) 
        data_th = vth

        self.x += data_x
        self.y += data_y
        self.th += data_th

        odometry_quaternion = tf.transformations.quaternion_from_euler(0,0,self.th)

        tf_br = tf.TransformBroadcaster()

        #Det opprettes en melding for TransformBroadcast som fungerer som node /tf

        odometry_trans = geometry_msgs.msg.TransformStamped()

        odometry_trans.header.stamp = self.current_time
        odometry_trans.header.frame_id = "odom"
        odometry_trans.child_frame_id = "base_link"

        odometry_trans.transform.translation.x = self.x
        odometry_trans.transform.translation.y = self.y
        odometry_trans.transform.translation.z = 0.0
        odometry_trans.transform.rotation = odometry_quaternion

        tf_br.sendTransform((self.x, self.y, 0.0),
                            odometry_quaternion,
                            rospy.Time.now(),
                            "base_link",
                            "odom")

        #Setter opp og utfyller en odometri melding før den publiseres.
        odom_msg = Odometry()
        
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "/odom"
        odom_msg.child_frame_id = "/base_link"
        
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.pose.pose.orientation.x = odometry_quaternion[0]
        odom_msg.pose.pose.orientation.y = odometry_quaternion[1]
        odom_msg.pose.pose.orientation.z = odometry_quaternion[2]
        odom_msg.pose.pose.orientation.w = odometry_quaternion[3]
        
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = vth
        
        #Printing the Odom msg
        print(odom_msg)
        
        self.odomPub.publish(odom_msg)
        

if __name__ == '__main__':
    node = dt_duckie_odometry_node(node_name='dt_duckie_odometry')
    
    # Det kjøres rospy.spin for å holde noden i live.
    rospy.spin()