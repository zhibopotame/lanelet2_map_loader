#!/usr/bin/env python


import rospy 
import tf
import random 
from dynamic_reconfigure.server import Server
from map_loader.cfg import pcl_mapConfig
import math

M00=0.9437411916931052
M02=-253.44994097919124 /5.0
M12=499.4035693549777 / 5.0

class DynamicTF():
    def __init__(self):
        self.br = tf.TransformBroadcaster()
        self.parent_frame_id = rospy.get_param("parent_frame_id", "map")
        self.child_frame_id = rospy.get_param("child_frame_id", "pcl_map")
        self.srv = Server(pcl_mapConfig, self.callback)
        self.x_offset = 0
        self.x_scale = 1
        self.y_offset = 0
        self.y_scale = 1
        self.theta_offset = 0

    def callback(self, config, level):
        print("config.x_offset: ", config.x_offset)
        self.x_offset = config.x_offset * config.x_scale
        self.y_offset = config.y_offset * config.y_scale
        self.theta_offset = config.theta_offset
        print("x_offset: ", self.x_offset)
        print("y_offset: ", self.y_offset)
        print("theta_offset: ", self.theta_offset)
        return config
        
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.x_offset = -M02
            self.y_offset = M12
            self.theta_offset = math.acos((M00))
            print("x_offset: ", self.x_offset)
            print("y_offset: ", self.y_offset)
            print("theta_offset: ", self.theta_offset)
            self.br.sendTransform((self.x_offset, self.y_offset, 0),
                     tf.transformations.quaternion_from_euler(0, 0, self.theta_offset),
                     rospy.Time.now(),
                     self.child_frame_id,
                     self.parent_frame_id)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node("DynamicTF")
    node = DynamicTF()
    try:
        node.run()
    except rospy.ROSInterruptException: pass