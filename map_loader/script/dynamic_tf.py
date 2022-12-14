#!/usr/bin/env python


import rospy 
import tf
import random 
from dynamic_reconfigure.server import Server
from map_loader.cfg import pcl_mapConfig
import math
import numpy as np 


POSITION_X=1.3358426758909915E7
POSITION_Y=3539644.973059331
INITIAL_SCALE=5.0
M00=0.9437411916931052
M11=0.9437411916931052
M10=0.33068499074145674
M01=-0.33068499074145674

M02=-253.44994097919124
M12=499.4035693549777
origin= [-105.599997, -63.249998, 0.05]

height = 292.85
width = 450.75
res = 0.05

A = np.array([[M00, M01, M02 ],
              [M10, M11, -M12],
              [0,0,1]])
              
B = np.array([[1.0, 0.0, width/2.0],
              [0.0, 1.0, height/2.0],
              [0,0,1]])

S = np.array([[INITIAL_SCALE/100.0, 0.0, 0],
              [0.0, INITIAL_SCALE/100.0, 0],
              [0,0,1]])
print(A.shape)
print(S.shape)
print(np.matmul(A,S))
T_world2map = np.dot(A, S)
print(T_world2map)
alpha = -2.45236185e+02
beta = -4.88767798e+02

relative_origin = np.array([9015/2.0 * 0.05, 5857/2.0 * 0.05, 0]) + np.array(origin)
# print(relative_origin)
# origin = np.dot(C, relative_origin)
# print(origin)

new_offset_x = M02 * INITIAL_SCALE/100.0
new_offset_y = -M12 * INITIAL_SCALE/100.0
print(new_offset_x, new_offset_y)
class DynamicTF():
    def __init__(self):
        self.br = tf.TransformBroadcaster()
        self.pcl_image_frame_id = "pcl_image"
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
        # print("theta_offset: ", self.theta_offset)
        # self.x_offset += origin[0]
        # self.y_offset += origin[1]
        return config
        
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # self.x_offset = -M12 - origin[0]
            # self.y_offset = M02 - origin[1]
            # self.x_offset = new_offset_x 
            # self.y_offset = new_offset_y 

            # self.x_offset = origin[0]
            # self.y_offset = origin[1]

            # self.x_offset = alpha
            # self.y_offset = beta

            self.br.sendTransform((-relative_origin[0], -relative_origin[1], 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0.0),
                     rospy.Time.now(),
                     self.child_frame_id,
                     self.pcl_image_frame_id)

            # self.theta_offset = 0.0
            # self.x_offset = new_offset_x
            # self.y_offset = new_offset_y

            self.br.sendTransform((self.x_offset, self.y_offset, 0),
                     tf.transformations.quaternion_from_euler(0, 0, self.theta_offset),
                     rospy.Time.now(),
                     self.pcl_image_frame_id,
                     self.parent_frame_id)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node("DynamicTF")
    node = DynamicTF()
    try:
        node.run()
    except rospy.ROSInterruptException: pass