#!/usr/bin/env python
import math 
import numpy as np 
import rospy 
import rosparam

POSITION_X=1.3358426758909915E7
POSITION_Y=3539644.973059331
INITIAL_SCALE=5.0
M00=0.9437411916931052
M11=0.9437411916931052
M10=0.33068499074145674
M02=-253.44994097919124
M01=-0.33068499074145674
M12=499.4035693549777

def convert(x, y):
    from pyproj import Proj, transform
    P3857 = Proj(init='epsg:3857')
    P4326 = Proj(init='epsg:4326')
    
    lon, lat = transform(P3857, P4326, x, y)
    return lon, lat

def convert_custom(x, y):
    # ref https://gist.github.com/onderaltintas/6649521
    k = 20037508.34
    # k = 20037508.34278924575
    lon = x * 180.0 / k
    lat = math.atan(math.exp(y * math.pi / k)) * 360.0 / math.pi - 90.0; 
    return lon, lat

def get_rotation(m00):
    return math.acos((m00))


def main():
    # lon, lat = convert(POSITION_X, POSITION_Y)
    print(convert(POSITION_X, POSITION_Y))
    print(convert_custom(POSITION_X, POSITION_Y))
    lon, lat = convert_custom(POSITION_X, POSITION_Y)
    theta = get_rotation(M00)

    dict_file = {"origin_lon" : lon, "origin_lat" : lat, "theta" : theta}
    # rospy.set_param("origin_lon", lon)
    # rospy.set_param("origin_lat", lat)
    # rospy.set_param( "theta" , theta)
    rospy.set_param("hd_map_calibration", dict_file)

    # file = open("./hd_map_calibration.yaml", 'w')
    file_name = "./hd_map_calibration.yaml"
    documents = rosparam.dump_params(file_name, "hd_map_calibration")

if __name__ == '__main__':
    rospy.init_node("DynamicTF")
    main()

