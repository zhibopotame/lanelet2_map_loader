/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Authors: Simon Thompson, Ryohsuke Mitsudome
 *
 */

#include <ros/ros.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include <lanelet2_extension/io/autoware_osm_parser.h>
#include <lanelet2_extension/projection/mgrs_projector.h>
#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/utilities.h>

#include <autoware_lanelet2_msgs/MapBin.h>

#include <string>
#include <dynamic_reconfigure/server.h>
#include <map_loader/mapConfig.h>

void printUsage()
{
  ROS_ERROR_STREAM("Usage:");
  ROS_ERROR_STREAM("rosrun map_loader lanelet2_map_loader [.OSM]");
  ROS_ERROR_STREAM(
    "rosrun map_loader lanelet2_map_loader download [X] [Y]: WARNING not implemented");
}

double lat = 30.28;
double lon = 120;
bool received_dc = false;

float RandomFloat(float a, float b) {
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = b - a;
    float r = random * diff;
    return a + r;
}

void callback(map_loader::mapConfig &config, uint32_t level, ros::NodeHandle pnh) {
  std::cout<<config.lat<< " "<< config.lon<< std::endl;
  lat = config.lat;
  lon = config.lon;
  received_dc = true;
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "lanelet_map_loader");
  ros::NodeHandle pnh("~");

  dynamic_reconfigure::Server<map_loader::mapConfig> server;
  dynamic_reconfigure::Server<map_loader::mapConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2, pnh);
  server.setCallback(f);

  if (argc < 2) {
    printUsage();
    return EXIT_FAILURE;
  }

  std::string mode(argv[1]);
  if (mode == "download" && argc < 4) {
    printUsage();
    return EXIT_FAILURE;
  }

  std::string lanelet2_filename(argv[1]);

  lanelet::ErrorMessages errors;
  ros::Rate loop_rate(1);

  
  while (ros::ok())  
  {
  if (! received_dc)
  {
    ros::spinOnce();
    loop_rate.sleep();
    continue;
  }
  // lanelet::projection::UtmProjector projector;
  // lat += RandomFloat(-0.01, 0.01);
  // lon += RandomFloat(-0.01, 0.01);
  lanelet::LaneletMapPtr map = lanelet::load(lanelet2_filename, lanelet::projection::UtmProjector(lanelet::Origin({lat, lon})), &errors);

  for (const auto & error : errors) {
    ROS_ERROR_STREAM(error);
  }
  if (!errors.empty()) {
    return EXIT_FAILURE;
  }

  double center_line_resolution;
  pnh.param<double>("center_line_resolution", center_line_resolution, 5.0);
  lanelet::utils::overwriteLaneletsCenterline(map, center_line_resolution, false);

  std::string format_version, map_version;
  lanelet::io_handlers::AutowareOsmParser::parseVersions(
    lanelet2_filename, &format_version, &map_version);

  ros::Publisher map_bin_pub =
    pnh.advertise<autoware_lanelet2_msgs::MapBin>("output/lanelet2_map", 1, true);
  autoware_lanelet2_msgs::MapBin map_bin_msg;
  map_bin_msg.header.stamp = ros::Time::now();
  map_bin_msg.header.frame_id = "map";
  map_bin_msg.format_version = format_version;
  map_bin_msg.map_version = map_version;
  lanelet::utils::conversion::toBinMsg(map, &map_bin_msg);

  map_bin_pub.publish(map_bin_msg);
  std::cout<<"updated map with: "<< lat<< " "<< lon<< std::endl;
  received_dc = false;
  ros::spinOnce();
  loop_rate.sleep();
  }
  // ros::spin();

  return 0;
}
