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

#include <visualization_msgs/MarkerArray.h>

#include <autoware_lanelet2_msgs/MapBin.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_projection/UTM.h>

#include <lanelet2_extension/regulatory_elements/autoware_traffic_light.h>
#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_extension/visualization/visualization.h>

#include <vector>
#include <nav_msgs/OccupancyGrid.h>

static bool g_viz_lanelets_centerline = true;
static ros::Publisher g_map_pub;

static ros::Publisher occ_map_pub;

void insertMarkerArray(
  visualization_msgs::MarkerArray * a1, const visualization_msgs::MarkerArray & a2)
{
  a1->markers.insert(a1->markers.end(), a2.markers.begin(), a2.markers.end());
}

void setColor(std_msgs::ColorRGBA * cl, double r, double g, double b, double a)
{
  cl->r = r;
  cl->g = g;
  cl->b = b;
  cl->a = a;
}

void binMapCallback(autoware_lanelet2_msgs::MapBin msg)
{
  lanelet::LaneletMapPtr viz_lanelet_map(new lanelet::LaneletMap);

  lanelet::utils::conversion::fromBinMsg(msg, viz_lanelet_map);
  ROS_INFO("Map is loaded\n");

  // get lanelets etc to visualize
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(viz_lanelet_map);
  lanelet::ConstLanelets road_lanelets = lanelet::utils::query::roadLanelets(all_lanelets);
  lanelet::ConstLanelets crosswalk_lanelets =
    lanelet::utils::query::crosswalkLanelets(all_lanelets);
  lanelet::ConstLineStrings3d pedestrian_markings =
    lanelet::utils::query::getAllPedestrianMarkings(viz_lanelet_map);
  lanelet::ConstLanelets walkway_lanelets = lanelet::utils::query::walkwayLanelets(all_lanelets);
  std::vector<lanelet::ConstLineString3d> stop_lines =
    lanelet::utils::query::stopLinesLanelets(road_lanelets);
  std::vector<lanelet::TrafficLightConstPtr> tl_reg_elems =
    lanelet::utils::query::trafficLights(all_lanelets);
  std::vector<lanelet::AutowareTrafficLightConstPtr> aw_tl_reg_elems =
    lanelet::utils::query::autowareTrafficLights(all_lanelets);
  std::vector<lanelet::DetectionAreaConstPtr> da_reg_elems =
    lanelet::utils::query::detectionAreas(all_lanelets);
  lanelet::ConstLineStrings3d parking_spaces =
    lanelet::utils::query::getAllParkingSpaces(viz_lanelet_map);
  lanelet::ConstPolygons3d parking_lots = lanelet::utils::query::getAllParkingLots(viz_lanelet_map);
  lanelet::ConstPolygons3d obstacle_polygons =
    lanelet::utils::query::getAllObstaclePolygons(viz_lanelet_map);

  std_msgs::ColorRGBA cl_road, cl_cross, cl_pedestrian_markings, cl_ll_borders, cl_stoplines,
    cl_trafficlights, cl_detection_areas, cl_parking_lots, cl_parking_spaces, cl_lanelet_id,
    cl_obstacle_polygons;
  setColor(&cl_road, 0.27, 0.27, 0.27, 0.999);
  setColor(&cl_cross, 0.27, 0.3, 0.27, 0.5);
  setColor(&cl_pedestrian_markings, 0.5, 0.5, 0.5, 0.999);
  setColor(&cl_ll_borders, 0.5, 0.5, 0.5, 0.999);
  setColor(&cl_stoplines, 0.5, 0.5, 0.5, 0.999);
  setColor(&cl_trafficlights, 0.5, 0.5, 0.5, 0.8);
  setColor(&cl_detection_areas, 0.27, 0.27, 0.37, 0.5);
  setColor(&cl_obstacle_polygons, 0.4, 0.27, 0.27, 0.5);
  setColor(&cl_parking_lots, 0.5, 0.5, 0.0, 0.3);
  setColor(&cl_parking_spaces, 1.0, 0.647, 0.0, 0.6);
  setColor(&cl_lanelet_id, 0.5, 0.5, 0.5, 0.999);

  visualization_msgs::MarkerArray map_marker_array;

  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::lineStringsAsMarkerArray(stop_lines, "stop_lines", cl_stoplines, 0.5));
  insertMarkerArray(
    &map_marker_array, lanelet::visualization::laneletDirectionAsMarkerArray(road_lanelets));
  insertMarkerArray(
    &map_marker_array, lanelet::visualization::laneletsAsTriangleMarkerArray(
                         "crosswalk_lanelets", crosswalk_lanelets, cl_cross));
  insertMarkerArray(
    &map_marker_array, lanelet::visualization::pedestrianMarkingsAsMarkerArray(
                         pedestrian_markings, cl_pedestrian_markings));
  insertMarkerArray(
    &map_marker_array, lanelet::visualization::laneletsAsTriangleMarkerArray(
                         "walkway_lanelets", walkway_lanelets, cl_cross));
  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::obstaclePolygonsAsMarkerArray(obstacle_polygons, cl_obstacle_polygons));
  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::detectionAreasAsMarkerArray(da_reg_elems, cl_detection_areas));
  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::parkingLotsAsMarkerArray(parking_lots, cl_parking_lots));
  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::parkingSpacesAsMarkerArray(parking_spaces, cl_parking_spaces));
  insertMarkerArray(
    &map_marker_array, lanelet::visualization::laneletsBoundaryAsMarkerArray(
                         road_lanelets, cl_ll_borders, g_viz_lanelets_centerline));
  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::autowareTrafficLightsAsMarkerArray(aw_tl_reg_elems, cl_trafficlights));
  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::generateLaneletIdMarker(road_lanelets, cl_lanelet_id));
  insertMarkerArray(
    &map_marker_array,
    lanelet::visualization::laneletsAsTriangleMarkerArray("road_lanelets", road_lanelets, cl_road));

  g_map_pub.publish(map_marker_array);

  bool publish_occ_map = false;

  if (!publish_occ_map)
    return;

  nav_msgs::OccupancyGrid occ_map;
  occ_map.header.frame_id = "map";
  occ_map.header.stamp = ros::Time::now();
  occ_map.info.width = 9015;
  occ_map.info.height = 5857;
  occ_map.info.resolution = 0.05;
  occ_map.info.origin.position.x = -9015/2 * 0.05;
  occ_map.info.origin.position.y = -5857/2 * 0.05;
  occ_map.info.origin.orientation.w = 1.0;
  occ_map.data.assign(9015 * 5857, -1);


  std::cout<<"marker size "<<map_marker_array.markers.size()<<std::endl;
  for (auto marker : map_marker_array.markers)
  {
    for (auto point : marker.points)
    {
      unsigned int grid_x = (unsigned int)((point.x - occ_map.info.origin.position.x) / occ_map.info.resolution);
      unsigned int grid_y = (unsigned int)((point.y - occ_map.info.origin.position.y) / occ_map.info.resolution);
      unsigned int idx = grid_x + grid_y * occ_map.info.width;
      // std::cout<<idx <<std::endl;
      if (idx > occ_map.info.width * occ_map.info.height )
        continue;
      occ_map.data[idx] = 100;
      // std::cout<<"idx = "<<idx << " value = " << occ_map.data[idx] <<std::endl;
    }
  }
  std::cout<<"publishing marker occ map"<<std::endl;
  occ_map_pub.publish(occ_map);

}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "lanelet2_map_visualizer");
  ros::NodeHandle pnh("~");
  ros::Subscriber bin_map_sub;

  bin_map_sub = pnh.subscribe("input/lanelet2_map", 1, binMapCallback);
  g_map_pub = pnh.advertise<visualization_msgs::MarkerArray>("output/lanelet2_map_marker", 1, true);

  occ_map_pub = pnh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);

  ros::spin();

  return 0;
}
