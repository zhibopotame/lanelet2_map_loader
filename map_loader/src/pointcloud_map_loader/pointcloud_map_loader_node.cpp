/*
 * Copyright 2020 TierIV. All rights reserved.
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
 */

/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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
 */

#include <map_loader/pointcloud_map_loader_node.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/filesystem.hpp>
#include <ros/ros.h>

PointCloudMapLoaderNode::PointCloudMapLoaderNode(const std::vector<std::string> & pcd_paths)
{
  pub_pointcloud_map_ = private_nh_.advertise<sensor_msgs::PointCloud2>("output/pointcloud_map", 1, true);
  map_metadata_sub = nh_.subscribe("map_metadata", 1, &PointCloudMapLoaderNode::callbackMapMetaData, this);

  const auto pcd = loadPCDFiles(pcd_paths);

  if (pcd.width == 0) {
    ROS_ERROR("No PCD was loaded: pcd_paths.size() = %zu", pcd_paths.size());
    return;
  }
  pub_pointcloud_map_.publish(pcd);
}

void PointCloudMapLoaderNode::callbackMapMetaData(const nav_msgs::MapMetaData &msg)
{
  map_metadata = msg;
  
  publish_tf();
}

void PointCloudMapLoaderNode::publish_tf()
{
  tf::TransformBroadcaster br; 
  tf::Transform transform;
  double new_origin_x =  map_metadata.origin.position.x + map_metadata.width/2.0 * map_metadata.resolution;
  double new_origin_y =  map_metadata.origin.position.y + map_metadata.height/2.0 * map_metadata.resolution;
  transform.setOrigin(tf::Vector3(-new_origin_x, -new_origin_y, 1.2));
  tf::Quaternion q;
  q.setRPY(0,0,0);
  transform.setRotation(q);
  std::cout<<"publishing tf pcl_map map"<<std::endl;
  ros::Rate loop_rate(50);
  while (ros::ok())
  {
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "pcl_map"));
    ros::spinOnce();
    loop_rate.sleep();
  }
}

sensor_msgs::PointCloud2 PointCloudMapLoaderNode::loadPCDFiles(
  const std::vector<std::string> & pcd_paths)
{
  sensor_msgs::PointCloud2 whole_pcd{};

  sensor_msgs::PointCloud2 partial_pcd;
  for (const auto & path : pcd_paths) {
    if (pcl::io::loadPCDFile(path, partial_pcd) == -1) {
      ROS_ERROR_STREAM("PCD load failed: " << path);
    }

    if (whole_pcd.width == 0) {
      whole_pcd = partial_pcd;
    } else {
      whole_pcd.width += partial_pcd.width;
      whole_pcd.row_step += partial_pcd.row_step;
      whole_pcd.data.reserve(whole_pcd.data.size() + partial_pcd.data.size());
      whole_pcd.data.insert(whole_pcd.data.end(), partial_pcd.data.begin(), partial_pcd.data.end());
    }
  }

  whole_pcd.header.frame_id = "pcl_map";

  return whole_pcd;
}
