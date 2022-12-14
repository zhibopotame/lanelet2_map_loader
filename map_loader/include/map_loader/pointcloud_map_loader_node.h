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

#include <string>
#include <vector>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/MapMetaData.h>

class PointCloudMapLoaderNode
{
public:
  explicit PointCloudMapLoaderNode(const std::vector<std::string> & pcd_paths);

private:
  ros::NodeHandle nh_{""};
  ros::NodeHandle private_nh_{"~"};

  ros::Publisher pub_pointcloud_map_;
  ros::Subscriber map_metadata_sub;
  nav_msgs::MapMetaData map_metadata;

  sensor_msgs::PointCloud2 loadPCDFiles(const std::vector<std::string> & pcd_paths);
  void publish_tf();
  void callbackMapMetaData(const nav_msgs::MapMetaData &msg);
};
