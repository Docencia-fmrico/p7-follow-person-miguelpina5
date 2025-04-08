// Copyright 2024 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NODE_3D_TO_TF_HPP_
#define NODE_3D_TO_TF_HPP_

#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "rclcpp/node.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/macros.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

using namespace std::chrono_literals;

namespace followperson
{

class Node_3d_to_tf : public rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Node_3d_to_tf)

  Node_3d_to_tf();
  void array3D_callback(vision_msgs::msg::Detection3DArray::UniquePtr msg);

private:

  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr array3D_sub_;
  tf2::Stamped<tf2::Transform> camera2person_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr vector3_pub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped transform_;
};

}  // namespace followperson

#endif  // NODE_3D_TO_TF_HPP_