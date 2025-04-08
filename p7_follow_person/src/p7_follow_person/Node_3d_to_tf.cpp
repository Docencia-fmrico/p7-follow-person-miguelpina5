// Copyright 2023 Intelligent Robotics Lab
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

#include <memory>

#include "p7_follow_person/Node_3d_to_tf.hpp"

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
using std::placeholders::_1;

namespace followperson
{

Node_3d_to_tf::Node_3d_to_tf()
: Node("Node_3d_to_tf")
{
    array3D_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
        "detection_3d", 10, std::bind(&Node_3d_to_tf::array3D_callback, this, _1));

    vector3_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
        "vector3_msg_3d", 100);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
}

void
Node_3d_to_tf::array3D_callback(vision_msgs::msg::Detection3DArray::UniquePtr detectionArray)
{
    for(const auto& detection : detectionArray->detections){
        if(detection.results[0].hypothesis.class_id == "person"){
            transform_.header.stamp = now();
            transform_.header.frame_id = "camera_rgb_optical_frame";
            transform_.child_frame_id = "target";
            
            transform_.transform.translation.x = detection.bbox.center.position.x;
            transform_.transform.translation.y = detection.bbox.center.position.y;
            transform_.transform.translation.z = detection.bbox.center.position.z;

            transform_.header.stamp = now();
            tf_broadcaster_->sendTransform(transform_);
        }
    }
}
} // namespace followperson