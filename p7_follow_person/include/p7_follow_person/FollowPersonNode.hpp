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

#ifndef FOLLOWPERSONNODE_HPP_
#define FOLLOWPERSONNODE_HPP_

#include "p7_follow_person/PIDController.hpp"

#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"

#include "rclcpp/node.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/macros.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

using namespace std::chrono_literals;

namespace followperson
{

class FollowPersonNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(FollowPersonNode)

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  FollowPersonNode();
  void control_cycle();
  void atrac_vector_cycle();

  void laser_callback(const geometry_msgs::msg::Vector3 msg);

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state);

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state);

private:
  static const int FORWARD = 0;
  static const int TURN = 1;
  static const int STOP = 2;
  double PESO_ATRAC = 1.0;
  double PESO_REPUL = 2.0;

  int state_;
  float dist_fwd_;
  float dist_turn_;

  void go_state(int new_state);
  bool check_forward_2_turn();
  bool check_turn_2_forward();

  static constexpr float SPEED_ANGULAR = 0.5f;
  static constexpr float SPEED_STOP = 0.0f;

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr hsv_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr atractive_vector_timer_;
  geometry_msgs::msg::Twist out_vel_;

  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2::Stamped<tf2::Transform> bf2camera_;

  geometry_msgs::msg::Vector3 atractive_vector_;

  bool person_detected_;
  PIDController vlin_pid_, vrot_pid_;
};

}  // namespace followperson

#endif  // FOLLOWPERSONNODE_HPP_
