// Copyright 2021 Intelligent Robotics Lab
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

#include <utility>
#include <chrono>

#include "p7_follow_person/FollowPersonNode.hpp"

#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/twist.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace followperson
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

FollowPersonNode::FollowPersonNode()
: LifecycleNode("FollowPersonNode"),
  tf_buffer_(),
  tf_listener_(tf_buffer_),
  vlin_pid_(0.0, 1.0, 0.0, 0.7),
  vrot_pid_(0.0, 1.0, 0.3, 1.0),
  person_detected_(false)
{

  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

}

CallbackReturn
FollowPersonNode::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Configuring...");

  return CallbackReturn::SUCCESS;
}

CallbackReturn
FollowPersonNode::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Activating...");

  state_ = FORWARD;

  std::string error;
  while (!tf_buffer_.canTransform("base_footprint", "camera_rgb_optical_frame", tf2::TimePointZero, &error)) {}
  auto bf2camera_msg_ = tf_buffer_.lookupTransform(
    "base_footprint", "camera_rgb_optical_frame", tf2::TimePointZero);

  tf2::fromMsg(bf2camera_msg_, bf2camera_);
  
  vel_pub_->on_activate();
  timer_ = create_wall_timer(10ms, std::bind(&FollowPersonNode::control_cycle, this));
  atractive_vector_timer_ = create_wall_timer(15ms, std::bind(&FollowPersonNode::atrac_vector_cycle, this));

  return CallbackReturn::SUCCESS;
}

CallbackReturn
FollowPersonNode::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Deactivating...");

  geometry_msgs::msg::Twist stop_vel;
  stop_vel.linear.x = SPEED_STOP;
  stop_vel.angular.z =SPEED_STOP;
  vel_pub_->publish(stop_vel);

  timer_->cancel();
  vel_pub_->on_deactivate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn
FollowPersonNode::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Cleaning Up...");

  return CallbackReturn::SUCCESS;
}

CallbackReturn
FollowPersonNode::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Shutting Down...");

  return CallbackReturn::SUCCESS;
}

CallbackReturn
FollowPersonNode::on_error(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Error State");

  return CallbackReturn::SUCCESS;
}

void
FollowPersonNode::atrac_vector_cycle()
{
  tf2::Stamped<tf2::Transform> bf2person;
  std::string error;

  if (tf_buffer_.canTransform("base_footprint", "target", tf2::TimePointZero, &error)) {
    auto bf2person_msg = tf_buffer_.lookupTransform(
      "base_footprint", "target", tf2::TimePointZero);

    tf2::fromMsg(bf2person_msg, bf2person);

    atractive_vector_.x = bf2person.getOrigin().x();
    atractive_vector_.y = bf2person.getOrigin().y();
    atractive_vector_.z = bf2person.getOrigin().z();

    go_state(FORWARD);

  } else {
    RCLCPP_WARN_STREAM(get_logger(), "Error in TF base_footprint -> person [<< " << error << "]");
    go_state(TURN);
  }
}

void
FollowPersonNode::control_cycle()
{
  double x, y, vel_rot, vel_lin;
  geometry_msgs::msg::Twist out_vel;
  x = atractive_vector_.x;
  y = atractive_vector_.y;

  double angle = atan2(y, x);
  double dist = sqrt(y * y + x * x);

  vel_rot = std::clamp(vrot_pid_.get_output(angle), -0.5, 0.5);
  vel_lin = std::clamp(vlin_pid_.get_output(dist - 1), -0.3 , 0.3);

  RCLCPP_INFO(get_logger(), "vel_rot: %lf, vel_lin: %lf", vel_rot, vel_lin);

  switch (state_) {
    case FORWARD:
      out_vel.linear.x = vel_lin;
      out_vel.angular.z = vel_rot;

      if (check_forward_2_turn()) {
        go_state(TURN);
        out_vel.linear.x = SPEED_STOP;
        out_vel.angular.z = SPEED_STOP;
      }
      break;

    case TURN:
      out_vel.angular.z = SPEED_ANGULAR;

      if (check_turn_2_forward()) {
        go_state(FORWARD);
        out_vel.angular.z = SPEED_STOP;
      }
      break;

  }
  vel_pub_->publish(out_vel);
}

void
FollowPersonNode::go_state(int new_state)
{
  state_ = new_state;
}

bool
FollowPersonNode::check_forward_2_turn()
{
  //cuando pierde la pelota y gira para ver doden está
  return !person_detected_;
}

bool
FollowPersonNode::check_turn_2_forward()
{
  //cuando detecta donde esta la pelota
  return person_detected_;
}

}  // namespace followperson
