/************************
 * Copyright (c) 2022 Aniruddh Balram
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ************************/
/**
 *  @file    publisher_member_function.cpp
 *  @author  Aniruddh Balram
 *  @date    12/05/2022
 *  @version 2.0
 *
 *  @brief This file is a combined publisher/server
 *
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;


/**
 * @brief MinimalPublisher class which contains publisher/server constructor and methods
 * 
 */
class MinimalPublisher : public rclcpp::Node {
 public:
  std::string base_message = "Hi there! Waiting on numbers to be summed!";
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0) {
    // Parameter for publisher frequency
    auto freq_p = rcl_interfaces::msg::ParameterDescriptor();
    freq_p.description = "Sets the frequency in Hz";
    this->declare_parameter("freq", 3.0, freq_p);
    auto frequency =
      this->get_parameter("freq").
      get_parameter_value().get<std::float_t>();
    // Publisher initialization
    publisher_ = this->create_publisher<std_msgs::msg
    ::String>("topic", 10);
    // Set timer to 500ms(calls callback twice in a second)
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    auto serviceCallbackPtr =
        std::bind(&MinimalPublisher::add_two_integers, this,
                  std::placeholders::_1, std::placeholders::_2);
    // Server initialization
    service_ = create_service<example_interfaces::srv::AddTwoInts>(
        "add_two_ints", serviceCallbackPtr);
    if (this->count_subscribers("topic") == 0) {
    RCLCPP_WARN_STREAM(this->get_logger(), "There is no-one subscribing!");
  }
    this->get_logger().set_level(rclcpp::Logger::Level::Debug);
     // Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

 private:
 /**
  * @brief timer callback to publish message
  * 
  */
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = std::to_string(count_++) + ") " + base_message;
    RCLCPP_WARN_STREAM(this->get_logger(), "Publishing: "
    << message.data.c_str());
    publisher_->publish(message);
    handle_pose();
  }
  /**
   * @brief function to obtain the TF Messages
   * 
   */
  void handle_pose() {
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "talk";

    // Turtle only exists in 2D, thus we get x and y translation
    // coordinates from the message and set the z coordinate to 0
    t.transform.translation.x = 1.2;
    t.transform.translation.y = 2.0;
    t.transform.translation.z = 3.0;

    // For the same reason, turtle can only rotate around one axis
    // and this why we set rotation in x and y to 0 and obtain
    // rotation in z axis from the message
    tf2::Quaternion q;
    q.setRPY(3.14, 2.1, 1.7);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Send the transformation
    tf_broadcaster_->sendTransform(t);
  }
/**
 * @brief basic request-response to add two integers
 * 
 * @param request server request
 * @param response server response
 */
  void add_two_integers(const std::shared_ptr<example_interfaces
  ::srv::AddTwoInts::Request> request,
          std::shared_ptr<example_interfaces
          ::srv::AddTwoInts::Response>      response) {
  response->sum = request->a + request->b;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
  "Incoming request\na: %ld" " b: %ld",
                request->a, request->b);
  if(response->sum > INT32_MAX) {
  RCLCPP_ERROR_STREAM (this->get_logger(),
  "value overflow!");
    }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
  "sending back response: [%ld]", (long int)response->sum);
  base_message = "The sum is: " + std::to_string(response->sum);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>
  ::SharedPtr publisher_;
  rclcpp::Service<example_interfaces::srv::AddTwoInts>
  ::SharedPtr service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  size_t count_;
};

void quit(int signum) {
  if (signum == 2) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("rclcpp"),
                        "Process ended with process-code 2!");
  }
}

int main(int argc, char * argv[]) {
  signal(SIGINT, quit);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
