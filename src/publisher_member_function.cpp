// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
 public:
  std::string base_message = "Hi there! Waiting on numbers to be summed!";
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    auto serviceCallbackPtr =
        std::bind(&MinimalPublisher::add_two_integers, this,
                  std::placeholders::_1, std::placeholders::_2);
    service_ = create_service<example_interfaces::srv::AddTwoInts>(
        "add_two_ints", serviceCallbackPtr);
    if (this->count_subscribers("topic") == 0) {
    RCLCPP_WARN_STREAM(this->get_logger(), "There is no-one subscribing!");
  }
    this->get_logger().set_level(rclcpp::Logger::Level::Debug);
  }

 private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = std::to_string(count_++) + ") " + base_message;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  void add_two_integers(const std::shared_ptr<example_interfaces
  ::srv::AddTwoInts::Request> request,
          std::shared_ptr<example_interfaces
          ::srv::AddTwoInts::Response>      response) {
  response->sum = request->a + request->b;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
  "Incoming request\na: %ld" " b: %ld",
                request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
  "sending back response: [%ld]", (long int)response->sum);
  base_message = "The sum is: " + std::to_string(response->sum);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>
  ::SharedPtr publisher_;
  rclcpp::Service<example_interfaces::srv::AddTwoInts>
  ::SharedPtr service_;
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
