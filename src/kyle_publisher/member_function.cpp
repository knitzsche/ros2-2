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
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <random>
#include <cstdlib>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("kyle_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic");
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
    std::string adjectives[55] = {"awesome","helpful","southern","able","actual","entire","latter","boring","pleasant","tiny","afraid","massive","foreign","severe","saucy","unfair","sorry","civil","alive","former","pregnant","ugly","lucky","basic","legal","inner","recent","distinct","ugly","hungry","sudden","former","actual","inner","foreign","tiny","angry","alive","pleasant","lucky","willing","mental","civil","global","aware","decent","distinct","pregnant","asleep","sorry","massive","southern","eastern","wooden","nervous"};
    std::string nouns[55] = {"hall","insurance","assistance","worker","transportation","dinner","election","technology","surgery","speaker","paper","university","speech","friendship","recipe","definition","description","reflection","advertising","variety","person","knowledge","chest","basis","patience","warning","connection","drama","chocolate","performance","county","nature","union","activity","information","resolution","organization","television","tale","success","growth","fortune","story","newspaper","employer","editor","science","sympathy","republic","tennis","song","celebration","complaint","engineering","king"}; 

  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    std::random_device rd; // obtain a random number from hardware
    std::mt19937 eng(rd()); // seed the generator
    std::uniform_int_distribution<> distr(0, 54); // define the range
    std::string adj = adjectives[distr(eng)];
    std::string noun = nouns[distr(eng)];
    message.data = adj + " " + noun;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
