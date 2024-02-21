#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#define _USE_MATH_DEFINES

#include <math.h>

using namespace std::chrono_literals;

class StrokePiblisher : public rclcpp::Node {
 public:
  StrokePiblisher() : Node("stroke_publisher") {
    this->declare_parameter("depth", 530.0f);
    joint_angle_publisher = this->create_publisher<sensor_msgs::msg::JointState>("Machine/ActuatorStroke/Feedback", 10);
    timer_ = this->create_wall_timer(10ms, std::bind(&StrokePiblisher::timer_callback, this));
    stroke_message.name.resize(4);
    stroke_message.position.resize(4, 0.0f);
    stroke_message.velocity.resize(4, 0.0f);
    stroke_message.effort.resize(4, 0.0f);

    stroke_message.name[0] = "S0";
    stroke_message.name[1] = "S1";
    stroke_message.name[2] = "S2";
    stroke_message.name[3] = "S3";

    start_time = this->get_clock()->now();
    step_time = 5.0f;
  }

 private:
  void timer_callback() {
    auto time = this->get_clock()->now() - start_time;
    float ts = std::fmod(time.seconds(), 6.0);

    if (ts < 3.0)
      stroke_message.position.at(0) = 0.0;
    else if (ts < 4.0)
      stroke_message.position.at(0) = 0.0 + 90.0 * std::sin(std::fmod(time.seconds(), 1.0) * M_PI_2);
    else if (ts < 5.0)
      stroke_message.position.at(0) = 90.0;
    else
      stroke_message.position.at(0) = 90.0 - 90.0 * std::sin(std::fmod(time.seconds(), 1.0) * M_PI_2);

    if (ts < 1.0)
      stroke_message.position.at(1) = 390.0 + (130 + time.seconds()) * std::sin(std::fmod(ts, 1.0) * M_PI_2);
    else if (ts < 2.0)
      stroke_message.position.at(1) = 520.0 + time.seconds();
    else if (ts < 3.0)
      stroke_message.position.at(1) = (520 + time.seconds()) - (130.0 + time.seconds()) * std::sin(std::fmod(ts, 1.0) * M_PI_2);
    else
      stroke_message.position.at(1) = 390.0;

    if (ts < 1.0)
      stroke_message.position.at(2) = 580.0;
    else if (ts < 2.0)
      stroke_message.position.at(2) = 580.0 - 130 * std::sin(std::fmod(ts, 1.0) * M_PI_2);
    else if (ts < 4.0)
      stroke_message.position.at(2) = 450.0;
    else if (ts < 5.0)
      stroke_message.position.at(2) = 450.0 + 130 * std::sin(std::fmod(ts, 1.0) * M_PI_2);
    else
      stroke_message.position.at(2) = 580.0;

    if (ts < 1.0)
      stroke_message.position.at(3) = 420.0;
    else if (ts < 2.0)
      stroke_message.position.at(3) = 420.0 + 160.0 * std::sin(std::fmod(ts, 1.0) * M_PI_2);
    else if (ts < 4.0)
      stroke_message.position.at(3) = 580.0;
    else if (ts < 5.0)
      stroke_message.position.at(3) = 580.0 - 160.0 * std::sin(std::fmod(ts, 1.0) * M_PI_2);
    else
      stroke_message.position.at(3) = 420.0;

    stroke_message.header.stamp = this->get_clock()->now();
    joint_angle_publisher->publish(stroke_message);
  }

  rclcpp::Time start_time;
  float step_time;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::JointState stroke_message;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_angle_publisher;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StrokePiblisher>());
  rclcpp::shutdown();
  return 0;
}
