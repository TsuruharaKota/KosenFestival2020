#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include "basic_utility/raspi/include/rotary_inc.hpp"
#include "basic_utility/raspi/include/pigpiod.hpp"
#include <array>
#include <std_msgs/Int16.h>
#include "MainRobot/pid.h"
using namespace Eigen;
using std::vector;
using std::array;
using ros::RotaryInc;
using ros::Pigpiod;

double v_x, v_y, v_rad, robot_rad;

constexpr float voltage_max_vel = 5;
constexpr float voltage_min_vel = 2;
std::array<float, 4> ref_wheel_vel;
void velCallback(const std_msgs::Float32MultiArray &msg_){
  for(int i = 0; i < 4; ++i){
    ref_wheel_vel[i] = msg_.data[i];
  }
}
int velPWMConv(float &&vel_){
  //FIXME:このmap式は間違っている可能性がある
  return static_cast<int>((vel_ - voltage_min_vel) * (255 - 0) / (voltage_max_vel - voltage_min_vel) + 0);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "wheel");
  ros::NodeHandle n;

  ros::Subscriber robot_vel_sub = n.subscribe("/cmd_vel_array", 10, velCallback);
  ros::ServiceClient motor_speed = n.serviceClient<motor_serial::motor_serial>("motor_info");
  
  motor_serial::motor_serial srv;
  
  std::array<float, 4> wheel_vel_ref;
  std::array<float, 4> wheel_vel_now;
  std::array<float, 4> wheel_vel_prev;

  constexpr std::array<unsigned int, 4> WHEEL_ID = {0, 1, 2, 3};
  constexpr std::array<unsigned int, 4> WHEEL_CMD = {0, 1, 2 ,3};
  
  PID wheelPID<float>[4] = {(1, 1, 1), (1, 1, 1), (1, 1, 1), (1, 1, 1)};
  RotaryInc wheelEnc[4] = {RotaryInc(26, 19, 1), RotaryInc(6, 5, 1), RotaryInc(11, 9, 1), RotaryInc(1, 2, 3)};
  ros::Rate loop_rate(100);

  //FIXME:パラメータ設定が必要
  constexpr int resolution = 1;
  constexpr int multiplication = 1;
  
  while(ros::ok()){
    for(int i = 0; i < 4; ++i){
      wheel_vel_now[i] = (((static_cast<double>(wheelEnc[i].get()) / (resolution * multiplication)) * 101.6) / 1000);
      srv.request.id = WHEEL_ID[i];
	    srv.request.cmd = WHEEL_CMD[i];
      srv.request.data = velPWMConv(wheelPID[i](wheel_vel_ref[i], wheel_vel_now[i], wheel_vel_prev[i]));
	    motor_speed.call(srv);
      wheel_vel_prev[i] = wheel_vel_now[i]
    }
    loop_rate.sleep();
    ros::spinOnce();
  }
}