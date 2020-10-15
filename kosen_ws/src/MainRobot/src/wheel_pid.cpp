#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "basic_utility/raspi/include/rotary_inc.hpp"
#include "basic_utility/raspi/include/pigpiod.hpp"
#include <vector>
#include <array>
#include <std_msgs/Int16.h>
#include</usr/local/include/eigen3/Dense>

using namespace Eigen;
using std::vector;
using std::array;
using ros::RotaryInc;
using ros::Pigpiod;

double v_x, v_y, v_rad, robot_rad;

class PID{
  public:
    PID(float _p_gain, float _i_gain, float _d_gain, float _freq):p_gain_(_p_gain), i_gain_(_i_gain), d_gain_(_d_gain), freq_(_freq){}
    ~PID(){}
    float operator()(float _ref_val, float _now_val){
      return p_gain_ * (_ref_val - _now_val) + I - D;
    }
  private:
    const float p_gain_;
    const float i_gain_;
    const float d_gain_;
    const float freq_;
};

void velCallback(const std_msgs::Float32 &msg){
  v_x = ;
  v_y = ;
}

void velRadCallback(const std_msgs::Float32 &msg){
  v_rad = ;
}

void robotRadCallback(const std_msgs::Float32 &msg){
  robot_rad = ;
}

void omuni4Inv(float(&_wheel_vel)[4]){
  MatrixXd inv_matrix(3, 4);
  inv_matrix << -1, 0, L,
                 0, 1, L,
                -1, 0, L,
                 0,-1, L;
  Vector4f vel_vec(v_x, v_y, v_rad);
  Vector4f vel_wheel = inv_matrix * vel_vec;
  for(int i = 0; i < 4; ++i){
    _wheel_vel[i] = vel_wheel(i);
  }
}

constexpr float voltage_max_vel = 5;
constexpr float voltage_min_vel = 2;
int velPWMConv(float &&vel){
  //FIXME:このmap式は間違っている可能性がある
  return static_cast<int>((vel - voltage_min_vel) * (255 - 0) / (voltage_max_vel - voltage_min_vel) + 0);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "wheel");
  ros::NodeHandle n;

  ros::Subscriber robot_vel_sub = n.subscribe("/cmd_vel", 10, velCallback);
  ros::Subscriber robot_vel_rad_sub = n.subscribe("/vel_rad", 10, velRadCallback);
  ros::Subscriber robot_rad_sub = n.subscribe("robot_rad", 10, robotRadCallback);
  ros::ServiceClient motor_speed = n.serviceClient<motor_serial::motor_serial>("motor_info");
  
  motor_serial::motor_serial srv;
  
  std::array<float, 4> wheel_vel_ref;
  std::array<float, 4> wheel_vel_now;
  constexpr std::array<unsigned int, 4> WHEEL_ID = {0, 1, 2, 3};
  constexpr std::array<unsigned int, 4> WHEEL_CMD = {0, 1, 2 ,3};
  
  PID wheelPID[4] = {(1, 1, 1), (1, 1, 1), (1, 1, 1), (1, 1, 1)};
  RotaryInc wheelEnc[4] = {RotaryInc(26, 19, 1), RotaryInc(6, 5, 1), RotaryInc(11, 9, 1), RotaryInc(1, 2, 3)};
  ros::Rate loop_rate(100);

  while(ros::ok()){
    wheel_now[i] = ((((double)rotary[i].get() / (resolution * multiplication)) * 101.6) / 1000);
    omuni4Inv(wheel_vel_ref);    //各タイヤの目標速度を取得
    for(int i = 0; i < 4; ++i){
      srv.request.id = WHEEL_ID[i];
	    srv.request.cmd = WHEEL_CMD[i];
      srv.request.data = velPWMConv(wheelPID[i](wheel_vel_ref[i], wheel_vel_now[i]));
	    motor_speed.call(srv); 
    }
    loop_rate.sleep();
    ros::spinOnce();
  }
}