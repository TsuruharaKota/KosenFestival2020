#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <vector>
#include <array>
#include <std_msgs/Int16.h>
#include</usr/local/include/eigen3/Dense>
#include</usr/local/include/eigen3/Core>
#include</usr/local/include/eigen3/Geometry>
#include<cmath>

using namespace Eigen;
using std::vector;
using std::array;

float v_x = 0, v_y = 0, v_rad = 0, robot_rad = 0;
constexpr float L = 50;
void velXCallback(const std_msgs::Float32 &msg){
    v_x = msg.data;
}
void velYCallback(const std_msgs::Float32 &msg){
    v_y = msg.data;
}

void omuni4Inv(float(&_wheel_vel)[4]){
    MatrixXf inv_matrix(4, 3);
    inv_matrix << -sin(robot_rad + (M_PI / 4)),       cos(robot_rad + (M_PI / 4))      , L,
                  -sin(robot_rad + ((M_PI * 3) / 4)), cos(robot_rad + ((M_PI * 3) / 4)), L,
                  -sin(robot_rad + ((M_PI * 5) / 4)), cos(robot_rad + ((M_PI * 3) / 4)), L,
                  -sin(robot_rad + ((M_PI * 7) / 4)), cos(robot_rad + ((M_PI * 7) / 4)), L;
    Vector3f vel_vec(v_x, v_y, v_rad);
    Vector4f vel_wheel = inv_matrix * vel_vec;
    for(int i = 0; i < 4; ++i){
      _wheel_vel[i] = vel_wheel(i);
    }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "wheel");
  ros::NodeHandle n;

  ros::Subscriber robot_vel_x_sub = n.subscribe("/cmd_vel_x", 10, velXCallback);
  ros::Subscriber robot_vel_y_sub = n.subscribe("/cmd_vel_y", 10, velYCallback);

  float wheel_vel_ref[4]{};
  ros::Rate loop_rate(100);
  
  while(ros::ok()){
    omuni4Inv(wheel_vel_ref);    //各タイヤの目標速度を取得
    ROS_INFO("%3lf %3lf %3lf %3lf", wheel_vel_ref[0], wheel_vel_ref[1], wheel_vel_ref[2], wheel_vel_ref[3]);
    loop_rate.sleep();
    ros::spinOnce();
  }
}