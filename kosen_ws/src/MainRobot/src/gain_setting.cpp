#include<ros/ros.h>
#include"MainRobot/pid.h"
#include"MainRobot/motor_serial.h"
#include "basic_utility/raspi/include/rotary_inc.hpp"
#include "basic_utility/raspi/include/pigpiod.hpp"

using ros::RotaryInc;
using ros::Pigpiod;

constexpr float voltage_max_vel = 6;
constexpr float voltage_min_vel = 2;
int velPWMConv(float &&vel_){
  //FIXME:このmap式は間違っている可能性がある
  return static_cast<int>((vel_ - voltage_min_vel) * (255 - 0) / (voltage_max_vel - voltage_min_vel) + 0);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "gain_setting");
    ros::NodeHandle n;

    ros::Rate loop_rate(100);
    
    ros::ServiceClient motor_speed = n.serviceClient<MainRobot::motor_serial>("motor_info");

    MainRobot::motor_serial srv;

    PID wheelPID_single(1, 1, 1, 1);
    RotaryInc wheelEnc_single(26, 19, 1);
    float prev_speed = 0;
    constexpr float ref_speed = 2.0;
    constexpr int resolution = 1;
    constexpr int multiplication = 1;
    constexpr int WHEEL_ID = 1;
    constexpr int WHEEL_CMD = 1;
    
    while(ros::ok()){
        float now_speed = (((static_cast<double>(wheelEnc_single.get()) / (resolution * multiplication)) * 101.6) / 1000);;
        srv.request.id = WHEEL_ID;
	    srv.request.cmd = WHEEL_CMD;
        srv.request.data = velPWMConv(wheelPID_single(ref_speed, now_speed, prev_speed));
	    motor_speed.call(srv);
        prev_speed = now_speed;
        loop_rate.sleep();
        ros::spinOnce();
    }
}