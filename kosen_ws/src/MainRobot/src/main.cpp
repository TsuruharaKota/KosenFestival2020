#include<ros/ros.h>
#include"MainRobot/catmull_rom.h"
#include</usr/local/include/eigen3/Dense>
#include<string>
#include<math.h>
#include<cmath>
#include<vector>
#include<tuple>
#include<array>
#include<utility>
#include</usr/local/include/eigen3/Dense>
#include<iostream>
#include<queue>
#include<fstream>
#include<string>
#include<iomanip>
#include<nav_msgs/Path.h>
#include<std_msgs/String.h>
#include<geometry_msgs/PoseStamped.h>
#include<tf/transform_broadcaster.h>

using namespace Eigen;
using catmull_tuple = std::tuple<float, float, float>;
using route_tuple = std::tuple<float, float, float, float>;
using std::cout;
using std::endl;

enum class CoursePt{
  acrobatic,
  qualifying,
  final
};

template<CoursePt color>
std::vector<route_tuple> routeInit(){
  std::vector<route_tuple> point;
  switch(color){
    case CoursePt::acrobatic:
      point.push_back(route_tuple( 0.0f, 0.0f,  1.0f,   0.0f));
      point.push_back(route_tuple( 0.0f, 1.0f,  1.0f,  10.0f));
      point.push_back(route_tuple( 0.0f, 2.0f,  1.0f,  20.0f));
      point.push_back(route_tuple( 0.0f, 3.0f,  1.0f,  10.0f));
      point.push_back(route_tuple(0.15f, 3.5f, -1.0f,   5.0f));
      point.push_back(route_tuple( 0.5f, 3.9f, -1.0f,  10.0f));
      point.push_back(route_tuple( 1.0f, 4.0f,  1.0f,  20.0f));
      point.push_back(route_tuple( 2.0f, 4.0f,  1.0f,  20.0f));
      point.push_back(route_tuple( 3.0f, 4.0f,  1.0f,  10.0f));
      point.push_back(route_tuple( 4.0f, 4.0f,  1.0f,  0.0f));
      break;
    case CoursePt::qualifying:
      break;
    case CoursePt::final:
      break;
  }
  return point;
}

bool update(double ref_time, double timer_counter){
    static double elapsed_time_prev{};
    double elapsed_time = (timer_counter * 0.01) - elapsed_time_prev;
    if(ref_time < elapsed_time){
        elapsed_time_prev = elapsed_time;
        return true;
    }else{
        //ROS_INFO("%lf %lf", ref_time, elapsed_time);
        return false;
    }
}
int main(int argc, char **argv){
    ros::init(argc, argv, "main");
    ros::NodeHandle n;

    std::string source_frame = "base_link";
    std::string target_frame = "unit0_link";
       
    CatmullRomSpline splineObject(routeInit<CoursePt::acrobatic>(), 100);
    std::vector<Vector4f> route_info = splineObject();

    //0.01秒ごとに更新される
    ros::Rate loop_rate(100);
    int route_counter{};
    int loop_counter{};
    double ref_x = 0, ref_y = 0, ref_rad = 0, ref_speed = 0, ref_time = 0;
    while(ros::ok()){
        ++loop_counter;
        if(update(ref_time, loop_counter)){
            ++route_counter;
            Vector4f nowParam = route_info.at(route_counter);
            ref_x = nowParam(0);
            ref_y = nowParam(1);
            ref_speed = nowParam(2);
            ref_rad = nowParam(3);
            ref_time = ((ref_x * ref_x) + (ref_y * ref_y)) / ref_speed;
            //ROS_INFO("ref_time = %lf\n", ref_speed);
            ROS_INFO("%lf %lf\n", ref_x, ref_y);
            //ROS_INFO("%lf %lf %lf %lf %lf\n", ref_x, ref_y, ref_rad, ref_speed, ref_time);
        }
        
        geometry_msgs::Pose t_pose;

        t_pose.position.x = ref_x / 10;
        t_pose.position.y = ref_y / 10;
        t_pose.orientation.w = 1.0;
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        poseMsgToTF(t_pose, transform);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), source_frame, target_frame));
        ros::spinOnce();
        loop_rate.sleep();
    }
}