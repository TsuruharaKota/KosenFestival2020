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
#include<iostream>
#include<queue>
#include<fstream>
#include<iomanip>
#include<chrono>
#include<nav_msgs/Path.h>
#include<std_msgs/String.h>
#include<geometry_msgs/PoseStamped.h>
#include<tf/transform_broadcaster.h>
#include<std_msgs/Float32.h>

using namespace std::chrono;
using namespace Eigen;
using catmull_tuple = std::tuple<float, float, float>;
using route_tuple = std::tuple<float, float, float, float>;
using std::cout;
using std::endl;

inline double get_time_sec(void){
    return static_cast<double>(duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count())/1000000000;
}

enum class CoursePt{
  acrobatic,
  straight,
  qualifying,
  final
};

template<CoursePt color>
std::vector<route_tuple> routeInit(){
  std::vector<route_tuple> point;
  switch(color){
    case CoursePt::acrobatic:
    //point.push_back(route_tuple(  x[m], y[m], angle[deg],speed[mm/s]))
      point.push_back(route_tuple(  0.0f,  1000.0f,  0.0f,   1000.0f));
      point.push_back(route_tuple(  0.0f,  2000.0f,  1.0f,   1000.0f));
      point.push_back(route_tuple(  0.0f,  3000.0f,  2.0f,   1000.0f));
      point.push_back(route_tuple(  0.0f,  4000.0f,  3.0f,   1000.0f));
      point.push_back(route_tuple(  0.0f,  5000.0f,  3.0f,   1000.0f));
      point.push_back(route_tuple(  0.0f,  6000.0f,  3.0f,   1000.0f));
      point.push_back(route_tuple(  0.0f,  7000.0f,  3.0f,   1000.0f));
      point.push_back(route_tuple(  0.0f,  8000.0f,  2.0f,   1000.0f));
      point.push_back(route_tuple(  0.0f,  9000.0f,  1.0f,   1000.0f));
      point.push_back(route_tuple(  0.0f, 10000.0f,  0.0f,   1000.0f));
      break;
    case CoursePt::straight:
      point.push_back(route_tuple(  0.0f, 0.0f,  0.0f,   0.0f));
      point.push_back(route_tuple(  0.0f, 1.0f,  0.0f,   1.0f));
      point.push_back(route_tuple(  0.0f, 2.0f,  0.0f,   2.0f));
      point.push_back(route_tuple(  0.0f, 3.0f,  0.0f,   3.0f));
      point.push_back(route_tuple(  0.0f, 4.0f,  0.0f,   6.0f));
      point.push_back(route_tuple(  0.0f, 5.0f,  0.0f,   9.0f));
      point.push_back(route_tuple(  0.0f, 6.0f,  0.0f,  12.0f));
      point.push_back(route_tuple(  0.0f, 7.0f,  0.0f,   6.0f));
      point.push_back(route_tuple(  0.0f, 8.0f,  0.0f,   3.0f));
      point.push_back(route_tuple(  0.0f, 9.0f,  0.0f,   0.0f));
      break;
    case CoursePt::qualifying:
      break;
    case CoursePt::final:
      break;
  }
  return point;
}

bool update(double ref_time, double timer){
    //elapsed_time_prevは初期化しないといけない(微小の誤差が出る)
    static double time_prev{};
    double elapsed_time = timer - time_prev;
    //ROS_INFO("ref_time = %lf, elapsed_time = %lf, time = %lf, time_prev = %lf ",ref_time, elapsed_time, timer, time_prev);
    if(ref_time < elapsed_time || fabs(ref_time-elapsed_time)<1e-5){
        //ROS_INFO("true\n");
        time_prev = timer;
        return true;
    }else{
        return false;
    }
}
int main(int argc, char **argv){
    ros::init(argc, argv, "main");
    ros::NodeHandle n;

    ros::Publisher vel_x_pub = n.advertise<std_msgs::Float32>("/cmd_vel_x", 10);
    ros::Publisher vel_y_pub = n.advertise<std_msgs::Float32>("/cmd_vel_y", 10);
    ros::Publisher rad_pub = n.advertise<std_msgs::Float32>("/cmd_vel_rad", 10);  
    std::string source_frame = "base_link";
    std::string target_frame = "unit0_link";
       
    CatmullRomSpline splineObject(routeInit<CoursePt::acrobatic>(), 100);
    //経路生成終了
    std::vector<Vector4f> route_info = splineObject();

    //0.01秒ごとに更新される
    constexpr int FREQ = 200;
    ros::Rate loop_rate(200);
    int route_counter{};
    unsigned long int loop_counter{};
    double ref_x = 0, ref_x_diff = 0, ref_x_prev = 0, ref_y = 0, ref_y_diff = 0, ref_y_prev = 0, ref_rad = 0, ref_speed = 0, ref_time = 0;
    double ref_x_speed = 0, ref_y_speed = 0;
    std_msgs::Float32 send_data_vel_x;
    std_msgs::Float32 send_data_vel_y;
    double len{};
    double loop_time{};
    while(ros::ok()){
        ++loop_counter;
        loop_time = std::floor(loop_counter * 5) / 1000;
        if(update(ref_time, loop_time)){
            ++route_counter;
            if(route_info.size() < route_counter){return 0;}
            Vector4f nowParam = route_info.at(route_counter - 1);
            ref_x = nowParam(0);
            ref_y = nowParam(1);
            ref_x_diff = ref_x - ref_x_prev;
            ref_y_diff = ref_y - ref_y_prev;
            ref_rad = nowParam(2);
            ref_speed = nowParam(3);
            if(ref_speed == 0){
              ref_time = 0;
              ref_x_speed = 0;
              ref_y_speed = 0;
            }else{
              ref_time = std::sqrt(((ref_x_diff * ref_x_diff) + (ref_y_diff * ref_y_diff))) / ref_speed;
              //ROS_INFO("length = %f\n", std::sqrt(((ref_x_diff * ref_x_diff) + (ref_y_diff * ref_y_diff))));
              
              ref_x_speed = ref_x_diff / ref_time;
              ref_y_speed = ref_y_diff / ref_time;
            }
            //ROS_INFO("%lf %lf %lf %lf\n", ref_x, ref_y, ref_speed, ref_time);
            ROS_INFO("speed = %lf, ref_y = %lf, ref_time = %lf, time = %lf", std::sqrt((ref_x_speed * ref_x_speed) + (ref_y_speed * ref_y_speed)), ref_y, ref_time, loop_time);
            ref_x_prev = ref_x;
            ref_y_prev = ref_y;
        }
        geometry_msgs::Pose t_pose;

        t_pose.position.x = ref_x;
        t_pose.position.y = ref_y;
        t_pose.orientation.w = 1.0;
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        poseMsgToTF(t_pose, transform);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), source_frame, target_frame));

        send_data_vel_x.data = ref_x_speed;
        send_data_vel_y.data = ref_y_speed;

        vel_x_pub.publish(send_data_vel_x);
        vel_y_pub.publish(send_data_vel_y);

        ros::spinOnce();
        loop_rate.sleep();
    }
}