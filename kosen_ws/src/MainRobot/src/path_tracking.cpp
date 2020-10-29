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
    //point.push_back(route_tuple(  x[m], y[m], angle[deg],speed[m/s]))
      point.push_back(route_tuple(  0.0f, 0.0f,  0.0f,   0.0f));
      point.push_back(route_tuple(  0.0f, 1.0f,  1.0f,   1.0f));
      point.push_back(route_tuple(  0.0f, 2.0f,  2.0f,   2.0f));
      point.push_back(route_tuple(  0.0f, 3.0f,  3.0f,   2.0f));
      point.push_back(route_tuple( 0.15f, 3.5f,  3.0f,   2.0f));
      point.push_back(route_tuple(  0.5f, 3.9f,  3.0f,   2.0f));
      point.push_back(route_tuple(  1.0f, 4.0f,  3.0f,   2.0f));
      point.push_back(route_tuple(  2.0f, 4.0f,  2.0f,   2.0f));
      point.push_back(route_tuple(  3.0f, 4.0f,  1.0f,   1.0f));
      point.push_back(route_tuple(  4.0f, 4.0f,  0.0f,   0.0f));
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
    if(ref_time < elapsed_time){
        time_prev = timer;
        return true;
    }else{
        return false;
    }
}

float pos_x = 0, pos_y = 0;
void posXCallback(const std_msgs::Float32 &msg){
    pos_x = msg.data;
}
void posYCallback(const std_msgs::Float32 &msg){
    pos_y = msg.data;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "main");
    ros::NodeHandle n;

    ros::Publisher vel_x_pub = n.advertise<std_msgs::Float32>("/cmd_vel_x", 10);
    ros::Publisher vel_y_pub = n.advertise<std_msgs::Float32>("/cmd_vel_y", 10);
    ros::Publisher rad_pub = n.advertise<std_msgs::Float32>("/cmd_vel_rad", 10);  
    ros::Subscriber robot_pos_x_sub = n.subscribe("/pos_x", 10, posXCallback);
    ros::Subscriber robot_pos_y_sub = n.subscribe("/pos_y", 10, posXCallback);
  
    std::string source_frame = "base_link";
    std::string target_frame = "unit0_link";
       
    CatmullRomSpline splineObject(routeInit<CoursePt::acrobatic>(), 100);
    //経路生成終了
    std::vector<Vector4f> route_info = splineObject();

    //0.01秒ごとに更新される
    ros::Rate loop_rate(100);
    int route_counter{};
    unsigned long int loop_counter{};
    double ref_x = 0, ref_x_diff = 0, ref_y = 0, ref_y_diff = 0, ref_rad = 0, ref_speed = 0, ref_time = 0;
    double ref_x_speed = 0, ref_y_speed = 0;
    std_msgs::Float32 send_data_vel_x;
    std_msgs::Float32 send_data_vel_y;
    while(ros::ok()){
        ros::spinOnce();
        ++loop_counter;
        if(((ref_x < pos_x + 30) or (ref_x > pos_x - 30)) and ((ref_y < pos_y + 30) or (ref_y > pos_y - 30))){
        }
        if(update(ref_time, loop_counter * 0.01)){
            if(((ref_x < pos_x + 30) or (ref_x > pos_x - 30)) and ((ref_y < pos_y + 30) or (ref_y > pos_y - 30))){
                ++route_counter;
            }
            if(route_info.size() < route_counter){return 0;}
            Vector4f nowParam = route_info.at(route_counter - 1);
            ref_x = nowParam(0);
            ref_y = nowParam(1);
            ref_x_diff = ref_x - pos_x;
            ref_y_diff = ref_y - pos_y;
            ref_rad = nowParam(2);
            ref_speed = nowParam(3);
            if(ref_speed == 0){
              ref_time = 0;
              ref_x_speed = 0;
              ref_y_speed = 0;
            }else{
              ref_time = std::sqrt(((ref_x_diff * ref_x_diff) + (ref_y_diff * ref_y_diff))) / ref_speed;
              ref_x_speed = ref_x_diff / ref_time;
              ref_y_speed = ref_y_diff / ref_time;
            }
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
        loop_rate.sleep();
    }
}