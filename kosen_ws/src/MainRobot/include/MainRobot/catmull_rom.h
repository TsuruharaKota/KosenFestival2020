#pragma once

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
#include<ros/ros.h>

using namespace Eigen;
using catmull_tuple = std::tuple<float, float, float>;
using route_tuple = std::tuple<float, float, float, float>;
using std::cout;
using std::endl;
template<typename T>
class PosData4{
    public:
        T x;
        T y;
        T theta;
        T speed;
        explicit PosData4(T _x, T _y, T _theta, T _speed):x(_x), y(_y), theta(_theta), speed(_speed){}
        PosData4 operator+(PosData4 object){
            x += object.x;
            y += object.y;
            return *this;
        }
        PosData4 operator-(PosData4 object){
            x -= object.x;
            y -= object.y;
            return *this;
        }
        PosData4 operator*(PosData4 object){
            x *= object.x;
            y *= object.y;
            return *this;
        }
        void pow(){
            x *= x;
            y *= y;
        }
};
class CatmullRomSpline{
    public:
        explicit CatmullRomSpline(std::vector<route_tuple> &&input_param, int _frequency):frequency(_frequency){
            //全ての通過点を取得
            for(auto &point : input_param){
                PosData4<float> point4d(std::get<0>(point), std::get<1>(point), std::get<2>(point), std::get<3>(point));
                transit_point.push_back(point4d);
            }
        }
        std::vector<Vector4f> operator()(){
            //始点通過、終点通過、一般経路を分類
            int angle_count = 0;
            float x_prev = 0;
            float y_prev = 0;
            float angle_prev = 0;
            float angle_sum = 0;
            for(int counter = 0; counter < transit_point.size() - 1; ++counter){
                auto NomalCal = [&](int n, float p0, float p1, float p2, float p3){
                    //printf("NomalCal n = %d, p0 = %f, p1 = %f, p2 = %f, p3 = %f\n", n, p0, p1, p2, p3);
                    a[n] = -p0 + 3.0f * p1 - 3.0f * p2 + p3;
                    b[n] = 2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3;
                    c[n] = -p0 + p2;
                    d[n] = 2.0f * p1;
                    //printf("NomalCal n = %d, b = %f, c = %f, d = %f\n", n, b[n], c[n], d[n]);
                };
                auto ExceptionCal = [&](int n, float p0, float p1, float p2){
                    //printf("ExceptionCal n = %d, p0 = %f, p1 = %f, p2 = %f\n", n, p0, p1, p2);
                    if(counter == 0){
                        //First Curve
                        b[n] = p0 - 2.0f * p1 + p2;
                        c[n] = -3.0f * p0 + 4.0f * p1 - p2;
                        d[n] = 2.0f * p0;
                    }else{
                        //Last Curve
                        b[n] = p0 - 2.0f * p1 + p2;
                        c[n] = -p0 + p2;
                        d[n] = 2.0f * p1;
                    }
                    //printf("ExceptionCal n = %d, b = %f, c = %f, d = %f\n", n, b[n], c[n], d[n]);
                };
                if(counter == 0){
                    ExceptionCal(0, transit_point.at(counter).x,     transit_point.at(counter + 1).x,     transit_point.at(counter + 2).x);
                    ExceptionCal(1, transit_point.at(counter).y,     transit_point.at(counter + 1).y,     transit_point.at(counter + 2).y);
                    ExceptionCal(2, transit_point.at(counter).theta, transit_point.at(counter + 1).theta, transit_point.at(counter + 2).theta);
                    ExceptionCal(3, transit_point.at(counter).speed, transit_point.at(counter + 1).speed, transit_point.at(counter + 2).speed);
                }else if(counter == transit_point.size() - 2){
                    ExceptionCal(0, transit_point.at(counter - 1).x,     transit_point.at(counter).x,     transit_point.at(counter + 1).x);
                    ExceptionCal(1, transit_point.at(counter - 1).y,     transit_point.at(counter).y,     transit_point.at(counter + 1).y);
                    ExceptionCal(2, transit_point.at(counter - 1).theta, transit_point.at(counter).theta, transit_point.at(counter + 1).theta);
                    ExceptionCal(3, transit_point.at(counter - 1).speed, transit_point.at(counter).speed, transit_point.at(counter + 1).speed);
                }else{
                    NomalCal(0, transit_point.at(counter - 1).x,     transit_point.at(counter).x,     transit_point.at(counter + 1).x,     transit_point.at(counter + 2).x);
                    NomalCal(1, transit_point.at(counter - 1).y,     transit_point.at(counter).y,     transit_point.at(counter + 1).y,     transit_point.at(counter + 2).y);
                    NomalCal(2, transit_point.at(counter - 1).theta, transit_point.at(counter).theta, transit_point.at(counter + 1).theta, transit_point.at(counter + 2).theta);
                    NomalCal(3, transit_point.at(counter - 1).speed, transit_point.at(counter).speed, transit_point.at(counter + 1).speed, transit_point.at(counter + 2).speed);
                }
                for(int i = 0; i < frequency; ++i){
                    float t = static_cast<float>(i) / frequency;
                    float temp[4];
                    for(int k = 0; k < 4; ++k){
                        if(counter == 0){
                            temp[k] = 0.5f * ((b[k] * t * t) + (c[k] * t) + d[k]);
                        }else if(counter == transit_point.size() - 2){
                            temp[k] = 0.5f * ((b[k] * t * t) + (c[k] * t) + d[k]);
                        }else{
                            temp[k] = 0.5f * ((a[k] * t * t * t) + (b[k] * t * t) + (c[k] * t) + d[k]);
                        }
                    }
                    Vector4f Pos(temp[0], temp[1], temp[2], temp[3]);
                    goal_point.push_back(Pos);
                    x_prev = temp[0];
                    y_prev = temp[1];
                }
            }
            //この時点で座標は生成できている
            std::ofstream outputFile("Route.txt");
            for(auto &point : goal_point){outputFile << std::fixed << std::setprecision(5) << point(0) << " " << point(1) << " " << point(2) << " " << point(3) << "\n";}
            outputFile.close();
            return goal_point;
        }
    private:
        std::vector<float> transit_theta;
        std::vector<Vector4f> goal_point;
        std::vector<PosData4<float>> transit_point;
        int frequency;
        float a[4], b[4], c[4], d[4];
};