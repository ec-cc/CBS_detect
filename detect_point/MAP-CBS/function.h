#pragma once
#include<iostream>
#include<vector>
#include"extern.h"
#include"CBSDataStructures.h"
extern const double collideDis ;


double DisOfTwoPoint(double slog, double slat, double elog, double elat);
float Cal_Heading_T(double Long, double Lat, double long_T, double lat_T);
std::pair<double, double> CompPosition(float delta_de_inm, float delta_dn_inm, double longitude, double latitude);
double angle_standard(double& azimuth);   //将进入角度规范化，转化为[0-360)的角度
std::pair<double, double> angle_3D(Point mine, Point enemy);		//三维方位角
double point_angle(double mine_x, double mine_y, double enemy_x, double enemy_y);   //计算二维方位角   
double distance(double mine_x, double mine_y, double mine_z, double enemy_x, double enemy_y, double enemy_z);   //计算三维距离
double distance(double mine_x, double mine_y, double enemy_x, double enemy_y);   //计算二维距离
double perdictFlyTime(Vertex *now, Vertex *future);     //大致预测飞行时间
Point2f predictPlace(const Point &now, const double v[], const double azi[], const double &num, const double &sampleTime);     //以后可能需要加权重
int enterAngleIndex(double angel);
int vectorIndex(double &tt, const double &start, double &perLen, double mframe);		//加入原点0，边长5000，[0,4999],[-5000,-1]
bool isEqualDouble(double a , double b);
bool getDTPSolution(std::vector<detectTimePeriod>& dtp, std::vector<Path*>solutions);             // 得到时间段求解方法
Point exLine(Point &p, double time);                                                    // 预测直线位置   



