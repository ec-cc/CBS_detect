#pragma once
#include<iostream>
#include<vector>
#include"extern.h"
#include"CBSDataStructures.h"
extern const double collideDis ;


double DisOfTwoPoint(double slog, double slat, double elog, double elat);
float Cal_Heading_T(double Long, double Lat, double long_T, double lat_T);
std::pair<double, double> CompPosition(float delta_de_inm, float delta_dn_inm, double longitude, double latitude);
double angle_standard(double& azimuth);   //������Ƕȹ淶����ת��Ϊ[0-360)�ĽǶ�
std::pair<double, double> angle_3D(Point mine, Point enemy);		//��ά��λ��
double point_angle(double mine_x, double mine_y, double enemy_x, double enemy_y);   //�����ά��λ��   
double distance(double mine_x, double mine_y, double mine_z, double enemy_x, double enemy_y, double enemy_z);   //������ά����
double distance(double mine_x, double mine_y, double enemy_x, double enemy_y);   //�����ά����
double perdictFlyTime(Vertex *now, Vertex *future);     //����Ԥ�����ʱ��
Point2f predictPlace(const Point &now, const double v[], const double azi[], const double &num, const double &sampleTime);     //�Ժ������Ҫ��Ȩ��
int enterAngleIndex(double angel);
int vectorIndex(double &tt, const double &start, double &perLen, double mframe);		//����ԭ��0���߳�5000��[0,4999],[-5000,-1]
bool isEqualDouble(double a , double b);
bool getDTPSolution(std::vector<detectTimePeriod>& dtp, std::vector<Path*>solutions);             // �õ�ʱ�����ⷽ��
Point exLine(Point &p, double time);                                                    // Ԥ��ֱ��λ��   



