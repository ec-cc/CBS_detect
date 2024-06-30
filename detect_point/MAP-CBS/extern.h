#pragma once
#include<vector>
const double PI = 3.14159265358;
const double R_to_GR = (180.0 / PI);
const double GR_to_R = (PI / 180.0);
const double EARTH_RADIUS = 6371000.0;
const double EARTH_G = 9.8;
const float KNOT_ONE = 0.5144f;
const double bestDetectD = 45000;//最佳探测距离
const double graphL = 5000;


extern std::vector<std::vector<double>>deteMap;   //探测得分地图，格内为1代表可以探测到，为0不可探测到，地图大小为10*10


//定义Point2f结构体  
struct Point2f
{
	double x;
	double y;
	double heading;
	Point2f() = default;
	Point2f(double mx, double my) :x(mx), y(my) {};
	Point2f(double mx, double my,double heading) :x(mx), y(my),heading(heading) {};
};

extern std::vector<Point2f>action;		//8格方向
struct Point {
	//七个方向的点
	int planeID;
	double x;
	double y;
	double z;
	double roll;
	double pitch;
	double azimuth;
	double v;
	double time;

	float V_cmd;
	float H_cmd;
	float A_cmd;

	Point();
	Point(double x, double y, double z, double azimuth, double v, float V_cmd, float H_cmd, float A_cmd);
	Point(double x, double y, double z);
	Point(double x, double y, double z, double azimuth, double pitch, double v);
	Point(double x, double y, double z, double azimuth, double v);
	Point(int planeID, double x, double y, double z, double azimuth, double pitch, double roll, double v, float V_cmd, float H_cmd, float A_cmd);
	bool operator == (const Point& p)
	{
		return x == p.x && y == p.y;
	}
	bool operator < (const Point& pk) const
	{
		return  x < pk.x ;
	}
	//Point(int planeID, double x, double y, double z, double azimuth, double v);
};
struct maneuverOutput
{
	float yingfeiA;     //应飞角度
	float yingfeiH; 	//应飞高度
	float yingfeiV;    //应飞速度

	double changeA;		//实际角度变化量
	double changeH;		//实际高度变化量
	double changeV;					//实际速度变化量
	//double actualPitch;				//飞机俯仰角变化量
	//double actualHeight;
	double dist;				//采样距离
	double distA;				//采样距离方位角
	//double yingfeiDist;
	//double yingfeiAngle;
	double time;				//采样时间
	maneuverOutput(float yingfeiA, float yingfeiH, float yingfeiV, double changeA, double changeH, double changeV, double dist, double distA, double time) : yingfeiA(yingfeiA), yingfeiH(yingfeiH), yingfeiV(yingfeiV), changeA(changeA), changeH(changeH), changeV(changeV), dist(dist), distA(distA), time(time) {};
};
struct graphRoughAnction	//假设边长为5000    方格粗规划
{
	double dist;
	double v;
	double angle;
	graphRoughAnction(double dist, double v, double angle) :dist(dist), v(v), angle(angle) {};
};
struct endPointList
{
	Point endPoint;
	double timeCost;
	double score;
	endPointList(Point endPoint, double timeCost, double score) :endPoint(endPoint), timeCost(timeCost), score(score) {};
};



extern std::vector< graphRoughAnction>actionSample;
std::vector<maneuverOutput>&speedSelect(double speed);
void deteMapSetUp(std::vector<std::vector<double>>&deteMap);   //设置探测地图各个格子得分
std::vector<maneuverOutput>&assignmentSpeedSelect(double speed);
