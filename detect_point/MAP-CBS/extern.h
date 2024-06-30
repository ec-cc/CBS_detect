#pragma once
#include<vector>
const double PI = 3.14159265358;
const double R_to_GR = (180.0 / PI);
const double GR_to_R = (PI / 180.0);
const double EARTH_RADIUS = 6371000.0;
const double EARTH_G = 9.8;
const float KNOT_ONE = 0.5144f;
const double bestDetectD = 45000;//���̽�����
const double graphL = 5000;


extern std::vector<std::vector<double>>deteMap;   //̽��÷ֵ�ͼ������Ϊ1�������̽�⵽��Ϊ0����̽�⵽����ͼ��СΪ10*10


//����Point2f�ṹ��  
struct Point2f
{
	double x;
	double y;
	double heading;
	Point2f() = default;
	Point2f(double mx, double my) :x(mx), y(my) {};
	Point2f(double mx, double my,double heading) :x(mx), y(my),heading(heading) {};
};

extern std::vector<Point2f>action;		//8����
struct Point {
	//�߸�����ĵ�
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
	float yingfeiA;     //Ӧ�ɽǶ�
	float yingfeiH; 	//Ӧ�ɸ߶�
	float yingfeiV;    //Ӧ���ٶ�

	double changeA;		//ʵ�ʽǶȱ仯��
	double changeH;		//ʵ�ʸ߶ȱ仯��
	double changeV;					//ʵ���ٶȱ仯��
	//double actualPitch;				//�ɻ������Ǳ仯��
	//double actualHeight;
	double dist;				//��������
	double distA;				//�������뷽λ��
	//double yingfeiDist;
	//double yingfeiAngle;
	double time;				//����ʱ��
	maneuverOutput(float yingfeiA, float yingfeiH, float yingfeiV, double changeA, double changeH, double changeV, double dist, double distA, double time) : yingfeiA(yingfeiA), yingfeiH(yingfeiH), yingfeiV(yingfeiV), changeA(changeA), changeH(changeH), changeV(changeV), dist(dist), distA(distA), time(time) {};
};
struct graphRoughAnction	//����߳�Ϊ5000    ����ֹ滮
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
void deteMapSetUp(std::vector<std::vector<double>>&deteMap);   //����̽���ͼ�������ӵ÷�
std::vector<maneuverOutput>&assignmentSpeedSelect(double speed);
