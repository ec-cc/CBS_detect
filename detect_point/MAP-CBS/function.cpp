#pragma once
#include"function.h"
#include"math.h"
using namespace std;

double DisOfTwoPoint(double slog, double slat, double elog, double elat)   //计算大地两点之间的距离
{
	double x1, x2, y_1, y2, z1, z2, alfa1, seit1, alfa2,
		seit2, beit, x, distance = 0.0, temppara;

	if (slog > 179.999 || slog < 0.001 || slat>70.0 || slat < 0.001 || elog>179.999 || elog < 0.001 || elat>70.0 || elat < 0.001)
		return 0.0;
	alfa1 = slog;
	seit1 = 90.0 - slat;
	alfa2 = elog;
	seit2 = 90.0 - elat;

	x1 = EARTH_RADIUS * sin(seit1 * GR_to_R) * cos(alfa1 * GR_to_R);
	y_1 = EARTH_RADIUS * sin(seit1 * GR_to_R) * sin(alfa1 * GR_to_R);
	z1 = EARTH_RADIUS * cos(seit1 * GR_to_R);

	x2 = EARTH_RADIUS * sin(seit2 * GR_to_R) * cos(alfa2 * GR_to_R);
	y2 = EARTH_RADIUS * sin(seit2 * GR_to_R) * sin(alfa2 * GR_to_R);
	z2 = EARTH_RADIUS * cos(seit2 * GR_to_R);

	temppara = pow(x2 - x1, 2.0) + pow(y2 - y_1, 2.0) + pow(z2 - z1, 2.0);
	if (temppara < 0)
		temppara = 0;
	x = sqrt(temppara);

	x = x / 2 / EARTH_RADIUS;
	beit = asin(x);

	beit *= R_to_GR;
	beit *= 2;

	distance = 2 * PI * EARTH_RADIUS * beit / 360.0;

	return distance;
}

//【计算两个经纬度之间的角度】
float Cal_Heading_T(double Long, double Lat, double long_T, double lat_T)  //期望航向计算  坐标系为 正北0度，顺时针为正， 0到360
{
	double  heading_T;
	double	DLong, DLat;
	double  distance_to_target;
	double  sinheading;

	DLong = long_T - Long;
	DLat = lat_T - Lat;
	//计算两点间距离
	distance_to_target = DisOfTwoPoint(Long, Lat, long_T, lat_T);
	//如果距离小于200米
	if (distance_to_target < 200.)
	{
		//如果纬度差小于0.00001
		if (fabs(DLat) < 0.00001)
		{
			if (DLong > 0)
				return 90.;
			else
				return 270.;
		}
		//否则按平面计算方向
		heading_T = atan2(DLong * cos(lat_T * GR_to_R), DLat) * R_to_GR;
		return (float)heading_T;
	}
	//计算航向的正弦值
	sinheading = sin(fabs(DLong) * GR_to_R) * sin((90 - lat_T) * GR_to_R) / sin(distance_to_target / EARTH_RADIUS);
	//限制极值
	if (sinheading > 1)
	{
		sinheading = 1;
	}
	else if (sinheading < -1)
	{
		sinheading = -1;
	}
	//按不同象限计算出目标航向角
	if (DLong >= 0 && DLat >= 0)		//1
	{
		heading_T = asin(sinheading) * R_to_GR;
	}
	else if (DLong < 0 && DLat >= 0)		//2
	{
		heading_T = -asin(sinheading) * R_to_GR;
	}
	else if (DLong < 0 && DLat < 0)		//3
	{
		heading_T = -(90. + acos(sinheading) * R_to_GR);
	}
	else							//4
	{
		heading_T = 90. + acos(sinheading) * R_to_GR;
	}
	if (heading_T < 0.0)
		heading_T += 360.0;
	else if (heading_T > 360.0)
		heading_T -= 360.0;
	return (float)heading_T;
}

pair<double, double> CompPosition(float delta_de_inm, float delta_dn_inm, double longitude, double latitude) //东向位移 北向位移    自身经纬度引用
{
	//********************************************
	//增加变量distin1,distin2,分别表示相应经纬度下，每度代表多少米。
	//增加变量theta,表示对应经纬度的大地纬度。
	//增加变量earthr,表示对应经纬度到地心的距离。
	//********************************************
	double distin1, distin2, theta, earthr, Long0, Lati0, Out_Long, Out_Lati;
	double a, b, temppara;
	Long0 = longitude;
	Lati0 = latitude;
	//if(fabs(delta_de_inm) > 20.0 || fabs(delta_dn_inm) > 20.0)
	//return;
	if (Long0 < 0.001 || Long0 > 179.999 || Lati0 < 0.001 || Lati0 > 70.0)
		return{ Long0, Lati0 };
	a = 6378136.0;
	b = 6356751.0;

	theta = atan(b * b * tan(Lati0 * PI / 180.0) / a / a);
	earthr = 1.0 / sqrt(cos(theta) * cos(theta) / a / a + sin(theta) * sin(theta) / b / b);
	distin1 = PI / 180.0 * earthr * cos(theta); //latitude纬度 一度代表的距离（米）
	temppara = sqrt((earthr / a * earthr / a - earthr / b * earthr / b) * (earthr / a * earthr / a - earthr / b * earthr / b) * (sin(2.0 * theta) / 2.0) * (sin(2.0 * theta) / 2.0) + 1.0);
	distin2 = temppara * earthr * PI / 180.0;
	Out_Long = Long0 + delta_de_inm / distin1;
	Out_Lati = atan(a / b * a / b * tan(theta + delta_dn_inm / distin2 * PI / 180.0)) / PI * 180.0;
	/**longitude = Out_Long;
	*latitude = Out_Lati;*/
	return make_pair(Out_Long, Out_Lati);
}
double angle_standard(double& azimuth)   //将进入角度规范化，转化为[0-360)的角度
{
	while (azimuth < 0 || azimuth>360)
	{
		if (azimuth < 0)
		{
			azimuth += 360;
		}
		else if (azimuth > 360)
		{
			azimuth -= 360;
		}
	}
	return azimuth;
}

pair<double, double> angle_3D(Point mine, Point enemy)		//三维方位角
{ //first:敌方相对于我方，second：我方相对于敌方
	pair<double, double> res;
	double q[3] = { 0 };
	double qTrans[3] = { 0 };
	double my_angle;
	double en_angle;
	double matrix[3][3] = { 0 };
	double ref[3] = { 1,0,0 };
	q[0] = enemy.x - mine.x;
	q[1] = enemy.y - mine.y;
	q[2] = enemy.z - mine.z;

	//先计算我方的夹角
	double p_yaw = mine.azimuth * 0.0174;
	double p_pitch = mine.pitch * 0.0174;
	double p_roll = mine.roll * 0.0174;
	matrix[0][0] = cos(p_yaw)*cos(p_pitch);
	matrix[0][1] = sin(p_yaw)*cos(p_pitch);
	matrix[0][2] = -sin(p_pitch);
	matrix[1][0] = cos(p_yaw)*sin(p_pitch)*sin(p_roll) - sin(p_yaw)*cos(p_roll);
	matrix[1][1] = sin(p_yaw)*sin(p_pitch)*sin(p_roll) + cos(p_yaw)*cos(p_roll);
	matrix[1][2] = cos(p_pitch)*sin(p_roll);
	matrix[2][0] = cos(p_yaw)*cos(p_roll)*sin(p_pitch) + sin(p_yaw)*sin(p_roll);
	matrix[2][1] = sin(p_yaw)*cos(p_roll)*sin(p_pitch) - cos(p_yaw)*sin(p_roll);
	matrix[2][2] = cos(p_pitch)*cos(p_roll);
	qTrans[0] = q[1] * matrix[0][0] + q[0] * matrix[0][1] + q[2] * matrix[0][2];
	qTrans[1] = q[1] * matrix[1][0] + q[0] * matrix[1][1] + q[2] * matrix[1][2];
	qTrans[2] = q[1] * matrix[2][0] + q[0] * matrix[2][1] + q[2] * matrix[2][2];
	my_angle = acos((qTrans[0]) / sqrt(pow(qTrans[0], 2) + pow(qTrans[1], 2) + pow(qTrans[2], 2)));
	my_angle = my_angle * 57.3;

	//计算敌方与方位角的夹角
	p_yaw = enemy.azimuth * 0.0174;
	p_pitch = enemy.pitch * 0.0174;
	p_roll = enemy.roll * 0.0174;
	matrix[0][0] = cos(p_yaw)*cos(p_pitch);
	matrix[0][1] = sin(p_yaw)*cos(p_pitch);
	matrix[0][2] = -sin(p_pitch);
	matrix[1][0] = cos(p_yaw)*sin(p_pitch)*sin(p_roll) - sin(p_yaw)*cos(p_roll);
	matrix[1][1] = sin(p_yaw)*sin(p_pitch)*sin(p_roll) + cos(p_yaw)*cos(p_roll);
	matrix[1][2] = cos(p_pitch)*sin(p_roll);
	matrix[2][0] = cos(p_yaw)*cos(p_roll)*sin(p_pitch) + sin(p_yaw)*sin(p_roll);
	matrix[2][1] = sin(p_yaw)*cos(p_roll)*sin(p_pitch) - cos(p_yaw)*sin(p_roll);
	matrix[2][2] = cos(p_pitch)*cos(p_roll);
	qTrans[0] = q[1] * matrix[0][0] + q[0] * matrix[0][1] + q[2] * matrix[0][2];
	qTrans[1] = q[1] * matrix[1][0] + q[0] * matrix[1][1] + q[2] * matrix[1][2];
	qTrans[2] = q[1] * matrix[2][0] + q[0] * matrix[2][1] + q[2] * matrix[2][2];
	en_angle = acos((qTrans[0]) / sqrt(pow(qTrans[0], 2) + pow(qTrans[1], 2) + pow(qTrans[2], 2)));
	en_angle = en_angle * 57.3;
	res = make_pair(my_angle, en_angle);
	return res;
}

double point_angle(double mine_x, double mine_y, double enemy_x, double enemy_y)   //计算敌方的方位角   //二维
{
	if (mine_x == enemy_x && mine_y == enemy_y)
		return 0;
	if ((mine_x < enemy_x) && (mine_y < enemy_y))
	{
		return atan2(abs(mine_x - enemy_x), abs(enemy_y - mine_y)) * 57.29;
	}
	else if ((mine_x < enemy_x) && (mine_y > enemy_y))
	{
		return atan2(abs(enemy_y - mine_y), abs(mine_x - enemy_x)) * 57.29 + 90;
	}
	else if ((mine_x > enemy_x) && (mine_y > enemy_y))
	{
		return  atan2(abs(mine_x - enemy_x), abs(mine_y - enemy_y)) * 57.29 + 180;
	}
	else if ((mine_x > enemy_x) && (mine_y < enemy_y))
	{
		return 270 + atan2(abs(enemy_y - mine_y), abs(enemy_x - mine_x)) * 57.29;
	}
	else if ((mine_x == enemy_x) && (mine_y < enemy_y))
		return 0;
	else if ((mine_x < enemy_x) && (mine_y == enemy_y))
		return 90;
	else if ((mine_x == enemy_x) && (mine_y > enemy_y))
		return 180;
	else if ((mine_x > enemy_x) && (mine_y == enemy_y))
		return 270;
}


double distance(double mine_x, double mine_y, double mine_z, double enemy_x, double enemy_y, double enemy_z)   //计算敌我距离
{
	double x_diff = abs(mine_x - enemy_x);
	double y_diff = abs(mine_y - enemy_y);
	double z_diff = abs(mine_z - enemy_z);
	return (sqrt(pow(x_diff, 2) + pow(y_diff, 2) + pow(z_diff, 2)));
}
double distance(double mine_x, double mine_y, double enemy_x, double enemy_y)   //计算敌我距离
{
	double x_diff = abs(mine_x - enemy_x);
	double y_diff = abs(mine_y - enemy_y);
	return (sqrt(pow(x_diff, 2) + pow(y_diff, 2)));
}

double perdictFlyTime(Vertex* now, Vertex* future)     //大致预测个飞行时间
{
	if (now->x == future->x &&now->y == future->y && abs(now->heading - future->heading) < 1)
		return 0;
	future->heading = angle_standard(future->heading);
	double angle1 = point_angle(now->x, now->y, future->x, future->y);
	angle1 = angle_standard(angle1);
	auto angle2 = abs(now->heading - angle1) > 180 ? (360 - abs(now->heading - angle1)) : abs(now->heading - angle1);
	//auto angleTime1 = angle2 / 1.;
	double angle3 = abs(future->heading - angle1) > 180 ? (360 - abs(future->heading - angle1)) : abs(future->heading - angle1);
	//auto angleTime2 = angle3 / 1.;
	double dis = distance(now->x, now->y, future->x, future->y);
	auto disTime = dis / 230;
	return angle2 + angle3 + disTime;
}

Point2f predictPlace(const Point &now,const double v[], const double azi[],const double &num,const double &sampleTime)     //以后可能需要加权重
{
	double dist = now.v*sampleTime;
	double predictX = now.x + sin(now.azimuth / 57.3)*dist;
	double predictY = now.y + cos(now.azimuth / 57.3)*dist;
	for (int i = 0; i < num; i++)
	{
		dist = v[i] * sampleTime;
		predictX += sin(azi[i] / 57.3)*dist;
		predictY += cos(azi[i] / 57.3)*dist;
	}
	return { predictX,predictY };
}

int vectorIndex(double &tt, const double &start, double &perLen, double mframe)		//加入原点0，边长5000，[0,4999],[-5000,-1]
{
	if (perLen == 0)
		perLen = 1;
	if (tt >= start)
	{
		return (int)((tt - start) / perLen) + (int)(mframe / 2);
	}
	else
	{
		if ((tt - start) / perLen - (int)((tt - start) / perLen) != 0)
			return (int)((tt - start) / perLen) - 1 + (int)(mframe / 2);
		else
			return(tt - start) / perLen + (int)(mframe / 2);
	}
}

int enterAngleIndex(double angel)
{
	if (angel >= 0 && angel < 10)
		return 0;
	else if (angel >= 10 && angel < 20)
		return 1;
	else if (angel >= 20 && angel < 30)
		return 2;
	else if (angel >= 30 && angel < 40)
		return 3;
	else if (angel >= 40 && angel < 50)
		return 4;
	else if (angel >= 50 && angel < 60)
		return 5;
	else if (angel >= 60 && angel < 70)
		return 6;
	else if (angel >= 70 && angel < 80)
		return 7;
	else if (angel >= 80 && angel < 90)
		return 8;
	else if (angel >= 90 && angel < 100)
		return 9;
	else if (angel >= 100 && angel < 110)
		return 10;
	else if (angel >= 110 && angel < 120)
		return 11;
	else if (angel >= 120 && angel < 130)
		return 12;
	else if (angel >= 130 && angel < 140)
		return 13;
	else if (angel >= 140 && angel < 150)
		return 14;
	else if (angel >= 150 && angel < 160)
		return 15;
	else if (angel >= 160 && angel < 170)
		return 16;
	else if (angel >= 170 && angel < 180)
		return 17;
	else if (angel >= 180 && angel < 190)
		return 18;
	else if (angel >= 190 && angel < 200)
		return 19;
	else if (angel >= 200 && angel < 210)
		return 20;
	else if (angel >= 210 && angel < 220)
		return 21;
	else if (angel >= 220 && angel < 230)
		return 22;
	else if (angel >= 230 && angel < 240)
		return 23;
	else if (angel >= 240 && angel < 250)
		return 24;
	else if (angel >= 250 && angel < 260)
		return 25;
	else if (angel >= 260 && angel < 270)
		return 26;
	else if (angel >= 270 && angel < 280)
		return 27;
	else if (angel >= 280 && angel < 290)
		return 28;
	else if (angel >= 290 && angel < 300)
		return 29;
	else if (angel >= 300 && angel < 310)
		return 30;
	else if (angel >= 310 && angel < 320)
		return 31;
	else if (angel >= 320 && angel < 330)
		return 32;
	else if (angel >= 330 && angel < 340)
		return 33;
	else if (angel >= 340 && angel < 350)
		return 34;
	else if (angel >= 350 && angel < 360)
		return 35;
	else
		return 36;
}
bool isEqualDouble(double a, double b)
{
	return fabs(a-b) < 0.000001;
}

Point exLine(Point &p, double time)                                              //
{
	double x = p.x + p.v * time*sin(p.azimuth / 57.3);
	double y = p.y + p.v * time*cos(p.azimuth / 57.3);
	return { x,y,p.z,p.azimuth,p.v };
}

bool getDTPSolution(vector<detectTimePeriod>& dtp, vector<Path*>solutions) {
	vector<Vertex*> mVertex;
	if (solutions.size() <= 0) return false;
	//  取出探测终点
	for (int i = 0; i < solutions.size(); i++) {
		mVertex.emplace_back(solutions[i]->Nodes[solutions[i]->Nodes.size() - 1]);
		solutions[i]->Nodes.pop_back();
	}
	// 先拉一段预测的直线 
	int maxPreTime = dtp[dtp.size() - 1].tEnd;
	for (int i = 0; i < dtp.size(); i++) {
		int agIndex = dtp[i].agentIndex;
		Vertex* detectVtex = solutions[agIndex]->Nodes[solutions[agIndex]->Nodes.size() - 1];
		Point mP(detectVtex->x, detectVtex->y, detectVtex->z, detectVtex->heading, detectVtex->v);
		//Point extendP = exLine(mP, dtp[i].tEnd - dtp[i].tStart);
		Point extendP = exLine(mP, maxPreTime - dtp[i].tStart);
		Vertex * child = new Vertex(extendP.x, extendP.y, extendP.azimuth, extendP.v);
		child->Parent = detectVtex;
		child->time = maxPreTime - dtp[i].tStart;
		child->totalTime = dtp[i].tEnd;
		solutions[agIndex]->Nodes.push_back(child);
	}

	// 判断  回转轨迹规划


	//  放回探测终点
	for (int i = 0; i < solutions.size(); i++) {
		solutions[i]->Nodes.push_back(mVertex[i]);
	}
	return true;
}
