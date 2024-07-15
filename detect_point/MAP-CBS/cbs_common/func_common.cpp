#include "func_common.h"

namespace zc::detect {

double FuncCommon::SelectR1(double v) {
	return (-1128 + 37.43 * v - 0.008036 * v * v);
}

double FuncCommon::DisOfTwoPoint(double slog, double slat, double elog, double elat) {
	double x1, x2, y1, y2, z1, z2, alfa1, seit1, alfa2,
		seit2, beit, x, distance = 0.0, temppara;

	CBS_EXCUTE(slog > 179.999 || slog < 0.001 || slat > 70.0 || slat < 0.001 || 
	elog > 179.999 || elog < 0.001 || elat > 70.0 || elat < 0.001, return 0.0);

	alfa1 = slog;
	seit1 = 90.0 - slat;
	alfa2 = elog;
	seit2 = 90.0 - elat;

	x1 = EARTH_RADIUS * sin(seit1 * GR_to_R) * cos(alfa1 * GR_to_R);
	y1 = EARTH_RADIUS * sin(seit1 * GR_to_R) * sin(alfa1 * GR_to_R);
	z1 = EARTH_RADIUS * cos(seit1 * GR_to_R);

	x2 = EARTH_RADIUS * sin(seit2 * GR_to_R) * cos(alfa2 * GR_to_R);
	y2 = EARTH_RADIUS * sin(seit2 * GR_to_R) * sin(alfa2 * GR_to_R);
	z2 = EARTH_RADIUS * cos(seit2 * GR_to_R);

	temppara = pow(x2 - x1, 2.0) + pow(y2 - y1, 2.0) + pow(z2 - z1, 2.0);
	CBS_EXCUTE(temppara < 0, temppara = 0);

	x = sqrt(temppara);
	x = x / 2 / EARTH_RADIUS;

	beit = asin(x);
	beit *= R_to_GR;
	beit *= 2;

	distance = 2 * PI * EARTH_RADIUS * beit / 360.0;

	return distance;
}

float FuncCommon::Cal_Heading_T(double Long, double Lat, double long_T, double lat_T) {
	double  heading_T;
	double	DLong, DLat;
	double  distance_to_target;
	double  sinheading;

	DLong = long_T - Long;
	DLat = lat_T - Lat;
	
	distance_to_target = DisOfTwoPoint(Long, Lat, long_T, lat_T);
	
	if (distance_to_target < 200.0) {
		
		CBS_EXCUTE(fabs(DLat) < 0.00001, return DLong > 0 ? 90.0 : 270.0);	
		
		heading_T = atan2(DLong * cos(lat_T * GR_to_R), DLat) * R_to_GR;
		return (float)heading_T;
	}
	
	sinheading = sin(fabs(DLong) * GR_to_R) * sin((90 - lat_T) * GR_to_R) / sin(distance_to_target / EARTH_RADIUS);
	
	CBS_EXCUTE(sinheading > 1, sinheading = 1);
	CBS_EXCUTE(sinheading < -1, sinheading = -1);
	
	if (DLong >= 0 && DLat >= 0) {  
		heading_T = asin(sinheading) * R_to_GR;
	} else if (DLong < 0 && DLat >= 0) {
		heading_T = -asin(sinheading) * R_to_GR;
	} else if (DLong < 0 && DLat < 0) {
		heading_T = -(90. + acos(sinheading) * R_to_GR);
	} else {
		heading_T = 90. + acos(sinheading) * R_to_GR;
	}

	CBS_EXCUTE(heading_T < 0.0, heading_T += 360.0);	
	CBS_EXCUTE(heading_T > 360.0, heading_T -= 360.0);

	return (float)heading_T;
}

std::pair<double, double> FuncCommon::CompPosition(float delta_de_inm, float delta_dn_inm, double longitude, double latitude) {
	//********************************************
	//���ӱ���distin1,distin2,�ֱ��ʾ��Ӧ��γ���£�ÿ�ȴ��������ס�
	//���ӱ���theta,��ʾ��Ӧ��γ�ȵĴ��γ�ȡ�
	//���ӱ���earthr,��ʾ��Ӧ��γ�ȵ����ĵľ��롣
	//********************************************
	double distin1, distin2, theta, earthr, Long0, Lati0, Out_Long, Out_Lati;
	double a, b, temppara;
	Long0 = longitude;
	Lati0 = latitude;
	//if(fabs(delta_de_inm) > 20.0 || fabs(delta_dn_inm) > 20.0)
	//return;
	CBS_EXCUTE(Long0 < 0.001 || Long0 > 179.999 || Lati0 < 0.001 || Lati0 > 70.0, 
			return std::make_pair(Long0, Lati0));

	a = 6378136.0;
	b = 6356751.0;

	theta = atan(b * b * tan(Lati0 * PI / 180.0) / a / a);
	earthr = 1.0 / sqrt(cos(theta) * cos(theta) / a / a + sin(theta) * sin(theta) / b / b);
	distin1 = PI / 180.0 * earthr * cos(theta); //latitudeγ�� һ�ȴ����ľ��루�ף�
	temppara = sqrt((earthr / a * earthr / a - earthr / b * earthr / b) * (earthr / a * earthr / a - earthr / b * earthr / b) * (sin(2.0 * theta) / 2.0) * (sin(2.0 * theta) / 2.0) + 1.0);
	distin2 = temppara * earthr * PI / 180.0;
	Out_Long = Long0 + delta_de_inm / distin1;
	Out_Lati = atan(a / b * a / b * tan(theta + delta_dn_inm / distin2 * PI / 180.0)) / PI * 180.0;
	/**longitude = Out_Long;
	*latitude = Out_Lati;*/
	return std::make_pair(Out_Long, Out_Lati);
}

double FuncCommon::angle_standard(double& azimuth) {
	while (azimuth < 0 || azimuth > 360) { 
		CBS_EXCUTE(azimuth < 0, azimuth += 360); 
		CBS_EXCUTE(azimuth > 360, azimuth -= 360); 
	} 
	return azimuth;
}

std::pair<double, double> FuncCommon::angle_3D(Point mine, Point enemy) { 
	std::pair<double, double> res;
	double q[3] = { 0 };
	double qTrans[3] = { 0 };
	double my_angle;
	double en_angle;
	double matrix[3][3] = { 0 };
	double ref[3] = { 1,0,0 };
	q[0] = enemy.x - mine.x;
	q[1] = enemy.y - mine.y;
	q[2] = enemy.z - mine.z;

	//�ȼ����ҷ��ļн�
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

	//����з��뷽λ�ǵļн�
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

	res = std::make_pair(my_angle, en_angle);
	return res;
}

double FuncCommon::point_angle(double mine_x, double mine_y, double enemy_x, double enemy_y){
	if (mine_x == enemy_x && mine_y == enemy_y) {
		return 0;
	} else if ((mine_x < enemy_x) && (mine_y < enemy_y)) {
		return atan2(abs(mine_x - enemy_x), abs(enemy_y - mine_y)) * 57.29;
	} else if ((mine_x < enemy_x) && (mine_y > enemy_y)) {
		return atan2(abs(enemy_y - mine_y), abs(mine_x - enemy_x)) * 57.29 + 90;
	} else if ((mine_x > enemy_x) && (mine_y > enemy_y)) {
		return  atan2(abs(mine_x - enemy_x), abs(mine_y - enemy_y)) * 57.29 + 180;
	} else if ((mine_x > enemy_x) && (mine_y < enemy_y)) {
		return 270 + atan2(abs(enemy_y - mine_y), abs(enemy_x - mine_x)) * 57.29;
	} else if ((mine_x == enemy_x) && (mine_y < enemy_y)) {
		return 0;
	} else if ((mine_x < enemy_x) && (mine_y == enemy_y)) {
		return 90;
	} else if ((mine_x == enemy_x) && (mine_y > enemy_y)) {
		return 180;
	} else if ((mine_x > enemy_x) && (mine_y == enemy_y)) {
		return 270;
	}

	return 0;
}

double FuncCommon::distance(double mine_x, double mine_y, double mine_z, double enemy_x, double enemy_y, double enemy_z) {
	double x_diff = abs(mine_x - enemy_x);
	double y_diff = abs(mine_y - enemy_y);
	double z_diff = abs(mine_z - enemy_z);
	return (sqrt(pow(x_diff, 2) + pow(y_diff, 2) + pow(z_diff, 2)));
}

double FuncCommon::distance(double mine_x, double mine_y, double enemy_x, double enemy_y) {
	double x_diff = abs(mine_x - enemy_x);
	double y_diff = abs(mine_y - enemy_y);
	return (sqrt(pow(x_diff, 2) + pow(y_diff, 2)));
}

Point2f FuncCommon::predictPlace(const Point& now, const double v[], const double azi[], const double &num, const double &sampleTime) {
	double dist = now.v*sampleTime;
	double predictX = now.x + sin(now.azimuth / 57.3)*dist;
	double predictY = now.y + cos(now.azimuth / 57.3)*dist;
	for (int i = 0; i < num; i++) {
		dist = v[i] * sampleTime;
		predictX += sin(azi[i] / 57.3)*dist;
		predictY += cos(azi[i] / 57.3)*dist;
	}
	return { predictX,predictY };
}

int FuncCommon::vectorIndex(double &tt, const double &start, double &perLen, double mframe) {
	CBS_EXCUTE(perLen == 0, perLen = 1);

	if (tt >= start) {
		return (int)((tt - start) / perLen) + (int)(mframe / 2);
	} else {
		if ((tt - start) / perLen - (int)((tt - start) / perLen) != 0) {
			return (int)((tt - start) / perLen) - 1 + (int)(mframe / 2);
		} else {
			return(tt - start) / perLen + (int)(mframe / 2);
		}
	}
}

int FuncCommon::enterAngleIndex(double angel) {
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

bool FuncCommon::isEqualDouble(double a, double b) {
	return fabs(a-b) < 0.000001;
}

Point FuncCommon::exLine(Point &p, double time) {
	double x = p.x + p.v * time*sin(p.azimuth / 57.3);
	double y = p.y + p.v * time*cos(p.azimuth / 57.3);
	return { x,y,p.z,p.azimuth,p.v };
}

const std::vector<maneuverOutput>& FuncCommon::speedSelect(double speed) {
	if (speed >= 260)
		return actionSample270;
	else if (speed >= 240)
		return actionSample250;
	else if (speed >= 220)
		return actionSample230;
	else if (speed >= 200)
		return actionSample210;
	else if (speed >= 180)
		return actionSample190;
	else
		return actionSample170;
}

const std::vector<maneuverOutput>& FuncCommon::assignmentSpeedSelect(double speed) {
	if (speed >= 260)
		return assignmentActionSample270;
	else if (speed >= 240)
		return assignmentActionSample250;
	else if (speed >= 220)
		return assignmentActionSample230;
	else if (speed >= 200)
		return assignmentActionSample210;
	else if (speed >= 180)
		return assignmentActionSample190;
	else
		return assignmentActionSample170;
}


void FuncCommon::getLinePara(double& x1, double& y1, double& x2, double& y2, LinePara & LP) {
	double m = 0;

	// �������  
	m = x2 - x1;

	if (m == 0){
		LP.k = 10000.0;
		LP.b = y1 - LP.k * x1;
	} else {
		LP.k = (y2 - y1) / (x2 - x1);
		LP.b = y1 - LP.k * x1;
	}
}

Point2f FuncCommon::getPoint(Point2f mine, double mineAzi, double enemyPosAng) {
	double GPSEnemyAng = enemyPosAng;
	GPSEnemyAng = angle_standard(GPSEnemyAng);
	auto bb = cos(GPSEnemyAng / 57.3);
	auto bbc = sin(GPSEnemyAng / 57.3);
	double pointX = mine.x + 60000 * sin(GPSEnemyAng / 57.3);
	double pointY = mine.y + 60000 * cos(GPSEnemyAng / 57.3);
	return Point2f{ pointX,pointY };
}

bool FuncCommon::getCross(LinePara para1, LinePara para2, Point2f & pt) {
	// �ж��Ƿ�ƽ��  
	if (abs(para1.k - para2.k) > 0.0001) {
		pt.x = (para2.b - para1.b) / (para1.k - para2.k);
		pt.y = para1.k * pt.x + para1.b;

		return true;
	} else {
		return false;
	}
}

void FuncCommon::crossP(Point mine, Point2f B, Point2f C, double enemyToA, double enemyToB, double enemyToC, double GPSEnemyAngL,
	LinePara enemyLinePara, Point2f&crossPoint) {
	//ǰ�᣺AΪ�ɻ���ǰ��ͷ������λ�ڵ�һ���ޣ�B��C�Ϸ�
	//���BС��180�������ֱ�Ӽ���
	if (enemyToB < 180) {
		auto nextB = enemyToB + 360;
		if (GPSEnemyAngL > enemyToB && GPSEnemyAngL <= enemyToC) {
			LinePara BToC;
			getLinePara(B.x, B.y, C.x, C.y, BToC);
			getCross(enemyLinePara, BToC, crossPoint);
		} else if (GPSEnemyAngL > enemyToC && GPSEnemyAngL <= enemyToA) {
			LinePara AToC;
			getLinePara(mine.x, mine.y, C.x, C.y, AToC);
			getCross(enemyLinePara, AToC, crossPoint);
		} else {
			LinePara AToB;
			getLinePara(B.x, B.y, mine.x, mine.y, AToB);
			getCross(enemyLinePara, AToB, crossPoint);
		}
	} else {
		auto nextB = enemyToB - 360;
		if (GPSEnemyAngL > nextB && GPSEnemyAngL <= enemyToC) {
			LinePara BToC;
			getLinePara(B.x, B.y, C.x, C.y, BToC);
			getCross(enemyLinePara, BToC, crossPoint);
		} else if (GPSEnemyAngL > enemyToC && GPSEnemyAngL <= enemyToA) {
			LinePara AToC;
			getLinePara(mine.x, mine.y, C.x, C.y, AToC);    //��   6.14
			getCross(enemyLinePara, AToC, crossPoint);
		} else {
			LinePara AToB;
			getLinePara(B.x, B.y, mine.x, mine.y, AToB);    //��   6.14
			getCross(enemyLinePara, AToB, crossPoint);
		}
	}
}

std::vector<Point2f> FuncCommon::positionTrianglePoint(Point mine, double L) {
	std::vector<Point2f> result;
	Point2f A, B, C;
	A.x = mine.x + L * sin((mine.azimuth + 30) / 57.3);
	A.y = mine.y + L * cos((mine.azimuth + 30) / 57.3);
	C.x = mine.x + 75000 * sin((mine.azimuth + 30) / 57.3);
	C.y = mine.y + 75000 * cos((mine.azimuth + 30) / 57.3);
	B.x = mine.x + L * sin((mine.azimuth + 30) / 57.3) + (75000 - L)*sin((mine.azimuth - 30) / 57.3);
	B.y = mine.y + L * cos((mine.azimuth + 30) / 57.3) + (75000 - L)*cos((mine.azimuth - 30) / 57.3);
	auto aaa = distance(A.x, A.y, B.x, B.y);
	auto aaaa = distance(A.x, A.y, C.x, C.y);
	result.push_back(A);
	result.push_back(B);
	result.push_back(C);
	return result;
}

double FuncCommon::product(Point2f p1, Point2f p2, Point2f p3) {
	//���ȸ����������p1p2��p1p3��������Ȼ���ټ�����
	return (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
}

bool FuncCommon::isInTriangle(Point2f p1, Point2f p2, Point2f p3, Point2f o) {
	//��֤p1��p2��p3����ʱ��˳��,���˸��ݹ�
	if (product(p1, p2, p3) < 0)
		return isInTriangle(p1, p3, p2, o);
	if (product(p1, p2, o) > 0 && product(p2, p3, o) > 0 && product(p3, p1, o) > 0)
		return true;
	return false;
}

std::vector<Point2f> FuncCommon::trianglePoint(Point mine,double sideLength) {
	std::vector<Point2f> result;
	Point2f A, B;
	A.x = mine.x + sideLength * cos((mine.azimuth + 30) / 57.296);
	A.y = mine.y + sideLength * sin((mine.azimuth + 30) / 57.296);
	B.x = mine.x + sideLength * cos((mine.azimuth - 30) / 57.296);
	B.y = mine.y + sideLength * sin((mine.azimuth - 30) / 57.296);
	result.push_back(A);
	result.push_back(B);
	return result;
}

double FuncCommon::detectTime(Point mine, Point enemy, Point2f B, Point2f C) {
	//B�ϣ�C��
	mine.azimuth = angle_standard(mine.azimuth);
	//double temp = mine.azimuth - 180;
	//double mine180 = angle_standard(temp);
	//pair<double, double>mineLineAngle = mine180 < mine.azimuth ? make_pair(mine180, mine.azimuth) : make_pair(mine.azimuth, mine180);
	enemy.azimuth = angle_standard(enemy.azimuth);
	//�����з�ԭʼ����
	double originalX = enemy.x;
	double originalY = enemy.y;
	double originalBX = B.x;
	double originalBY = B.y;
	double originalCX = C.x;
	double originalCY = C.y;
	//auto vvdsv = std::atan2(originalY, originalX);
	double enemyPlaceAngle = atan2(originalY - mine.y, originalX - mine.x) > 0 ? atan2(originalY - mine.y, originalX - mine.x)*57.296 : (atan2(originalY - mine.y, originalX - mine.x)*57.296 + 360);
	double diffAngle = enemyPlaceAngle - mine.azimuth;   //�õ��з����Һ����ļн�
	double disEnemy = distance(originalX, originalY, mine.x, mine.y);
	enemy.x = disEnemy * cos(diffAngle / 57.296);       //��̽��������ת�Ƶ�������x���غϵ������Σ���˵з������궼��䶯
	enemy.y = disEnemy * sin(diffAngle / 57.296);
	Point2f enemyOtherPoint = getPoint({ originalX,originalY }, mine.azimuth, enemy.azimuth);
	LinePara enemyLinePara;
	//����з�ֱ�߷���
	getLinePara(originalX, originalY, enemyOtherPoint.x, enemyOtherPoint.y, enemyLinePara);
	Point2f mineOtherPoint = getPoint({ mine.x,mine.y }, mine.azimuth, mine.azimuth);
	LinePara mineLinePara;
	//�����ҷ�ֱ�߷���
	getLinePara(mine.x, mine.y, mineOtherPoint.x, mineOtherPoint.y, mineLinePara);
	bool isIntersect;      //�жϵз�ֱ��ֱ�����Һ����ཻ�����ӳ����ཻ

	Point2f nowIntersectP;
	if (getCross(mineLinePara, enemyLinePara, nowIntersectP)) {   //�õ���ʱ��ֱ�߽���
		double disIntersectP = distance(nowIntersectP.x, nowIntersectP.y, mine.x, mine.y);
		Point2f afterIntersectP;
		afterIntersectP.x = disIntersectP * cos(0);     //����ת�Ƶ�x����
		afterIntersectP.y = 0;
		if (atan2((nowIntersectP.y - originalY), (nowIntersectP.x - originalX)) == enemy.azimuth)
			isIntersect = true;
		else
			isIntersect = false;
		if (nowIntersectP.x == originalX && nowIntersectP.y == originalY)     //����з��㱾�������ҷ������ϣ���Ҫͼ�鿼��
		{
			enemy.azimuth -= mine.azimuth;
		}
		else
		{
			if (isIntersect)
			{
				enemy.azimuth = atan2((afterIntersectP.y - enemy.y), (afterIntersectP.x - enemy.x))*57.29;
			}
			else
			{
				enemy.azimuth = atan2((afterIntersectP.y - enemy.y), (afterIntersectP.x - enemy.x))*57.296 + 180;
			}
			enemy.azimuth = angle_standard(enemy.azimuth);
		}
		//��ת���ĵз�����Ƿ����ж�   �жϴ��ĸ��߳���
		if (enemy.azimuth >= 0 && enemy.azimuth < 90)
		{
			double enemySpeedX = enemy.v*cos(enemy.azimuth / 57.296);
			double enemySpeedY = enemy.v*sin(enemy.azimuth / 57.296);
			double lengthX = 70000 * cos(30 / 57.296) - enemy.x;
			double lengthY = 70000 * sin(30 / 57.296) / (70000 * cos(30 / 57.296) / enemy.x) - enemy.y;
			double timeY = abs(lengthY / enemySpeedY);
			double timeX;
			if (enemySpeedX < mine.v)
			{
				lengthX = (70000 * sin(30 / 57.296) - abs(enemy.y)) / tan(30 / 57.296) - (70000 * cos(30 / 57.296) - enemy.x);
				timeX = lengthX / (mine.v - enemySpeedX);
			}
			else if (enemySpeedX == mine.v)
				return timeY;
			else
				timeX = lengthX / (enemySpeedX - mine.v);
			return timeX < timeY ? timeX : timeY;
		}
		else if (enemy.azimuth >= 90 && enemy.azimuth < 180)
		{
			double enemySpeedX = enemy.v*cos(enemy.azimuth / 57.296);
			double enemySpeedY = enemy.v*sin(enemy.azimuth / 57.296);
			double lengthX = (70000 * sin(30 / 57.296) - abs(enemy.y)) / tan(30 / 57.296) - (70000 * cos(30 / 57.296) - enemy.x);
			double lengthY = 70000 * sin(30 / 57.296) / (70000 * cos(30 / 57.296) / enemy.x) - enemy.y;
			double timeY = abs(lengthY / enemySpeedY);
			double timeX = lengthX / (mine.v - enemySpeedX);
			return timeX < timeY ? timeX : timeY;
		}
		else if (enemy.azimuth >= 180 && enemy.azimuth < 270)
		{
			double enemySpeedX = enemy.v*cos(enemy.azimuth / 57.296);
			double enemySpeedY = enemy.v*sin(enemy.azimuth / 57.296);
			double lengthX = (70000 * sin(30 / 57.296) - abs(enemy.y)) / tan(30 / 57.296) - (70000 * cos(30 / 57.296) - enemy.x);
			double lengthY = 70000 * sin(30 / 57.296) / (70000 * cos(30 / 57.296) / enemy.x) + enemy.y;
			double timeY = abs(lengthY / enemySpeedY);
			double timeX = lengthX / (mine.v - enemySpeedX);
			return timeX < timeY ? timeX : timeY;
		}
		else
		{
			double enemySpeedX = enemy.v*cos(enemy.azimuth / 57.296);
			double enemySpeedY = enemy.v*sin(enemy.azimuth / 57.296);
			double lengthX = 70000 * cos(30 / 57.296) - enemy.x;
			double lengthY = 70000 * sin(30 / 57.296) / (70000 * cos(30 / 57.296) / enemy.x) + enemy.y;
			double timeY = abs(lengthY / enemySpeedY);
			double timeX;
			if (enemySpeedX < mine.v)
			{
				lengthX = (70000 * sin(30 / 57.296) - abs(enemy.y)) / tan(30 / 57.296) - (70000 * cos(30 / 57.296) - enemy.x);
				timeX = lengthX / (mine.v - enemySpeedX);
			}
			else
				timeX = lengthX / (enemySpeedX - mine.v);
			return timeX < timeY ? timeX : timeY;
		}
	}
	else
	{
		if (enemy.azimuth == mine.azimuth)
		{
			if (enemy.v > mine.v)
				return (70000 * cos(30 / 57.296) - enemy.x) / (enemy.v - mine.v);
			else if (enemy.v == mine.v)
			{
				std::cout << "ʱ�����޴󣡣�" << std::endl;
			}
			else
				return enemy.x / (mine.v - enemy.v);
		}
		else
		{
			return enemy.x / (mine.v + enemy.v);
		}
	}

}

double FuncCommon::detectScore(const Point &mine,const Point &enemy) {
	auto dist = distance(mine.x, mine.y, enemy.x, enemy.y);
	double enemyAzi = point_angle(mine.x, mine.y, enemy.x, enemy.y);
	double angleDiff = abs(mine.azimuth - enemyAzi) > 180 ? 360 - abs(mine.azimuth - enemyAzi) : abs(mine.azimuth - enemyAzi);
	double angleS, distanceS;
	angleS = exp(-PI * pow((angleDiff / 30.0),2));
	distanceS = exp(-PI * pow(((dist- bestDetectD) / 30000.0),2));
	return 0.5*angleS + 0.5*distanceS;
}

double FuncCommon::encapsulationTime(Point &mine, Point &enemy) {
	mine.azimuth = -mine.azimuth + 90;   //ת��Ϊ��ѧ����ϵ
	enemy.azimuth = -enemy.azimuth + 90; //ת��Ϊ��ѧ����ϵ
	auto otherPoint = trianglePoint(mine, 70000);
	//cout << "�ж��Ƿ����������ڲ�  " << isInTriangle({ mine.x,mine.y }, otherPoint[0], otherPoint[1], { enemy.x,enemy.y }) << endl;
	if (isInTriangle({ mine.x,mine.y }, otherPoint[0], otherPoint[1], { enemy.x,enemy.y }))
		return detectTime(mine, enemy, otherPoint[0], otherPoint[1]);
	return 0;
}

    
} // namespace zc::detect
