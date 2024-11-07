#ifndef STRUCT_COMMON_H
#define STRUCT_COMMON_H


namespace zc::detect {

struct LinePara {
	double k;
	double b;

};

//����Point2f�ṹ��  
struct Point2f {
	Point2f() = default;
	Point2f(double _x, double _y) :x(_x), y(_y) {};
	Point2f(double _x, double _y,double _heading) :x(_x), y(_y), heading(_heading) {};
    
    double x;
	double y;
	double heading;
};

//extern std::vector<Point2f>action;		//8����

struct Point {
	Point() {}
	Point(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
	Point(double _x, double _y, double _z, double _azimuth, double _v) 
        : x(_x), y(_y), z(_z), azimuth(_azimuth), v(_v) {}
	Point(int _planeID, double _x, double _y, double _z, double _azimuth, double _v) 
        : planeID(_planeID), x(_x), y(_y), z(_z), azimuth(_azimuth), v(_v) {}
	// Point(double _x, double _y, double _z, double _azimuth, double _v, double _pitch)
    //     : x(_x), y(_y), z(_z), azimuth(_azimuth), v(_v), pitch(_pitch) {}
	Point(double _x, double _y, double _z, double _azimuth, double _pitch, double _v)
        : x(_x), y(_y), z(_z), azimuth(_azimuth), v(_v), pitch(_pitch) {}
	Point(double _x, double _y, double _z, double _azimuth, double _v, float _V_cmd, float _H_cmd, float _A_cmd)
        : x(_x), y(_y), z(_z), azimuth(_azimuth), v(_v), V_cmd(_V_cmd), H_cmd(_H_cmd), A_cmd(_A_cmd) {}
	Point(int _planeID, double _x, double _y, double _z, double _azimuth, double _v, double _pitch, double _roll, float _V_cmd, float _H_cmd, float _A_cmd)
        : x(_x), y(_y), z(_z), azimuth(_azimuth), v(_v),  pitch(_pitch), roll(_roll), V_cmd(_V_cmd), H_cmd(_H_cmd), A_cmd(_A_cmd) {}
	bool operator == (const Point& p) {
		return x == p.x && y == p.y;
	}
	bool operator < (const Point& pk) const {
		return  x < pk.x ;
	}

    //�߸�����ĵ�
	int planeID;
	double x;
	double y;
	double z;
	double roll = 0;
	double pitch = 0;
	double azimuth;
	double v;
	double time = 0;

	float V_cmd = 0;
	float H_cmd = 0;
	float A_cmd = 0;

	//Point(int planeID, double x, double y, double z, double azimuth, double v);
};

struct maneuverOutput {
    maneuverOutput(float yingfeiA, float yingfeiH, float yingfeiV, double changeA, double changeH, double changeV, double dist, double distA, double time)
        : yingfeiA(yingfeiA), yingfeiH(yingfeiH), yingfeiV(yingfeiV), changeA(changeA), changeH(changeH), changeV(changeV), dist(dist), distA(distA), time(time) {};

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
};

struct graphRoughAnction {	//����߳�Ϊ5000    ����ֹ滮
	graphRoughAnction(double dist, double v, double angle) : dist(dist), v(v), angle(angle) {};

    double dist;
	double v;
	double angle;
};

struct endPointList {
	endPointList(Point endPoint, double timeCost, double score) : endPoint(endPoint), timeCost(timeCost), score(score) {};

	double timeCost;
	double score;
	Point endPoint;
};

} // namespace zc::cbs {


#endif //STRUCT_COMMON_H