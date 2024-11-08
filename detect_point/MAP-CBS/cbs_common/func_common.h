#ifndef FUNC_COMMON_H
#define FUNC_COMMON_H

#include <iostream>
#include <math.h>
#include <memory>
#include <algorithm>
#include "data_common.h"

const double PI = 3.14159265358;
const double R_to_GR = (180.0 / PI);
const double GR_to_R = (PI / 180.0);
const double EARTH_RADIUS = 6371000.0;
const double EARTH_G = 9.8;
const float KNOT_ONE = 0.5144f;
const double bestDetectD = 45000;//���̽�����
const double graphL = 5000;

#define CBS_EXCUTE(flag, exptr) {    \
        if (flag) {                  \
            exptr;                   \
        }                            \
}       

#define CBS_EXCUTE_PLUS(flag, exptr1, exptr2) {     \
    if (flag) {                                     \
            exptr1;                                 \
            exptr2;                                 \
        }                                           \
} 


namespace zc::detect {

inline static bool NearEq(double a, double b, double threshold) {
    return std::abs((a) - (b)) < (threshold);
}


class FuncCommon {
public:
    static void DeteMapSetUp(std::vector<std::vector<double>>&deteMap);   

    static void GetLinePara(double& x1, double& y1, double& x2, double& y2, LinePara & LP);//�õ�ֱ�߲���
    
    static void CrossP(Point mine, Point2f B, Point2f C, double enemyToA, double enemyToB, double enemyToC, double GPSEnemyAngL,
        LinePara enemyLinePara, Point2f&crossPoint);//��λ�÷ּ��㽻��
    
    static bool IsEqualDouble(double a , double b);
        
    static bool GetCross(LinePara para1, LinePara para2, Point2f & pt);  //�ж��Ƿ�ƽ�У���ƽ�еĻ��󽻵�
    
    static bool IsInTriangle(Point2f p1, Point2f p2, Point2f p3, Point2f o);  //�жϵ�o�Ƿ����������ڲ�
    
    static int EnterAngleIndex(double angel);

    static double SelectR1(double v);
    
    static int VectorIndex(double &tt, const double &start, double &perLen, double mframe);

    template <typename Point>
    static double Distance(const Point& pta, const Point& ptb) {
	    return sqrt(pow(((pta).x - (ptb).x), 2) + pow(((pta).y - (ptb).y), 2));
    }
    
    static double Product(Point2f p1, Point2f p2, Point2f p3); //������
    
    static double DetectTime(Point mine, Point enemy, Point2f B, Point2f C);     

    static double DetectScore(const Point &mine, const Point &enemy);

    static double EncapsulationTime(Point &mine, Point &enemy);//�����ζ�λʱ���װ

    static double PointAngle(double mine_x, double mine_y, double enemy_x, double enemy_y);   //�����ά��λ��   
   
    static double Distance(double mine_x, double mine_y, double mine_z, double enemy_x, double enemy_y, double enemy_z);   //������ά����
    
    static double Distance(double mine_x, double mine_y, double enemy_x, double enemy_y);   //�����ά����
       
    static double AngleStandard(double& azimuth);   //������Ƕȹ淶����ת��Ϊ[0-360)�ĽǶ�
   
    static double DisOfTwoPoint(double slog, double slat, double elog, double elat);

    static float CalHeadingT(double Long, double Lat, double long_T, double lat_T);

    static const std::vector<maneuverOutput>& SpeedSelect(double speed);
    
    static const std::vector<maneuverOutput>& AssignmentSpeedSelect(double speed);
    
    static std::vector<Point2f> PositionTrianglePoint(Point mine, double L);//����һ��ͻ����������ཻ������

    static std::vector<Point2f> TrianglePoint(Point mine, double sideLength);//�����ҷ��õ���λ������

    static std::pair<double, double> CompPosition(float delta_de_inm, float delta_dn_inm, double longitude, double latitude);

    static std::pair<double, double> Angle3D(Point mine, Point enemy);		//��ά��λ��
    
    static Point Exline(Point &p, double time);   

    static Point2f GetPoint(Point2f mine, double mineAzi, double enemyPosAng);   //��֪һ���뷽����ֱ��������һ����

    static Point2f PredictPlace(const Point &now, const double v[], const double azi[], const double &num, const double &sampleTime);     //�Ժ������Ҫ��Ȩ��

};

}


#endif //FUNC_COMMON_H