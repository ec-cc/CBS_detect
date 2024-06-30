#pragma once
#include"extern.h"
#include"function.h"
#include<iostream>
#include <vector>
#include <queue>
#include <list>
#include<map>


// ����ֱ�߲����ṹ��  
struct LinePara
{
	double k;
	double b;

};
void getLinePara(double& x1, double& y1, double& x2, double& y2, LinePara & LP);//�õ�ֱ�߲���

Point2f getPoint(Point2f mine, double mineAzi, double enemyPosAng);   //��֪һ���뷽����ֱ��������һ����

bool getCross(LinePara para1, LinePara para2, Point2f & pt);  //�ж��Ƿ�ƽ�У���ƽ�еĻ��󽻵�

void crossP(Point mine, Point2f B, Point2f C, double enemyToA, double enemyToB, double enemyToC, double GPSEnemyAngL,
	LinePara enemyLinePara, Point2f&crossPoint);//��λ�÷ּ��㽻��

std::vector<Point2f>positionTrianglePoint(Point mine, double L);//����һ��ͻ����������ཻ������

std::vector<Point2f>trianglePoint(Point mine, double sideLength);//�����ҷ��õ���λ������


double product(Point2f p1, Point2f p2, Point2f p3); //������

bool isInTriangle(Point2f p1, Point2f p2, Point2f p3, Point2f o);  //�жϵ�o�Ƿ����������ڲ�

double detectTime(Point mine, Point enemy, Point2f B, Point2f C);           //�����ζ�λʱ��   (ǰ�᣺�з������ҵĶ�λ��Χ�ڣ�

double detectScore(const Point &mine, const Point &enemy);

double encapsulationTime(Point &mine, Point &enemy);//�����ζ�λʱ���װ

std::vector<Vertex*>lineInterpolate(Vertex*start, Vertex*end, double time);	//�����߽���5sһ��ֵ���õ�һϵ��Point��

