#pragma once
#include"extern.h"
#include"function.h"
#include<iostream>
#include <vector>
#include <queue>
#include <list>
#include<map>


// 定义直线参数结构体  
struct LinePara
{
	double k;
	double b;

};
void getLinePara(double& x1, double& y1, double& x2, double& y2, LinePara & LP);//得到直线参数

Point2f getPoint(Point2f mine, double mineAzi, double enemyPosAng);   //已知一点与方向，求直线上另外一个点

bool getCross(LinePara para1, LinePara para2, Point2f & pt);  //判断是否平行，不平行的话求交点

void crossP(Point mine, Point2f B, Point2f C, double enemyToA, double enemyToB, double enemyToC, double GPSEnemyAngL,
	LinePara enemyLinePara, Point2f&crossPoint);//定位得分计算交点

std::vector<Point2f>positionTrianglePoint(Point mine, double L);//根据一点和基线求两机相交三角形

std::vector<Point2f>trianglePoint(Point mine, double sideLength);//根据我方得到定位三角形


double product(Point2f p1, Point2f p2, Point2f p3); //计算叉乘

bool isInTriangle(Point2f p1, Point2f p2, Point2f p3, Point2f o);  //判断点o是否在三角形内部

double detectTime(Point mine, Point enemy, Point2f B, Point2f C);           //三角形定位时间   (前提：敌方已在我的定位范围内）

double detectScore(const Point &mine, const Point &enemy);

double encapsulationTime(Point &mine, Point &enemy);//三角形定位时间封装

std::vector<Vertex*>lineInterpolate(Vertex*start, Vertex*end, double time);	//在曲线进行5s一插值，得到一系列Point点

