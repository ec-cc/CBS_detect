#include "geometry.h"
using namespace std;
void getLinePara(double& x1, double& y1, double& x2, double& y2, LinePara & LP)
{
	double m = 0;

	// 计算分子  
	m = x2 - x1;

	if (0 == m)
	{
		LP.k = 10000.0;
		LP.b = y1 - LP.k * x1;
	}
	else
	{
		LP.k = (y2 - y1) / (x2 - x1);
		LP.b = y1 - LP.k * x1;
	}
}

Point2f getPoint(Point2f mine, double mineAzi, double enemyPosAng)   
{
	double GPSEnemyAng = enemyPosAng;
	GPSEnemyAng = angle_standard(GPSEnemyAng);
	auto bb = cos(GPSEnemyAng / 57.3);
	auto bbc = sin(GPSEnemyAng / 57.3);
	double pointX = mine.x + 60000 * sin(GPSEnemyAng / 57.3);
	double pointY = mine.y + 60000 * cos(GPSEnemyAng / 57.3);
	return Point2f{ pointX,pointY };
}

bool getCross(LinePara para1, LinePara para2, Point2f & pt)
{
	// 判断是否平行  
	if (abs(para1.k - para2.k) > 0.0001)
	{
		pt.x = (para2.b - para1.b) / (para1.k - para2.k);
		pt.y = para1.k * pt.x + para1.b;

		return true;
	}
	else
	{
		return false;
	}
}

void crossP(Point mine, Point2f B, Point2f C, double enemyToA, double enemyToB, double enemyToC, double GPSEnemyAngL,
	LinePara enemyLinePara, Point2f&crossPoint)
{
	//前提：A为飞机当前机头方向，且位于第一象限，B在C上方
	//如果B小于180，则可以直接计算
	if (enemyToB < 180)
	{
		auto nextB = enemyToB + 360;
		//穿越BC
		if (GPSEnemyAngL > enemyToB && GPSEnemyAngL <= enemyToC)
		{
			LinePara BToC;
			getLinePara(B.x, B.y, C.x, C.y, BToC);
			getCross(enemyLinePara, BToC, crossPoint);
		}
		//穿越AC
		else if (GPSEnemyAngL > enemyToC && GPSEnemyAngL <= enemyToA)
		{
			LinePara AToC;
			getLinePara(mine.x, mine.y, C.x, C.y, AToC);
			getCross(enemyLinePara, AToC, crossPoint);
		}
		//穿越AB
		else
		{
			LinePara AToB;
			getLinePara(B.x, B.y, mine.x, mine.y, AToB);
			getCross(enemyLinePara, AToB, crossPoint);
		}
	}
	//	//如果B大于180，B-360得到一个负值，大于负值，小于正值为穿越2
	else
	{
		//穿越BC
		auto nextB = enemyToB - 360;
		if (GPSEnemyAngL > nextB && GPSEnemyAngL <= enemyToC)
		{
			LinePara BToC;
			getLinePara(B.x, B.y, C.x, C.y, BToC);
			getCross(enemyLinePara, BToC, crossPoint);
		}
		//穿越AC
		else if (GPSEnemyAngL > enemyToC && GPSEnemyAngL <= enemyToA)
		{
			LinePara AToC;
			getLinePara(mine.x, mine.y, C.x, C.y, AToC);    //改   6.14
			getCross(enemyLinePara, AToC, crossPoint);
		}
		//穿越AB
		else
		{
			LinePara AToB;
			getLinePara(B.x, B.y, mine.x, mine.y, AToB);    //改   6.14
			getCross(enemyLinePara, AToB, crossPoint);
		}
	}
}

vector<Point2f>positionTrianglePoint(Point mine, double L)
{
	vector<Point2f>result;
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

double product(Point2f p1, Point2f p2, Point2f p3) //计算叉乘
{
	//首先根据坐标计算p1p2和p1p3的向量，然后再计算叉乘
	return (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
}

bool isInTriangle(Point2f p1, Point2f p2, Point2f p3, Point2f o)  //判断点o是否在三角形内部
{
	//保证p1，p2，p3是逆时针顺序,用了个递归
	if (product(p1, p2, p3) < 0)
		return isInTriangle(p1, p3, p2, o);
	if (product(p1, p2, o) > 0 && product(p2, p3, o) > 0 && product(p3, p1, o) > 0)
		return true;
	return false;
}

vector<Point2f>trianglePoint(Point mine,double sideLength)
{
	vector<Point2f> result;
	Point2f A, B;
	A.x = mine.x + sideLength * cos((mine.azimuth + 30) / 57.296);
	A.y = mine.y + sideLength * sin((mine.azimuth + 30) / 57.296);
	B.x = mine.x + sideLength * cos((mine.azimuth - 30) / 57.296);
	B.y = mine.y + sideLength * sin((mine.azimuth - 30) / 57.296);
	result.push_back(A);
	result.push_back(B);
	return result;
}

double detectTime(Point mine, Point enemy, Point2f B, Point2f C)            //敌人航向角是大地坐标系
{
	//B上，C下
	mine.azimuth = angle_standard(mine.azimuth);
	//double temp = mine.azimuth - 180;
	//double mine180 = angle_standard(temp);
	//pair<double, double>mineLineAngle = mine180 < mine.azimuth ? make_pair(mine180, mine.azimuth) : make_pair(mine.azimuth, mine180);
	enemy.azimuth = angle_standard(enemy.azimuth);
	//保留敌方原始坐标
	double originalX = enemy.x;
	double originalY = enemy.y;
	double originalBX = B.x;
	double originalBY = B.y;
	double originalCX = C.x;
	double originalCY = C.y;
	//auto vvdsv = std::atan2(originalY, originalX);
	double enemyPlaceAngle = atan2(originalY - mine.y, originalX - mine.x) > 0 ? atan2(originalY - mine.y, originalX - mine.x)*57.296 : (atan2(originalY - mine.y, originalX - mine.x)*57.296 + 360);
	double diffAngle = enemyPlaceAngle - mine.azimuth;   //得到敌方与我航机的夹角
	double disEnemy = distance(originalX, originalY, mine.x, mine.y);
	enemy.x = disEnemy * cos(diffAngle / 57.296);       //把探测三角形转移到航向与x轴重合的三角形，因此敌方的坐标都需变动
	enemy.y = disEnemy * sin(diffAngle / 57.296);
	Point2f enemyOtherPoint = getPoint({ originalX,originalY }, mine.azimuth, enemy.azimuth);
	LinePara enemyLinePara;
	//计算敌方直线方程
	getLinePara(originalX, originalY, enemyOtherPoint.x, enemyOtherPoint.y, enemyLinePara);
	Point2f mineOtherPoint = getPoint({ mine.x,mine.y }, mine.azimuth, mine.azimuth);
	LinePara mineLinePara;
	//计算我方直线方程
	getLinePara(mine.x, mine.y, mineOtherPoint.x, mineOtherPoint.y, mineLinePara);
	bool isIntersect;      //判断敌方直线直接与我航线相交还是延长线相交

	Point2f nowIntersectP;
	if (getCross(mineLinePara, enemyLinePara, nowIntersectP))    //得到此时两直线交点
	{
		double disIntersectP = distance(nowIntersectP.x, nowIntersectP.y, mine.x, mine.y);
		Point2f afterIntersectP;
		afterIntersectP.x = disIntersectP * cos(0);     //交点转移到x轴上
		afterIntersectP.y = 0;
		if (atan2((nowIntersectP.y - originalY), (nowIntersectP.x - originalX)) == enemy.azimuth)
			isIntersect = true;
		else
			isIntersect = false;
		if (nowIntersectP.x == originalX && nowIntersectP.y == originalY)     //如果敌方点本身就在我方航线上，需要图书考虑
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
		//对转变后的敌方航向角分类判断   判断从哪个边出来
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
				cout << "时间无限大！！" << endl;
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

double detectScore(const Point &mine,const Point &enemy)
{
	auto dist = distance(mine.x, mine.y, enemy.x, enemy.y);
	double enemyAzi = point_angle(mine.x, mine.y, enemy.x, enemy.y);
	double angleDiff = abs(mine.azimuth - enemyAzi) > 180 ? 360 - abs(mine.azimuth - enemyAzi) : abs(mine.azimuth - enemyAzi);
	double angleS, distanceS;
	angleS = exp(-PI * pow((angleDiff / 30.0),2));
	distanceS = exp(-PI * pow(((dist- bestDetectD) / 30000.0),2));
	return 0.5*angleS + 0.5*distanceS;
}

double encapsulationTime(Point &mine, Point &enemy)//三角形定位时间封装
{
	mine.azimuth = -mine.azimuth + 90;   //转化为数学坐标系
	enemy.azimuth = -enemy.azimuth + 90; //转化为数学坐标系
	auto otherPoint = trianglePoint(mine, 70000);
	//cout << "判断是否在三角形内部  " << isInTriangle({ mine.x,mine.y }, otherPoint[0], otherPoint[1], { enemy.x,enemy.y }) << endl;
	if (isInTriangle({ mine.x,mine.y }, otherPoint[0], otherPoint[1], { enemy.x,enemy.y }))
		return detectTime(mine, enemy, otherPoint[0], otherPoint[1]);
	return 0;
}

vector<Vertex*>lineInterpolate(Vertex*start, Vertex*end,double time)		//在曲线进行5s一插值，得到一系列Point点
{
	vector<Vertex*>result;
	double lineDis = distance(start->x, start->y, end->x, end->y);
	double interpolatePointNum = time / 5 ;
	double interpolateDis = lineDis / interpolatePointNum;
	double lineAngle = atan2((end->x - start->x), (end->y - start->y));
	for (int i = 0; i < interpolatePointNum; i++)
	{
		Vertex *interpolatePoint=new Vertex((start->x + interpolateDis * (i + 1) * sin(lineAngle)), (start->y + interpolateDis * (i + 1) * cos(lineAngle)));
		interpolatePoint->totalTime = start->totalTime + 5 * (i + 1);
		//lineMap[interpolatePoint] = end;
		result.push_back(interpolatePoint);
	}
	return result;
}