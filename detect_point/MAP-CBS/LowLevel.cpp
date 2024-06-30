#include "LowLevel.h"
using namespace std;
#define  AS_ANGLE_LIMIT 1
#define  AS_DIS_LIMIT 60000


int LowLevelCBS::selectR1(double& v)
{
	return (-1128 + 37.43 * v - 0.008036 * v * v);
}


void LowLevelCBS::pArrResize(double framey, double framex)
{
	_gridHeight = framey;
	_gridWidth = framex;
	map.resize(_gridHeight);
	for (int i = 0; i < _gridHeight; i++)
	{
		map[i].resize(_gridWidth);
		for (int j = 0; j < _gridWidth; j++)
		{
			map[i][j].resize(37);
		}
	}
}


bool LowLevelCBS::AStar(Vertex* start, Vertex* goal, Path *path)
{
	double dis = distance(start->x, start->y, goal->x, goal->y);
	// ClearMapAStarValues();
	double gAngle = 0;
	double gDis = 0;
	start->g = gAngle + gDis;
	double hAngle = 1;
	double hDis = dis / dis;
	double h = hAngle + hDis;
	start->f = start->g + h;
	start->depth = 0;
	map[start->yIndex][start->xIndex][start->angleIndex] = start;
	openList.push(start);

	//int timeStep = 0;
	bool seek = true;
	//TODO_BAHAR need better limit case to terminate a*
	while (!openList.empty() && seek)
	{
		auto front = openList.top();
		auto actionSample = speedSelect(front->v);
		front->visited = true;
		openList.pop();
		vector<Vertex*> successors;
		FillNeighboors(front, successors, seek);                              
		for (int i = 0; i < successors.size(); i++)
		{
			double widthIndex = vectorIndex(successors[i]->y, centerY, lengthY, frameY);
			double lengthIndex = vectorIndex(successors[i]->x, centerX, lengthX, frameX);
			double angleIndex = enterAngleIndex(successors[i]->heading);
			if (map[widthIndex][lengthIndex][angleIndex] && !map[widthIndex][lengthIndex][angleIndex]->visited)  //visit?    机动空间visit
			{
				gAngle = abs(actionSample[i].changeA) / 360.0;													//  0628修改  采样角度除以360 
				gDis = actionSample[i].dist / dis;															 //采样距离除以总距离
				auto diff = abs(point_angle(successors[i]->x, successors[i]->y, goal->x, goal->y) - successors[i]->heading) > 180 ? 360 - abs(point_angle(successors[i]->x, successors[i]->y, goal->x, goal->y) - successors[i]->heading) : abs(point_angle(successors[i]->x, successors[i]->y, goal->x, goal->y) - successors[i]->heading);
				hAngle = angle_standard(diff) / 360.0;													 // 下一个动作方位角与航向角的差值  
				hDis = distance(successors[i]->x, successors[i]->y, goal->x, goal->y) / dis;			// 当前点到目标点距离启发函数
				auto g = gAngle + gDis + front->g;
				auto h = hAngle + hDis;
				auto f = g + h;
				if (f < map[widthIndex][lengthIndex][angleIndex]->f)
				{
					successors[i]->g = g;
					successors[i]->h = h;
					successors[i]->f = f;
					map[widthIndex][lengthIndex][angleIndex] = successors[i];
					openList.push(successors[i]);
					if (hAngle * 360 < AS_ANGLE_LIMIT && hDis*dis < AS_DIS_LIMIT)
					{
						seek = false;
						goal->Parent = successors[i];
						ReconstructPath(goal,path);
						return true;
					}
				}
			}
			else if (!map[widthIndex][lengthIndex][angleIndex])
			{
				gAngle = abs(actionSample[i].changeA) / 360.0;                                                  //0628修改
				gDis = actionSample[i].dist / dis;
				auto diff = abs(point_angle(successors[i]->x, successors[i]->y, goal->x, goal->y) - successors[i]->heading) > 180 ? 360 - abs(point_angle(successors[i]->x, successors[i]->y, goal->x, goal->y) - successors[i]->heading) : abs(point_angle(successors[i]->x, successors[i]->y, goal->x, goal->y) - successors[i]->heading);
				hAngle = angle_standard(diff) / 360.0;
				hDis = distance(successors[i]->x, successors[i]->y, goal->x, goal->y) / dis;
				auto g = gAngle + gDis + front->g;
				auto h = hAngle + hDis;
				auto f = g + h;
				map[widthIndex][lengthIndex][angleIndex] = successors[i];
				openList.push(successors[i]);
				if (hAngle * 360 < AS_ANGLE_LIMIT && hDis*dis < AS_DIS_LIMIT)
				{
					seek = false;
					goal->Parent = successors[i];
					ReconstructPath(goal,path);
					return true;
				}
			}
			else if (map[widthIndex][lengthIndex][angleIndex] && map[widthIndex][lengthIndex][angleIndex]->visited)
			{
				auto diff = abs(point_angle(successors[i]->x, successors[i]->y, goal->x, goal->y) - successors[i]->heading) > 180 ? 360 - abs(point_angle(successors[i]->x, successors[i]->y, goal->x, goal->y) - successors[i]->heading) : abs(point_angle(successors[i]->x, successors[i]->y, goal->x, goal->y) - successors[i]->heading);
				auto hAngle = angle_standard(diff) / 360.0;
				auto hDis = distance(successors[i]->x, successors[i]->y, goal->x, goal->y) / dis;
				if (hAngle * 360 < AS_ANGLE_LIMIT && hDis*dis < AS_DIS_LIMIT)
				{
					seek = false;
					goal->Parent = successors[i];
				    ReconstructPath(goal,path);
					return true;
				}
			}
			else {
				cout << "nnnnnnnnnnnnnnnnn" << endl;
			}
		}
	}
	return false;
}

bool LowLevelCBS:: extendTime(Vertex* start, vector<Vertex*> Allend, Path& pathA, double extendtime)   //改成只与时间相关的启发函数   效果更好
{
	Vertex* end = Allend[0];
	bool isPath = false;
	end->Parent = NULL;
	centerX = (start->x + end->x) / 2;
	centerY = (start->y + end->y) / 2;
	double xLength = abs(start->x - end->x);
	double yLength = abs(start->y - end->y);
	int _gridWidth = ceil(xLength / lengthX + 15);
	int _gridHeight = ceil(yLength / lengthY + 15);
	end->heading = angle_standard(end->heading);
	Vertex* newFront = NULL;
	ClearMapAStarValues();
	//double minTime = pathA.Nodes[0]->totalTime;
	double minTime = pathA.Nodes[pathA.Nodes.size() - 2]->totalTime;
	double rate = extendtime / minTime;
	//if (rate < 1) {
	//	cout << "提前到达无法保证！" << endl;
	//	return false;
	//}

	double g = 0;
	double diff = abs(end->heading - start->heading) > 180 ? 360 - abs(end->heading - start->heading) : abs(end->heading - start->heading);
	double hAngle = angle_standard(diff) / 360.0;
	double hDis;
	auto firstTime = minTime * rate;    //修改最短时间

	bool seek = true;
	Point endd(end->x, end->y, end->z, (-end->heading + 90) / 57.3, end->v);
	double q1[] = { endd.x,endd.y,endd.azimuth };
	Point startt(start->x, start->y, start->z, (-start->heading + 90) / 57.3, start->v);
	DubinsPath path;
	r0 = selectR1(start->v);
	double q0[] = { startt.x,startt.y,startt.azimuth };
	dubins_shortest_path(&path, q0, q1, r0);  //起点，终点和转弯半径
	//double shortestPath = path.param[0] + path.param[1] + path.param[2];
	//h = (shortestPath * r0 / mine.v);
	double dis = (path.param[0] + path.param[1] + path.param[2]) * r0 * rate;
	double h = dis / start->v;
	double f = h;
	dis = distance(start->x, start->y, end->x, end->y);
	Vertex* pt = new Vertex(f, g, h, start->x, start->y, start->z, start->heading, 0, start->v);
	allPoint.push_back(pt);
	pt->stepp = 0;
	pt->visited = 1;
	auto index = enterAngleIndex(angle_standard(start->heading));
	map[vectorIndex(start->y, centerY, lengthY, frameY)][vectorIndex(start->x, centerX, lengthX, frameX)][index] = pt;
	openList.push(pt);

	Vertex* front = NULL;
	int widthIndex, lengthIndex, angleIndex;
	double shortestPath;
	while (seek && !openList.empty())
	{
		front = openList.top();
		openList.pop();
		widthIndex = vectorIndex(front->y, centerY, lengthY, frameY);
		lengthIndex = vectorIndex(front->x, centerX, lengthX, frameX);
		angleIndex = enterAngleIndex(front->heading);
		front->visited = true;
		auto actionSample = speedSelect(front->v);
		for (int i = 0; i < actionSample.size() && seek; i++)
		{
			double newX = front->x + sin((actionSample[i].distA + front->heading) / 180 * 3.1415926) * actionSample[i].dist;
			double newY = front->y + cos((actionSample[i].distA + front->heading) / 180 * 3.1415926) * actionSample[i].dist;
			double newZ = front->z + actionSample[i].changeH;
			double sampleTime = actionSample[i].time;
			double stepp = front->stepp + 1;
			double newV = actionSample[i].changeV;
			auto aziEnd = front->heading + actionSample[i].changeA;
			double newAzi = angle_standard(aziEnd);
			Point t(newX, newY, newZ, newAzi, 0, newV);    //0629修改
			widthIndex = vectorIndex(t.y, centerY, lengthY, frameY);
			lengthIndex = vectorIndex(t.x, centerX, lengthX, frameX);
			angleIndex = enterAngleIndex(aziEnd);
			if (t.x<centerX - (int)(frameX / 2) * lengthX || t.x>centerX + (int)(frameX / 2) * lengthX || t.y<centerY - (int)(frameY / 2) * lengthY || t.y>centerY + (int)(frameY / 2) * lengthY)
				continue;
			diff = abs(end->heading - t.azimuth) > 180 ? 360 - abs(end->heading - t.azimuth) : abs(end->heading - t.azimuth);
			hAngle = angle_standard(diff) / 360.0;
			hDis = distance(t.x, t.y, end->x, end->y) / dis;
			//g = actionSample[i].time + front->totalTime;
			g = actionSample[i].time + front->g;


			if (hDis < 0.2 && hAngle * 360 < 5 && g > firstTime * 1 && g < firstTime * 1.2)
			{
				seek = false;
				Vertex* newPoint = new Vertex(f, g, h, t.x, t.y, t.z, t.azimuth, t.pitch, t.v);    //0629修改
				allPoint.push_back(newPoint);
				newPoint->time = sampleTime;
				newPoint->totalTime = front->totalTime + sampleTime;
				newPoint->Parent = front;
				newPoint->stepp = stepp;
				end->Parent = newPoint;
				isPath = true;
			}

			if (map[widthIndex][lengthIndex][angleIndex] && !map[widthIndex][lengthIndex][angleIndex]->visited)
			{
				///************采用dubins曲线计算时间*********/

				auto tempM = (-t.azimuth + 90) / 57.3;

				Point mine(t.x, t.y, t.z, tempM, t.v);

				double q0[] = { mine.x,mine.y,mine.azimuth };
				DubinsPath path;
				r0 = selectR1(mine.v);
				dubins_shortest_path(&path, q0, q1, r0);  //起点，终点和转弯半径
				shortestPath = path.param[0] + path.param[1] + path.param[2];
				h = (shortestPath * r0 / 270);
				//h = min(h, perdictFlyTime(t, end));
				/************采用dubins曲线计算时间*********/


				/********延长时间**********/

				f = abs(g + h * 1. - firstTime);
				/********延长时间**********/

				if (f < map[widthIndex][lengthIndex][angleIndex]->f)
				{
					Vertex* newPoint = new Vertex(f, g, h, t.x, t.y, t.z, t.azimuth, t.pitch, t.v);    //0629修改
					allPoint.push_back(newPoint);
					newPoint->visited = 0;
					newPoint->time = sampleTime;
					newPoint->totalTime = front->totalTime + sampleTime;
					newPoint->Parent = front;
					newPoint->stepp = stepp;
					map[widthIndex][lengthIndex][angleIndex] = newPoint;
					openList.push(newPoint);
					if (hDis < 0.2 && hAngle * 360 < 5 && g > firstTime * 1 && g < firstTime * 1.2)
					{
						seek = false;
						end->Parent = newPoint;
						isPath = true;
					}
				}
			}
			else if (!map[widthIndex][lengthIndex][angleIndex])
			{
				diff = abs(end->heading - t.azimuth) > 180 ? 360 - abs(end->heading - t.azimuth) : abs(end->heading - t.azimuth);
				hAngle = angle_standard(diff) / 360.0;
				hDis = distance(t.x, t.y, end->x, end->y) / dis;
				g = actionSample[i].time + front->g;



				/************采用dubins曲线计算时间*********/
				auto tempM = (-t.azimuth + 90) / 57.3;

				Point mine(t.x, t.y, t.z, tempM, t.v);

				double q0[] = { mine.x,mine.y,mine.azimuth };
				DubinsPath path;
				r0 = selectR1(mine.v);
				dubins_shortest_path(&path, q0, q1, r0);  //起点，终点和转弯半径
				shortestPath = path.param[0] + path.param[1] + path.param[2];
				h = (shortestPath * r0 / 270);
				//h = min(h, perdictFlyTime(t, end));
				/************采用dubins曲线计算时间*********/

				/********延长时间**********/

				f = abs(g + h * 1. - firstTime);
				/********延长时间**********/

				Vertex* newPoint = new Vertex(f, g, h, t.x, t.y, t.z, t.azimuth, t.pitch, t.v);
				allPoint.push_back(newPoint);
				newPoint->visited = 0;
				newPoint->time = sampleTime;
				newPoint->totalTime = front->totalTime + sampleTime;
				newPoint->Parent = front;
				newPoint->stepp = stepp;

				map[widthIndex][lengthIndex][angleIndex] = newPoint;
				openList.push(newPoint);
				if (hDis < 0.2 && hAngle * 360 < 5 && g > firstTime * 1 && g < firstTime * 1.2)
				{
					seek = false;
					end->Parent = newPoint;
					isPath = true;
				}
			}
		}
	}
	auto p = end;

	if (p->Parent)
	{
		pathA.Nodes.clear();
		while (true)
		{
			if (p->time < 5 || p->time > 45)
			{
				if (p->Parent)
				{
					p = p->Parent;
					continue;
				}
				else
					break;
			}
			pathA.Nodes.push_back(p);
			p = p->Parent;
		}
		pathA.Nodes.push_back(p);
		pathA.Nodes[0] = Allend[1];
		std::reverse(pathA.Nodes.begin(), pathA.Nodes.end());
		pathA.Nodes[0]->totalTime = pathA.Nodes[pathA.Nodes.size() - 2]->g;
	}
	else
	{
		cout << "   规划无果！ " << endl;
		isPath = false;
	}
	//Allend[1]->Parent = pathA.Nodes[pathA.Nodes.size() - 1];
	end = Allend[1];
	return isPath;
}


void LowLevelCBS::ClearMapAStarValues()
{
	for (int i = 0; i < allNode.size(); i++) {
		delete allNode[i];
	}
	allNode.clear();
	map.clear();
	openList = {};
	pArrResize(_gridHeight, _gridWidth);
}
void LowLevelCBS::FillNeighboors(Vertex* node, vector<Vertex*> &successors, bool seek)
{
	auto actionSample = speedSelect(node->v);
	vector<double>lastangle;
	bool hconflict = false;
	for (int i = 0; i < actionSample.size() && seek; i++)
	{
		if (hconflict)
		{
			bool con = false;
			for (int j = 0; j < lastangle.size(); j++)
			{
				if (actionSample[i].yingfeiA == lastangle[j])
				{
					con = true;
					break;
				}
			}
			if (con)
			{
				continue;
			}
		}
		double newX = node->x + sin((actionSample[i].distA + node->heading) / 180 * 3.1415926) * actionSample[i].dist;
		double newY = node->y + cos((actionSample[i].distA + node->heading) / 180 * 3.1415926) * actionSample[i].dist;
		double sampleTime = actionSample[i].time;
		double stepp = node->depth + 1;
		double newV = actionSample[i].changeV;
		auto aziEnd = node->heading + actionSample[i].changeA;
		if (aziEnd >= 360)
			aziEnd = aziEnd - 360;
		else if (aziEnd < 0)
			aziEnd = aziEnd + 360;
		double newAzi = aziEnd;
		Point t(newX, newY, 0, newAzi, 0, newV);    //0629修改
		Vertex *child = new Vertex(newX, newY, newAzi, newV);
		child->xIndex = vectorIndex(child->x, centerX, lengthX, frameX);
		child->yIndex = vectorIndex(child->y, centerY, lengthY, frameY);
		child->time = sampleTime;
		child->totalTime = node->totalTime + sampleTime;
		if (t.x<centerX - (int)(frameX / 2) * lengthX || t.x>centerX + (int)(frameX / 2) * lengthX 
			|| t.y<centerY - (int)(frameY / 2) * lengthY || t.y>centerY + (int)(frameY / 2) * lengthY)
		{
			lastangle.push_back(actionSample[i].yingfeiA);                       //lastangle?
			hconflict = true;
			continue;
		}
		child->depth = node->depth + 1;
		child->visited = false;
		child->Parent = node;
		allNode.push_back(child);
		successors.push_back(child);
	}
}

int LowLevelCBS::HeuristicCostEstimate(const Vertex& a, const Vertex& b) const
{
	int manhattan_distance = get_manhattan_distance(a, b);
	return manhattan_distance == 0 ? 1 : manhattan_distance;
}

int LowLevelCBS::get_manhattan_distance(const Vertex& a, const Vertex& b) const
{
	return abs(a.x - b.x) + abs(a.y - b.y);
}

double  LowLevelCBS::get_square_distance_between_nodes(const Vertex& a, const Vertex& b) const
{
	return pow((a.x - b.x), 2) + pow((a.y - b.y), 2);
}
int  LowLevelCBS::get_node_with_least_f(const std::vector<Vertex*> &list) const
{
	//TODO
	int min = INT_MAX;
	int min_index = 0;
	for (int i = 0; i < list.size(); i++)
	{
		if (list[i]->f < min)
		{
			min = list[i]->f;
			min_index = i;
		}
	}

	return min_index;
}
void LowLevelCBS::ReconstructPath(Vertex* node , Path * path)
{
	//for (int i = 0; i < path->Nodes.size(); i++) {
	//	delete path->Nodes[i];
	//}

	path->Nodes.clear();
	std::vector<Vertex*> path_reverse;

	while (node != NULL)
	{
		path_reverse.push_back(node);
		node = node->Parent;
	}
	// 下层的规划的节点
	for (int i = path_reverse.size() - 1; i >= 0; i--)
	{
		auto tmp = new Vertex(path_reverse[i]);
		path->Nodes.push_back(tmp);
	}

	return;
}
bool  LowLevelCBS::HasConflict(Vertex* child, double time, const std::vector<Constraint*> &constraints)
{
	for (int i = 0; i < constraints.size(); i++)
	{
		if ((time == (constraints[i]->Vertex->totalTime)) && (fabs(child->x - (constraints[i]->Vertex->x) < 1e-8)) && (fabs(child->y - (constraints[i]->Vertex->y) < 1e-8)))
		{
			return true;
		}
	}
	return false;
}