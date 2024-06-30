#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include<queue>
#include<vector>
#include "CBSDataStructures.h"
#include"function.h"
#include"geometry.h"
#include"dubins.h"

struct compppp //��д�º���
{
	bool operator() (Vertex *a, Vertex *b)
	{
		return a->f > b->f; //С����
	}
};

class LowLevelCBS
{

public:
	//Vertex map[10][10];
	//TODO_BAHAR 
	std::vector<std::vector<std::vector<Vertex*>>> map;
	std::vector<Vertex*> allNode;
	double lengthX;
	double lengthY;
	double frameX;
	double frameY;
	double centerX;
	double centerY;
	double r0 = 7000;		//dubins����ת��뾶
	std::vector<Vertex*> allPoint;

	int _gridWidth;
	int _gridHeight;
	std::priority_queue<Vertex*, std::vector<Vertex*>, compppp> openList;   //���ȶ���

public:
	LowLevelCBS() {}
	int selectR1(double& v);
	void pArrResize(double framey, double framex);
	bool AStar(Vertex* start, Vertex* goal, Path* path);
	bool extendTime(Vertex* start, std::vector<Vertex*> end_two, Path& pathA, double extendtime);   //�ĳ�ֻ��ʱ����ص���������   Ч������
	void ClearMapAStarValues();
	void FillNeighboors(Vertex* node, std::vector<Vertex*>& successors, bool seek);
	int HeuristicCostEstimate(const Vertex& a, const Vertex& b) const;
	int get_manhattan_distance(const Vertex& a, const Vertex& b) const;
	double get_square_distance_between_nodes(const Vertex& a, const Vertex& b) const;
	int get_node_with_least_f(const std::vector<Vertex*>& list) const;
	void ReconstructPath(Vertex* node, Path* path);
	bool HasConflict(Vertex* child, double time, const std::vector<Constraint*>& constraints);
	int GetWidth()const { return _gridWidth; }
	int GetHeight()const { return _gridHeight; }
};
