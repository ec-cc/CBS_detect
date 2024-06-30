#pragma once
#include <vector>
#include <climits>
#include <algorithm>

struct Constraint;

struct Vertex
{
	Vertex() :
		g(0), h(0), f(0)
	{
		Parent = NULL;
	}

	Vertex(double x, double y, double heading, double v) :
		g(0), h(0), f(0), x(x), y(y), heading(heading), v(v)
	{
		Parent = NULL;
	}

	Vertex(double x, double y, bool obstacle = false) :
		x(x), y(y), g(0), h(0), f(0)
	{
		Parent = NULL;
		Obstacle = obstacle;
		depth = 0;
	}
	Vertex(double f, double g, double h, double x, double y, double z, double azimuth, double a, double v) :
		x(x), y(y), g(g), h(h), f(f), heading(azimuth), v(v)
	{
		Parent = NULL;
		Obstacle = false;
		depth = 0;
	}
	Vertex(Vertex* node) {
		x = node->x;
		y = node->y;
		z = node->z;
		heading = node->heading;
		v = node->v;
		g = node->g;
		h = node->h;
		f = node->f;
		visited = node->visited;
		time = node->time;
		totalTime = node->totalTime;
		endtime = node->endtime;
		nov = node->nov;
		depth = node->depth;
	}

	inline bool operator == (const Vertex &v) const
	{
		return v.xIndex == this->xIndex && v.yIndex == this->yIndex;
	}

	double x;
	double y;
	double z = 0;
	double heading;
	double v;
	double g;
	double h;
	double f;
	bool visited;
	double time;
	double totalTime = 0;
	double endtime = 0; //探测的结束时间
	int stepp = 0;
	double nov;

	double xIndex;
	double yIndex;
	double angleIndex;

	
	Vertex* Parent;
	int depth;

	bool Obstacle;
};


struct Path
{
	Path()
	{
	}
	Path(int index) : agentIndex(index)
	{
	}
	int agentIndex;
	std::vector<Vertex*> Nodes;
	//std::vector<Vertex*> constraints;
	std::vector<Constraint*> Constraints;                   // 每个路径里面的约束是？ 
	//TODO get real cost with nodes cost
	int get_cost() { return Nodes.size(); }
};

struct Agent
{
	Agent(double index, double startStateX1, double startStateY1, double startStateHeading, double startStateV, 
		  double goalStateX1, double goalStateY1, double goalStateHeading, double v)
		: Index(index), StartStateX(startStateX1), StartStateY(startStateY1), StartStateHeading(startStateHeading),
		StartStateV(startStateV), GoalStateX(goalStateX1), GoalStateY(goalStateY1), GoalStateHeading(goalStateHeading), goalV(v)
	{
		path = new Path(index);
	}
	int Index;
	//Better implementation
	double StartStateX;
	double StartStateY;
	double StartStateHeading;
	double StartStateV;
	double GoalStateX;
	double GoalStateY;
	double GoalStateHeading;
	double goalV;  //用于计算探测敌方的时间，定义一下敌方目标速度
	double xStartIndex;
	double yStartIndex;
	double angleStartIndex;
	double xEndIndex;
	double yEndIndex;
	double angleEndIndex;
	//TODO this probably not needed here
	Path *path;
	//std::vector<Constraint*> constraints;
};


struct Constraint
{
	Constraint(Agent* agent, Vertex* vertex, double timeStep) :
		Agent(agent), Vertex(vertex), TimeStep(timeStep)
	{
	}
	Agent* Agent;
	Vertex* Vertex;
	double TimeStep;
};

struct detectTimePeriod
{
	int agentIndex;
	double tStart;
	double tEnd;
	Vertex* vertexSTgt;
};
struct Conflict
{
	Conflict(Agent* agent1, Agent* agent2, Vertex* vertex, double timeStep) :
		Vertex(vertex), TimeStep(timeStep)
	{
		Agents[0] = agent1;
		Agents[1] = agent2;
	}
	//private:
	Agent* Agents[2];
	Vertex* Vertex;
	double TimeStep;
};


float Clip(float n, float lower, float upper);

