#pragma once
//#include "CBSDataStructures.h"
#include "LowLevel.h"
#include"function.h"
#include"geometry.h"

class CTNode
{
private:
	Constraint* _constraint;                   // 约束 
	std::vector<Path*> _solution;                            // 每个智能体的轨迹点，最后一个是探测点
	std::vector<Conflict*> _conflicts;                       // 冲突点
	std::vector<detectTimePeriod> _dtp;                      // 对固定点探测的起始时间
public:
	int debugIndex;
	double cost;
	double totalTime;                                   // 总探测覆盖时间 
	//TODO_BAHAR_1 destructiooooon
	const std::vector <Path*>& get_solution() { return _solution; }
	std::vector<Conflict*>& get_conflict() { return _conflicts; }
	Constraint* get_constraint() const { return _constraint; }
	const std::vector<detectTimePeriod>&get_dtp() const { return _dtp; }
	void add_conflict(Conflict* new_conflict) { _conflicts.push_back(new_conflict); }
	void set_solutionInit(const std::vector<Path*> &new_solution, std::vector<Path*>&allPath);
	void set_solution(const std::vector<Path*>& new_solution, std::vector<Vertex*> allVertex, std::vector<Path*> allPaths);
	void set_solution_for_agent(Agent& agent, std::vector<Vertex*> allVertex, std::vector<Path*>&allPath);
	void set_constraint(Constraint* constraint);
	void set_dtp();
	void clear_conflicts();
	void clear_solution();

};

struct comppp //重写仿函数
{
	bool operator() (CTNode *a, CTNode *b)
	{
		return a->cost < b->cost; //大顶堆
	}
};

class HighLevelCBS
{
public:
	LowLevelCBS _lowLevelSolver;
	std::vector <Agent*> _agents;
	std::vector<Vertex*> agentStart;
	std::vector<Vertex*> agentEnd;

	std::priority_queue<CTNode*, std::vector<CTNode*>, comppp> openListHigh;        //  优先队列
	double handoverDetectRate;                                            //  设置的交班时间比率
	double coLocateTimeRate;                                              //  设置协同定位时间比率
	double maxCost;                                                       //  最大代价
	double totalTimeOfmaxCost;                                            //  最大代价对应的探测总时间
	int SearchTimes;                                                      //  搜索次数
	int EqualTimes;                                                       //  搜索过程中与最大值相等的次数
	int maxSearchTimes;                                                   //  最大搜索次数限制
	int maxEqualTimes;                                                    //  最大相同次数限制    有约束情况下的局部较优解
	CTNode* resCTN;                                                       //  最终输出的约束结点
	std::vector<Path*> resultPath;
	std::vector<Conflict*>  allConficts;
	std::vector<Constraint*> allConstraints;
	std::vector<Path*> allPaths;
	std::vector<CTNode*> allCTNodes;
	std::vector<Vertex*> allVertex;
public:
	HighLevelCBS();
	~HighLevelCBS();                                                         // 析构函数中要 delete 创建的变量 
	std::vector <Path*>& RunCBS();
	void addAgent(double idx, double stStateX1, double stStateY1, double stStateHeading, double stStateV,
		double glStateX1, double glStateY1, double glStateHeading, double v);
	double GetSIC(const std::vector <Path*> &solution, double& totalTime);
	bool ValidatePathsInNode(CTNode& node);
	void FindPathsForAllAgents(CTNode &node, std::vector <Path*>& allPath);
	bool UpdateSolutionByInvokingLowLevel(CTNode& node);
	void InitialCBS();
	void clearCBS();
	void getRes(CTNode& node);
};

