#pragma once
//#include "CBSDataStructures.h"
#include "LowLevel.h"
#include"function.h"
#include"geometry.h"

class CTNode
{
private:
	Constraint* _constraint;                   // Լ�� 
	std::vector<Path*> _solution;                            // ÿ��������Ĺ켣�㣬���һ����̽���
	std::vector<Conflict*> _conflicts;                       // ��ͻ��
	std::vector<detectTimePeriod> _dtp;                      // �Թ̶���̽�����ʼʱ��
public:
	int debugIndex;
	double cost;
	double totalTime;                                   // ��̽�⸲��ʱ�� 
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

struct comppp //��д�º���
{
	bool operator() (CTNode *a, CTNode *b)
	{
		return a->cost < b->cost; //�󶥶�
	}
};

class HighLevelCBS
{
public:
	LowLevelCBS _lowLevelSolver;
	std::vector <Agent*> _agents;
	std::vector<Vertex*> agentStart;
	std::vector<Vertex*> agentEnd;

	std::priority_queue<CTNode*, std::vector<CTNode*>, comppp> openListHigh;        //  ���ȶ���
	double handoverDetectRate;                                            //  ���õĽ���ʱ�����
	double coLocateTimeRate;                                              //  ����Эͬ��λʱ�����
	double maxCost;                                                       //  ������
	double totalTimeOfmaxCost;                                            //  �����۶�Ӧ��̽����ʱ��
	int SearchTimes;                                                      //  ��������
	int EqualTimes;                                                       //  ���������������ֵ��ȵĴ���
	int maxSearchTimes;                                                   //  ���������������
	int maxEqualTimes;                                                    //  �����ͬ��������    ��Լ������µľֲ����Ž�
	CTNode* resCTN;                                                       //  ���������Լ�����
	std::vector<Path*> resultPath;
	std::vector<Conflict*>  allConficts;
	std::vector<Constraint*> allConstraints;
	std::vector<Path*> allPaths;
	std::vector<CTNode*> allCTNodes;
	std::vector<Vertex*> allVertex;
public:
	HighLevelCBS();
	~HighLevelCBS();                                                         // ����������Ҫ delete �����ı��� 
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

