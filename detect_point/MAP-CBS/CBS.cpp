#include "CBS.h"
#include <iostream>
#include <algorithm>
#include <map>
using namespace std;
int quickScore[5] = { -16 , 8 , 2 , 1 , 0 };     // 黑(0,-4) ， 灰(1,8) , 白Ⅱ（2，4），白Ⅲ(3,1) ，白Ⅳ(4,0) 



void CTNode::set_solution(const std::vector<Path*>& new_solution, std::vector<Vertex*> allVertex, std::vector<Path*> allPaths)
{
	for (int i = 0; i < new_solution.size(); i++)
	{
		Path* tmpPath = new Path();
		tmpPath->agentIndex = new_solution[i]->agentIndex;
		for (int j = 0; j < new_solution[i]->Nodes.size(); j++)
		{
			Vertex* tmpVertex = new Vertex(new_solution[i]->Nodes[j]);
			allVertex.push_back(tmpVertex);
			tmpPath->Nodes.push_back(tmpVertex);
		}
		allPaths.push_back(tmpPath);
		_solution.push_back(tmpPath);
	}
}

void CTNode::set_solutionInit(const std::vector<Path*> &new_solution, vector<Path*>&allPath)
{
	for (int i = 0; i < new_solution.size(); i++)
	{
		_solution.push_back(new_solution[i]);
		allPath.push_back(new_solution[i]);
	}
}
void CTNode::set_solution_for_agent(Agent& agent, std::vector<Vertex*> allVertex, vector<Path*>&allPath)
{
	_solution[agent.Index] = new Path(agent.Index);
	for (int i = 0; i < agent.path->Nodes.size(); i++) {
		Vertex* tmpVertexNode = new Vertex(agent.path->Nodes[i]);
		allVertex.push_back(tmpVertexNode);
		_solution[agent.Index]->Nodes.push_back(tmpVertexNode);
	}
	allPath.push_back(_solution[agent.Index]);
	//for (auto s : allPath) {
	//	cout << s->Nodes.size() << endl;
	//}
	//cout <<"*************************************"<< endl;
}

void CTNode::set_dtp() {
	std::vector <Path*> solution = this->get_solution();
	if (this->get_dtp().size()) return;
	for (int i = 0; i < solution.size(); i++)
	{
		Point mine, enemy;
		detectTimePeriod dtpTemp;
		mine.x = solution[i]->Nodes[solution[i]->Nodes.size() - 2]->x;
		mine.y = solution[i]->Nodes[solution[i]->Nodes.size() - 2]->y;
		mine.v = solution[i]->Nodes[solution[i]->Nodes.size() - 2]->v;
		mine.azimuth = solution[i]->Nodes[solution[i]->Nodes.size() - 2]->heading;
		enemy.x = solution[i]->Nodes[solution[i]->Nodes.size() - 1]->x;
		enemy.y = solution[i]->Nodes[solution[i]->Nodes.size() - 1]->y;
		enemy.azimuth = solution[i]->Nodes[solution[i]->Nodes.size() - 1]->heading;
		enemy.v = solution[i]->Nodes[solution[i]->Nodes.size() - 1]->v;

		dtpTemp.agentIndex = i;
		dtpTemp.tStart = solution[i]->Nodes[solution[i]->Nodes.size() - 2]->totalTime;
		dtpTemp.tEnd = dtpTemp.tStart + encapsulationTime(mine, enemy);
		dtpTemp.vertexSTgt = solution[i]->Nodes[solution[i]->Nodes.size() - 2];
		_dtp.push_back(dtpTemp);
	}
}

void CTNode::set_constraint(Constraint* constraint)
{
	_constraint = constraint;
}


void CTNode::clear_solution() {
	for (int i = 0; i < _solution.size(); i++) {
		for (int j = 0; j < _solution[i]->Nodes.size(); j++) {
			if (_solution[i]->Nodes[j] != nullptr) {
				delete _solution[i]->Nodes[j];
				_solution[i]->Nodes[j] = nullptr;
			}
		}
		if (_solution[i] != nullptr) {
			delete _solution[i];
			_solution[i] = nullptr;
		}
	}
	_solution.clear();
}

void CTNode::clear_conflicts() 
{
	for (int i = 0; i < _conflicts.size(); i++) {
		if (_conflicts[i] != nullptr)
		delete _conflicts[i];
	}
	_conflicts.clear();
}


HighLevelCBS::HighLevelCBS()
{
	handoverDetectRate = 0.3;
	coLocateTimeRate = 0.5;
	maxCost = INT_MIN;
	totalTimeOfmaxCost = 0;
	SearchTimes = 0;
	EqualTimes = 0;
	maxSearchTimes = 10000;
	maxEqualTimes = 1000;
}
HighLevelCBS::~HighLevelCBS()
{
	// delete;  
}

void HighLevelCBS::addAgent(double idx, double stStateX1, double stStateY1, double stStateHeading, double stStateV,
	double glStateX1, double glStateY1, double glStateHeading, double v)
{
	_agents.push_back(new Agent(idx, stStateX1, stStateY1, stStateHeading, stStateV, glStateX1, glStateY1, glStateHeading, v));
}
bool HighLevelCBS::ValidatePathsInNode(CTNode& node)
{
	bool valid_solution = true;
	double lastTimeStep = 0;
	vector <Path*> solution = node.get_solution();
	if (solution.size() == 0)
	{
		return false;
	}
	vector<detectTimePeriod> dtp = node.get_dtp();
	sort(dtp.begin(), dtp.end(), [](const detectTimePeriod& a , const detectTimePeriod& b) {
		if (isEqualDouble(a.tStart, b.tStart)) {
			return a.tEnd < b.tEnd;
		}
		else {
			return a.tStart < b.tStart;
		}
	});
	for (int j = 0; j < dtp.size(); j++) {
		double handoverTime = (dtp[j].tEnd - dtp[j].tStart) * handoverDetectRate;
		handoverTime = max(handoverTime, 15.0);
		for (int k = j + 1; k < dtp.size(); k++) {
			if (isEqualDouble(dtp[j].tStart, dtp[k].tStart)) {
				node.add_conflict(new Conflict(_agents[dtp[k].agentIndex], _agents[dtp[j].agentIndex], dtp[k].vertexSTgt, dtp[j].tEnd));     //最后一个变量为延长时间
				valid_solution = false;
				return valid_solution;
				if (isEqualDouble(dtp[j].tEnd, dtp[k].tEnd)){
					node.add_conflict(new Conflict(_agents[dtp[k].agentIndex], _agents[dtp[j].agentIndex], dtp[k].vertexSTgt, dtp[j].tEnd));     //最后一个变量为延长时间
					return valid_solution;
				}
			}
			else {
				if (dtp[k].tStart > dtp[j].tStart && dtp[k].tStart < dtp[j].tEnd) {
					if (dtp[j].tEnd - dtp[k].tStart > handoverTime){
						node.add_conflict(new Conflict(_agents[dtp[k].agentIndex], _agents[dtp[j].agentIndex], dtp[k].vertexSTgt, dtp[j].tEnd));     //最后一个变量为延长时间
						valid_solution = false;
						return valid_solution;
					}
				}
			}
		}
	}
	return valid_solution;
}


void HighLevelCBS::FindPathsForAllAgents(CTNode &node, std::vector <Path*>& allPath)
{
	for (int i = 0; i < _agents.size(); i++)
	{
		Vertex *start = agentStart[i];
		start->xIndex = _agents[i]->xStartIndex;
		start->yIndex = _agents[i]->yStartIndex;
		start->angleIndex = _agents[i]->angleStartIndex;
		Vertex *goal = agentEnd[i];
		goal->xIndex = _agents[i]->xEndIndex;
		goal->yIndex = _agents[i]->yEndIndex;
		_lowLevelSolver.AStar(start, goal, _agents[i]->path);
		_lowLevelSolver.ClearMapAStarValues();
		_agents[i]->path->agentIndex = i;
		allPath.push_back(_agents[i]->path);
	}
}

bool HighLevelCBS::UpdateSolutionByInvokingLowLevel(CTNode &node)
{
	int agentIndex = node.get_constraint()->Agent->Index;
	Vertex *start = agentStart[agentIndex];
	start->xIndex = _agents[agentIndex]->xStartIndex;
	start->yIndex = _agents[agentIndex]->yStartIndex;
	vector<Vertex*> goal;
	goal.push_back(node.get_constraint()->Vertex);
	goal.push_back(agentEnd[agentIndex]);
	double time = node.get_constraint()->TimeStep;
	if (_lowLevelSolver.extendTime(start, goal, *(_agents[agentIndex]->path), time))	//一个智能体路径不变，另外一个改变 constraints没看到又啥操作啊   
	{
		_lowLevelSolver.ClearMapAStarValues();
		_agents[agentIndex]->path->agentIndex = agentIndex;
		node.set_solution_for_agent(*_agents[agentIndex], allVertex, allPaths);
		return true;
	}
	else
	{
		cout << " can't get path" << endl;
		return false;
	}
}

void HighLevelCBS::getRes(CTNode& node)
{
	CTNode* tmpNode = new CTNode();
	tmpNode->set_solution(node.get_solution(), allVertex, allPaths);
	tmpNode->set_dtp();
	tmpNode->cost = node.cost;
	tmpNode->totalTime = node.totalTime;
	resCTN = tmpNode;
	resultPath = resCTN->get_solution();
	allCTNodes.push_back(tmpNode);
}

vector <Path*>& HighLevelCBS::RunCBS()
{
	CTNode* root = new CTNode();
	vector <Path*> allAgentPath;
	FindPathsForAllAgents(*root, allAgentPath);                         //找到初始化路径
	root->set_solution(allAgentPath,allVertex,allPaths);
	root->cost = GetSIC(root->get_solution() , root->totalTime);		//初始化cost     //先都5s   以这个时间为指标的
	root->set_dtp();
	vector<detectTimePeriod> dtp = root->get_dtp();
	openListHigh.push(root);
	allCTNodes.push_back(root);
	for (int i = 0; i < allAgentPath.size(); i++) {
		auto tmp = new Path(allAgentPath[i]->agentIndex);
		for (int j = 0; j < allAgentPath[i]->Nodes.size(); j++) {
			tmp->Nodes.push_back(allAgentPath[i]->Nodes[j]);
		}
		allPaths.push_back(tmp);
	}
	while (!openListHigh.empty())
	{
		auto P = openListHigh.top();
		openListHigh.pop();
		EqualTimes++;
		SearchTimes++;
		if (P->cost > maxCost) {
			maxCost = P->cost;
			totalTimeOfmaxCost = P->totalTime;
			EqualTimes = 0;
			cout << "  maxCost:  " << maxCost << " time: " << totalTimeOfmaxCost << endl;
			cout << "-------------------------------------" << endl;
			getRes(*P);
		}  
		bool valid = ValidatePathsInNode(*P);	                        //查看路径是否不包含碰撞
		if ( valid )	                                                //如果没有碰撞，返回路径
		{
			cout << "最高得分：" << maxCost << " 对应探测覆盖时间  " << totalTimeOfmaxCost << endl;

			return resultPath;
		}
		vector<Conflict*> conflict = P->get_conflict();
	    for( int i = 0 ; i < conflict.size() ; i++ )
		{
			allConficts.push_back(conflict[i]);

		    CTNode* new_ct_node = new CTNode();		//新的结点
			Constraint* newConstraint = new Constraint(conflict[i]->Agents[0], conflict[i]->Vertex, conflict[i]->TimeStep);
			allConstraints.push_back(newConstraint);
			new_ct_node->set_constraint(newConstraint);//加入限制条件
		    new_ct_node->set_solution(P->get_solution(),allVertex,allPaths);														
			bool path_found = UpdateSolutionByInvokingLowLevel(*new_ct_node);       //每次只将一个agents的路径更新
			allCTNodes.push_back(new_ct_node);
			if (path_found)
		    {
		    	new_ct_node->cost = GetSIC(new_ct_node->get_solution(), new_ct_node->totalTime);                                  
				new_ct_node->set_dtp();
		    	if (new_ct_node->cost < DBL_MAX)
		    	{
		    		openListHigh.push(new_ct_node);
		    	}
				else {
					cout << "err: " << endl;
				}
		    }
		    else {
		    	cout << " path_found   none " << endl;
		    }
		}
	}

}
double HighLevelCBS::GetSIC(const vector <Path*> &solution, double& totalTime)
{
	Point mine, enemy;
	double time1, time2;
	vector<vector<double>> Time;
	for (int i = 0; i < solution.size(); i++)
	{
		vector<double> timee;
	    time1 = solution[i]->Nodes[solution[i]->Nodes.size() - 2]->totalTime;
		mine.x = solution[i]->Nodes[solution[i]->Nodes.size() - 2]->x;
		mine.y = solution[i]->Nodes[solution[i]->Nodes.size() - 2]->y;
		mine.v = solution[i]->Nodes[solution[i]->Nodes.size() - 2]->v;
		mine.azimuth = solution[i]->Nodes[solution[i]->Nodes.size() - 2]->heading;
		enemy.x = solution[i]->Nodes[solution[i]->Nodes.size() - 1]->x;
		enemy.y = solution[i]->Nodes[solution[i]->Nodes.size() - 1]->y;
		enemy.azimuth = solution[i]->Nodes[solution[i]->Nodes.size() - 1]->heading;
		enemy.v = solution[i]->Nodes[solution[i]->Nodes.size() - 1]->v;
		time2 = time1 + encapsulationTime(mine, enemy);
		timee.push_back(time1);
		timee.push_back(time2);
		Time.push_back(timee);
	}
	sort(Time.begin(), Time.end(), [](const vector<double>& u, const vector<double>& v) {
		if (isEqualDouble(u[0], v[0]))
			return u[1] < v[1];
		else
		    return u[0] < v[0];
		});//给定一组区间进行排序（区间左端点升序）
	int maxTime = int(Time[Time.size() - 1][1]);
	double score = 0;
	int stateFlag[5] = { 0 };
	for (int i = 5; i < maxTime; i += 5) {
		int detectNum = 0;
		for (int j = 0; j < Time.size(); j++) {
			if (i >= int(Time[j][0]) && i < int(Time[j][1]))                // 把时间取整了   
				detectNum++;
		}
		score += quickScore[detectNum]; 

		stateFlag[detectNum]++;
	}
	totalTime = maxTime;
	return score;
}

void HighLevelCBS::InitialCBS()
{
	_lowLevelSolver.lengthX = 3000;
	_lowLevelSolver.lengthY = 3000;
	double xMin = DBL_MAX;
	double yMin = DBL_MAX;
	double xMax = DBL_MIN;
	double yMax = DBL_MIN;
	for (auto i : _agents)
	{
		xMin = (std::min)(xMin, i->GoalStateX);
		xMin = (std::min)(xMin, i->StartStateX);
		yMin = (std::min)(yMin, i->GoalStateY);
		yMin = (std::min)(yMin, i->StartStateY);
		xMax = (std::max)(xMax, i->GoalStateX);
		xMax = (std::max)(xMax, i->StartStateX);
		yMax = (std::max)(yMax, i->GoalStateY);
		yMax = (std::max)(yMax, i->StartStateY);
	}
	_lowLevelSolver.frameX= ceil((xMax - xMin) / _lowLevelSolver.lengthX + 10);
	_lowLevelSolver.frameY= ceil((yMax - yMin) / _lowLevelSolver.lengthY + 10);
	_lowLevelSolver.centerX = (xMax + xMin) / 2;
	_lowLevelSolver.centerY = (yMax + yMin) / 2;
	for (auto &i : _agents)
	{
		i->xStartIndex= vectorIndex(i->StartStateX, _lowLevelSolver.centerX, _lowLevelSolver.lengthX, _lowLevelSolver.frameX);
		i->yStartIndex= vectorIndex(i->StartStateY, _lowLevelSolver.centerY, _lowLevelSolver.lengthY, _lowLevelSolver.frameY);
		i->angleStartIndex = enterAngleIndex(i->StartStateHeading);
		i->xEndIndex = vectorIndex(i->GoalStateX, _lowLevelSolver.centerX, _lowLevelSolver.lengthX, _lowLevelSolver.frameX);
		i->yEndIndex = vectorIndex(i->GoalStateY, _lowLevelSolver.centerY, _lowLevelSolver.lengthY, _lowLevelSolver.frameY);
		i->angleEndIndex = enterAngleIndex(i->GoalStateHeading);
	}

	for (int i = 0; i < _agents.size(); i++)
	{
		Vertex*pStart = new Vertex(_agents[i]->StartStateX, _agents[i]->StartStateY, _agents[i]->StartStateHeading, _agents[i]->StartStateV);
		Vertex*pEnd = new Vertex(_agents[i]->GoalStateX, _agents[i]->GoalStateY, _agents[i]->GoalStateHeading, _agents[i]->goalV);
		agentStart.push_back(pStart);
		agentEnd.push_back(pEnd);
	}
	_lowLevelSolver._gridHeight = _lowLevelSolver.frameY;
	_lowLevelSolver._gridWidth = _lowLevelSolver.frameX;
	_lowLevelSolver.pArrResize(_lowLevelSolver.frameY, _lowLevelSolver.frameX);
}

void HighLevelCBS::clearCBS() {
	for (int i = 0; i < allConficts.size(); i++) {
		if (allConficts[i] != nullptr)
			delete allConficts[i];
	}
	
	for (int i = 0; i < allConstraints.size(); i++) {
		if (allConstraints[i] != nullptr)
			delete allConstraints[i];
	}
	
	for (int i = 0; i < allPaths.size(); i++) {
		// 释放节点
		for (int j = 0; j < allPaths[i]->Nodes.size(); j++) {
			if (allPaths[i]->Nodes[j] != nullptr) {
				delete allPaths[i]->Nodes[j];
				//allPaths[i]->Nodes[j] = nullptr;
			}
		}
		if (allPaths[i] != nullptr)
			delete allPaths[i];
	}

	for (int i = 0; i < allCTNodes.size(); i++) {
		if(allCTNodes[i] != nullptr)
		delete allCTNodes[i];
	}

	for (int i = 0; i < agentStart.size(); i++) {
		if (agentStart[i] != nullptr)
			delete agentStart[i];
		if (agentEnd[i] != nullptr)
			delete agentEnd[i];
	}


	for (int i = 0; i < _agents.size(); i++) {
		if(_agents[i] != nullptr)
			delete _agents[i];
	}

	cout << endl;
}


