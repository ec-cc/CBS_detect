#ifndef CBS_H_
#define CBS_H_


#include "CBSDataStructures.h"
#include "LowLevel.h"
#include <algorithm>

namespace zc::cbs {

#define FLOAT_IN_RANGE(input, benchmark)  zc::detect::NearEq((input), (benchmark), 1e-6)

const double detect_threshold_time = 15.0;
	
const int quickScore[5] = { -16 , 8 , 2 , 1 , 0 };     

class CTNode {
private:
	int debug_index;
	double cost;
	double total_time;  
	
	std::vector<ConstraintPtr> constraints;                  
	std::vector<PathPtr> solutions;                           
	std::vector<ConflictPtr> conflicts;                   
	std::vector<detectTimePeriodPtr> dtps;                     

public:
   
	CTNode() {}

	CTNode(const std::shared_ptr<CTNode>& node);                             
	
	const std::vector<PathPtr>& GetSolution() const { 
		return solutions; 
	}

	const std::vector<ConflictPtr>& GetConflict() const { 
		return conflicts; 
	}

	const std::vector<ConstraintPtr>& GetConstraint() const {
		 return constraints; 
	}

	const std::vector<detectTimePeriodPtr>& GetDtp() const { 
		return dtps; 
	}

	const double& GetCost() const {
		return cost;
	}

	const double& GetTotalTime() const {
		return total_time;
	}

	void SetConflict(const ConflictPtr& new_conflict);

	void SetConstraint(const std::vector<ConstraintPtr> old_constraint_list, ConstraintPtr new_constraint);


	void SetSolution(const std::vector<PathPtr> &new_solution);


	void SetSolutionForSingleAgent(AgentPtr& agent);

	// void set_solutionInit(const std::vector<Path*> &new_solution, std::vector<Path*>&allPath);

	void SetSIC();
	
	void SetDtp();

	void ClearConflict() {
		conflicts.clear();
	}

	void ClearConstraints() {
		constraints.clear();
	}

	void ClearDtps() {
		dtps.clear();
	}

	void ClearSolution() {
		solutions.clear();
	}
};
using CTNodePtr = std::shared_ptr<CTNode>;



class HighLevelCBS {
private:

struct comppp {
	bool operator() (const CTNodePtr& a, const CTNodePtr& b) {
		return a->GetCost() < b->GetCost();
	}
};

	int search_times;                                                      
	int equal_times;                                                       
	int max_search_times;                                                   
	int max_equal_times;                                                   
	double handover_detect_rate;                                           
	double co_locate_time_rate;                                          
	double max_cost;                                                   
	double total_time_of_max_cost;                                         

	std::vector<AgentPtr> agents;
	std::vector<VertexPtr> agent_start;
	std::vector<VertexPtr> agent_end;
	std::vector<PathPtr> agent_path;

	CTNodePtr res_CTN;
	LowLevelCBS _lowLevelSolver;
	std::priority_queue<CTNodePtr, std::vector<CTNodePtr>, comppp> openListHigh;       

	void UpdateCBSRes(const CTNodePtr& node);
		
	bool ValidatePathsInNode(CTNodePtr& node);
	
	void FindPathsForAllAgents(CTNodePtr& node);
	
	bool UpdateSolutionByInvokingLowLevel(CTNodePtr& node);
	
	void InitialCBS();

	void RunCBS();

public:

	HighLevelCBS();

	~HighLevelCBS() = default;                                                         
	
	void SetDataOfCBS(double id, double start_x, double start_y, double start_heading, double start_v,
		double goal_x, double goal_y, double goal_heading, double goal_state_v);

	void ProcessOfCBS();
	
	const CTNodePtr& GetResOfCBS() const {
		return res_CTN;
	}
};

} // namespace zc::cbs


#endif //CBS_H_

