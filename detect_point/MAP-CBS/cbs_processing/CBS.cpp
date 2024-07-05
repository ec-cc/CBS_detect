#include "CBS.h"

namespace zc::cbs {

CTNode::CTNode(const std::shared_ptr<CTNode>& node) : debug_index(node->debug_index), cost(node->cost), 
			total_time(node->total_time) {

		conflicts.reserve(node->conflicts.size());
		for (const auto& conflict : node->conflicts) {
			ConflictPtr temp_conflict = std::make_shared<Conflict>(conflict);
			conflicts.emplace_back(temp_conflict);
		}

		constraints.reserve(node->constraints.size());
		for (const auto& constraint : node->constraints) {
			ConstraintPtr temp_constraint = std::make_shared<Constraint>(constraint);
			constraints.emplace_back(temp_constraint);
		}

		solutions.reserve(node->solutions.size());
		for (const auto& solution : node->solutions) {
			PathPtr temp_path = std::make_shared<Path>(solution);
			solutions.emplace_back(temp_path);
		}

		dtps.reserve(node->dtps.size());
		for (const auto& dtp : node->dtps) {
			detectTimePeriodPtr temp_dtp = std::make_shared<detectTimePeriod>(dtp);
			dtps.emplace_back(temp_dtp);
		}
}

void CTNode::SetConflict(const ConflictPtr& new_conflict) {
	conflicts.push_back(new_conflict);
}

void CTNode::SetConstraint(const std::vector<ConstraintPtr> old_constraint_list, ConstraintPtr new_constraint) {
	constraints.assign(old_constraint_list.begin(), old_constraint_list.end());
	constraints.push_back(new_constraint);
}

void CTNode::SetSolution(const std::vector<PathPtr> &new_solution) {
	solutions.reserve(new_solution.size());
	for (int i = 0; i < new_solution.size(); ++i) {
		auto temp_path_ptr = std::make_shared<Path>(new_solution[i]);
		solutions.emplace_back(temp_path_ptr);
	}
}

void CTNode::SetSolutionForSingleAgent(AgentPtr& agent) {
	agent->path->agent_index = agent->index;
	solutions[agent->index] = std::make_shared<Path>(agent->path);
}


void CTNode::SetDtp() {
	CBS_EXCUTE_PLUS(solutions.empty(), std::cout << "NO SOLUTIONS!" << std::endl, return);

	dtps.reserve(solutions.size());

	for (const auto& solution : solutions) {

		CBS_EXCUTE_PLUS(solution == nullptr, std::cout << "SOLUTION" << solution->agent_index << "NULL!" << std::endl, continue);
		CBS_EXCUTE_PLUS(solution->nodes.empty(), std::cout << "SOLUTION" << solution->agent_index << "NO NODES!" << std::endl, continue);
		
		dtps.emplace_back(std::make_shared<detectTimePeriod>());
		auto dtp_temp = dtps.back();

		zc::detect::Point mine, enemy;
		mine.x = solution->nodes[solution->nodes.size() - 2]->x;
		mine.y = solution->nodes[solution->nodes.size() - 2]->y;
		mine.v = solution->nodes[solution->nodes.size() - 2]->v;
		mine.azimuth = solution->nodes[solution->nodes.size() - 2]->heading;

		enemy.x = solution->nodes.back()->x;
		enemy.y = solution->nodes.back()->y;
		enemy.v = solution->nodes.back()->v;
		enemy.azimuth = solution->nodes.back()->heading;

		dtp_temp->agent_index = solution->agent_index;
		dtp_temp->t_start = solution->nodes[solution->nodes.size() - 2]->total_time;
		dtp_temp->t_end = dtp_temp->t_start + zc::detect::FuncCommon::encapsulationTime(mine, enemy);
		dtp_temp->vertexSTgt = solution->nodes[solution->nodes.size() - 2];
	}
}

void CTNode::SetSIC() {
	CBS_EXCUTE_PLUS(dtps.empty(), std::cout << "NO DTPS!" << std::endl, return);

	std::vector<std::vector<double>> time_dtp;
	time_dtp.reserve(dtps.size());

	for (const auto& dtp : dtps) {

		CBS_EXCUTE_PLUS(dtp == nullptr, std::cout << "DTP" << dtp->agent_index << "NULL!" << std::endl, continue)

		std::vector<double> temp_time;
		temp_time.reserve(2);
		temp_time.emplace_back(dtp->t_start);
		temp_time.emplace_back(dtp->t_end);
		time_dtp.emplace_back(temp_time);
	}
	sort(time_dtp.begin(), time_dtp.end(), [](const std::vector<double>& u, const std::vector<double>& v) {
		return FLOAT_IN_RANGE(u[0], v[0]) ? u[1] < v[1] : u[0] < v[0]; });

	int max_time = int(time_dtp.back().back());
	double score = 0;
	int state_flag[5] = { 0 };
	for (int i = 5; i < max_time; i += 5) {
		int detect_num = 0;
		for (int j = 0; j < time_dtp.size(); j++) {
			CBS_EXCUTE(i >= int(time_dtp[j][0]) && i < int(time_dtp[j][1]), detect_num++);
		}
		score += quickScore[detect_num]; 
		state_flag[detect_num]++;
	}
	total_time = max_time;
	cost = score;
}	


HighLevelCBS::HighLevelCBS() {
	handover_detect_rate = 0.3;
	co_locate_time_rate = 0.5;
	max_cost = INT32_MIN;
	total_time_of_max_cost = 0;
	search_times = 0;
	equal_times = 0;
	max_search_times = 10000;
	max_equal_times = 1000;
}

void HighLevelCBS::ProcessOfCBS() {
	InitialCBS();
	RunCBS();
}

void HighLevelCBS::SetDataOfCBS(double id, double start_x, double start_y, double start_heading, double start_v,
		double goal_x, double goal_y, double goal_heading, double goal_v) {
	AgentPtr agent = std::make_shared<Agent>(id, start_x, start_y, start_heading, start_v, 
		goal_x, goal_y, goal_heading, goal_v);
	agents.emplace_back(agent);
}

bool HighLevelCBS::ValidatePathsInNode(CTNodePtr& node) {
	bool valid_solution = true;
	double last_time_step = 0;
	auto solution = node->GetSolution();

	CBS_EXCUTE(solution.empty(), return false);

	auto dtp = node->GetDtp();
	sort(dtp.begin(), dtp.end(), [](const detectTimePeriodPtr& a , const detectTimePeriodPtr& b) {
			return FLOAT_IN_RANGE(a->t_start, b->t_start) ? a->t_end < b->t_end : a->t_start < b->t_start;
	});

	for (int j = 0; j < dtp.size(); ++j) {
		CBS_EXCUTE_PLUS(dtp[j] == nullptr, std::cout << "dtp: " << j << "NULL!" << std::endl, continue);
		
		double handover_time = (dtp[j]->t_end - dtp[j]->t_start) * handover_detect_rate;
		CBS_EXCUTE(handover_time < detect_threshold_time, handover_time = detect_threshold_time); 
		
		for (int k = j + 1; k < dtp.size(); ++k) {
			CBS_EXCUTE_PLUS(dtp[k] == nullptr, std::cout << "dtp: " << k << "NULL!" << std::endl, continue);
			
			CBS_EXCUTE_PLUS(FLOAT_IN_RANGE(dtp[j]->t_start, dtp[k]->t_start), valid_solution = false, break);
			if (!FLOAT_IN_RANGE(dtp[j]->t_start, dtp[k]->t_start) && dtp[k]->t_start < dtp[j]->t_end) {
				CBS_EXCUTE_PLUS(dtp[j]->t_end - dtp[k]->t_start > handover_time, valid_solution = false, break);
			} else {
				std::cout << "CANT CLEAR CONFLICT!" << std::endl;
			}
		}
		CBS_EXCUTE_PLUS(!valid_solution,
			node->SetConflict(std::make_shared<Conflict>(agents[dtp[j + 1]->agent_index], agents[dtp[j]->agent_index], dtp[j + 1]->vertexSTgt, dtp[j]->t_end)),
			return valid_solution);
	}

	return valid_solution;
}


void HighLevelCBS::FindPathsForAllAgents(CTNodePtr& node) {
	for (int i = 0; i < agents.size(); ++i) {

		auto start = agent_start[i];
		start->x_index = agents[i]->x_start_index;
		start->y_index = agents[i]->y_start_index;
		start->angle_index = agents[i]->angle_start_index;

		auto goal = agent_end[i];
		goal->x_index = agents[i]->x_end_index;
		goal->y_index = agents[i]->y_end_index;

		_lowLevelSolver.AStar(start, goal, agents[i]->path, node->GetConstraint());
		agents[i]->path->agent_index = i;
		agent_path.emplace_back(agents[i]->path);
	}
}

bool HighLevelCBS::UpdateSolutionByInvokingLowLevel(CTNodePtr& node) {
	int agent_index = node->GetConstraint().front()->agent->index;

	VertexPtr start = agent_start[agent_index];
	start->x_index = agents[agent_index]->x_start_index;
	start->y_index = agents[agent_index]->y_start_index;

	VertexPtr goal = agent_end[agent_index];
	goal->x_index = agents[agent_index]->x_end_index;
	goal->y_index = agents[agent_index]->y_end_index;

	CBS_EXCUTE_PLUS(_lowLevelSolver.AStar(start, goal, agents[agent_index]->path, node->GetConstraint())
			, node->SetSolutionForSingleAgent(agents[agent_index]), return true);
	std::cout << " can't get path" << std::endl;
	return false;
}

void HighLevelCBS::UpdateCBSRes(const CTNodePtr& node) {
	equal_times = 0;
	max_cost = node->GetCost();
	total_time_of_max_cost = node->GetTotalTime();
	std::cout << "  max_cost:  " << max_cost << " time: " << total_time_of_max_cost << std::endl;
	std::cout << "-------------------------------------" << std::endl;
	
	res_CTN = std::make_shared<CTNode>(node);

	// result_CTN_path.reserve(node->GetSolution().size());
	// for (const auto& path : node->GetSolution()) {
	// 	PathPtr temp_path = std::make_shared<Path>(path);
	// 	result_CTN_path.emplace_back(temp_path);
	// }
}

void HighLevelCBS::RunCBS() {
	CTNodePtr root = std::make_shared<CTNode>();
	FindPathsForAllAgents(root);                     
	root->SetSolution(agent_path);
	root->SetDtp();
	root->SetSIC();		

	auto dtp = root->GetDtp();
	openListHigh.push(root);

	while (!openListHigh.empty()) {
		auto P = openListHigh.top();
		openListHigh.pop();
		equal_times++;
		search_times++;

		CBS_EXCUTE(P->GetCost() > max_cost, UpdateCBSRes(P));

		bool valid = ValidatePathsInNode(P);	
		CBS_EXCUTE_PLUS(valid, std::cout << "NO CONFLICT!" << std::endl, return);

		auto conflict = P->GetConflict();
	    for (int i = 0 ; i < conflict.size() ; ++i) {
		    CTNodePtr new_ct_node = std::make_shared<CTNode>();		
			ConstraintPtr new_constraint = std::make_shared<Constraint>(conflict[i]->agents[0], conflict[i]->vertex, conflict[i]->time_step);
			new_ct_node->SetConstraint(P->GetConstraint(), new_constraint);
		    new_ct_node->SetSolution(P->GetSolution());														
			bool path_found = UpdateSolutionByInvokingLowLevel(new_ct_node);      
			if (path_found) {
				new_ct_node->SetDtp();
		    	new_ct_node->SetSIC();                                  
		    	CBS_EXCUTE(new_ct_node->GetCost() < __DBL_MAX__, openListHigh.push(new_ct_node));
		    } else {
		    	std::cout << " PATH FOUND NONE!" << std::endl;
		    }
		}
	}

	std::cout << "OPENLIST EMPTY!" << std::endl;
	return;
}

void HighLevelCBS::InitialCBS() {
	_lowLevelSolver.SetLengthX(3000);
	_lowLevelSolver.SetLengthY(3000);
	double xMin = __DBL_MAX__;
	double yMin = __DBL_MAX__;
	double xMax = __DBL_MIN__;
	double yMax = __DBL_MIN__;

	for (auto agent : agents) {
		xMin = (std::min)(xMin, agent->goal_state_x);
		xMin = (std::min)(xMin, agent->start_state_x);
		yMin = (std::min)(yMin, agent->goal_state_y);
		yMin = (std::min)(yMin, agent->start_state_y);
		xMax = (std::max)(xMax, agent->goal_state_x);
		xMax = (std::max)(xMax, agent->start_state_x);
		yMax = (std::max)(yMax, agent->goal_state_y);
		yMax = (std::max)(yMax, agent->start_state_y);
	}

	_lowLevelSolver.SetFrameX(ceil((xMax - xMin) / _lowLevelSolver.GetLengthX() + 10));
	_lowLevelSolver.SetFrameY(ceil((yMax - yMin) / _lowLevelSolver.GetLengthY() + 10));
	_lowLevelSolver.SetCentreX((xMax + xMin) / 2);
	_lowLevelSolver.SetCentreY((yMax + yMin) / 2);

	for (auto &agent : agents) {
		agent->x_start_index= zc::detect::FuncCommon::vectorIndex(agent->start_state_x, _lowLevelSolver.GetCentreX(), _lowLevelSolver.GetLengthX(), _lowLevelSolver.GetFrameX());
		agent->y_start_index= zc::detect::FuncCommon::vectorIndex(agent->start_state_y, _lowLevelSolver.GetCentreY(), _lowLevelSolver.GetLengthY(), _lowLevelSolver.GetFrameY());
		agent->angle_start_index = zc::detect::FuncCommon::enterAngleIndex(agent->start_state_heading);
		agent->x_end_index = zc::detect::FuncCommon::vectorIndex(agent->goal_state_x, _lowLevelSolver.GetCentreX(), _lowLevelSolver.GetLengthX(), _lowLevelSolver.GetFrameX());
		agent->y_end_index = zc::detect::FuncCommon::vectorIndex(agent->goal_state_y, _lowLevelSolver.GetCentreY(), _lowLevelSolver.GetLengthY(), _lowLevelSolver.GetFrameY());
		agent->angle_end_index = zc::detect::FuncCommon::enterAngleIndex(agent->goal_state_heading);
	}

	for (int i = 0; i < agents.size(); i++) {
		VertexPtr pStart = std::make_shared<Vertex>(agents[i]->start_state_x, agents[i]->start_state_y, agents[i]->start_state_heading, agents[i]->start_state_v);
		VertexPtr pEnd = std::make_shared<Vertex>(agents[i]->goal_state_x, agents[i]->goal_state_y, agents[i]->goal_state_heading, agents[i]->goal_v);
		agent_start.emplace_back(pStart);
		agent_end.emplace_back(pEnd);
	}

	_lowLevelSolver.SetGridWidth(_lowLevelSolver.GetFrameX());
	_lowLevelSolver.SetGridHeight(_lowLevelSolver.GetFrameY());
	// _lowLevelSolver.pArrResize(_lowLevelSolver.GetFrameY(), _lowLevelSolver.GetFrameX());
}



} // namespace zc::cbs



