#ifndef CBS_DATA_STRUCTRUES_H_
#define CBS_DATA_STRUCTRUES_H_

#include "../cbs_common/func_common.h"

namespace zc::cbs {
	

struct Constraint;
using ConstraintPtr = std::shared_ptr<Constraint>;

struct Vertex {
	Vertex() :
		g(0), h(0), f(0) {
		parent = NULL;
	}

	Vertex(double x, double y, double heading, double v) :
		g(0), h(0), f(0), x(x), y(y), heading(heading), v(v) {
		parent = NULL;
	}

	Vertex(double x, double y, bool obstacle = false) :
		x(x), y(y), g(0), h(0), f(0) {
		parent = NULL;
		obstacle = obstacle;
		depth = 0;
	}

	Vertex(double f, double g, double h, double x, double y, double z, double azimuth, double a, double v) :
		x(x), y(y), g(g), h(h), f(f), heading(azimuth), v(v) {
		parent = NULL;
		obstacle = false;
		depth = 0;
	}

	Vertex(std::shared_ptr<Vertex> node) {
		CBS_EXCUTE_PLUS(node == nullptr, std::cout << "VERTEX COPY CONSTRUCTOR FAILED!" << std::endl, return);
		
		x = node->x; 
		y = node->y; 
		z = node->z; 
		v = node->v; 
		heading = node->heading; 

		g = node->g;
		h = node->h; 
		f = node->f; 

		time = node->time;
		end_time = node->end_time;
		total_time = node->total_time;
		visited = node->visited;
		nov = node->nov;
		depth = node->depth;

		parent = nullptr;
		CBS_EXCUTE(node->parent != nullptr, parent = std::make_shared<Vertex>(node->parent));
}

	inline bool operator == (const Vertex &v) const {
		return v.x_index == this->x_index && v.y_index == this->y_index;
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
	double total_time = 0;
	double end_time = 0; //̽��Ľ���ʱ��
	int step = 0;
	double nov;

	double x_index;
	double y_index;
	double angle_index;

	
	std::shared_ptr<Vertex> parent;
	int depth;

	bool obstacle;
};
using VertexPtr = std::shared_ptr<Vertex>;

struct Path {
	Path() {

	}

	Path(int index) : agent_index(index) {

	}

	Path(std::shared_ptr<Path>&& new_path) {
		CBS_EXCUTE_PLUS(new_path == nullptr, std::cout << "PATH MOVE CONSTRUCTOR FAILED!" << std::endl, return);
		
		agent_index = new_path->agent_index;

		nodes = std::move(new_path->nodes);
		constraints = std::move(new_path->constraints);
	}

	Path(const std::shared_ptr<Path>& path) {
		CBS_EXCUTE_PLUS(path == nullptr, std::cout << "PATH COPY CONSTRUCTOR FAILED!" << std::endl, return);
		
		agent_index = path->agent_index;

		nodes.reserve(path->nodes.size());
		for (int j = 0; j < path->nodes.size(); ++j) {
			CBS_EXCUTE(path->nodes[j] == nullptr, continue);
			nodes.emplace_back(std::make_shared<Vertex>(path->nodes[j]));
		}

		constraints.reserve(constraints.size());
		for (int j = 0; j < path->constraints.size(); ++j) {
			CBS_EXCUTE(path->constraints[j] == nullptr, continue);
			constraints.emplace_back(std::make_shared<Constraint>(path->constraints[j]));
		}
	}

	int agent_index;

	std::vector<VertexPtr> nodes;

	std::vector<ConstraintPtr> constraints;                   // ÿ��·�������Լ���ǣ� 

	int get_cost() { 
		return nodes.size(); 
	}
};
using PathPtr = std::shared_ptr<Path>;

struct Agent {
	Agent(double index1, double start_state_x1, double start_state_y1, double start_state_heading1, double start_state_v1, 
			double goal_state_x1, double goal_state_y1, double goal_state_heading1, double v)
			: index(index1), start_state_x(start_state_x1), start_state_y(start_state_y1), start_state_heading(start_state_heading1),
		start_state_v(start_state_v1), goal_state_x(goal_state_x1), goal_state_y(goal_state_y1), goal_state_heading(goal_state_heading1), goal_v(v) {
		path = std::make_shared<Path>(index1);
	}

	Agent(const std::shared_ptr<Agent>& agent) {
		CBS_EXCUTE_PLUS(agent == nullptr, std::cout << "AGENT COPY CONSTRUCTOR FAILED!" << std::endl, return);
		index = agent->index;

		start_state_x = agent->start_state_x; 
		start_state_y = agent->start_state_y; 
		start_state_v = agent->start_state_v;
		start_state_heading = agent->start_state_heading; 

		goal_state_x = agent->goal_state_x; 
		goal_state_y = agent->goal_state_y; 
		goal_v = agent->goal_v; 
		goal_state_heading = agent->goal_state_heading;

		x_start_index = agent->x_start_index; 
		y_start_index = agent->y_start_index; 
		angle_start_index = agent->angle_start_index; 

		x_end_index = agent->x_end_index; 
		y_end_index = agent->x_end_index; 
		angle_end_index = agent->angle_end_index;

		path = nullptr;
		CBS_EXCUTE(agent->path != nullptr, path = std::make_shared<Path>(agent->path));
	}

	int index;
	
	double start_state_x;
	double start_state_y;
	double start_state_heading;
	double start_state_v;
	double goal_state_x;
	double goal_state_y;
	double goal_state_heading;
	double goal_v;  
	double x_start_index;
	double y_start_index;
	double angle_start_index;
	double x_end_index;
	double y_end_index;
	double angle_end_index;

	PathPtr path;
};
using AgentPtr = std::shared_ptr<Agent>;

struct Constraint {
	Constraint(AgentPtr agent1, VertexPtr vertex1, double time_step1) :
		agent(agent1), vertex(vertex1), time_step(time_step1) {
			
	}

	Constraint(const ConstraintPtr& constraint) {
		CBS_EXCUTE_PLUS(constraint == nullptr, std::cout << "CONSTRAINT COPY CONSTRUCTOR FAILED!" << std::endl, return);

		time_step = constraint->time_step;

		agent = nullptr;
		CBS_EXCUTE(constraint->agent != nullptr, agent = std::make_shared<Agent>(constraint->agent));
		vertex = nullptr;
		CBS_EXCUTE(constraint->vertex != nullptr, vertex = std::make_shared<Vertex>(constraint->vertex));
	}

	double time_step;
	AgentPtr agent;
	VertexPtr vertex;
};

struct Conflict {
	Conflict(AgentPtr agent1, AgentPtr agent2, VertexPtr vertex1, int time_step) :
		vertex(vertex1), time_step(time_step) {
		agents[0] = agent1;
		agents[1] = agent2;
	}

	Conflict(const std::shared_ptr<Conflict>& conflict) { 
		CBS_EXCUTE_PLUS(conflict == nullptr, std::cout << "CONFLICT COPY CONSTRUCTOR FAILED!", return);

		time_step = conflict->time_step;

		for (int i = 0; i < 2; ++i) {
			agents[i] = nullptr;
			CBS_EXCUTE(conflict->agents[i] != nullptr, agents[i] = std::make_shared<Agent>(conflict->agents[i]));
		}

		vertex = nullptr;
		CBS_EXCUTE(conflict->vertex != nullptr, vertex = std::make_shared<Vertex>(conflict->vertex));
	}
	//private:
	AgentPtr agents[2];
	VertexPtr vertex;
	double time_step;
};
using ConflictPtr = std::shared_ptr<Conflict>;

struct detectTimePeriod {
	detectTimePeriod() {}
	
	detectTimePeriod(const std::shared_ptr<detectTimePeriod>& dtp) {
		CBS_EXCUTE_PLUS(dtp == nullptr, std::cout << "DTP COPY CONSTRUCTOR FAILED!", return);

		agent_index = dtp->agent_index; 
		t_start = dtp->t_start;
		t_end = dtp->t_end;

		vertexSTgt = nullptr;
		CBS_EXCUTE(dtp->vertexSTgt != nullptr, vertexSTgt = std::make_shared<Vertex>(dtp->vertexSTgt)) ;
	}

	int agent_index;
	double t_start;
	double t_end;
	VertexPtr vertexSTgt;
};
using detectTimePeriodPtr = std::shared_ptr<detectTimePeriod>;

class CBSCommonFunc {
public:

	static bool getDTPSolution(const std::vector<detectTimePeriodPtr>& dtp, std::vector<PathPtr> solutions);             // �õ�ʱ�����ⷽ��
	
	static double perdictFlyTime(VertexPtr& now, VertexPtr& future);     //����Ԥ�����ʱ��

	static float Clip(float n, float lower, float upper);

	static std::vector<VertexPtr>& lineInterpolate(VertexPtr& start, VertexPtr& end, double time);	//�����߽���5sһ��ֵ���õ�һϵ��Point��
};

} // namespace zc::cbs

#endif //CBS_DATA_STRUCTRUES_H_