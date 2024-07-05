#include "LowLevel.h"

namespace zc::cbs {
	
void LowLevelCBS::pArrResize() {
	map.resize(grid_height);
	for (int i = 0; i < grid_height; i++) {
		map[i].resize(grid_width);
		for (int j = 0; j < grid_width; j++) {
			map[i][j].resize(37);
		}
	}
}

bool LowLevelCBS::AStar(VertexPtr& start, VertexPtr& goal, PathPtr& path, const std::vector<ConstraintPtr>& constraints) {
	ClearMapAStarValues();
	double dis = zc::detect::FuncCommon::Distance(*start, *goal);
	double g_angle = 0;
	double g_dis = 0;
	start->g = g_angle + g_dis;
	double h_angle = 1;
	double h_dis = dis / dis;
	double h = h_angle + h_dis;
	start->f = start->g + h;
	start->depth = 0;
	map[start->y_index][start->x_index][start->angle_index] = start;
	openList.push(start);

	bool seek = true;
	
	while (!openList.empty() && seek) {
		auto front = openList.top();
		auto action_sample = zc::detect::FuncCommon::speedSelect(front->v);
		front->visited = true;
		openList.pop();

		std::vector<VertexPtr> successors;
		FillNeighboors(front, successors, seek, path, constraints);   

		for (int i = 0; i < successors.size(); i++) {
			double width_index = successors[i]->y_index;
			double length_index = successors[i]->x_index;
			double angle_index = successors[i]->angle_index; 

			if (map[width_index][length_index][angle_index] && !map[width_index][length_index][angle_index]->visited) {
				g_angle = abs(action_sample[i].changeA) / 360.0;													//  0628�޸�  �����Ƕȳ���360 
				g_dis = action_sample[i].dist / dis;															 //������������ܾ���
				auto diff = abs(zc::detect::FuncCommon::point_angle(successors[i]->x, successors[i]->y, goal->x, goal->y) - successors[i]->heading) > 180 ? 
						360 - abs(zc::detect::FuncCommon::point_angle(successors[i]->x, successors[i]->y, goal->x, goal->y) - successors[i]->heading) : 
							abs(zc::detect::FuncCommon::point_angle(successors[i]->x, successors[i]->y, goal->x, goal->y) - successors[i]->heading);
				h_angle = zc::detect::FuncCommon::angle_standard(diff) / 360.0;													 // ��һ��������λ���뺽��ǵĲ�ֵ  
				h_dis = zc::detect::FuncCommon::Distance(*successors[i], *goal) / dis;			// ��ǰ�㵽Ŀ��������������
				auto g = g_angle + g_dis + front->g;
				auto h = h_angle + h_dis;
				auto f = g + h;
				if (f < map[width_index][length_index][angle_index]->f) {
					successors[i]->g = g;
					successors[i]->h = h;
					successors[i]->f = f;
					map[width_index][length_index][angle_index] = successors[i];
					openList.push(successors[i]);
					if (h_angle * 360 < AS_ANGLE_LIMIT && h_dis*dis < AS_DIS_LIMIT) {
						if (constraints.empty()) {
							seek = false;
							goal->parent = successors[i];
							ReconstructPath(goal, path);
							return true;
						} else if (constraints.back() != nullptr) {
							if (successors[i]->total_time > constraints.back()->time_step - 10
									&& successors[i]->total_time < constraints.back()->time_step + 10) {
								seek = false;
								goal->parent = successors[i];
								ReconstructPath(goal, path);
								return true;
							}
						}
					}
				}
			} else if (!map[width_index][length_index][angle_index]) {
				g_angle = abs(action_sample[i].changeA) / 360.0;                                                  //0628�޸�
				g_dis = action_sample[i].dist / dis;
				auto diff = abs(zc::detect::FuncCommon::point_angle(successors[i]->x, successors[i]->y, goal->x, goal->y) - successors[i]->heading) > 180 ? 
						360 - abs(zc::detect::FuncCommon::point_angle(successors[i]->x, successors[i]->y, goal->x, goal->y) - successors[i]->heading) : 
							abs(zc::detect::FuncCommon::point_angle(successors[i]->x, successors[i]->y, goal->x, goal->y) - successors[i]->heading);
				h_angle = zc::detect::FuncCommon::angle_standard(diff) / 360.0;
				h_dis = zc::detect::FuncCommon::Distance(*successors[i], *goal) / dis;
				auto g = g_angle + g_dis + front->g;
				auto h = h_angle + h_dis;
				auto f = g + h;
				map[width_index][length_index][angle_index] = successors[i];
				openList.push(successors[i]);
				if (h_angle * 360 < AS_ANGLE_LIMIT && h_dis*dis < AS_DIS_LIMIT) {
					if (constraints.empty()) {
						seek = false;
						goal->parent = successors[i];
						ReconstructPath(goal, path);
						return true;
					} else if (constraints.back() != nullptr) {
						if (successors[i]->total_time > constraints.back()->time_step - 10
								&& successors[i]->total_time < constraints.back()->time_step + 10) {
							seek = false;
							goal->parent = successors[i];
							ReconstructPath(goal, path);
							return true;
						}
					}
				}
			} else if (map[width_index][length_index][angle_index] && map[width_index][length_index][angle_index]->visited) {
				auto diff = abs(zc::detect::FuncCommon::point_angle(successors[i]->x, successors[i]->y, goal->x, goal->y) - successors[i]->heading) > 180 ? 
						360 - abs(zc::detect::FuncCommon::point_angle(successors[i]->x, successors[i]->y, goal->x, goal->y) - successors[i]->heading) : 
							abs(zc::detect::FuncCommon::point_angle(successors[i]->x, successors[i]->y, goal->x, goal->y) - successors[i]->heading);
				auto h_angle = zc::detect::FuncCommon::angle_standard(diff) / 360.0;
				auto h_dis = zc::detect::FuncCommon::Distance(*successors[i], *goal) / dis;
				if (h_angle * 360 < AS_ANGLE_LIMIT && h_dis*dis < AS_DIS_LIMIT) {
					if (constraints.empty()) {
						seek = false;
						goal->parent = successors[i];
						ReconstructPath(goal, path);
						return true;
					} else if (constraints.back() != nullptr) {
						if (successors[i]->total_time > constraints.back()->time_step - 10
								&& successors[i]->total_time < constraints.back()->time_step + 10) {
							seek = false;
							goal->parent = successors[i];
							ReconstructPath(goal, path);
							return true;
						}
					}
				}
			} else {
				std::cout << "ASTAR ERROR!!!" << std::endl;
			}
		}
	}
	return false;
}

void LowLevelCBS::ClearMapAStarValues() {
	map.clear();
	openList = {};
	pArrResize();
}

void LowLevelCBS::FillNeighboors(VertexPtr& node, std::vector<VertexPtr>& successors, bool seek, const PathPtr& path, const std::vector<ConstraintPtr>& constraints) {
	auto action_sample = zc::detect::FuncCommon::speedSelect(node->v);
	std::vector<double> last_angle;
	bool is_conflict = false;

	for (int i = 0; i < action_sample.size() && seek; i++) {
		if (is_conflict) {
			bool flag = false;
			for (int j = 0; j < last_angle.size(); j++) {
				CBS_EXCUTE_PLUS(action_sample[i].yingfeiA == last_angle[j], flag = true, break);
			}
			CBS_EXCUTE(flag, continue);
		}

		double new_x = node->x + sin((action_sample[i].distA + node->heading) / 180 * 3.1415926) * action_sample[i].dist;
		double new_y = node->y + cos((action_sample[i].distA + node->heading) / 180 * 3.1415926) * action_sample[i].dist;
		double sample_time = action_sample[i].time;
		double step = node->depth + 1;
		double new_v = action_sample[i].changeV;
		auto azi_end = node->heading + action_sample[i].changeA;
		CBS_EXCUTE(azi_end >= 360, azi_end = azi_end - 360);
		CBS_EXCUTE(azi_end < 0, azi_end = azi_end + 360);
		double new_azi = azi_end;

		zc::detect::Point t(path->agent_index, new_x, new_y, 0, new_azi, new_v);    
		VertexPtr child = std::make_shared<Vertex>(new_x, new_y, new_azi, new_v);
		child->x_index = zc::detect::FuncCommon::vectorIndex(child->x, center_x, length_x, frame_x);
		child->y_index = zc::detect::FuncCommon::vectorIndex(child->y, center_y, length_y, frame_y);
		child->angle_index = zc::detect::FuncCommon::enterAngleIndex(child->heading);
		child->time = sample_time;
		child->total_time = node->total_time + sample_time;

		if (t.x < center_x - (int)(frame_x / 2) * length_x || t.x > center_x + (int)(frame_x / 2) * length_x 
			|| t.y < center_y - (int)(frame_y / 2) * length_y || t.y > center_y + (int)(frame_y / 2) * length_y /*|| 
			HasConflict(child, child->total_time, constraints)*/) {
			last_angle.push_back(action_sample[i].yingfeiA);                       //last_angle?
			is_conflict = true;
			continue;
		}

		child->depth = node->depth + 1;
		child->visited = false;
		child->parent = node;
		successors.push_back(child);
	}
}

void LowLevelCBS::ReconstructPath(VertexPtr& node , PathPtr& path) {
	path->nodes.clear();
	std::vector<VertexPtr> path_reverse;

	while (node != NULL) {
		path_reverse.push_back(node);
		node = node->parent;
	}
	
	path->nodes.reserve(path_reverse.size());
	for(auto it = path_reverse.rbegin(); it != path_reverse.rend(); ++it) {
    	path->nodes.emplace_back(*it);
		// VertexPtr temp_node = std::make_shared<Vertex>(it);
		// path->nodes.emplace_back(temp_node);
	}	
}

bool LowLevelCBS::HasConflict(VertexPtr& child, double time, const std::vector<ConstraintPtr>& constraints) {
	for (const auto& constraint : constraints) {
		CBS_EXCUTE((time == (constraint->time_step)) && (zc::detect::NearEq(child->x , constraint->vertex->x, 1e-8)) && (zc::detect::NearEq(child->y, constraint->vertex->y, 1e-8)),
				return true);
	}
	return false;
}


} // namespace zc::cbs
