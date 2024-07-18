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

bool LowLevelCBS::GetPathOfAStar(bool& seek, VertexPtr& goal, VertexPtr& detect_point, PathPtr& path) {
	seek = false;
	goal->parent = detect_point;
	ReconstructPath(goal, path);
	return true;
}

bool LowLevelCBS::AStar(VertexPtr& start, VertexPtr& goal, PathPtr& path, const std::vector<ConstraintPtr>& constraints) {
	ClearMapAStarValues();
	double dis = zc::detect::FuncCommon::Distance(*start, *goal);
	double g_angle = 0;
	double g_dis = 0;
	start->g = g_angle + g_dis;
	double h_angle = 1;
	double h_dis = dis / dis;
	start->h  = h_angle + h_dis;
	start->f = start->g + start->h;
	start->depth = 0;
	map[start->y_index][start->x_index][start->angle_index] = start;
	open_list.push(start);

	bool seek = true;
	
	while (!open_list.empty() && seek) {
		auto front = open_list.top();
		auto action_sample = zc::detect::FuncCommon::SpeedSelect(front->v);
		front->visited = true;
		open_list.pop();

		std::vector<VertexPtr> successors;
		FillNeighboors(front, successors, seek, path, constraints);   

		for (int i = 0; i < successors.size(); i++) {
			double width_index = successors[i]->y_index;
			double length_index = successors[i]->x_index;
			double angle_index = successors[i]->angle_index; 

			if (map[width_index][length_index][angle_index] && !map[width_index][length_index][angle_index]->visited) {
				g_angle = abs(action_sample[i].changeA) / 360.0;												
				g_dis = action_sample[i].dist / dis;														
				auto diff = abs(zc::detect::FuncCommon::PointAngle(successors[i]->x, successors[i]->y, goal->x, goal->y) - successors[i]->heading) > 180 ? 
						360 - abs(zc::detect::FuncCommon::PointAngle(successors[i]->x, successors[i]->y, goal->x, goal->y) - successors[i]->heading) : 
							abs(zc::detect::FuncCommon::PointAngle(successors[i]->x, successors[i]->y, goal->x, goal->y) - successors[i]->heading);
				h_angle = zc::detect::FuncCommon::AngleStandard(diff) / 360.0;													
				h_dis = zc::detect::FuncCommon::Distance(*successors[i], *goal) / dis;			
				auto g = g_angle + g_dis + front->g;
				auto h = h_angle + h_dis;
				auto f = g + h;
				if (f < map[width_index][length_index][angle_index]->f) {
					successors[i]->g = g;
					successors[i]->h = h;
					successors[i]->f = f;
					map[width_index][length_index][angle_index] = successors[i];
					open_list.push(successors[i]);

					CBS_EXCUTE(h_angle * 360 < AS_ANGLE_LIMIT && h_dis*dis < AS_DIS_LIMIT, 
						return GetPathOfAStar(seek, goal, successors[i], path)); 	
				}
			} else if (!map[width_index][length_index][angle_index]) {
				g_angle = abs(action_sample[i].changeA) / 360.0;                                                
				g_dis = action_sample[i].dist / dis;
				auto diff = abs(zc::detect::FuncCommon::PointAngle(successors[i]->x, successors[i]->y, goal->x, goal->y) - successors[i]->heading) > 180 ? 
						360 - abs(zc::detect::FuncCommon::PointAngle(successors[i]->x, successors[i]->y, goal->x, goal->y) - successors[i]->heading) : 
							abs(zc::detect::FuncCommon::PointAngle(successors[i]->x, successors[i]->y, goal->x, goal->y) - successors[i]->heading);
				h_angle = zc::detect::FuncCommon::AngleStandard(diff) / 360.0;
				h_dis = zc::detect::FuncCommon::Distance(*successors[i], *goal) / dis;
				auto g = g_angle + g_dis + front->g;
				auto h = h_angle + h_dis;
				auto f = g + h;
				map[width_index][length_index][angle_index] = successors[i];
				open_list.push(successors[i]);

				CBS_EXCUTE(h_angle * 360 < AS_ANGLE_LIMIT && h_dis*dis < AS_DIS_LIMIT, 
					return GetPathOfAStar(seek, goal, successors[i], path)); 

			} else if (map[width_index][length_index][angle_index] && map[width_index][length_index][angle_index]->visited) {
				auto diff = abs(zc::detect::FuncCommon::PointAngle(successors[i]->x, successors[i]->y, goal->x, goal->y) - successors[i]->heading) > 180 ? 
						360 - abs(zc::detect::FuncCommon::PointAngle(successors[i]->x, successors[i]->y, goal->x, goal->y) - successors[i]->heading) : 
							abs(zc::detect::FuncCommon::PointAngle(successors[i]->x, successors[i]->y, goal->x, goal->y) - successors[i]->heading);
				auto h_angle = zc::detect::FuncCommon::AngleStandard(diff) / 360.0;
				auto h_dis = zc::detect::FuncCommon::Distance(*successors[i], *goal) / dis;

				CBS_EXCUTE(h_angle * 360 < AS_ANGLE_LIMIT && h_dis*dis < AS_DIS_LIMIT, 
					return GetPathOfAStar(seek, goal, successors[i], path)); 

			} else {
				std::cout << "ASTAR ERROR!!!" << std::endl;
			}
		}
	}
	return false;
}

bool LowLevelCBS:: ExtendTime(VertexPtr& start, std::vector<VertexPtr>& end_two, PathPtr& path_extend, double extend_time) {
	VertexPtr end = end_two[0];
	bool is_path = false;
	end->parent = NULL;
	center_x = (start->x + end->x) / 2;
	center_y = (start->y + end->y) / 2;
	double x_length = abs(start->x - end->x);
	double y_length = abs(start->y - end->y);
	int grid_width = ceil(x_length / length_x + 15);
	int grid_height = ceil(y_length / length_y + 15);
	end->heading = zc::detect::FuncCommon::AngleStandard(end->heading);
	ClearMapAStarValues();
	//double min_time = path_extend->nodes[0]->totalTime;
	double min_time = path_extend->nodes[path_extend->nodes.size() - 2]->total_time;
	double rate = extend_time / min_time;
	//if (rate < 1) {
	//	cout << "ÌáÇ°µœŽïÎÞ·š±£Ö€£¡" << endl;
	//	return false;
	//}

	double g = 0;
	double diff = abs(end->heading - start->heading) > 180 ? 360 - abs(end->heading - start->heading) : abs(end->heading - start->heading);
	double h_angle = zc::detect::FuncCommon::AngleStandard(diff) / 360.0;
	double h_dis;
	auto first_time = min_time * rate;    

	bool seek = true;
	zc::detect::Point endd(end->x, end->y, end->z, (-end->heading + 90) / 57.3, end->v);
	double q1[] = { endd.x,endd.y,endd.azimuth };
	zc::detect::Point startt(start->x, start->y, start->z, (-start->heading + 90) / 57.3, start->v);
	dubins::DubinsPath path;
	r0 = zc::detect::FuncCommon::SelectR1(start->v);
	double q0[] = { startt.x,startt.y,startt.azimuth };
	dubins::dubins_shortest_path(&path, q0, q1, r0);
	//double shortestPath = path.param[0] + path.param[1] + path.param[2];
	//h = (shortestPath * r0 / mine.v);
	double dis = (path.param[0] + path.param[1] + path.param[2]) * r0 * rate;
	double h = dis / start->v;
	double f = h;
	dis = zc::detect::FuncCommon::Distance(start->x, start->y, end->x, end->y);
	VertexPtr pt = std::make_shared<Vertex>(f, g, h, start->x, start->y, start->z, start->heading, start->v, start->x_index, start->y_index, start->angle_index);
	pt->step = 0;
	pt->visited = 1;
	auto index = zc::detect::FuncCommon::EnterAngleIndex(zc::detect::FuncCommon::AngleStandard(start->heading));
	map[zc::detect::FuncCommon::VectorIndex(start->y, center_y, length_y, frame_y)][zc::detect::FuncCommon::VectorIndex(start->x, center_x, length_x, frame_x)][index] = pt;
	open_list.push(pt);

	VertexPtr front = NULL;
	int width_index, length_index, angle_index;
	double shortestPath;
	while (seek && !open_list.empty()) {
		front = open_list.top();
		open_list.pop();
		width_index = zc::detect::FuncCommon::VectorIndex(front->y, center_y, length_y, frame_y);
		length_index = zc::detect::FuncCommon::VectorIndex(front->x, center_x, length_x, frame_x);
		angle_index = zc::detect::FuncCommon::EnterAngleIndex(front->heading);
		front->visited = true;
		auto action_sample = zc::detect::FuncCommon::SpeedSelect(front->v);
		for (int i = 0; i < action_sample.size() && seek; i++) {
			double new_x = front->x + sin((action_sample[i].distA + front->heading) / 180 * 3.1415926) * action_sample[i].dist;
			double new_y = front->y + cos((action_sample[i].distA + front->heading) / 180 * 3.1415926) * action_sample[i].dist;
			double new_z = front->z + action_sample[i].changeH;
			double sample_time = action_sample[i].time;
			double step = front->step + 1;
			double new_v = action_sample[i].changeV;
			auto azi_end = front->heading + action_sample[i].changeA;
			double new_azi = zc::detect::FuncCommon::AngleStandard(azi_end);
			zc::detect::Point t(new_x, new_y, new_z, new_azi, 0, new_v);    
			width_index = zc::detect::FuncCommon::VectorIndex(t.y, center_y, length_y, frame_y);
			length_index = zc::detect::FuncCommon::VectorIndex(t.x, center_x, length_x, frame_x);
			angle_index = zc::detect::FuncCommon::EnterAngleIndex(azi_end);
			CBS_EXCUTE(t.x < center_x - (int)(frame_x / 2) * length_x || t.x > center_x + (int)(frame_x / 2) * length_x || 
					t.y < center_y - (int)(frame_y / 2) * length_y || t.y > center_y + (int)(frame_y / 2) * length_y, continue);
			diff = abs(end->heading - t.azimuth) > 180 ? 360 - abs(end->heading - t.azimuth) : abs(end->heading - t.azimuth);
			h_angle = zc::detect::FuncCommon::AngleStandard(diff) / 360.0;
			h_dis = zc::detect::FuncCommon::Distance(t.x, t.y, end->x, end->y) / dis;
			//g = action_sample[i].time + front->totalTime;
			g = action_sample[i].time + front->g;


			if (h_dis < 0.2 && h_angle * 360 < 5 && g > first_time * 1 && g < first_time * 1.2) {
				seek = false;
				VertexPtr new_point = std::make_shared<Vertex>(f, g, h, t.x, t.y, t.z, t.azimuth, t.v, length_index, width_index, angle_index);    
				new_point->time = sample_time;
				new_point->total_time = front->total_time + sample_time;
				new_point->parent = front;
				new_point->step = step;
				end->parent = new_point;
				is_path = true;
			}

			if (map[width_index][length_index][angle_index] && !map[width_index][length_index][angle_index]->visited) {

				auto tempM = (-t.azimuth + 90) / 57.3;

				zc::detect::Point mine(t.x, t.y, t.z, tempM, t.v);

				double q0[] = { mine.x,mine.y,mine.azimuth };
				dubins::DubinsPath path;
				r0 = zc::detect::FuncCommon::SelectR1(mine.v);
				dubins::dubins_shortest_path(&path, q0, q1, r0);  
				shortestPath = path.param[0] + path.param[1] + path.param[2];
				h = (shortestPath * r0 / 270);
				//h = min(h, perdictFlyTime(t, end));

				f = abs(g + h * 1. - first_time);

				if (f < map[width_index][length_index][angle_index]->f) {
					VertexPtr new_point = std::make_shared<Vertex>(f, g, h, t.x, t.y, t.z, t.azimuth, t.v, length_index, width_index, angle_index);   
					new_point->visited = 0;
					new_point->time = sample_time;
					new_point->total_time = front->total_time + sample_time;
					new_point->parent = front;
					new_point->step = step;
					map[width_index][length_index][angle_index] = new_point;
					open_list.push(new_point);
					if (h_dis < 0.2 && h_angle * 360 < 5 && g > first_time * 1 && g < first_time * 1.2) {
						seek = false;
						end->parent = new_point;
						is_path = true;
					}
				}
			} else if (!map[width_index][length_index][angle_index]) {
				diff = abs(end->heading - t.azimuth) > 180 ? 360 - abs(end->heading - t.azimuth) : abs(end->heading - t.azimuth);
				h_angle = zc::detect::FuncCommon::AngleStandard(diff) / 360.0;
				h_dis = zc::detect::FuncCommon::Distance(t.x, t.y, end->x, end->y) / dis;
				g = action_sample[i].time + front->g;

				auto tempM = (-t.azimuth + 90) / 57.3;

				zc::detect::Point mine(t.x, t.y, t.z, tempM, t.v);

				double q0[] = { mine.x,mine.y,mine.azimuth };
				dubins::DubinsPath path;
				r0 = zc::detect::FuncCommon::SelectR1(mine.v);
				dubins_shortest_path(&path, q0, q1, r0); 
				shortestPath = path.param[0] + path.param[1] + path.param[2];
				h = (shortestPath * r0 / 270);
				//h = min(h, perdictFlyTime(t, end));


				f = abs(g + h * 1. - first_time);

				VertexPtr new_point = std::make_shared<Vertex>(f, g, h, t.x, t.y, t.z, t.azimuth, t.v, length_index, width_index, angle_index);
				new_point->visited = 0;
				new_point->time = sample_time;
				new_point->total_time = front->total_time + sample_time;
				new_point->parent = front;
				new_point->step = step;

				map[width_index][length_index][angle_index] = new_point;
				open_list.push(new_point);
				if (h_dis < 0.2 && h_angle * 360 < 5 && g > first_time * 1 && g < first_time * 1.2) {
					seek = false;
					end->parent = new_point;
					is_path = true;
				}
			}
		}
	}
	auto p = end;

	if (p->parent) {
		path_extend->nodes.clear();
		while (true) {
			if (p->time < 5 || p->time > 45)
			{
				CBS_EXCUTE_PLUS (p->parent, p = p->parent, continue);
				break;
			}
			path_extend->nodes.push_back(p);
			p = p->parent;
		}
		path_extend->nodes.push_back(p);
		path_extend->nodes[0] = end_two[1];
		std::reverse(path_extend->nodes.begin(), path_extend->nodes.end());
		path_extend->nodes[0]->total_time = path_extend->nodes[path_extend->nodes.size() - 2]->g;
	} else {
		std::cout << "EXTEND PATH FAILED!!! " << std::endl;
		is_path = false;
	}
	//end_two[1]->Parent = path_extend->nodes[path_extend->nodes.size() - 1];
	end = end_two[1];
	return is_path;
}

void LowLevelCBS::ClearMapAStarValues() {
	map.clear();
	open_list = {};
	pArrResize();
}

void LowLevelCBS::FillNeighboors(VertexPtr& node, std::vector<VertexPtr>& successors, bool seek, const PathPtr& path, const std::vector<ConstraintPtr>& constraints) {
	auto action_sample = zc::detect::FuncCommon::SpeedSelect(node->v);
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
		child->x_index = zc::detect::FuncCommon::VectorIndex(child->x, center_x, length_x, frame_x);
		child->y_index = zc::detect::FuncCommon::VectorIndex(child->y, center_y, length_y, frame_y);
		child->angle_index = zc::detect::FuncCommon::EnterAngleIndex(child->heading);
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
