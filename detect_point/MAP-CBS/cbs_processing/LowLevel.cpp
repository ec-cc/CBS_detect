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

					CBS_EXCUTE(h_angle * 360 < AS_ANGLE_LIMIT && h_dis*dis < AS_DIS_LIMIT, 
						return GetPathOfAStar(seek, goal, successors[i], path)); 	
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

				CBS_EXCUTE(h_angle * 360 < AS_ANGLE_LIMIT && h_dis*dis < AS_DIS_LIMIT, 
					return GetPathOfAStar(seek, goal, successors[i], path)); 

			} else if (map[width_index][length_index][angle_index] && map[width_index][length_index][angle_index]->visited) {
				auto diff = abs(zc::detect::FuncCommon::point_angle(successors[i]->x, successors[i]->y, goal->x, goal->y) - successors[i]->heading) > 180 ? 
						360 - abs(zc::detect::FuncCommon::point_angle(successors[i]->x, successors[i]->y, goal->x, goal->y) - successors[i]->heading) : 
							abs(zc::detect::FuncCommon::point_angle(successors[i]->x, successors[i]->y, goal->x, goal->y) - successors[i]->heading);
				auto h_angle = zc::detect::FuncCommon::angle_standard(diff) / 360.0;
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

bool LowLevelCBS::ExtendTime(VertexPtr& start, std::vector<VertexPtr>& end_two, PathPtr& path, double extend_time) {
	bool is_path = false;
	auto end = end_two[0];
	end->heading = zc::detect::FuncCommon::angle_standard(end->heading);
	end->parent = NULL;

	center_x = (start->x + end->x) / 2;
	center_y = (start->y + end->y) / 2;
	double x_length = abs(start->x - end->x);
	double y_length = abs(start->y - end->y);
	int grid_width = ceil(x_length / length_x + 15);
	int grid_height = ceil(y_length / length_y + 15);
	
	VertexPtr new_front = NULL;
	ClearMapAStarValues();

	//double minTime = pathA.Nodes[0]->totalTime;
	double min_time = path->nodes[path->nodes.size() - 2]->total_time;
	double rate = extend_time / min_time;
	//if (rate < 1) {
	//	cout << "ÌáÇ°µœŽïÎÞ·š±£Ö€£¡" << endl;
	//	return false;
	//}

	double g = 0;
	double diff = abs(end->heading - start->heading) > 180 ? 360 - abs(end->heading - start->heading) : abs(end->heading - start->heading);
	double hAngle = zc::detect::FuncCommon::angle_standard(diff) / 360.0;
	double hDis;
	auto first_time = min_time * rate;    //ÐÞžÄ×î¶ÌÊ±Œä

	bool seek = true;
	zc::detect::Point end_point(end->x, end->y, end->z, (-end->heading + 90) / 57.3, end->v);
	double q1[] = { end_point.x, end_point.y, end_point.azimuth };
	zc::detect::Point start_point(start->x, start->y, start->z, (-start->heading + 90) / 57.3, start->v);
	
	dubins::DubinsPath dubins_path;
	r0 = zc::detect::FuncCommon::SelectR1(start->v);
	double q0[] = { start_point.x,start_point.y,start_point.azimuth };
	dubins::dubins_shortest_path(&dubins_path, q0, q1, r0);  //Æðµã£¬ÖÕµãºÍ×ªÍä°ëŸ¶
	//double shortest_path = path.param[0] + path.param[1] + path.param[2];
	//h = (shortest_path * r0 / mine.v);
	double dis = (dubins_path.param[0] + dubins_path.param[1] + dubins_path.param[2]) * r0 * rate;
	double h = dis / start->v;
	double f = h;
	dis = zc::detect::FuncCommon::Distance(*start, *end);
	VertexPtr pt = std::make_shared<Vertex>(f, g, h, start->x, start->y, start->z, start->heading, 0, start->v);
	pt->step = 0;
	pt->visited = 1;
	auto index = zc::detect::FuncCommon::enterAngleIndex(zc::detect::FuncCommon::angle_standard(start->heading));
	map[zc::detect::FuncCommon::vectorIndex(start->y, center_y, length_y, frame_y)]
		[zc::detect::FuncCommon::vectorIndex(start->x, center_x, length_x, frame_x)][index] = pt;
	openList.push(pt);

	VertexPtr front = NULL;
	int width_index, length_index, angle_index;
	double shortest_path;
	while (seek && !openList.empty()) {
		front = openList.top();
		front->visited = true;
		openList.pop();

		width_index = zc::detect::FuncCommon::vectorIndex(front->y, center_y, length_y, frame_y);
		length_index = zc::detect::FuncCommon::vectorIndex(front->x, center_x, length_x, frame_x);
		angle_index = zc::detect::FuncCommon::enterAngleIndex(front->heading);

		auto action_sample = zc::detect::FuncCommon::speedSelect(front->v);
		for (int i = 0; i < action_sample.size() && seek; i++) {
			double new_x = front->x + sin((action_sample[i].distA + front->heading) / 180 * 3.1415926) * action_sample[i].dist;
			double new_y = front->y + cos((action_sample[i].distA + front->heading) / 180 * 3.1415926) * action_sample[i].dist;
			double new_z = front->z + action_sample[i].changeH;
			double sample_time = action_sample[i].time;
			double new_step = front->step + 1;
			double new_v = action_sample[i].changeV;
			auto azi_end = front->heading + action_sample[i].changeA;
			double new_azi = zc::detect::FuncCommon::angle_standard(azi_end);
			zc::detect::Point t(new_x, new_y, new_z, new_azi, new_v);    //0629ÐÞžÄ
			width_index = zc::detect::FuncCommon::vectorIndex(t.y, center_y, length_y, frame_y);
			length_index = zc::detect::FuncCommon::vectorIndex(t.x, center_x, length_x, frame_x);
			angle_index = zc::detect::FuncCommon::enterAngleIndex(azi_end);

			CBS_EXCUTE(t.x < center_x - (int)(frame_x / 2) * length_x || t.x > center_x + (int)(frame_x / 2) * length_x ||
				t.y < center_y - (int)(frame_y / 2) * length_y || t.y > center_y + (int)(frame_y / 2) * length_y, continue);
			
			diff = abs(end->heading - t.azimuth) > 180 ? 360 - abs(end->heading - t.azimuth) : abs(end->heading - t.azimuth);
			hAngle = zc::detect::FuncCommon::angle_standard(diff) / 360.0;
			hDis = zc::detect::FuncCommon::distance(t.x, t.y, end->x, end->y) / dis;
			//g = actionSample[i].time + front->totalTime;
			g = action_sample[i].time + front->g;


			if (hDis < 0.2 && hAngle * 360 < 5 && g > first_time * 1 && g < first_time * 1.2) {
				seek = false;
				VertexPtr new_vertex = std::make_shared<Vertex>(f, g, h, t.x, t.y, t.z, t.azimuth, t.pitch, t.v);    //0629ÐÞžÄ
				new_vertex->time = sample_time;
				new_vertex->total_time = front->total_time + sample_time;
				new_vertex->parent = front;
				new_vertex->step = new_step;
				end->parent = new_vertex;
				is_path = true;
			}

			if (map[width_index][length_index][angle_index] && !map[width_index][length_index][angle_index]->visited) {
				///************²ÉÓÃdubinsÇúÏßŒÆËãÊ±Œä*********/

				auto temp_azimuth = (-t.azimuth + 90) / 57.3;
				zc::detect::Point mine(t.x, t.y, t.z, temp_azimuth, t.v);

				double q0[] = { mine.x,mine.y,mine.azimuth };
				dubins::DubinsPath path;

				r0 = zc::detect::FuncCommon::SelectR1(mine.v);
				dubins::dubins_shortest_path(&path, q0, q1, r0);  //Æðµã£¬ÖÕµãºÍ×ªÍä°ëŸ¶
				shortest_path = path.param[0] + path.param[1] + path.param[2];
				h = (shortest_path * r0 / 270);
				//h = min(h, perdictFlyTime(t, end));
				/************²ÉÓÃdubinsÇúÏßŒÆËãÊ±Œä*********/


				/********ÑÓ³€Ê±Œä**********/

				f = abs(g + h * 1. - first_time);
				/********ÑÓ³€Ê±Œä**********/

				if (f < map[width_index][length_index][angle_index]->f) {
					VertexPtr new_vertex = std::make_shared<Vertex>(f, g, h, t.x, t.y, t.z, t.azimuth, t.pitch, t.v);    //0629ÐÞžÄ
					new_vertex->visited = 0;
					new_vertex->time = sample_time;
					new_vertex->total_time = front->total_time + sample_time;
					new_vertex->parent = front;
					new_vertex->step = new_step;
					map[width_index][length_index][angle_index] = new_vertex;
					openList.push(new_vertex);

					if (hDis < 0.2 && hAngle * 360 < 5 && g > first_time * 1 && g < first_time * 1.2) {
						seek = false;
						end->parent = new_vertex;
						is_path = true;
					}
				}
			} else if (!map[width_index][length_index][angle_index]) {
				diff = abs(end->heading - t.azimuth) > 180 ? 360 - abs(end->heading - t.azimuth) : abs(end->heading - t.azimuth);
				hAngle = zc::detect::FuncCommon::angle_standard(diff) / 360.0;
				hDis = zc::detect::FuncCommon::distance(t.x, t.y, end->x, end->y) / dis;
				g = action_sample[i].time + front->g;

				/************²ÉÓÃdubinsÇúÏßŒÆËãÊ±Œä*********/
				auto temp_azimuth = (-t.azimuth + 90) / 57.3;

				zc::detect::Point mine(t.x, t.y, t.z, temp_azimuth, t.v);

				double q0[] = { mine.x,mine.y,mine.azimuth };
				dubins::DubinsPath path;
				r0 = zc::detect::FuncCommon::SelectR1(mine.v);
				dubins::dubins_shortest_path(&path, q0, q1, r0);  //Æðµã£¬ÖÕµãºÍ×ªÍä°ëŸ¶
				shortest_path = path.param[0] + path.param[1] + path.param[2];
				h = (shortest_path * r0 / 270);
				//h = min(h, perdictFlyTime(t, end));
				/************²ÉÓÃdubinsÇúÏßŒÆËãÊ±Œä*********/

				/********ÑÓ³€Ê±Œä**********/

				f = abs(g + h * 1. - first_time);
				/********ÑÓ³€Ê±Œä**********/

				VertexPtr new_vertex = std::make_shared<Vertex>(f, g, h, t.x, t.y, t.z, t.azimuth, t.pitch, t.v);
				new_vertex->visited = 0;
				new_vertex->time = sample_time;
				new_vertex->total_time = front->total_time + sample_time;
				new_vertex->parent = front;
				new_vertex->step = new_step;

				map[width_index][length_index][angle_index] = new_vertex;
				openList.push(new_vertex);
				if (hDis < 0.2 && hAngle * 360 < 5 && g > first_time * 1 && g < first_time * 1.2) {
					seek = false;
					end->parent = new_vertex;
					is_path = true;
				}
			}
		}
	}
	auto p = end;

	if (p->parent) {
		path->nodes.clear();
		while (true) {
			if (p->time < 5 || p->time > 45) {
				CBS_EXCUTE_PLUS(p->parent, p = p->parent, continue);
				break;
			}
			path->nodes.push_back(p);
			p = p->parent;
		}
		path->nodes.push_back(p);
		path->nodes[0] = end_two[1];
		std::reverse(path->nodes.begin(), path->nodes.end());
		path->nodes[0]->total_time = path->nodes[path->nodes.size() - 2]->g;
	} else {
		std::cout << "   ¹æ»®ÎÞ¹û£¡ " << std::endl;
		is_path = false;
	}
	//Allend[1]->Parent = pathA.Nodes[pathA.Nodes.size() - 1];
	end = end_two[1];
	return is_path;
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
