#include "CBSDataStructures.h"

namespace zc::cbs {


// bool CBSCommonFunc::getDTPSolution(const std::vector<detectTimePeriodPtr>& dtp, std::vector<PathPtr> solutions) {
// 	std::vector<VertexPtr> m_vertex;
//     CBS_EXCUTE(solutions.empty(), return false);


// 	for (int i = 0; i < solutions.size(); ++i) {
// 		m_vertex.emplace_back(solutions[i]->nodes[solutions[i]->nodes.size() - 1]);
// 		solutions[i]->nodes.pop_back();
// 	}


// 	int max_pre_time = dtp[dtp.size() - 1]->t_end;
// 	for (int i = 0; i < dtp.size(); ++i) {
// 		int index = dtp[i]->agent_index;
// 		auto detect_vetex = solutions[index]->nodes[solutions[index]->nodes.size() - 1];
// 		zc::detect::Point mP(detect_vetex->x, detect_vetex->y, detect_vetex->z, detect_vetex->heading, detect_vetex->v);
		
// 		auto extendP = zc::detect::FuncCommon::Exline(mP, max_pre_time - dtp[i]->t_start);
// 		VertexPtr child = std::make_shared<Vertex>(extendP.x, extendP.y, extendP.azimuth, extendP.v);
// 		child->parent = detect_vetex;
// 		child->time = max_pre_time - dtp[i]->t_start;
// 		child->total_time = dtp[i]->t_end;
// 		solutions[index]->nodes.push_back(child);
// 	}


// 	for (int i = 0; i < solutions.size(); ++i) {
// 		solutions[i]->nodes.push_back(m_vertex[i]);
// 	}
// 	return true;
// }

double CBSCommonFunc::perdictFlyTime(VertexPtr& now, VertexPtr& future) {
	CBS_EXCUTE(now->x == future->x && now->y == future->y && abs(now->heading - future->heading) < 1, return 0);

	future->heading = zc::detect::FuncCommon::AngleStandard(future->heading);
	double angle1 = zc::detect::FuncCommon::PointAngle(now->x, now->y, future->x, future->y);
	angle1 = zc::detect::FuncCommon::AngleStandard(angle1);
	auto angle2 = abs(now->heading - angle1) > 180 ? (360 - abs(now->heading - angle1)) : abs(now->heading - angle1);
	//auto angleTime1 = angle2 / 1.;
	double angle3 = abs(future->heading - angle1) > 180 ? (360 - abs(future->heading - angle1)) : abs(future->heading - angle1);
	//auto angleTime2 = angle3 / 1.;
	double dis = zc::detect::FuncCommon::Distance(now->x, now->y, future->x, future->y);
	auto dis_time = dis / 230;
	return angle2 + angle3 + dis_time;
}

// std::vector<VertexPtr>& CBSCommonFunc::lineInterpolate(VertexPtr& start, VertexPtr& end, double time) {
// 	std::vector<VertexPtr> result;
// 	double line_dis = zc::detect::FuncCommon::Distance(start->x, start->y, end->x, end->y);
// 	double interpolate_point_num = time / 5 ;
// 	double interpolate_dis = line_dis / interpolate_point_num;
// 	double line_angle = atan2((end->x - start->x), (end->y - start->y));
// 	for (int i = 0; i < interpolate_point_num; i++) {
// 		VertexPtr interpolate_point=std::make_shared<Vertex>((start->x + interpolate_dis * (i + 1) * sin(line_angle)), (start->y + interpolate_dis * (i + 1) * cos(line_angle)));
// 		interpolate_point->total_time = start->total_time + 5 * (i + 1);
// 		result.push_back(interpolate_point);
// 	}
// 	return result;
// }

float CBSCommonFunc::Clip(float n, float lower, float upper) {

}

} // namespace zc::cbs
