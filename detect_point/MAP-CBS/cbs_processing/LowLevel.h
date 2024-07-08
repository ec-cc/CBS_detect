#ifndef LOWLEVEL_H_
#define LOWLEVEL_H_

#include <fstream>
#include <string>
#include <sstream>
#include <queue>
#include "CBSDataStructures.h"
#include "dubins.h"

#define  AS_ANGLE_LIMIT 15
#define  AS_DIS_LIMIT 60000

namespace zc::cbs {

class LowLevelCBS {
private:

struct compppp {
	bool operator() (const VertexPtr& a, const VertexPtr& b) {
		return a->f > b->f; 
	}
};

	double length_x;
	double length_y;
	double frame_x;
	double frame_y;
	double center_x;
	double center_y;
	int grid_width;
	int grid_height;
	double r0 = 7000;		

	std::vector<std::vector<std::vector<VertexPtr>>> map;
	std::priority_queue<VertexPtr, std::vector<VertexPtr>, compppp> openList;
	
	bool GetPathOfAStar(bool& seek, VertexPtr& goal, VertexPtr& detect_point, PathPtr& path);

	void ClearMapAStarValues();
	
	void FillNeighboors(VertexPtr& node, std::vector<VertexPtr>& successors, bool seek, const PathPtr& path, const std::vector<ConstraintPtr>& constraints);
	
	void ReconstructPath(VertexPtr& node, PathPtr& path);
	
	bool HasConflict(VertexPtr& child, double time, const std::vector<ConstraintPtr>& constraints);  

	void pArrResize();

public:

	LowLevelCBS() {}

	bool ExtendTime(VertexPtr& start, std::vector<VertexPtr>& end_two, PathPtr& path, double extend_time);

	bool AStar(VertexPtr& start, VertexPtr& goal, PathPtr& path, const std::vector<ConstraintPtr>& constraints);

	double& GetLengthX() {
		return length_x;
	}

	void SetLengthX(double x) {
		length_x = x;
	}

	double& GetLengthY() {
		return length_y;
	}

	void SetLengthY(double y) {
		length_y = y;
	}

	const int GetFrameX() const {
		return frame_x;
	}

	void SetFrameX(double x) {
		frame_x = x;
	}

	const int GetFrameY() const {
		return frame_y;
	}

	void SetFrameY(double y) {
		frame_y = y;
	}

	const int GetCentreX() const {
		return center_x;
	}

	void SetCentreX(double x) {
		center_x = x;
	}

	const int GetCentreY() const {
		return center_y;
	}

	void SetCentreY(double y) {
		center_y = y;
	}

	void SetGridWidth(int width) {
		grid_width = width;
	}

	void SetGridHeight(int height) {
		grid_height = height;
	}

};

} // namespace zc::cbs

#endif //LOWLEVEL_H_
