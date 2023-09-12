#pragma once
#ifndef LAYER_GRAPG_H_
#define LAYER_GRAPG_H_


#include "polygon.h"
#include "graph.h"
#include "data.h"

#include     <set>
#include  <vector>
#include  <string>
#include "..\slice\visual.h"
class Layer_Graph : public Graph
{
public:
	Layer_Graph(const Data& data);
	~Layer_Graph();
	void BuildLayerGraph(nozzle the_nozzle);
	void BuildDependencyGraph(std::vector<cv::Point3d>& all_points);
	void GetInitialOPP();
	Data data;
	std::vector<std::vector<int>> initial_opp_info;
	vector<pair<int, int>> temp_edges;
	int cont_normal_dependency_edges;
private:
	void DFS(int u, std::vector<int>& initial_opp);
	void OutputInitialOpp(const std::string& file_name);
};

#endif // !LAYER_GRAPG_H_