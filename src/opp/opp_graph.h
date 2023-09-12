#pragma once

#ifndef OPP_GRAPG_H_
#define OPP_GRAPG_H_


#include "polygon.h"
#include "graph.h"
#include "data.h"

#include     <queue>
#include   <cstring>
#include <algorithm>
#include<CGAL/Advancing_front_surface_reconstruction.h>
#include<direct.h>
#include<cmath>
typedef std::vector<int> path;
const int max_node_num = 5000;

class OPP_Graph : public Graph
{
public:
	OPP_Graph(const Data& data, const std::vector<std::vector<int>>& initial_opp_info, std::vector<std::pair<int, int>> all_edges,int cont_normal_dependency_edges);
	~OPP_Graph();
	void BuildOPPGraph_G0();
	void MergeOPP_First_One();
	void MergeOPP_First_All_BruteForce();
	void MergeOPP_First_ALL_SearchTree();
	void GenerateFinalOPPs();
	void GeneratePath();
	std::vector<std::vector<int>> medium_opp_info;
	//
private:
	bool** has_edge_G0;
	bool** has_collision_edge_G0;
	bool** has_collision_edge_G1;
	int com[max_node_num];
	Data data;
	std::vector<std::vector<int>> initial_opp_info;
	std::vector<std::vector<std::vector<int>>> all_medium_opp_info;
	std::vector<std::vector<int>> i_length_com[15];
	bool IsDepend(int i, int j);
	bool IsDepend_collision(int i, int j);
	void DFS_One(int u, std::vector<int>& medium_path);
	void DFS_All(int fa, int u, std::vector<int>& temp);
	void OutputMediumOpp(const std::string& file_name);
	void Combination(int n, int m, int M);
	bool compare_two_node(std::vector<int>a, std::vector<int>b);
	void update_tree_node(int now_node);
	void update_tree_node_2(int now_node);
	
	std::vector<std::vector<int>> tree_node;
	std::queue<int> Q;
	std::queue<int> Q_2;
	std::queue<int> NumLayer;
	int node;
	//std::vector<std::vector<int>> pre_tree_index;
	int* pre_tree_index;
	std::vector<std::pair<int, int>> temp_edges;
	int cont_normal_dependency_edges;
	std::vector<std::vector<std::vector<int>>> all_sub_nodes_of_current_layer;

	void BuildOPPGraph_G1();
	void SortBorder(std::vector<int> all_index_G0_vertex);
	void merge_border_point(int c_m, int c_n, int c_k, int l_m, int l_n, int l_k);
	bool jud_monotonous();
	double distance_3d(cv::Point3d a, cv::Point3d b);
	bool jud_OPP(std::vector<int> all_index_G0_vertex);
	bool inner_loop_merge(std::vector<int> all_index_G0_vertex);
	bool merge_layers(cv::Point2i current_layer, cv::Point2i last_layer, int i, int& last_layer_terminal);
	bool jud_allowable_slope(cv::Point3d a, cv::Point3d b);
	bool MergeOPP_G1();
	void try_merge_two_nodes_in_inner_loop(int i,int j, bool& jud_node_merge, int& index_group);
	std::vector<cv::Point3i> sequence_border_point;
	std::vector<std::vector<cv::Point2i>> Group;
	std::vector<bool> is_lower_group;
	std::vector<int> index_in_GO_vertex;
	std::vector<std::vector<bool>> is_deleted_layer;
	bool** has_edge_G1;
	std::vector<bool> have_connected_vertex;
	std::vector<bool> is_eliminate_part_all;
	std::vector<int> eliminate_part_depend_all;
	std::vector<bool> is_forked_part;

	std::vector<std::vector<int>> sub_OPPs;
	std::vector<std::vector<int>> edges_in_sub_OPPs;
	std::vector<std::vector<int>> collision_edges_in_sub_OPPs;
	int cont_current_layer_nodes;
	int cont_current_node_sub_nodes;

	int cont_merge_successfully_nodes;
};



#endif