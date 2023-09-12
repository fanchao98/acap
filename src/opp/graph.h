#pragma once

#ifndef GRAPH_H_
#define GRAPH_H_


#include "edge.h"
#include "helpers.h"

#include       <set>
#include       <map>
#include    <vector>
#include    <string>
#include   <fstream>
#include <algorithm>

class Graph
{
public:
	Graph();
	~Graph();
	void AddEdge(int from, int to);
protected:
	int total_node_num;
	std::vector<Edge> edges;
	std::vector<int>  G[maxn];
	std::vector<int>  in_degree;
	std::vector<int>  out_degree;
	std::vector<bool> node_visited, edge_deleted;
	void UpdateDegree(int node_id,int change);
};
#endif