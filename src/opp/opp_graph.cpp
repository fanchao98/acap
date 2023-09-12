#include "opp_graph.h"

OPP_Graph::OPP_Graph(const Data& data, const std::vector<std::vector<int>>& initial_opp_info, std::vector<std::pair<int, int>> all_edges, int cont_normal_dependency_edges)
{
	this->data = data;
	this->initial_opp_info = initial_opp_info;
	this->total_node_num = initial_opp_info.size();
	this->in_degree.resize(this->total_node_num, 0);
	this->out_degree.resize(this->total_node_num, 0);
	this->node_visited.resize(this->total_node_num, false);
	this->edge_deleted.resize(this->total_node_num * this->total_node_num, false);
	//pre_tree_index.resize(1000000);
	pre_tree_index = new int[10000000];
	has_edge_G0 = new bool* [max_node_num];
	has_edge_G1 = new bool* [max_node_num];
	has_collision_edge_G0 = new bool* [max_node_num];
	has_collision_edge_G1 = new bool* [max_node_num];
	for (int i = 0; i < max_node_num; i++) {
		has_edge_G0[i] = new bool[max_node_num];
		has_edge_G1[i] = new bool[max_node_num];
		has_collision_edge_G0[i] = new bool[max_node_num];
		has_collision_edge_G1[i] = new bool[max_node_num];
	}

	for (int i = 0; i < max_node_num; i++) {
		for (int j = 0; j < max_node_num; j++) {
			has_edge_G0[i][j] = false;
			has_edge_G1[i][j] = false;
			has_collision_edge_G0[i][j] = false;
			has_collision_edge_G1[i][j] = false;
		}
	}
	temp_edges = all_edges;
	this->cont_normal_dependency_edges = cont_normal_dependency_edges;
	cont_merge_successfully_nodes = 0;
}

OPP_Graph::~OPP_Graph()
{
}

void OPP_Graph::BuildOPPGraph_G0()
{
	clock_t start_time, end_time;
	start_time = clock();
	//std::cout << "Edges in G0 graph :" << std::endl;
	for (int i = 0; i < this->total_node_num; i++) {
		for (int j = 0; j < this->total_node_num; j++) {
			if (i == j) continue;
			if (IsDepend(i, j)) {
				this->has_edge_G0[i][j] = true;
				//std::cout << "edge: " << i << " " << j << std::endl;
				this->AddEdge(i, j);
				continue;
			}
			if (IsDepend_collision(i, j)) {
				this->has_collision_edge_G0[i][j] = true;
				//std::cout << "collision_edge: " << i << " " << j << std::endl;
				this->AddEdge(i, j);
			}
		}
	}
	std::cout << "number of edges in G_init:" << edges.size() << std::endl;
	end_time = clock();
	//std::cout << "&&&&&&& time of establish initial opp graph: " << double(end_time - start_time) / CLOCKS_PER_SEC << "s &&&&&&&" << std::endl;
	//std::cout << "G0 edges number: " << this->edges.size() << std::endl;
}
bool OPP_Graph::IsDepend(int i, int j)
{
	std::vector<std::vector<cv::Point2d>> i_opp_layers;
	std::vector<std::vector<cv::Point2d>> j_opp_layers;
	std::vector<double> i_z_value;
	std::vector<double> j_z_value;
	for (int m = 0; m < initial_opp_info[i].size(); m++) {
		std::pair<int, int> index_2d = data.index[initial_opp_info[i][m]];
		i_opp_layers.push_back(data.slice_points[index_2d.first][index_2d.second]);
		i_z_value.push_back(data.z_value[index_2d.first][index_2d.second][0]);
	}
	for (int m = 0; m < initial_opp_info[j].size(); m++) {
		std::pair<int, int> index_2d = data.index[initial_opp_info[j][m]];
		j_opp_layers.push_back(data.slice_points[index_2d.first][index_2d.second]);
		j_z_value.push_back(data.z_value[index_2d.first][index_2d.second][0]);
	}

	for (int m = 0; m < i_opp_layers.size(); m++) {
		Polygon polygon_m(ConstructPolygonPoints(i_opp_layers[m], dependence_offset));
		for (int n = 0; n < j_opp_layers.size(); n++) {
			if (abs(j_z_value[n] - i_z_value[m] - dh) < 0.01) {
				for (int k = 0; k < j_opp_layers[n].size(); k++) {
					cv::Point2d p = j_opp_layers[n][k];
					if (polygon_m.JudgePointInside(p)) {
						return true;
					}
				}
			}
		}
	}
	return false;
}

bool OPP_Graph::IsDepend_collision(int i, int j)
{
	int id_top_layer_from = initial_opp_info[i][initial_opp_info[i].size() - 1];
	int id_bottom_layer_to = initial_opp_info[j][0];
	for (int k = cont_normal_dependency_edges;k < temp_edges.size();k++) {
		if (temp_edges[k].first == id_top_layer_from && temp_edges[k].second == id_bottom_layer_to) {
			return true;
		}
	}
	return false;
}

void OPP_Graph::MergeOPP_First_One()
{
	for (int i = 0; i < this->total_node_num; i++) {
		if (this->node_visited[i] == false && this->in_degree[i] == 0) {
			this->UpdateDegree(i, -1);
			std::vector<int> medium_opp;
			medium_opp.push_back(i);
			this->node_visited[i] = true;
			DFS_One(i, medium_opp);
			this->medium_opp_info.push_back(medium_opp);
		}
	}
	std::cout << "first path cover medium opp number_greedy: " << medium_opp_info.size() << std::endl;
	this->OutputMediumOpp(file_name + "_medium_opp" + suffix_txt);
	for (int k = 0; k < this->total_node_num; k++) {
		this->UpdateDegree(k, 1);
		this->node_visited[k] = false;
	}
}
void OPP_Graph::MergeOPP_First_All_BruteForce()
{
	std::vector<int> temp;
	temp.push_back(this->total_node_num - 1);
	this->i_length_com[1].push_back(temp);
	for (int i = 1; i <= this->total_node_num - 1; i++) {
		this->Combination(this->total_node_num - 1, i, i);
		printf("_________\n");
	}
	
	int node[20];
	for (int i = 0; i < max_node_num; i++) {
		node[i] = i;
	}
	int i, j, k;
	int t = 0;
	bool flag_1, flag_2;
	int num = 0;
	do {
		for (i = 1; i <= medium_opp_info.size(); i++) {
			for (j = 0; j < this->i_length_com[i].size(); j++) {
				t = 0;
				flag_1 = true;
				flag_2 = true;
				for (k = 0; k < this->i_length_com[i][j].size(); k++,t++) {
					while (t < this->i_length_com[i][j][k]) {
						if (this->has_edge_G0[node[t]][node[t + 1]] == false) {
							flag_1 = false;
							break;
						}
						t++;
					}
					if (flag_1 == false) break;
				}
					
				if (flag_1 == true) {
						
					t = 0;
					for (k = 0; k < this->i_length_com[i][j].size(); k++) {
						while (t <= this->i_length_com[i][j][k]) {
							if (this->in_degree[node[t]] != 0) {
								//printf("%d %d\n", node[t], this->in_degree[node[t]]);
								flag_2 = false;
							}
							this->UpdateDegree(node[t], -1);
							t++;
						}
					}
					for (int k = 0; k < this->total_node_num; k++) {
						this->UpdateDegree(k, 1);
					}
				}
				//flag_2 = true;
				if (flag_1 == true && flag_2 == true) {
					t = 0;
					printf("------------\n");
					for (k = 0; k < this->i_length_com[i][j].size(); k++) {
						while (t <= this->i_length_com[i][j][k]) {
							printf("%d_", node[t]);
							t++;
						}
						printf("\n");
					}
					num++;
				}
			}
		}
	} while (std::next_permutation(node,node + this->total_node_num));
	std::cout << num << std::endl;
}

//void OPP_Graph::update_tree_node(int now_node)
//{
//	for (int i = 0; i < tree_node[now_node].size(); i++) {
//		this->UpdateDegree(tree_node[now_node][i], -1);
//		this->node_visited[tree_node[now_node][i]] = true;
//	}
//	if (pre_tree_index[now_node][0] != -1) {
//		int temp_node;
//		for (int k = 0; k < pre_tree_index[now_node].size(); k++) {
//			temp_node = pre_tree_index[now_node][k];
//			update_tree_node(temp_node);
//		}
//	}
//}
//
//void OPP_Graph::update_tree_node_2(int now_node)
//{
//	for (int i = 0; i < tree_node[now_node].size(); i++) {
//		this->UpdateDegree(tree_node[now_node][i], 1);
//		this->node_visited[tree_node[now_node][i]] = false;
//	}
//	if (pre_tree_index[now_node][0] != -1) {
//		int temp_node;
//		for (int k = 0; k < pre_tree_index[now_node].size(); k++) {
//			temp_node = pre_tree_index[now_node][k];
//			update_tree_node_2(temp_node);
//		}
//	}
//}

//void OPP_Graph::MergeOPP_First_ALL_SearchTree()
//{
//	clock_t start_time, end_time;
//	start_time = clock();
//	int w = 10000;
//	int height = 0;
//	cont_current_layer_nodes = 0;
//	all_sub_nodes_of_current_layer.resize(1);
//	pre_tree_index.resize(1000000);
//	for (int i = 0; i < 1000000; i++)
//		pre_tree_index[i].resize(1);
//	for (int i = 0; i < 1000000; i++)
//			pre_tree_index[i][0] = -1;
//	std::vector<int> root;
//	bool jud_break = false;
//	tree_node.push_back(root);
//	node = 0;
//	Q.push(node);
//	while (Q.size() != 0 || Q_2.size() != 0) {
//		int now_node = Q.front();
//		update_tree_node(now_node);
//		now_node = Q.front();
//		
//		cont_current_node_sub_nodes = 0;
//		for (int i = 0; i < this->total_node_num; i++) {
//			if (this->node_visited[i] == false && this->in_degree[i] == 0) {
//				std::vector<int> temp;
//				temp.push_back(i);
//				this->UpdateDegree(i, -1);
//				this->node_visited[i] = true;
//				this->DFS_All(now_node, i, temp);   //save in candidate_paths
//				this->UpdateDegree(i, 1);
//				this->node_visited[i] = false;
//			}
//		}
//		cont_current_layer_nodes++;
//		std::vector<std::vector<int>> temp_vec;
//		all_sub_nodes_of_current_layer.push_back(temp_vec);
//		//////merge redundancy paths///
//		bool jud_redundancy = true;
//		for (int i = 0; i < all_sub_nodes_of_current_layer.size()-1; i++) {
//			for (int j = 0; j < all_sub_nodes_of_current_layer[i].size(); j++) {
//				bool jud_redundancy_2 = false;
//				for (int k = 0; k < all_sub_nodes_of_current_layer[all_sub_nodes_of_current_layer.size() - 1].size(); k++) {
//					if(compare_two_node(all_sub_nodes_of_current_layer[i][j], all_sub_nodes_of_current_layer[all_sub_nodes_of_current_layer.size() - 1][k])) {
//						jud_redundancy_2 = true;
//						break;
//					}
//				}
//				if (jud_redundancy_2 == false) {
//					jud_redundancy = false;
//					break;
//				}	
//			}
//			if (jud_redundancy == true) {
//				int id_the_same_node = 0;
//				for (int j = 0; j < i; j++) {
//					for (int k = 0; k < all_sub_nodes_of_current_layer[j].size(); k++)
//						id_the_same_node++;
//				}
//				for (int j = 0; j < all_sub_nodes_of_current_layer[i].size(); j++) {
//					int the_same_node = node - cont_current_layer_nodes + 1 + j+id_the_same_node;
//					for (int k = 0; k < pre_tree_index[node].size(); k++)
//						pre_tree_index[the_same_node].push_back(pre_tree_index[node][k]);
//				}
//				for (int j = 0; j < cont_current_node_sub_nodes; j++) {
//					tree_node.pop_back();
//					//pre_tree_index[node].pop_back();
//					//pre_tree_index[node] = pre_tree_index[the_same_node];
//					node--;
//					Q_2.pop();
//				}
//				break;
//			}
//		}
//		///////////////////////////////
//
//		/*if (Q.size() == 1) {
//			bool jud_break = true;
//			for (int i = 0; i < this->total_node_num; i++)
//				if (this->node_visited[i] == false)
//					jud_break = false;
//			if (jud_break == true)
//				break;
//		}*/
//		update_tree_node_2(now_node);
//		Q.pop();
//		std::vector<int> id_pre_node;
//		id_pre_node.resize(tree_node.size());
//		if (Q.size() == 0) {
//			cont_current_layer_nodes = 0;
//			all_sub_nodes_of_current_layer.clear();
//			all_sub_nodes_of_current_layer.resize(1);
//			height++;
//			int cont_w = 0;
//			std::vector<std::vector<std::vector<int>>> temp_all_medium_opp_info = all_medium_opp_info;
//			all_medium_opp_info.clear();
//			while (Q_2.size() !=0 && cont_w<w) {
//				Q.push(Q_2.front());
//				for (int m = 0; m < pre_tree_index[Q_2.front()].size(); m++) {
//					std::vector<std::vector<int>> temp_vec;
//					if (pre_tree_index[Q_2.front()][0] != 0)
//						temp_vec = temp_all_medium_opp_info[id_pre_node[pre_tree_index[Q_2.front()][m]]];
//					else
//						temp_vec.clear();
//					temp_vec.push_back(tree_node[Q_2.front()]);
//					all_medium_opp_info.push_back(temp_vec);
//					id_pre_node[Q_2.front()] = all_medium_opp_info.size() - 1;
//				}
//				
//				Q_2.pop();
//				cont_w++;
//			}
//			if (cont_w >= w)
//				std::cout << "have reached W" << std::endl;
//			Q_2 = std::queue<int>();
//
//			////pruning////
//			for (int i = 0; i < all_medium_opp_info.size(); i++) {
//				if (all_medium_opp_info[i].size() == height-1) {
//					jud_break = true;
//					all_medium_opp_info = temp_all_medium_opp_info;
//					break;
//				}
//			}
//			if (jud_break == true)
//				break;
//			if(all_medium_opp_info.size() == 0)
//				all_medium_opp_info = temp_all_medium_opp_info;
//			//////////////
//		}
//	}
//	
//	int min_path = maxn;
//	for (int i = 0; i < all_medium_opp_info.size(); i++) {
//		if (min_path < all_medium_opp_info[i].size())
//			min_path = all_medium_opp_info[i].size();
//	}
//	for (int i = 0; i < all_medium_opp_info.size(); i++) {
//		std::vector<std::vector<std::vector<int>>>::iterator itor;
//		for (itor = all_medium_opp_info.begin(); itor != all_medium_opp_info.end(); itor++) {
//			if (itor->size()>min_path) {
//				all_medium_opp_info.erase(itor);
//				itor--;
//			}
//		}
//	}
//
//	/*int min_path = MAX_I;
//	int cnt1, cnt2;
//	int now_node;
//	for (int i = node; i >= 0; i--) {
//		now_node = i;
//		cnt1 = cnt2 = 0;
//		while (pre_tree_index[now_node] != -1) {
//			cnt1++;
//			for (int j = 0; j < tree_node[now_node].size(); j++) {
//				cnt2++;
//			}
//			now_node = pre_tree_index[now_node];
//		}
//		if(cnt2 == this->total_node_num) min_path = std::min(cnt1, min_path);	
//	}*/
//	//std::cout << "first path cover medium opp number_search_tree: " << min_path << std::endl;
//	/*for (int i = node; i >= 0; i--) {
//		now_node = i;
//		cnt2 = 0;
//		std::vector<std::vector<int>> temp_1;
//		while (pre_tree_index[now_node][0] != -1) {
//			for (int k = 0; k < pre_tree_index[now_node].size(); k++) {
//				std::vector<int> temp_2;
//				for (int j = 0; j < tree_node[now_node].size(); j++) {
//					temp_2.push_back(tree_node[now_node][j]);
//					cnt2++;
//				}
//				temp_1.push_back(temp_2);
//				now_node = pre_tree_index[now_node][k];
//			}
//		}
//		std::reverse(temp_1.begin(), temp_1.end());
//		if (cnt2 == this->total_node_num && temp_1.size() == min_path) {
//			this->all_medium_opp_info.push_back(temp_1);
//		}
//	}*/
//	//std::cout << this->all_medium_opp_info.size() << std::endl;
//	/*for (int i = 0; i < this->all_medium_opp_info.size(); i++) {
//		std::cout << "search tree solution "  << i + 1 << std::endl;
//		for (int j = 0; j < this->all_medium_opp_info[i].size(); j++) {
//			for (int k = 0; k < this->all_medium_opp_info[i][j].size(); k++) {
//				std::cout << this->all_medium_opp_info[i][j][k] << " ";
//			}
//			std::cout << std::endl;
//		}
//		std::cout << "--------" << std::endl;
//	}*/
//
//	/*for (int i = 0; i < all_medium_opp_info.size(); i++)
//	{
//		if (all_medium_opp_info[i][14].size() == 45)
//			std::cout << "dd";
//	}*/
//
//	////delete redundancy path////
//	for(int i =0;i< all_medium_opp_info.size();i++)
//		for (int j = i + 1;j < all_medium_opp_info.size();j++) {
//			bool jud_different = true;
//			for (int k = 0;k < all_medium_opp_info[i].size();) {
//				bool jud_different_2 = false;
//				for (int l = 0;l < all_medium_opp_info[j].size();) {
//					if (compare_two_node(all_medium_opp_info[i][k], all_medium_opp_info[j][l]) == true) {
//						k++;
//						jud_different_2 = true;
//						break;
//					}
//					else {
//						l++;
//					}
//				}
//				if (jud_different_2 == false) {
//					jud_different = false;
//					break;
//				}
//					
//			}
//			if (jud_different == true) {
//				std::vector<std::vector<std::vector<int>>>::iterator itor;
//				int cont_itor = 0;
//				for (itor = all_medium_opp_info.begin(); itor != all_medium_opp_info.end(); itor++) {
//					if (cont_itor == i) {
//						all_medium_opp_info.erase(itor);
//						break;
//					}
//					cont_itor++;
//				}
//				i--;
//				//cont_needless_node++;
//				break;
//			}
//			else
//				std::cout << "dd";
//		}
//
//	end_time = clock();
//	std::cout << "&&&&&&& time of PC-MPC calculation: " << double(end_time - start_time) / CLOCKS_PER_SEC << "s &&&&&&&" << std::endl;
//
//	pre_tree_index.clear();
//	medium_opp_info = all_medium_opp_info[10];
//	std::cout << "number of array:" << all_medium_opp_info.size()<< std::endl << std::endl;
//	this->OutputMediumOpp(file_name + "_medium_opp" + suffix_txt);
//
//
//	//update has_collision_edge_G0
//	for (int i = 0;i < medium_opp_info.size();i++)
//		for (int j = 0;j < medium_opp_info[i].size();j++)
//			for (int ii = i + 1;ii < medium_opp_info.size();ii++)
//				for (int jj = 0;jj < medium_opp_info[ii].size();jj++) {
//					if (has_collision_edge_G0[medium_opp_info[i][j]][medium_opp_info[ii][jj]] == true) {
//						has_collision_edge_G1[i][ii] = true;
//						break;
//					}
//					else if (has_collision_edge_G0[medium_opp_info[ii][jj]][medium_opp_info[i][j]] == true) {
//						has_collision_edge_G1[ii][i] = true;
//						break;
//					}
//				}
//	for (int i = 0;i < max_node_num;i++)
//		for (int j = 0;j < max_node_num;j++) {
//			if (has_collision_edge_G1[i][j] == true)
//				std::cout << "collision edges in G1: " << i << " " << j << std::endl;
//		}
//}



void OPP_Graph::MergeOPP_First_ALL_SearchTree()
{
	clock_t start_time, end_time;
	start_time = clock();
	int w = 10000;
	int height = 0;
	memset(pre_tree_index, -1, sizeof(pre_tree_index));
	std::vector<int> root;
	tree_node.push_back(root);
	node = 0;
	Q.push(node);
	while (Q.size() != 0 || Q_2.size() != 0) {
		int now_node = Q.front();
		while (pre_tree_index[now_node] != -1) {
			for (int i = 0; i < tree_node[now_node].size(); i++) {
				this->UpdateDegree(tree_node[now_node][i], -1);
				this->node_visited[tree_node[now_node][i]] = true;
			}
			now_node = pre_tree_index[now_node];
		}
		now_node = Q.front();
		for (int i = 0; i < this->total_node_num; i++) {
			if (this->node_visited[i] == false && this->in_degree[i] == 0) {
				std::vector<int> temp;
				temp.push_back(i);
				this->UpdateDegree(i, -1);
				this->node_visited[i] = true;
				this->DFS_All(now_node, i, temp);
				this->UpdateDegree(i, 1);
				this->node_visited[i] = false;
			}
		}
		/*if (Q.size() == 1) {
			bool jud_break = true;
			for (int i = 0; i < this->total_node_num; i++)
				if (this->node_visited[i] == false)
					jud_break = false;
			if (jud_break == true)
				break;
		}*/

		while (pre_tree_index[now_node] != -1) {
			for (int i = 0; i < tree_node[now_node].size(); i++) {
				this->UpdateDegree(tree_node[now_node][i], 1);
				this->node_visited[tree_node[now_node][i]] = false;
			}
			now_node = pre_tree_index[now_node];
		}
		Q.pop();
		if (Q.size() == 0) {
			height++;
			int cont_w = 0;
			while (Q_2.size() != 0 && cont_w < w) {
				Q.push(Q_2.front());
				Q_2.pop();
				cont_w++;
			}
			if (cont_w >= w)
				std::cout << "have reached W" << std::endl;
			Q_2 = std::queue<int>();
		}
	}
	/*for (int i = 0; i < tree_node.size(); i++) {
		for (int j = 0; j < tree_node[i].size(); j++) {
			std::cout << tree_node[i][j] << " ";
		}
		std::cout << std::endl;
	}*/
	int min_path = MAX_I;
	int cnt1, cnt2;
	int now_node;
	for (int i = node; i >= 0; i--) {
		now_node = i;
		cnt1 = cnt2 = 0;
		while (pre_tree_index[now_node] != -1) {
			cnt1++;
			for (int j = 0; j < tree_node[now_node].size(); j++) {
				cnt2++;
			}
			now_node = pre_tree_index[now_node];
		}
		if (cnt2 == this->total_node_num) min_path = std::min(cnt1, min_path);
	}
	std::cout << "first path cover medium opp number_search_tree: " << min_path << std::endl;
	for (int i = node; i >= 0; i--) {
		now_node = i;
		cnt2 = 0;
		std::vector<std::vector<int>> temp_1;
		while (pre_tree_index[now_node] != -1) {
			std::vector<int> temp_2;
			for (int j = 0; j < tree_node[now_node].size(); j++) {
				temp_2.push_back(tree_node[now_node][j]);
				cnt2++;
			}
			temp_1.push_back(temp_2);
			now_node = pre_tree_index[now_node];
		}
		std::reverse(temp_1.begin(), temp_1.end());
		if (cnt2 == this->total_node_num && temp_1.size() == min_path) {
			this->all_medium_opp_info.push_back(temp_1);
		}
	}
	//std::cout << this->all_medium_opp_info.size() << std::endl;
	/*for (int i = 0; i < this->all_medium_opp_info.size(); i++) {
		std::cout << "search tree solution "  << i + 1 << std::endl;
		for (int j = 0; j < this->all_medium_opp_info[i].size(); j++) {
			for (int k = 0; k < this->all_medium_opp_info[i][j].size(); k++) {
				std::cout << this->all_medium_opp_info[i][j][k] << " ";
			}
			std::cout << std::endl;
		}
		std::cout << "--------" << std::endl;
	}*/

	//int cont_needless_node = 0;
	for (int i = 0; i < all_medium_opp_info.size(); i++)
		for (int j = i + 1; j < all_medium_opp_info.size(); j++) {
			bool jud_different = true;
			for (int k = 0; k < all_medium_opp_info[i].size();) {
				bool jud_different_2 = false;
				for (int l = 0; l < all_medium_opp_info[j].size();) {
					if (compare_two_node(all_medium_opp_info[i][k], all_medium_opp_info[j][l]) == true) {
						k++;
						jud_different_2 = true;
						break;
					}
					else {
						l++;
					}
				}
				if (jud_different_2 == false) {
					jud_different = false;
					break;
				}

			}
			if (jud_different == true) {
				std::vector<std::vector<std::vector<int>>>::iterator itor;
				int cont_itor = 0;
				for (itor = all_medium_opp_info.begin(); itor != all_medium_opp_info.end(); itor++) {
					if (cont_itor == i) {
						all_medium_opp_info.erase(itor);
						break;
					}
					cont_itor++;
				}
				i--;
				//cont_needless_node++;
				break;
			}
		}

	end_time = clock();
	std::cout << "&&&&&&& time of PC-MPC calculation: " << double(end_time - start_time) / CLOCKS_PER_SEC << "s &&&&&&&" << std::endl;

	delete[] pre_tree_index;
	medium_opp_info = all_medium_opp_info[1];
	std::cout << "number of array:" << all_medium_opp_info.size() << std::endl << std::endl;
	this->OutputMediumOpp(file_name + "_medium_opp" + suffix_txt);


	//update has_collision_edge_G0
	for (int i = 0; i < medium_opp_info.size(); i++)
		for (int j = 0; j < medium_opp_info[i].size(); j++)
			for (int ii = i + 1; ii < medium_opp_info.size(); ii++)
				for (int jj = 0; jj < medium_opp_info[ii].size(); jj++) {
					if (has_collision_edge_G0[medium_opp_info[i][j]][medium_opp_info[ii][jj]] == true) {
						has_collision_edge_G1[i][ii] = true;
						break;
					}
					else if (has_collision_edge_G0[medium_opp_info[ii][jj]][medium_opp_info[i][j]] == true) {
						has_collision_edge_G1[ii][i] = true;
						break;
					}
				}
	for (int i = 0; i < max_node_num; i++)
		for (int j = 0; j < max_node_num; j++) {
			if (has_collision_edge_G1[i][j] == true)
				std::cout << "collision edges in G1: " << i << " " << j << std::endl;
		}
}


bool OPP_Graph::compare_two_node(std::vector<int>a, std::vector<int>b)
{
	bool jud_different = true;
	for (int k = 0;k < a.size();) {
		bool jud_different_2 = false;
		for (int l = 0;l < b.size();) {
			if (a[k] == b[l]) {
				k++;
				jud_different_2 = true;
				break;
			}
			else {
				l++;
			}
		}
		if (jud_different_2 == false) {
			jud_different = false;
			break;
		}	
	}
	return jud_different;
}

void OPP_Graph::DFS_One(int u, std::vector<int>& medium_path)
{
	for (int i = 0; i < this->G[u].size(); i++) {
		int v = this->edges[G[u][i]].GetTo();
		if (this->node_visited[v]) continue;
		if (this->in_degree[v] != 0) continue;
		this->node_visited[v] = true;
		this->UpdateDegree(v, -1);
		medium_path.push_back(v);
		DFS_One(v, medium_path);
		break;
	}
}

//void OPP_Graph::DFS_All(int fa, int u, std::vector<int>& temp)
//{
//	bool go_on = false;
//	for (int i = 0; i < this->G[u].size(); i++) {
//		int v = this->edges[G[u][i]].GetTo();
//		if (this->node_visited[v]) continue;
//		if (this->in_degree[v] != 0) continue;
//
//		//////////collision dependency edges///////////////
//		if (IsDepend_collision(u,v)) 
//			continue;
//		///////////////////////////////////////////////////
//		
//		this->node_visited[v] = true;
//		this->UpdateDegree(v, -1);
//		go_on = true;
//		temp.push_back(v);
//		DFS_All(fa, v, temp);
//		temp.pop_back();
//		this->node_visited[v] = false;
//		this->UpdateDegree(v, 1);
//	}
//	if (go_on == false) {
//		node++;
//		tree_node.push_back(temp);
//		pre_tree_index[node][0] = fa;
//		Q_2.push(node);
//		all_sub_nodes_of_current_layer[cont_current_layer_nodes].push_back(temp);
//		cont_current_node_sub_nodes++;
//		//NumLayer.push();
//	}
//}

void OPP_Graph::DFS_All(int fa, int u, std::vector<int>& temp)
{
	bool go_on = false;
	for (int i = 0; i < this->G[u].size(); i++) {
		int v = this->edges[G[u][i]].GetTo();
		if (this->node_visited[v]) continue;
		if (this->in_degree[v] != 0) continue;

		//////////collision dependency edges///////////////
		if (IsDepend_collision(u, v))
			continue;
		///////////////////////////////////////////////////

		this->node_visited[v] = true;
		this->UpdateDegree(v, -1);
		go_on = true;
		temp.push_back(v);
		DFS_All(fa, v, temp);
		temp.pop_back();
		this->node_visited[v] = false;
		this->UpdateDegree(v, 1);
	}
	if (go_on == false) {
		node++;
		tree_node.push_back(temp);
		pre_tree_index[node] = fa;
		Q_2.push(node);
		//all_sub_nodes_of_current_layer[cont_current_layer_nodes].push_back(temp);
		//cont_current_node_sub_nodes++;
		//NumLayer.push();
	}
}

void OPP_Graph::OutputMediumOpp(const std::string& file_name)
{
	std::ofstream dstream(file_name.c_str());
	if (!dstream.is_open()) {
		std::cout << "can not open " << file_name << std::endl;
		return;
	}
	dstream << medium_opp_info.size() << std::endl;
	for (int i = 0; i < medium_opp_info.size(); i++) {
		int layer_num = 0;
		for (int j = 0; j < medium_opp_info[i].size(); j++) {
			layer_num += initial_opp_info[medium_opp_info[i][j]].size();
		}
		dstream << layer_num << std::endl;
		int m, n;
		for (int j = 0; j < medium_opp_info[i].size(); j++) {
			for (int k = 0; k < initial_opp_info[medium_opp_info[i][j]].size(); k++) {
				m = data.index[initial_opp_info[medium_opp_info[i][j]][k]].first;
				n = data.index[initial_opp_info[medium_opp_info[i][j]][k]].second;
				dstream << data.slice_points[m][n].size() << std::endl;
				for (int k = 0; k < data.slice_points[m][n].size(); k++) {
					dstream << this->data.slice_points[m][n][k].x << " " << this->data.slice_points[m][n][k].y << " " << data.z_value[m][n][k] << std::endl;
				}
			}
		}
	}
	dstream.close();
}

void OPP_Graph::Combination(int n, int m, int M)
{
	for (int i = n; i >= 1; i--) {
		com[m] = i;
		if (m > 1) Combination(i - 1, m - 1,M);
		else {
			std::vector<int> temp;
			for (int j = 1; j <= M; j++) {
				printf("%d-", com[j] - 1);
				temp.push_back(com[j] - 1);
			}
			printf("%d-", this->total_node_num - 1);
			temp.push_back(this->total_node_num - 1);
			printf("\n");
			this->i_length_com[M + 1].push_back(temp);
		}
	}
}

void OPP_Graph::GenerateFinalOPPs()
{
	bool jud_merge = true;
	eliminate_part_depend_all.resize(total_node_num);
	while(jud_merge) {  // every time merge two G1's nodes
		is_eliminate_part_all.resize(total_node_num);
		for (int i = 0;i < total_node_num;i++)
			is_eliminate_part_all[i] = false;
		BuildOPPGraph_G1();
		jud_merge = MergeOPP_G1();
	}
	OutputMediumOpp(file_name + "_final_OPPs" + suffix_txt);
	std::cout << "Final OPP number: " << medium_opp_info.size() << std::endl;

	std::cout << "cont_meger_successful_nodes:" << cont_merge_successfully_nodes << std::endl;
}

void OPP_Graph::BuildOPPGraph_G1()
{
	//have_connected_vertex.resize(medium_opp_info.size());
	//for (int i = 0;i < medium_opp_info.size();i++)
		//have_connected_vertex[i] = false;
	bool is_OPP = false;
	for (int i = 0;i < medium_opp_info.size();i++) {
		for (int j = i + 1;j < medium_opp_info.size();j++) {
			//if (have_connected_vertex[i] == true || have_connected_vertex[j] == true)
				//continue;

			//if is depend && did not exist collision dependency
			bool jud_depend = false;
			for (int k = 0;k < medium_opp_info[i].size();k++)
				for (int l = 0;l < medium_opp_info[j].size();l++)
					if (has_edge_G0[medium_opp_info[i][k]][medium_opp_info[j][l]] == true || has_edge_G0[medium_opp_info[j][l]][medium_opp_info[i][k]] == true) //judge dependency by G0's nodes
						jud_depend = true;
			if (jud_depend == false)
				continue;

			/////////////deadlock detection/////////////
			bool jud_deadlock = false;
			for (int k = 0; k < medium_opp_info.size(); k++) {
				if (has_edge_G1[k][i] == true || has_edge_G1[k][j] == true || has_collision_edge_G1[k][i] == true || has_collision_edge_G1[k][j] == true) {
					if (has_edge_G1[i][k] == true || has_edge_G1[j][k] == true || has_collision_edge_G1[i][k] == true || has_collision_edge_G1[j][k] == true)
						jud_deadlock = true;
				}
				/*else if (has_edge_G1[i][k] == true || has_edge_G1[j][k] == true || has_collision_edge_G1[i][k] == true || has_collision_edge_G1[j][k] == true) {
					if (has_edge_G1[k][i] == true || has_edge_G1[k][j] == true || has_collision_edge_G1[k][i] == true || has_collision_edge_G1[k][j] == true)
						jud_deadlock = true;
				}*/
			}
			if (jud_deadlock == true)
				continue;
			////////////////////////////////////////////

			//try merge
			std::vector<int> all_index_G0_vertex;
			for (int k = 0;k < medium_opp_info[i].size();k++)
				all_index_G0_vertex.push_back(medium_opp_info[i][k]);
			for (int k = 0;k < medium_opp_info[j].size();k++)
				all_index_G0_vertex.push_back(medium_opp_info[j][k]);
			//is_OPP = jud_OPP(all_index_G0_vertex);
			is_OPP = inner_loop_merge(all_index_G0_vertex);
			//if(IsSegment)
			//SortBorder(all_index_G0_vertex);
			//bool is_monotonous = jud_monotonous();
					/*if (i == 0 && j == 1 && medium_opp_info.size() == 7)
						is_OPP = true;
					else if (i == 0 && j == 2 && medium_opp_info.size() == 6)
						is_OPP = true;
					else if (i == 0 && j == 1 && medium_opp_info.size() == 5)
						is_OPP = true;
					else if (i == 0 && j == 1 && medium_opp_info.size() == 4)
						is_OPP = true;
					else
						is_OPP = false;*/
			//add edges
			if (is_OPP == true) {
				has_edge_G1[i][j] = true;
				//have_connected_vertex[i] = true;
				//have_connected_vertex[j] = true;
				std::cout << std::endl << "Merge results (by G1 vertexs): " << std::endl;
				std::cout <<i<<" "<<j << std::endl;
				cont_merge_successfully_nodes++;
				break;
			}
			sequence_border_point.clear();
		}
		if (is_OPP == true)
			break;
	}

	if (is_OPP == true) {
		/*std::cout << std::endl << "Merge results (by G0 vertexs): " << std::endl;
		for (int i = 0;i < medium_opp_info.size();i++)
			for (int j = i + 1;j < medium_opp_info.size();j++)
				if (has_edge_G1[i][j] == true) {
					for (int k = 0;k < medium_opp_info[i].size();k++)
						std::cout << medium_opp_info[i][k] << " ";
					for (int k = 0;k < medium_opp_info[j].size();k++)
						std::cout << medium_opp_info[j][k] << " ";
					std::cout << std::endl;
				}
		std::cout << std::endl;*/
	}
}

void OPP_Graph::GeneratePath()
{
	std::vector<int> all_index_G0_vertex;
	std::vector<std::vector<int>> all_curved_block;
	std::vector<int> all_fermat_block;
	all_curved_block.resize(total_node_num);
	is_eliminate_part_all.clear();
	eliminate_part_depend_all.clear();
	is_eliminate_part_all.resize(total_node_num);
	eliminate_part_depend_all.resize(total_node_num);

	//calculate all curved block and fermat block
	for (int i = 0;i < medium_opp_info.size();i++)
	{
		all_index_G0_vertex.clear();
		for (int k = 0;k < medium_opp_info[i].size();k++)
			all_index_G0_vertex.push_back(medium_opp_info[i][k]);
		jud_OPP(all_index_G0_vertex);
	}
	for (int i = 0;i < total_node_num;i++) {
		all_curved_block[i].push_back(i);
	}

	std::vector<bool> is_group_block;
	is_group_block.resize(total_node_num);
	for (int i = 0;i < total_node_num;i++)
		is_group_block[i] = false;
	std::vector<int> temp_curved_block;
	for (int i = 0;i < all_curved_block.size();i++) {
		if (is_group_block[i] == false && all_curved_block[i].size() > 1) {
			for (int j = 1;j < all_curved_block[i].size();j++) {
				for (int k = i + 1;k < all_curved_block.size();k++) {
					if (all_curved_block[k].size()>1 && all_curved_block[i][j] == all_curved_block[k][0]) {
						for (int m = 1;m < all_curved_block[k].size();m++) {
							all_curved_block[i].push_back(all_curved_block[k][m]);
							for (int n = 0;n < all_curved_block[i].size()-1;n++)
								if (all_curved_block[i][n] == all_curved_block[k][m]) {
									all_curved_block[i].pop_back();
									break;
								}
						}
						all_curved_block[k].clear();
						break;
					}
				}
			}
		}
	}
	
	//calculate flat blocks
	std::vector<bool> is_flat_blocks;
	std::vector<std::vector<int>> all_flat_blocks;
	is_flat_blocks.resize(total_node_num);
	for (int i = 0;i < total_node_num;i++)
		is_flat_blocks[i] = true;
	for (int i = 0;i < all_curved_block.size();i++) {
		if (all_curved_block[i].size() > 1) {
			for (int j = 0;j < all_curved_block[i].size();j++) {
				is_flat_blocks[all_curved_block[i][j]] = false;
			}
		}
	}
	for (int i = 0;i < all_fermat_block.size();i++)
		is_flat_blocks[all_fermat_block[i]] = false;
	for (int i = 0;i < medium_opp_info.size();i++) {
		/*std::vector<int> temp_flat_blocks;
		all_flat_blocks.push_back(temp_flat_blocks);*/
		for (int j = 0;j < medium_opp_info[i].size();j++) {
			if (is_flat_blocks[medium_opp_info[i][j]] == true) {
				/*if (all_flat_blocks[all_flat_blocks.size() - 1].size() == 0)
					all_flat_blocks[all_flat_blocks.size() - 1].push_back(medium_opp_info[i][j]);*/
				bool jud_add = false;
				for(int l = 0;l<all_flat_blocks.size();l++)
					for (int k = 0;k < all_flat_blocks[l].size();k++) {
						if (all_flat_blocks[l][k] != medium_opp_info[i][j] &&
							(has_edge_G0[all_flat_blocks[l][k]][medium_opp_info[i][j]] == true)) {
							bool jud_add_2 = false;
							for(int t = 0;t<medium_opp_info[i].size();t++)
								if (t != j && medium_opp_info[i][t] == all_flat_blocks[l][k]) {
									all_flat_blocks[l].push_back(medium_opp_info[i][j]);
									jud_add = true;
									jud_add_2 = true;
									break;
								}
							if (jud_add_2 == true)
								break;
						}
				}
				if (jud_add == false) {
					std::vector<int> temp_flat_blocks;
					all_flat_blocks.push_back(temp_flat_blocks);
					all_flat_blocks[all_flat_blocks.size() - 1].push_back(medium_opp_info[i][j]);
				}
			}
		}
	}

	//output flat blocks
	double temp_dh = 0.2;
	int number_block = 0;
	std::string derectory_2 = "../segmentation/" + file_name + "_all_flat_blocks";
	_mkdir(derectory_2.c_str());
	for (int i = 0;i < all_flat_blocks.size();i++) {
		std::string sub_file_name = "_block" + std::to_string(number_block);
		std::ofstream dstream("../Blocks_files/" + file_name + sub_file_name + ".obj");
		std::ofstream dstream_txt("../Blocks_files/" + file_name + sub_file_name + ".txt");
		int sum_layer = 0;
		for (int j = 0; j < all_flat_blocks[i].size(); j++)
			sum_layer += initial_opp_info[all_flat_blocks[i][j]].size();
		dstream_txt << sum_layer << std::endl;
		for (int j = 0; j < all_flat_blocks[i].size(); j++) {
			int m, n;
			for (int k = 0; k < initial_opp_info[all_flat_blocks[i][j]].size(); k++) {
				m = data.index[initial_opp_info[all_flat_blocks[i][j]][k]].first;
				n = data.index[initial_opp_info[all_flat_blocks[i][j]][k]].second;
				dstream_txt << data.slice_points[m][n].size() << std::endl;
				for (int l = 0; l < data.slice_points[m][n].size(); l++) {
					dstream << "v " << this->data.slice_points[m][n][l].x << " " << this->data.slice_points[m][n][l].y << " " << data.z_value[m][n][l] << std::endl;
					dstream_txt << data.slice_points[m][n][l].x << " " << data.slice_points[m][n][l].y << " " << data.z_value[m][n][l] << " " << dh << std::endl;

					///////for thickness constrant/////////
					//dstream_txt << data.slice_points[m][n][l].x << " " << data.slice_points[m][n][l].y << " " << data.z_value[m][n][l] << " " << temp_dh << std::endl;
				}
				//temp_dh *= 1.04;
			}
		}
		dstream.close();
		number_block++;
	}


}

void OPP_Graph::SortBorder(std::vector<int> all_index_G0_vertex)
{
	std::vector<bool> uppest_is_border;
	std::vector<bool> lowwest_is_border;
	uppest_is_border.resize(all_index_G0_vertex.size());
	lowwest_is_border.resize(all_index_G0_vertex.size());
	for (int i = 0; i < all_index_G0_vertex.size(); i++) {
		uppest_is_border[i] = true;
		lowwest_is_border[i] = true;
	}
	for (int i = 0; i < all_index_G0_vertex.size(); i++)
		for (int j = i + 1; j < all_index_G0_vertex.size(); j++) {
			if (has_edge_G0[i][j] == true) {
				uppest_is_border[i] = false;
				lowwest_is_border[j] = false;
			}
		}

	//Record information about the forked layer
	std::vector<cv::Point3i> sequence_bifurcate_layer;
	int loc_sequence_bifurcate_layer = 0;
	for (int i = 0; i < all_index_G0_vertex.size(); i++) {
		for (int j = 0; j < initial_opp_info[all_index_G0_vertex[i]].size(); j++) {
			int m = data.index[initial_opp_info[all_index_G0_vertex[i]][j]].first;
			int n = data.index[initial_opp_info[all_index_G0_vertex[i]][j]].second;
			if ((j == initial_opp_info[all_index_G0_vertex[i]].size() - 1 && uppest_is_border[i] == false)
				|| (j == 0 && lowwest_is_border[i] == false)) {
				sequence_bifurcate_layer.push_back(cv::Point3i(m, n, 0));
				sequence_bifurcate_layer.push_back(cv::Point3i(m, n, 1));
			}
		}
	}

	for (int i = 0; i < all_index_G0_vertex.size(); i++) {
		for (int j = 0; j < initial_opp_info[all_index_G0_vertex[i]].size(); j++) {
			int m = data.index[initial_opp_info[all_index_G0_vertex[i]][j]].first;
			int n = data.index[initial_opp_info[all_index_G0_vertex[i]][j]].second;
			int index_current_layer, index_current_mul_layer;
			//If it's a middle layer
			if (j != 0 && j != initial_opp_info[all_index_G0_vertex[i]].size() - 1) {
				index_current_layer = data.index[initial_opp_info[all_index_G0_vertex[i]][j + 1]].first;
				index_current_mul_layer = data.index[initial_opp_info[all_index_G0_vertex[i]][j + 1]].second;
				data.adjacent_points[m][n][0].first = cv::Point3i(index_current_layer, index_current_mul_layer, 0);
				data.adjacent_points[m][n][1].first = cv::Point3i(index_current_layer, index_current_mul_layer, 1);

				index_current_layer = data.index[initial_opp_info[all_index_G0_vertex[i]][j - 1]].first;
				index_current_mul_layer = data.index[initial_opp_info[all_index_G0_vertex[i]][j - 1]].second;
				data.adjacent_points[m][n][0].second = cv::Point3i(index_current_layer, index_current_mul_layer, 0);
				data.adjacent_points[m][n][1].second = cv::Point3i(index_current_layer, index_current_mul_layer, 1);
			}

			//If it's a boundary layer
			else if ((j == initial_opp_info[all_index_G0_vertex[i]].size() - 1 && uppest_is_border[i] == true)
				|| (j == 0 && lowwest_is_border[i] == true)) {
				int offset_layer;
				if (j == 0 && lowwest_is_border[i] == true)
					offset_layer = 1;
				else
					offset_layer = -1;
				index_current_layer = data.index[initial_opp_info[all_index_G0_vertex[i]][j + offset_layer]].first;
				index_current_mul_layer = data.index[initial_opp_info[all_index_G0_vertex[i]][j + offset_layer]].second;
				data.adjacent_points[m][n][0].first = cv::Point3i(index_current_layer, index_current_mul_layer, 0);
				data.adjacent_points[m][n][1].first = cv::Point3i(index_current_layer, index_current_mul_layer, 1);

				data.adjacent_points[m][n][0].second = cv::Point3i(m, n, 1);
				data.adjacent_points[m][n][1].second = cv::Point3i(m, n, 0);
			}

			//If it's a forked layer
			else if ((j == initial_opp_info[all_index_G0_vertex[i]].size() - 1 && uppest_is_border[i] == false)
				|| (j == 0 && lowwest_is_border[i] == false)) {
				int offset_layer;
				if (j == 0 && lowwest_is_border[i] == false)
					offset_layer = 1;
				else
					offset_layer = -1;
				index_current_layer = data.index[initial_opp_info[all_index_G0_vertex[i]][j + offset_layer]].first;
				index_current_mul_layer = data.index[initial_opp_info[all_index_G0_vertex[i]][j + offset_layer]].second;
				data.adjacent_points[m][n][0].first = cv::Point3i(index_current_layer, index_current_mul_layer, 0);
				data.adjacent_points[m][n][1].first = cv::Point3i(index_current_layer, index_current_mul_layer, 1);

				if (loc_sequence_bifurcate_layer == 0) {
					data.adjacent_points[m][n][0].second = sequence_bifurcate_layer[(loc_sequence_bifurcate_layer + 2) % sequence_bifurcate_layer.size()];
					loc_sequence_bifurcate_layer++;
					data.adjacent_points[m][n][1].second = sequence_bifurcate_layer[(loc_sequence_bifurcate_layer - 2 + sequence_bifurcate_layer.size()) % sequence_bifurcate_layer.size()];
					loc_sequence_bifurcate_layer++;
				}
				else if (loc_sequence_bifurcate_layer == 2) {
					data.adjacent_points[m][n][0].second = sequence_bifurcate_layer[(loc_sequence_bifurcate_layer - 2 + sequence_bifurcate_layer.size()) % sequence_bifurcate_layer.size()];
					loc_sequence_bifurcate_layer++;
					data.adjacent_points[m][n][1].second = sequence_bifurcate_layer[(loc_sequence_bifurcate_layer + 1) % sequence_bifurcate_layer.size()];
					loc_sequence_bifurcate_layer++;
				}
				else if (loc_sequence_bifurcate_layer != sequence_bifurcate_layer.size() - 2) {
					data.adjacent_points[m][n][0].second = sequence_bifurcate_layer[(loc_sequence_bifurcate_layer - 1 + sequence_bifurcate_layer.size()) % sequence_bifurcate_layer.size()];
					loc_sequence_bifurcate_layer++;
					data.adjacent_points[m][n][1].second = sequence_bifurcate_layer[(loc_sequence_bifurcate_layer + 1) % sequence_bifurcate_layer.size()];
					loc_sequence_bifurcate_layer++;
				}
				else if (loc_sequence_bifurcate_layer == sequence_bifurcate_layer.size() - 2) {
					data.adjacent_points[m][n][0].second = sequence_bifurcate_layer[(loc_sequence_bifurcate_layer - 1 + sequence_bifurcate_layer.size()) % sequence_bifurcate_layer.size()];
					loc_sequence_bifurcate_layer++;
					data.adjacent_points[m][n][1].second = sequence_bifurcate_layer[(loc_sequence_bifurcate_layer + 2) % sequence_bifurcate_layer.size()];
					loc_sequence_bifurcate_layer++;
				}

			}
		}

	}

	//Start sorting
	int last_m = data.index[initial_opp_info[all_index_G0_vertex[all_index_G0_vertex[0]]][0]].first;
	int last_n = data.index[initial_opp_info[all_index_G0_vertex[all_index_G0_vertex[0]]][0]].second;
	int last_k = 0;
	int begin_m = last_m, begin_n = last_n, begin_k = last_k;
	int current_m = data.adjacent_points[last_m][last_n][last_k].first.x;
	int current_n = data.adjacent_points[last_m][last_n][last_k].first.y;
	int current_k = data.adjacent_points[last_m][last_n][last_k].first.z;
	while (current_m != begin_m || current_n != begin_n || current_k != begin_k) {
		if (data.adjacent_points[current_m][current_n][current_k].first.x != last_m
			|| data.adjacent_points[current_m][current_n][current_k].first.y != last_n
			|| data.adjacent_points[current_m][current_n][current_k].first.z != last_k) {
			last_m = current_m;
			last_n = current_n;
			last_k = current_k;
			//update
			current_m = data.adjacent_points[last_m][last_n][last_k].first.x;
			current_n = data.adjacent_points[last_m][last_n][last_k].first.y;
			current_k = data.adjacent_points[last_m][last_n][last_k].first.z;
			merge_border_point(current_m, current_n, current_k, last_m, last_n, last_k);
		}
		else if (data.adjacent_points[current_m][current_n][current_k].second.x != last_m
			|| data.adjacent_points[current_m][current_n][current_k].second.y != last_n
			|| data.adjacent_points[current_m][current_n][current_k].second.z != last_k) {
			last_m = current_m;
			last_n = current_n;
			last_k = current_k;
			current_m = data.adjacent_points[last_m][last_n][last_k].second.x;
			current_n = data.adjacent_points[last_m][last_n][last_k].second.y;
			current_k = data.adjacent_points[last_m][last_n][last_k].second.z;
			merge_border_point(current_m, current_n, current_k, last_m, last_n, last_k);
		}
	}

	//output_test
	std::ofstream ofile_test("../test/" + file_name + "_test_sequence.obj");
	for (int i = 0; i < sequence_border_point.size(); i++)
		ofile_test << "v " << data.slice_points[sequence_border_point[i].x][sequence_border_point[i].y][sequence_border_point[i].z].x
		<< " " << data.slice_points[sequence_border_point[i].x][sequence_border_point[i].y][sequence_border_point[i].z].y
		<< " " << data.z_value[sequence_border_point[i].x][sequence_border_point[i].y][sequence_border_point[i].z] << std::endl;
}

bool OPP_Graph::MergeOPP_G1()
{
	bool jud_merge = false;
	bool have_been_used_opp[max_node_num];
	for (int i = 0; i < max_node_num; i++)
		have_been_used_opp[i] = false;
	std::vector<std::vector<int>> temp_medium_opp_info;
	for (int i = 0; i < medium_opp_info.size(); i++) {
		if (have_been_used_opp[i] == true)
			continue;
		std::vector<int> temp_medium_opp;
		for (int k = 0;k < medium_opp_info[i].size();k++)
			temp_medium_opp.push_back(medium_opp_info[i][k]);
		if(jud_merge == false)
			for (int j = i + 1;j < medium_opp_info.size();j++) {
				if (has_edge_G1[i][j] == true) {
					jud_merge = true;
					for (int k = 0;k < medium_opp_info[j].size();k++)
						temp_medium_opp.push_back(medium_opp_info[j][k]);
					have_been_used_opp[j] = true;    
				}
			}
		temp_medium_opp_info.push_back(temp_medium_opp);
	}
	medium_opp_info = temp_medium_opp_info;
	for (int i = 0;i < max_node_num;i++)
		for (int j = 0;j < max_node_num;j++) {
			has_edge_G1[i][j] = false;
			has_collision_edge_G1[i][j] = false;
		}
			
	//update has_collision_edge_G0
	for (int i = 0;i < medium_opp_info.size();i++)
		for (int j = 0;j < medium_opp_info[i].size();j++)
			for (int ii = i + 1;ii < medium_opp_info.size();ii++)
				for (int jj = 0;jj < medium_opp_info[ii].size();jj++) {
					if (has_collision_edge_G0[medium_opp_info[i][j]][medium_opp_info[ii][jj]] == true) {
						has_collision_edge_G1[i][ii] = true;
						break;
					}
					else if (has_collision_edge_G0[medium_opp_info[ii][jj]][medium_opp_info[i][j]] == true) {
						has_collision_edge_G1[ii][i] = true;
						break;
					}
				}
	return jud_merge;
}

void OPP_Graph::merge_border_point(int c_m, int c_n, int c_k, int l_m, int l_n, int l_k)
{
	if (c_k == 1)
		c_k = data.slice_points[c_m][c_n].size() - 1;
	if (l_k == 1)
		l_k = data.slice_points[l_m][l_n].size() - 1;

	sequence_border_point.push_back(cv::Point3i(c_m, c_n, c_k));
	//cv::Point3d temp_current_point = cv::Point3d(data.slice_points[c_m][c_n][0].x, data.slice_points[c_m][c_n][0].y, data.z_value[c_m][c_n][0]);
	//cv::Point3d temp_current_point_2 = cv::Point3d(data.slice_points[c_m][c_n][data.slice_points[c_m][c_n].size() - 1].x, data.slice_points[c_m][c_n][data.slice_points[c_m][c_n].size() - 1].y, data.z_value[c_m][c_n][data.slice_points[c_m][c_n].size() - 1]);
	//cv::Point3d last_point = cv::Point3d(data.slice_points[l_m][l_n][l_k].x, data.slice_points[l_m][l_n][l_k].y, data.z_value[l_m][l_n][l_k]);
	//if ((c_m != l_m || c_n != l_n) && (distance_3d(temp_current_point, last_point) > distance_3d(temp_current_point_2, last_point))) {
	//	c_k = data.slice_points[c_m][c_n].size() - 1;
	//}

	//cv::Point3d current_point = cv::Point3d(data.slice_points[c_m][c_n][c_k].x, data.slice_points[c_m][c_n][c_k].y, data.z_value[c_m][c_n][c_k]);
	//double tan_slope_angle_value = abs(current_point.z - last_point.z) / sqrt(pow(current_point.x - last_point.x, 2) + pow(current_point.y - last_point.y, 2));

	////set slope angle value is 45°
	//if (tan_slope_angle_value > 1) {
	//	sequence_border_point.push_back(cv::Point3i(c_m, c_n, c_k));
	//}

}

bool OPP_Graph::jud_monotonous()
{
	double heightest_value = -99999;
	int index_heightest_point;
	double lowwest_value = 99999;
	int index_lowwest_point;
	for (int i = 0; i < sequence_border_point.size(); i++) {
		if (sequence_border_point[i].z > heightest_value) {
			heightest_value = sequence_border_point[i].z;
			index_heightest_point = i;
		}
		if (sequence_border_point[i].z < lowwest_value) {
			lowwest_value = sequence_border_point[i].z;
			index_lowwest_point = i;
		}
	}
	bool is_monotonous = true;
	int current_point = index_heightest_point;
	int next_point = (index_heightest_point + 1) % sequence_border_point.size();
	while (current_point != index_lowwest_point) {
		if (data.z_value[sequence_border_point[current_point].x][sequence_border_point[current_point].y][sequence_border_point[current_point].z]
			< data.z_value[sequence_border_point[next_point].x][sequence_border_point[next_point].y][sequence_border_point[next_point].z])
			is_monotonous = false;
		current_point = next_point;
		next_point = (current_point + 1) % sequence_border_point.size();
	}
	current_point = index_heightest_point;
	next_point = (index_heightest_point - 1 + sequence_border_point.size()) % sequence_border_point.size();
	while (current_point != index_lowwest_point) {
		if (data.z_value[sequence_border_point[current_point].x][sequence_border_point[current_point].y][sequence_border_point[current_point].z]
			< data.z_value[sequence_border_point[next_point].x][sequence_border_point[next_point].y][sequence_border_point[next_point].z])
			is_monotonous = false;
		current_point = next_point;
		next_point = (current_point - 1 + sequence_border_point.size()) % sequence_border_point.size();
	}
	return is_monotonous;
}

double OPP_Graph::distance_3d(cv::Point3d a, cv::Point3d b) {
	double distance = sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
	return distance;
}

bool OPP_Graph::jud_OPP(std::vector<int> all_index_G0_vertex)
{
	is_deleted_layer.resize(data.slice_points.size());
	for (int i = 0;i < data.slice_points.size();i++)
		is_deleted_layer[i].resize(data.slice_points[i].size());
	for (int i = 0;i < data.slice_points.size();i++)
		for (int j = 0;j < data.slice_points[i].size();j++)
			is_deleted_layer[i][j] = false;

	//search forked part
	std::vector<int> current_out_degree,current_in_degree;
	current_out_degree.resize(all_index_G0_vertex.size());
	current_in_degree.resize(all_index_G0_vertex.size());
	for (int i = 0;i < all_index_G0_vertex.size();i++) {
		current_in_degree[i] = 0;
		current_out_degree[i] = 0;
	}
	for (int i = 0;i < all_index_G0_vertex.size();i++) {
		for (int j = 0;j < all_index_G0_vertex.size();j++) {
			if (i != j && has_edge_G0[all_index_G0_vertex[i]][all_index_G0_vertex[j]] == true) {
				current_out_degree[i]++;
				current_in_degree[j]++;
			}
		}
	}
	is_forked_part.clear();
	is_forked_part.resize(all_index_G0_vertex.size());
	Group.clear();
	index_in_GO_vertex.clear();
	Group.resize(all_index_G0_vertex.size());
	is_lower_group.resize(all_index_G0_vertex.size());
	int index_group = 0;
	std::vector<int> index_in_GO_of_Group;
	index_in_GO_of_Group.resize(total_node_num);
	for (int i = 0;i < all_index_G0_vertex.size();i++) {
		if (current_out_degree[i] == 0 && current_in_degree[i] == 1) {
			int index_layer = initial_opp_info[all_index_G0_vertex[i]][initial_opp_info[all_index_G0_vertex[i]].size() - 1];
			Group[index_group].push_back(cv::Point2i(data.index[index_layer].first, data.index[index_layer].second));
			index_in_GO_vertex.push_back(all_index_G0_vertex[i]);
			index_in_GO_of_Group[index_group] = all_index_G0_vertex[i];
			is_lower_group[index_group] = false;
			for (int j = 0;j < all_index_G0_vertex.size();j++)
				if (i != j && has_edge_G0[all_index_G0_vertex[j]][all_index_G0_vertex[i]] == true)
					eliminate_part_depend_all[all_index_G0_vertex[i]] = all_index_G0_vertex[j];
			index_group++;
			is_forked_part[i] = true;
		}
		else if (current_in_degree[i] == 0 && current_out_degree[i] == 1) {
			int index_layer = initial_opp_info[all_index_G0_vertex[i]][0];
			Group[index_group].push_back(cv::Point2i(data.index[index_layer].first, data.index[index_layer].second));
			index_in_GO_vertex.push_back(all_index_G0_vertex[i]);
			index_in_GO_of_Group[index_group] = all_index_G0_vertex[i];
			is_lower_group[index_group] = true;
			for (int j = 0;j < all_index_G0_vertex.size();j++)
				if (i != j && has_edge_G0[all_index_G0_vertex[i]][all_index_G0_vertex[j]] == true)
					eliminate_part_depend_all[all_index_G0_vertex[i]] = all_index_G0_vertex[j];
			index_group++;
			is_forked_part[i] = true;
		}
		else
			is_forked_part[i] = false;
	}
	Group.resize(index_group);

	//record connection relationships
	std::vector<std::vector<cv::Point3i>> four_boder_connection_terminal_point;
	four_boder_connection_terminal_point.resize(all_index_G0_vertex.size());
	for (int i = 0;i < all_index_G0_vertex.size();i++) {
		if (is_forked_part[i] == false &&
			data.is_contour[data.index[initial_opp_info[all_index_G0_vertex[i]][0]].first]
			[data.index[initial_opp_info[all_index_G0_vertex[i]][0]].second] == false) {
			std::vector<cv::Point2i> lowwer_connection_terminal_point;
			std::vector<cv::Point2i> upper_connection_terminal_point;
			cv::Point3i final_lowwer_connection_terminal_point_left;
			cv::Point3i final_lowwer_connection_terminal_point_right;
			cv::Point3i final_upper_connection_terminal_point_left;
			cv::Point3i final_upper_connection_terminal_point_right;
			for (int j = 0;j < all_index_G0_vertex.size();j++) {
				if (i != j && has_edge_G0[all_index_G0_vertex[i]][all_index_G0_vertex[j]] == true) {
					upper_connection_terminal_point.push_back(cv::Point2i(data.index[initial_opp_info[all_index_G0_vertex[j]][0]].first,
						data.index[initial_opp_info[all_index_G0_vertex[j]][0]].second));
				}
				else if (i != j && has_edge_G0[all_index_G0_vertex[j]][all_index_G0_vertex[i]] == true) {
					lowwer_connection_terminal_point.push_back(cv::Point2i(data.index[initial_opp_info[all_index_G0_vertex[j]][initial_opp_info[all_index_G0_vertex[j]].size() - 1]].first,
						data.index[initial_opp_info[all_index_G0_vertex[j]][initial_opp_info[all_index_G0_vertex[j]].size() - 1]].second));
				}
			}
			cv::Point2i lowwer_terminal_point, upper_terminal_point;
			lowwer_terminal_point= cv::Point2i(data.index[initial_opp_info[all_index_G0_vertex[i]][0]].first, data.index[initial_opp_info[all_index_G0_vertex[i]][0]].second);
			int loc_upper_terminal_point_right = initial_opp_info[all_index_G0_vertex[i]].size()-1;
			upper_terminal_point = cv::Point2i(data.index[initial_opp_info[all_index_G0_vertex[i]][loc_upper_terminal_point_right]].first,
				data.index[initial_opp_info[all_index_G0_vertex[i]][loc_upper_terminal_point_right]].second);
			double min_distance = 99999, min_distance_2 = 99999;
			cv::Point3i lowwer_terminal_point_left = cv::Point3i(data.slice_points[lowwer_terminal_point.x][lowwer_terminal_point.y][0]);
			cv::Point3i lowwer_terminal_point_right = cv::Point3i(data.slice_points[lowwer_terminal_point.x][lowwer_terminal_point.y][data.slice_points[lowwer_terminal_point.x][lowwer_terminal_point.y].size()-1]);
			cv::Point3i upper_terminal_point_left = cv::Point3i(data.slice_points[upper_terminal_point.x][upper_terminal_point.y][0]);
			cv::Point3i upper_terminal_point_right = cv::Point3i(data.slice_points[upper_terminal_point.x][upper_terminal_point.y][data.slice_points[upper_terminal_point.x][upper_terminal_point.y].size() - 1]);
			for (int j = 0;j < lowwer_connection_terminal_point.size();j++) {
				cv::Point3i P1 = cv::Point3i(data.slice_points[lowwer_connection_terminal_point[j].x][lowwer_connection_terminal_point[j].y][0].x,
					data.slice_points[lowwer_connection_terminal_point[j].x][lowwer_connection_terminal_point[j].y][0].y,
					data.z_value[lowwer_connection_terminal_point[j].x][lowwer_connection_terminal_point[j].y][0]);
				int loc_P2 = data.slice_points[lowwer_connection_terminal_point[j].x][lowwer_connection_terminal_point[j].y].size() - 1;
				cv::Point3i P2 = cv::Point3i(data.slice_points[lowwer_connection_terminal_point[j].x][lowwer_connection_terminal_point[j].y][loc_P2].x,
					data.slice_points[lowwer_connection_terminal_point[j].x][lowwer_connection_terminal_point[j].y][loc_P2].y,
					data.z_value[lowwer_connection_terminal_point[j].x][lowwer_connection_terminal_point[j].y][loc_P2]);
				double distance = distance_3d(P1, lowwer_terminal_point_left);
				double distance_2 = distance_3d(P1, lowwer_terminal_point_right);
				if (distance < min_distance) {
					min_distance = distance;
					final_lowwer_connection_terminal_point_left = cv::Point3i(lowwer_connection_terminal_point[j].x, lowwer_connection_terminal_point[j].y,0);
				}
				distance = distance_3d(P2, lowwer_terminal_point_left);
				if (distance < min_distance) {
					min_distance = distance;
					final_lowwer_connection_terminal_point_left = cv::Point3i(lowwer_connection_terminal_point[j].x, lowwer_connection_terminal_point[j].y, loc_P2);
				}
				if (distance_2 < min_distance_2) {
					min_distance_2 = distance_2;
					final_lowwer_connection_terminal_point_right = cv::Point3i(lowwer_connection_terminal_point[j].x, lowwer_connection_terminal_point[j].y, 0);
				}
				distance_2 = distance_3d(P2, lowwer_terminal_point_right);
				if (distance_2 < min_distance_2) {
					min_distance_2 = distance_2;
					final_lowwer_connection_terminal_point_right = cv::Point3i(lowwer_connection_terminal_point[j].x, lowwer_connection_terminal_point[j].y, loc_P2);
				}
			}
			for (int j = 0;j < upper_connection_terminal_point.size();j++) {
				cv::Point3i P1 = cv::Point3i(data.slice_points[upper_connection_terminal_point[j].x][upper_connection_terminal_point[j].y][0].x,
					data.slice_points[upper_connection_terminal_point[j].x][upper_connection_terminal_point[j].y][0].y,
					data.z_value[upper_connection_terminal_point[j].x][upper_connection_terminal_point[j].y][0]);
				int loc_P2 = data.slice_points[upper_connection_terminal_point[j].x][upper_connection_terminal_point[j].y].size() - 1;
				cv::Point3i P2 = cv::Point3i(data.slice_points[upper_connection_terminal_point[j].x][upper_connection_terminal_point[j].y][loc_P2].x,
					data.slice_points[upper_connection_terminal_point[j].x][upper_connection_terminal_point[j].y][loc_P2].y,
					data.z_value[upper_connection_terminal_point[j].x][upper_connection_terminal_point[j].y][loc_P2]);
				double distance = distance_3d(P1, upper_terminal_point_left);
				double distance_2 = distance_3d(P1, upper_terminal_point_right);
				if (distance < min_distance) {
					min_distance = distance;
					final_upper_connection_terminal_point_left = cv::Point3i(upper_connection_terminal_point[j].x, upper_connection_terminal_point[j].y, 0);
				}
				distance = distance_3d(P2, upper_terminal_point_left);
				if (distance < min_distance) {
					min_distance = distance;
					final_upper_connection_terminal_point_left = cv::Point3i(upper_connection_terminal_point[j].x, upper_connection_terminal_point[j].y, loc_P2);
				}
				if (distance_2 < min_distance_2) {
					min_distance_2 = distance_2;
					final_upper_connection_terminal_point_right = cv::Point3i(upper_connection_terminal_point[j].x, upper_connection_terminal_point[j].y, 0);
				}
				distance_2 = distance_3d(P2, upper_terminal_point_right);
				if (distance_2 < min_distance_2) {
					min_distance_2 = distance_2;
					final_upper_connection_terminal_point_right = cv::Point3i(upper_connection_terminal_point[j].x, upper_connection_terminal_point[j].y, loc_P2);
				}
			}
			four_boder_connection_terminal_point[i].push_back(final_lowwer_connection_terminal_point_left);
			four_boder_connection_terminal_point[i].push_back(final_lowwer_connection_terminal_point_right);
			four_boder_connection_terminal_point[i].push_back(final_upper_connection_terminal_point_left);
			four_boder_connection_terminal_point[i].push_back(final_upper_connection_terminal_point_right);
		}
	}

	//try merge layer by layer
	for (int i = 0;i < index_group;i++) {
		cv::Point2i last_layer = Group[i][0];
		cv::Point2i current_layer;
		cv::Point2i temp_current_layer;
		if (is_lower_group[i] == false)
			if (int(initial_opp_info[index_in_GO_vertex[i]].size() - 2) >= 0)
				current_layer = cv::Point2i(data.index[initial_opp_info[index_in_GO_vertex[i]][initial_opp_info[index_in_GO_vertex[i]].size() - 2]].first
					, data.index[initial_opp_info[index_in_GO_vertex[i]][initial_opp_info[index_in_GO_vertex[i]].size() - 2]].second);
			else
				continue;
		else
			if (int(initial_opp_info[index_in_GO_vertex[i]].size()) >= 2)
				current_layer = cv::Point2i(data.index[initial_opp_info[index_in_GO_vertex[i]][1]].first
					, data.index[initial_opp_info[index_in_GO_vertex[i]][1]].second);
			else
				continue;
		temp_current_layer = current_layer;
		int last_layer_terminal = 0;
		//int last_layer_terminal = -1;
		int last_layer_terminal_2 = -3;
		bool jud_merge = true;
		int  iteration = 2;
		///last_layer_terminal_2 = data.slice_points[last_layer.x][last_layer.y].size() - 1;????????????
		bool travers_every_layer = false;
		while (jud_merge == true) {
			if (data.is_contour[last_layer.x][last_layer.y] == true && data.is_contour[current_layer.x][current_layer.y] == true) {
				last_layer_terminal = last_layer_terminal_2 = -1;
				jud_merge = merge_layers(current_layer, last_layer, i, last_layer_terminal);
			}
			else if (data.is_contour[last_layer.x][last_layer.y] == false && data.is_contour[current_layer.x][current_layer.y] == false) {
				/*if (iteration == 2) {
					last_layer_terminal = 0;
					last_layer_terminal_2 = data.slice_points[last_layer.x][last_layer.y].size() - 1;
				}*/
				//先试一边再试另一边
				if (last_layer_terminal != -2) {
					jud_merge = merge_layers(current_layer, last_layer, i, last_layer_terminal);
					if (jud_merge == false) {
						last_layer = Group[i][0];
						current_layer = temp_current_layer;
						jud_merge = true;
						iteration = 2;
						if (last_layer.y)
							last_layer_terminal_2 = data.slice_points[Group[i][0].x][Group[i][0].y].size() - 1;
						else
							last_layer_terminal_2 = 0;
						last_layer_terminal = -2;
						continue;
					}
				}
				else
					jud_merge = merge_layers(current_layer, last_layer, i, last_layer_terminal_2);
			}
			if (jud_merge == false)
				break;
			last_layer = current_layer;
			if (is_lower_group[i] == false) {
				if (int(initial_opp_info[index_in_GO_vertex[i]].size() - 1 - iteration) >= 0)
					current_layer = cv::Point2i(data.index[initial_opp_info[index_in_GO_vertex[i]][initial_opp_info[index_in_GO_vertex[i]].size() - 1 - iteration]].first
						, data.index[initial_opp_info[index_in_GO_vertex[i]][initial_opp_info[index_in_GO_vertex[i]].size() - 1 - iteration]].second);
				else {
					travers_every_layer = true;
					break;
				}
					
			}
			else
				if (iteration < initial_opp_info[index_in_GO_vertex[i]].size())
					current_layer = cv::Point2i(data.index[initial_opp_info[index_in_GO_vertex[i]][iteration]].first
						, data.index[initial_opp_info[index_in_GO_vertex[i]][iteration]].second);
				else {
					travers_every_layer = true;
					break;
				}
			iteration++;
		}

		bool jud_break = false;
		for (int m = 0;m < all_index_G0_vertex.size();m++)
		{
			if(four_boder_connection_terminal_point[m].size() > 0)
				for (int k = 0;k < 4;k++) {
					if (four_boder_connection_terminal_point[m][k].x == last_layer.x && four_boder_connection_terminal_point[m][k].y == last_layer.y &&
						(four_boder_connection_terminal_point[m][k].z == last_layer_terminal || four_boder_connection_terminal_point[m][k].z == last_layer_terminal_2)) {
						jud_break = true;
					}
				}
		}
		//if (jud_break == true)
			//continue;
		if (travers_every_layer == false)   //All layers of the forked part must be removed
			continue;

		else {     //update fork parts
			for (int k = 0;k < all_index_G0_vertex.size();k++) {
				if (has_edge_G0[all_index_G0_vertex[k]][index_in_GO_vertex[i]] == true)
					current_out_degree[k]--;
				else if(has_edge_G0[index_in_GO_vertex[i]][all_index_G0_vertex[k]] == true)
					current_in_degree[k]--;
			}
			for (int k = 0;k < all_index_G0_vertex.size();k++) {
				if (current_out_degree[k] == 0 && current_in_degree[k] == 1 && is_forked_part[k] == false) {
					int index_layer = initial_opp_info[all_index_G0_vertex[k]][initial_opp_info[all_index_G0_vertex[k]].size() - 1];
					Group.resize(index_group+1);
					is_lower_group.resize(index_group + 1);
					Group[index_group].push_back(cv::Point2i(data.index[index_layer].first, data.index[index_layer].second));
					index_in_GO_vertex.push_back(all_index_G0_vertex[k]);
					index_in_GO_of_Group[index_group] = all_index_G0_vertex[k];
					is_lower_group[index_group] = false;
					for (int j = 0;j < all_index_G0_vertex.size();j++)
						if (k != j && has_edge_G0[all_index_G0_vertex[j]][all_index_G0_vertex[k]] == true)
							eliminate_part_depend_all[all_index_G0_vertex[k]] = all_index_G0_vertex[j];
					index_group++;
					is_forked_part[k] = true;
				}
				else if (current_in_degree[k] == 0 && current_out_degree[k] == 1 && is_forked_part[k] == false) {
					int index_layer = initial_opp_info[all_index_G0_vertex[k]][0];
					Group.resize(index_group + 1);
					is_lower_group.resize(index_group + 1);
					Group[index_group].push_back(cv::Point2i(data.index[index_layer].first, data.index[index_layer].second));
					index_in_GO_vertex.push_back(all_index_G0_vertex[k]);
					index_in_GO_of_Group[index_group] = all_index_G0_vertex[k];
					is_lower_group[index_group] = true;
					for (int j = 0;j < all_index_G0_vertex.size();j++)
						if (k != j && has_edge_G0[all_index_G0_vertex[k]][all_index_G0_vertex[j]] == true)
							eliminate_part_depend_all[all_index_G0_vertex[k]] = all_index_G0_vertex[j];
					index_group++;
					is_forked_part[k] = true;
				}
				//else
					//is_forked_part[k] = false;
			}
		}

		is_eliminate_part_all[index_in_GO_of_Group[i]] = true;
		//delete layers have been merged
		for (int j = 0;j < Group[i].size();j++) {
			is_deleted_layer[Group[i][j].x][Group[i][j].y] = true;
		}
	}


	//OPP is judged by monotonicity
	std::vector<std::vector<bool>> exist_layer;
	exist_layer = is_deleted_layer;
	for (int i = 0;i < all_index_G0_vertex.size();i++)
		for (int j = 0;j < initial_opp_info[all_index_G0_vertex[i]].size();j++) {
			exist_layer[data.index[initial_opp_info[all_index_G0_vertex[i]][j]].first][data.index[initial_opp_info[all_index_G0_vertex[i]][j]].second] = true;
		}
	bool is_OPP = true;
	for (int i = 0;i < data.slice_points.size();i++) {
		int cont_undeleted_layer = 0;
		for (int j = 0;j < data.slice_points[i].size();j++)
			if (is_deleted_layer[i][j] == false && exist_layer[i][j] == true)
				cont_undeleted_layer++;
		if (cont_undeleted_layer > 1)
			is_OPP = false;
	}
	return is_OPP;
}

bool OPP_Graph::inner_loop_merge(std::vector<int> all_index_G0_vertex)
{
	is_deleted_layer.resize(data.slice_points.size());
	for (int i = 0;i < data.slice_points.size();i++)
		is_deleted_layer[i].resize(data.slice_points[i].size());
	for (int i = 0;i < data.slice_points.size();i++)
		for (int j = 0;j < data.slice_points[i].size();j++)
			is_deleted_layer[i][j] = false;

	
	sub_OPPs.clear();
	sub_OPPs.resize(all_index_G0_vertex.size());
	for (int i = 0;i < all_index_G0_vertex.size();i++)
		sub_OPPs[i].push_back(all_index_G0_vertex[i]);
	edges_in_sub_OPPs.clear();
	collision_edges_in_sub_OPPs.clear();
	edges_in_sub_OPPs.resize(all_index_G0_vertex.size());
	collision_edges_in_sub_OPPs.resize(all_index_G0_vertex.size());
	for (int i = 0;i < all_index_G0_vertex.size();i++)
		for (int j = 0;j < all_index_G0_vertex.size();j++) {
			if (i != j && has_edge_G0[all_index_G0_vertex[i]][all_index_G0_vertex[j]] == true)
				edges_in_sub_OPPs[i].push_back(j);
			if (i != j && has_collision_edge_G0[all_index_G0_vertex[i]][all_index_G0_vertex[j]] == true)
				collision_edges_in_sub_OPPs[i].push_back(j);
		}
			

	std::vector<std::vector<int>> temp_edges_in_sub_OPPs = edges_in_sub_OPPs;
	bool jud_node_merge = true;
	int index_group = 0;
	Group.resize(max_node_num);
	while (jud_node_merge == true) {
		jud_node_merge = false;
		for (int i = 0;i < sub_OPPs.size();i++) {
			for (int j = 0;j < sub_OPPs.size();j++) {
				for (int k = 0;k < edges_in_sub_OPPs[i].size();k++) 
					//if (i != j && edges_in_sub_OPPs[i][k] == j && sub_OPPs[i].size() == 1) {
					if (i != j && edges_in_sub_OPPs[i][k] == j && sub_OPPs[i].size() == 1) {
						bool jud_collision = false;
						for (int t = 0;t < collision_edges_in_sub_OPPs[i].size();t++)
							if (collision_edges_in_sub_OPPs[i][t] == j)
								jud_collision = true;
						if (jud_collision == false) {
							////////////////////try merge in inner loop/////////////////////
							try_merge_two_nodes_in_inner_loop(i, j, jud_node_merge, index_group);
							if (jud_node_merge == true)
								break;
						}
						else
							break;
					}
				if (jud_node_merge == true)
					break;

				for(int k =0;k<edges_in_sub_OPPs[j].size();k++)
					if (i != j && edges_in_sub_OPPs[j][k] == i && sub_OPPs[i].size() == 1) {
						bool jud_collision = false;
						for (int t = 0;t < collision_edges_in_sub_OPPs[j].size();t++)
							if (collision_edges_in_sub_OPPs[j][t] == i)
								jud_collision = true;
						if (jud_collision == false) {
							////////////////////try merge in inner loop/////////////////////
							try_merge_two_nodes_in_inner_loop(i, j, jud_node_merge, index_group);
							if (jud_node_merge == true)
								break;
						}
						else
							break;
					}
				if (jud_node_merge == true)
					break;
			}
			if (jud_node_merge == true)
				break;
		}
			
	}
		
	//OPP is judged by monotonicity
	std::vector<std::vector<bool>> exist_layer;
	exist_layer = is_deleted_layer;
	for (int i = 0;i < all_index_G0_vertex.size();i++)
		for (int j = 0;j < initial_opp_info[all_index_G0_vertex[i]].size();j++) {
			exist_layer[data.index[initial_opp_info[all_index_G0_vertex[i]][j]].first][data.index[initial_opp_info[all_index_G0_vertex[i]][j]].second] = true;
		}
	bool is_OPP = true;
	for (int i = 0;i < data.slice_points.size();i++) {
		int cont_undeleted_layer = 0;
		for (int j = 0;j < data.slice_points[i].size();j++)
			if (is_deleted_layer[i][j] == false && exist_layer[i][j] == true)
				cont_undeleted_layer++;
		if (cont_undeleted_layer > 1)
			is_OPP = false;
	}
	return is_OPP;
}

void OPP_Graph::try_merge_two_nodes_in_inner_loop(int i,int j, bool& jud_node_merge, int& index_group)
{
	Group[index_group].clear();
	int index_layer = initial_opp_info[sub_OPPs[i][0]][0];
	Group[index_group].push_back(cv::Point2i(data.index[index_layer].first, data.index[index_layer].second));
	//index_in_GO_vertex.push_back(sub_OPPs[i][0]);


	cv::Point2i last_layer = Group[index_group][0];
	cv::Point2i current_layer;
	cv::Point2i temp_current_layer;
	if (int(initial_opp_info[sub_OPPs[i][0]].size()) >= 2)
		current_layer = cv::Point2i(data.index[initial_opp_info[sub_OPPs[i][0]][1]].first
			, data.index[initial_opp_info[sub_OPPs[i][0]][1]].second);
	else {
		current_layer = last_layer;
		for (int k = 0; k < sub_OPPs.size(); k++) {
			for (int m = 0; m < sub_OPPs[k].size(); m++) {
				if (has_edge_G0[sub_OPPs[k][m]][sub_OPPs[i][0]]) {
					last_layer = cv::Point2i(data.index[initial_opp_info[sub_OPPs[k][m]][initial_opp_info[sub_OPPs[k][m]].size() - 1]].first
						, data.index[initial_opp_info[sub_OPPs[k][m]][initial_opp_info[sub_OPPs[k][m]].size() - 1]].second);
				}
			}
			
		}
	}
		
	temp_current_layer = current_layer;
	int last_layer_terminal = 0;
	//int last_layer_terminal = -1;
	int last_layer_terminal_2 = -3;
	bool jud_merge = true;
	int  iteration = 2;
	///last_layer_terminal_2 = data.slice_points[last_layer.x][last_layer.y].size() - 1;????????????
	bool travers_every_layer = false;
	while (jud_merge == true) {
		if (data.is_contour[last_layer.x][last_layer.y] == true && data.is_contour[current_layer.x][current_layer.y] == true) {
			jud_merge = false;
			break;
		}
		else if (data.is_contour[last_layer.x][last_layer.y] == false && data.is_contour[current_layer.x][current_layer.y] == false) {
			/*if (iteration == 2) {
				last_layer_terminal = 0;
				last_layer_terminal_2 = data.slice_points[last_layer.x][last_layer.y].size() - 1;
			}*/
			//先试一边再试另一边
			if (last_layer_terminal != -2) {
				jud_merge = merge_layers(current_layer, last_layer, index_group, last_layer_terminal);
				if (jud_merge == false) {
					last_layer = Group[index_group][0];
					current_layer = temp_current_layer;
					jud_merge = true;
					iteration = 2;
					if (last_layer_terminal == 0)  //原来是last_layer.y
						last_layer_terminal_2 = data.slice_points[Group[index_group][0].x][Group[index_group][0].y].size() - 1;
					else
						last_layer_terminal_2 = 0;
					last_layer_terminal = -2;
					continue;
				}
			}
			else
				jud_merge = merge_layers(current_layer, last_layer, index_group, last_layer_terminal_2);
		}
		if (jud_merge == false)
			break;
		last_layer = current_layer;

		if (iteration < initial_opp_info[sub_OPPs[i][0]].size())
			current_layer = cv::Point2i(data.index[initial_opp_info[sub_OPPs[i][0]][iteration]].first
				, data.index[initial_opp_info[sub_OPPs[i][0]][iteration]].second);
		else {
			travers_every_layer = true;
			break;
		}
		iteration++;
	}

	if (travers_every_layer == false)   //All layers of the forked part must be removed
		return;

	//delete layers have been merged
	for (int k = 0;k < Group[index_group].size();k++) {
		is_deleted_layer[Group[index_group][k].x][Group[index_group][k].y] = true;
	}

	jud_node_merge = true;
	index_group++;

	//update sub_OPPs && edges_in_sub_OPPs && collision_edges_in_sub_OPPs
	sub_OPPs[j].push_back(sub_OPPs[i][0]);
	std::vector<std::vector<int>>::iterator itor;
	for (int k = 0; k < edges_in_sub_OPPs[i].size(); k++)  
		edges_in_sub_OPPs[j].push_back(edges_in_sub_OPPs[i][k]);
	for (int k = 0; k < collision_edges_in_sub_OPPs[i].size(); k++)
		collision_edges_in_sub_OPPs[j].push_back(collision_edges_in_sub_OPPs[i][k]);


	int cont_itor = 0;
	for (itor = sub_OPPs.begin();itor != sub_OPPs.end();itor++) {
		if (cont_itor == i) {
			sub_OPPs.erase(itor);
			break;
		}
		cont_itor++;
	}
	
	cont_itor = 0;
	for (itor = edges_in_sub_OPPs.begin(); itor != edges_in_sub_OPPs.end(); itor++) {
		if (cont_itor == i) {
			edges_in_sub_OPPs.erase(itor);
			break;
		}
		cont_itor++;
	}
	cont_itor = 0;
	for (itor = collision_edges_in_sub_OPPs.begin(); itor != collision_edges_in_sub_OPPs.end(); itor++) {
		if (cont_itor == i) {
			collision_edges_in_sub_OPPs.erase(itor);
			break;
		}
		cont_itor++;
	}

	for (int n = 0; n < sub_OPPs.size(); n++)
		for (int m = 0; m < edges_in_sub_OPPs[n].size(); m++) {
			if (edges_in_sub_OPPs[n][m] >= i)
				edges_in_sub_OPPs[n][m]--;
			if (edges_in_sub_OPPs[n][m] == i - 1)
				edges_in_sub_OPPs[n][m] = j;
		}
					
	for (int n = 0;n < sub_OPPs.size();n++)
		for (int m = 0; m < collision_edges_in_sub_OPPs[n].size(); m++) {
			if (collision_edges_in_sub_OPPs[n][m] >= i)
				collision_edges_in_sub_OPPs[n][m]--;
			if (collision_edges_in_sub_OPPs[n][m] == i - 1)
				collision_edges_in_sub_OPPs[n][m] = j;
		}
			

	return;
}

bool OPP_Graph::merge_layers(cv::Point2i current_layer, cv::Point2i last_layer, int i, int& last_layer_terminal)
{
	bool jud_merge = true;

	//all contours
	if (last_layer_terminal == -1) {
		for (int i = 0; i < data.slice_points[last_layer.x][last_layer.y].size(); i++) {
			double min_distance = 99999;
			int index_min_point;
			for (int j = 0; j < data.slice_points[current_layer.x][current_layer.y].size(); j++) {
				double distace_points = distance_3d(cv::Point3d(data.slice_points[last_layer.x][last_layer.y][i].x, data.slice_points[last_layer.x][last_layer.y][i].y
					, data.z_value[last_layer.x][last_layer.y][i]), cv::Point3d(data.slice_points[current_layer.x][current_layer.y][j].x,
						data.slice_points[current_layer.x][current_layer.y][j].y, data.z_value[current_layer.x][current_layer.y][j]));
				if (distace_points < min_distance) {
					min_distance = distace_points;
					index_min_point = j;
				}
			}
			cv::Point3d last_point = cv::Point3d(data.slice_points[last_layer.x][last_layer.y][i].x,
				data.slice_points[last_layer.x][last_layer.y][i].y,
				data.z_value[last_layer.x][last_layer.y][i]);
			cv::Point3d current_point = cv::Point3d(data.slice_points[current_layer.x][current_layer.y][index_min_point].x,
				data.slice_points[current_layer.x][current_layer.y][index_min_point].y,
				data.z_value[current_layer.x][current_layer.y][index_min_point]);
			if (jud_allowable_slope(last_point, current_point) == false) {
				jud_merge = false;
				break;
			}
		}
		if (jud_merge == true)
			Group[i].push_back(current_layer);
	}

	//all segments
	else if (last_layer_terminal != -1) {
		cv::Point3d last_point = cv::Point3d(data.slice_points[last_layer.x][last_layer.y][last_layer_terminal].x,
			data.slice_points[last_layer.x][last_layer.y][last_layer_terminal].y,
			data.z_value[last_layer.x][last_layer.y][last_layer_terminal]);
		cv::Point3d last_point_2;
		if(last_layer_terminal == 0)
			last_point_2 = cv::Point3d(data.slice_points[last_layer.x][last_layer.y][data.slice_points[last_layer.x][last_layer.y].size() - 1].x,
				data.slice_points[last_layer.x][last_layer.y][data.slice_points[last_layer.x][last_layer.y].size() - 1].y,
				data.z_value[last_layer.x][last_layer.y][data.slice_points[last_layer.x][last_layer.y].size() - 1]);
		else
			last_point_2 = cv::Point3d(data.slice_points[last_layer.x][last_layer.y][0].x,
				data.slice_points[last_layer.x][last_layer.y][0].y,
				data.z_value[last_layer.x][last_layer.y][0]);
		cv::Point3d current_point_1 = cv::Point3d(data.slice_points[current_layer.x][current_layer.y][0].x,
			data.slice_points[current_layer.x][current_layer.y][0].y,
			data.z_value[current_layer.x][current_layer.y][0]);
		cv::Point3d current_point_2 = cv::Point3d(data.slice_points[current_layer.x][current_layer.y][data.slice_points[current_layer.x][current_layer.y].size() - 1].x,
			data.slice_points[current_layer.x][current_layer.y][data.slice_points[current_layer.x][current_layer.y].size() - 1].y,
			data.z_value[current_layer.x][current_layer.y][data.slice_points[current_layer.x][current_layer.y].size() - 1]);
		if (distance_3d(last_point, current_point_1) < distance_3d(last_point_2, current_point_1)) {
			jud_merge = jud_allowable_slope(last_point, current_point_1);
			if (jud_merge == true) {
				last_layer_terminal = 0;
				Group[i].push_back(current_layer);
			}
		}
		else {
			jud_merge = jud_allowable_slope(last_point, current_point_2);
			if (jud_merge == true) {
				last_layer_terminal = data.slice_points[current_layer.x][current_layer.y].size() - 1;
				Group[i].push_back(current_layer);
			}
		}
	}
	return jud_merge;
}

bool OPP_Graph::jud_allowable_slope(cv::Point3d a, cv::Point3d b)
{
	double tan_slope_angle_value = abs(b.z - a.z) / sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2));
	if (tan_slope_angle_value >0.75)  //0.75  //0.6
		return false;
	else
		return true;
}