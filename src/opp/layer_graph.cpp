#include "layer_graph.h"

Layer_Graph::Layer_Graph(const Data& data)
{
	this->total_node_num = data.total_node_num;
	this->in_degree.resize(this->total_node_num, 0);
	this->out_degree.resize(this->total_node_num, 0);
	this->node_visited.resize(this->total_node_num, false);
	this->edge_deleted.resize(this->total_node_num * this->total_node_num, false);
	this->data = data;
}

Layer_Graph::~Layer_Graph()
{
}

void Layer_Graph::BuildLayerGraph(nozzle the_nozzle)
{
	clock_t start_time, end_time;
	start_time = clock();
	vector<pair<int, int>> temp_collision_edges;
	vector<pair<int, int>> all_id_layers;
	cv::Point2d dir;
	for (int i = 1; i < data.slice_points.size(); i++) {
		//get (i-1) layer segment's contour
		std::vector<Polygon> Last_layer_polygons;
		for (int j = 0; j < data.slice_points[i - 1].size(); j++) {
			Last_layer_polygons.push_back(Polygon(ConstructPolygonPoints(data.slice_points[i - 1][j], dependence_offset)));
		}
		// build graph
		for (int j = 0; j < data.slice_points[i].size(); j++) {
			for (int m = 0; m < Last_layer_polygons.size(); m++) {
				for (int k = 0; k < data.slice_points[i][j].size(); k++) {
					if (Last_layer_polygons[m].JudgePointInside(data.slice_points[i][j][k])) {
						this->AddEdge(data.index_inv[std::make_pair(i - 1, m)], data.index_inv[std::make_pair(i, j)]);
						temp_edges.push_back(make_pair(data.index_inv[std::make_pair(i - 1, m)], data.index_inv[std::make_pair(i, j)]));
						all_id_layers.push_back(make_pair(i - 1 , i));
						break;
					}
				}
			}
		}
	}
	cont_normal_dependency_edges = temp_edges.size();

	//collision detect, add dependency edge
	for (int i = 0; i < data.slice_points.size(); i++) {
		for (int j = 0; j < data.slice_points[i].size(); j++) {
			for (int ii = i + 1; ii < data.slice_points.size(); ii++) {
				double circle_r;
				if ((ii - i) * dh < the_nozzle.nozzle_H_half)
					circle_r = the_nozzle.lowwer_surface_r + (ii - i) * dh * (the_nozzle.upper_surface_r - the_nozzle.lowwer_surface_r) / the_nozzle.nozzle_H_half;
				else
					circle_r = the_nozzle.upper_surface_r;

				for (int jj = 0; jj < data.slice_points[ii].size(); jj++) {
					bool jud_collision = false;
					if ((ii - i) * dh > the_nozzle.nozzle__H_total) //exceed nozzle_H
					{
						jud_collision = true;
						temp_collision_edges.push_back(make_pair(data.index_inv[std::make_pair(i, j)], data.index_inv[std::make_pair(ii, jj)]));
						continue;
					}
					for (int k = 0; k < data.slice_points[i][j].size(); k+=20) { //step == 2
						for (int kk = 0; kk < data.slice_points[ii][jj].size(); kk+=20) {
							if (pow(data.slice_points[ii][jj][kk].x - data.slice_points[i][j][k].x,2) + pow(data.slice_points[ii][jj][kk].y - data.slice_points[i][j][k].y,2) - pow(circle_r,2) < 0) {
								jud_collision = true;
								break;
							}
						}
						if (jud_collision == true)
							break;
					}
					if (jud_collision == true) {
						temp_collision_edges.push_back(make_pair(data.index_inv[std::make_pair(i, j)], data.index_inv[std::make_pair(ii, jj)]));
					}
				}
			}
		}
	}
	
	//establish hase graph
	bool** dependency_relationship = new bool*[10000];
	for (int i = 0;i < 10000;i++)
		dependency_relationship[i] = new bool[10000];
	for (int i = 0;i < 10000;i++)
		for (int j = 0;j < 10000;j++)
			dependency_relationship[i][j] = false;

	for (int i = 1; i < data.slice_points.size(); i++) {
		for (int j = 0; j < data.slice_points[i].size(); j++) {
			int id_layer = i;
			vector<int> id_current_layer;
			vector<int> temp_id_current_layer;
			id_current_layer.push_back(data.index_inv[std::make_pair(i, j)]);
			while (id_layer > 0) {
				for (int k = 0;k < temp_edges.size();k++) {
					for (int m = 0;m < id_current_layer.size();m++) {
						if (temp_edges[k].second == id_current_layer[m]) {
							dependency_relationship[temp_edges[k].first][data.index_inv[std::make_pair(i, j)]] = true;
							temp_id_current_layer.push_back(temp_edges[k].first);
							break;
						}
					}
				}
				id_layer--;
				id_current_layer = temp_id_current_layer;
				temp_id_current_layer.clear();
			}
		}
	}
	cout << "the number of dependency edges: " << temp_edges.size() << endl;
	for (int i = 0;i < temp_collision_edges.size();i++) {
		if (dependency_relationship[temp_collision_edges[i].first][temp_collision_edges[i].second] == false) {
			dependency_relationship[temp_collision_edges[i].first][temp_collision_edges[i].second] = true;
			for (int j = 0;j < 10000;j++) {   //update
				if (dependency_relationship[temp_collision_edges[i].second][j] == true) {
					dependency_relationship[temp_collision_edges[i].first][j] = true;
				}
			}
			this->AddEdge(temp_collision_edges[i].first, temp_collision_edges[i].second);
			temp_edges.push_back(make_pair(temp_collision_edges[i].first, temp_collision_edges[i].second));
		}
	}
	cout << "the number of edges: " << temp_edges.size()<<endl;
	end_time = clock();
	cout << "&&&&&&& time of establish dependency graph (contain collision detection): " << double(end_time - start_time) / CLOCKS_PER_SEC << "s &&&&&&&" << endl;
}


void Layer_Graph::BuildDependencyGraph(std::vector<cv::Point3d>& all_points)
{
	
	bool** Dependen_edges;
	Dependen_edges = new bool*[3000];
	for (int i = 0;i < 3000;i++)
		Dependen_edges[i] = new bool[3000];
	for (int i = 0;i < 3000;i++)
		for (int j = 0;j < 3000;j++)
			Dependen_edges[i][j] = false;
	std::vector<cv::Point2d> all_2d_points;
	std::vector<Polygon> All_polygons;
	for (int i = 0;i < all_points.size();i++) {
		cv::Point2d temp_point(all_points[i].x, all_points[i].y);
		all_2d_points.push_back(temp_point);

		std::vector<cv::Point2d> temp_2d_point;
		cv::Point2d temp_2d_one_point = all_2d_points[i];
		double r = 0.2;
		//temp_2d_point.push_back(temp_2d_one_point);
		//temp_2d_one_point.x -= r;
		temp_2d_one_point.y -= 2 *r;
		temp_2d_point.push_back(temp_2d_one_point);
		temp_2d_one_point.y += 2*r;
		temp_2d_point.push_back(temp_2d_one_point);
		temp_2d_one_point.y += 2*r;
		temp_2d_point.push_back(temp_2d_one_point);
		//temp_2d_one_point.x += r;
		//temp_2d_one_point.x += r;
		//temp_2d_one_point.y += r;
		
		/*for (double sita = 0;sita < 2 * PI;sita += 1) {
			temp_2d_one_point.x += r * sin(sita);
			temp_2d_one_point.y += r * cos(sita);
			temp_2d_point.push_back(temp_2d_one_point);
			temp_2d_one_point.x -= r * sin(sita);
			temp_2d_one_point.y -= r * cos(sita);
		}*/
		All_polygons.push_back(Polygon(ConstructPolygonPoints_2(temp_2d_point, dependence_offset/4*0.48)));
	}
	
	int cont_edges = 0;
	for (int i = 0;i < all_points.size();i++) {
		for (int j = 0;j < all_points.size();j++) {
			if (i != j && all_points[j].z < all_points[i].z)
				if (All_polygons[j].JudgePointInside(all_2d_points[i])) {
					std::pair<int, int> Pair(i, j);
					Dependen_edges[i][j] = true;
					cont_edges++;
				}
		}
	}
	for (int i = 0;i < 3000;i++) {
		for (int j = 0;j < 3000;j++) {
			if (Dependen_edges[i][j] == true) {
				for (int t = 0;t < 3000;t++) {
					if (t != j && Dependen_edges[i][t] == true) {
						if (Dependen_edges[t][j] == true) {
							Dependen_edges[i][j] = false;
							break;
						}
					}
				}
			}
		}
	}

	ofstream all_edges("D:\\360MoveData\\Users\\zhong\\Desktop\\Others\\Open_Surface_Ceramics_Printing-master\\base_line\\all_edges.txt");
	all_edges << cont_edges << endl;;
	for (int i = 0;i < 3000;i++)
		for (int j = 0;j < 3000;j++)
			if (Dependen_edges[i][j] == true)
				all_edges << i << " " << j << endl;
	Visual Vis;
	Vis.generateModelForRendering_6(all_points, Dependen_edges);
}


void Layer_Graph::GetInitialOPP()
{
	clock_t start_time, end_time;
	start_time = clock();
	//std::cout << this->total_node_num << std::endl;
	int d_num = 0;
	for (int u = 0; u < this->total_node_num; u++) {
		for (int i = 0; i < G[u].size(); i++) {
			int v = this->edges[G[u][i]].GetTo(); d_num++;
			if (in_degree[v] >= 2 || out_degree[u] >= 2) {
				edge_deleted[G[u][i]] = true;
				d_num++;
			}
		}
	}
	std::cout << "d_num: " << d_num << std::endl;
	for (int i = 0; i < this->edges.size(); i++) {
		if (edge_deleted[i]) {
			int u = this->edges[i].GetFrom();
			int v = this->edges[i].GetTo();
			//std::cout << u  << " " << v << std::endl;
			in_degree[v]--;
			out_degree[u]--;
		}
	}
	for (int i = 0; i < this->total_node_num; i++) {
		if ((this->node_visited[i] == false) && (this->in_degree[i] == 0)) {
			//std::cout << i << std::endl;
			this->UpdateDegree(i, -1);
			std::vector<int> initial_opp;
			initial_opp.push_back(i);
			this->node_visited[i] = true;
			DFS(i, initial_opp);
			this->initial_opp_info.push_back(initial_opp);
		}
	}
	end_time = clock();
	std::cout << "***initial opp num***: " << this->initial_opp_info.size() << std::endl << std::endl;
	std::cout << "&&&&&&& time of establish initial opp graph: " << double(end_time - start_time) / CLOCKS_PER_SEC << "s &&&&&&&" << std::endl;
	//std::cout<<"&&&&&&& time of establish initial opp graph(vertex): " << double(end_time - start_time) / CLOCKS_PER_SEC << "s &&&&&&&" << endl;
#if DEFAULT_PRINTING_DIRECTION
	std::cout << "initial opp num: " << this->initial_opp_info.size() << std::endl;
#else

#endif
	this->OutputInitialOpp(file_name + "_initial_opp" + suffix_txt);
}

void Layer_Graph::DFS(int u, std::vector<int>& initial_opp)
{
	for (int i = 0; i < this->G[u].size(); i++) {
		int v = this->edges[G[u][i]].GetTo();
		if (edge_deleted[G[u][i]]) continue;
		if (this->node_visited[v]) continue;
		if (this->in_degree[v] != 0) continue;

		bool f1 = data.is_contour[data.index[u].first][data.index[u].second];
		bool f2 = data.is_contour[data.index[v].first][data.index[v].second];
		if (f1 != f2) {
			/*std::cout << data.index[u].first << " " << data.index[u].second << std::endl;
			std::cout << data.index[v].first << " " << data.index[v].second << std::endl;
			std::cout << *(data.slice_points[data.index[u].first][data.index[u].second].begin()) << std::endl;
			std::cout << *(data.slice_points[data.index[u].first][data.index[u].second].end() - 1) << std::endl;
			std::cout << *(data.slice_points[data.index[v].first][data.index[v].second].begin()) << std::endl;
			std::cout << *(data.slice_points[data.index[v].first][data.index[v].second].end() - 1) << std::endl;
			std::cout << "contour and segment " <<  f1 << " " << f2 << std::endl;*/
			continue;
		}
		this->UpdateDegree(v, -1);
		this->node_visited[v] = true;
		initial_opp.push_back(v);
		DFS(v, initial_opp);
		break;
	}
}

void Layer_Graph::OutputInitialOpp(const std::string& file_name)
{
	std::ofstream dstream(file_name.c_str());
	if (!dstream.is_open()) {
		std::cout << "can not open " << file_name << std::endl;
		return;
	}
	dstream << initial_opp_info.size() << std::endl;
	for (int i = 0; i < initial_opp_info.size(); i++) {
		dstream << initial_opp_info[i].size() << std::endl;
		int m, n;
		for (int j = 0; j < initial_opp_info[i].size(); j++) {
			m = data.index[initial_opp_info[i][j]].first;
			n = data.index[initial_opp_info[i][j]].second;
			dstream << data.slice_points[m][n].size() << std::endl;
			for (int k = 0; k < data.slice_points[m][n].size(); k++) {
				dstream << this->data.slice_points[m][n][k].x << " " << this->data.slice_points[m][n][k].y << " " << data.z_value[m][n][k] << std::endl;
			}
		}
	}
	dstream.close();
}