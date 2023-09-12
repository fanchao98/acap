#include "data.h"

void Data::ReadData(const std::string& file_name)
{
	std::ifstream dstream;
	dstream.open(file_name.c_str(), std::ios_base::in);
	if (!dstream.is_open()) {
		std::cout << "can not open " << file_name << std::endl;
	}
	int total_layer, s_num, p_num;
	double x, y, z;
	dstream >> total_layer;
	this->slice_points.resize(total_layer);
	this->z_value.resize(total_layer);
	this->is_contour.resize(total_layer);
	this->adjacent_points.resize(total_layer);
	this->total_node_num = 0;
	for (int i = 0; i < total_layer; i++) {
		dstream >> s_num;
		this->total_node_num += s_num;
		this->slice_points[i].resize(s_num);
		this->z_value[i].resize(s_num);
		this->is_contour[i].resize(s_num);
		this->adjacent_points[i].resize(s_num);
		for (int j = 0; j < s_num; j++) {
			dstream >> p_num;
			for (int k = 0; k < p_num; k++) {
				dstream >> x >> y >> z;
				this->slice_points[i][j].push_back(cv::Point2d(x, y));
				this->z_value[i][j].push_back(z);
			}
			// avoid the segment only has one point
			if (p_num == 1) {
				this->slice_points[i][j].push_back(cv::Point2d(x + 1, y + 1));
				this->z_value[i][j].push_back(z);
			}
			if (IsContour(i, j)) {

				this->is_contour[i][j] = true;
			}
			this->adjacent_points[i][j].push_back(std::make_pair(cv::Point3d(-1, -1, -1), cv::Point3d(-1, -1, -1)));
			this->adjacent_points[i][j].push_back(std::make_pair(cv::Point3d(-1, -1, -1), cv::Point3d(-1, -1, -1)));
		}
	}
	dstream.close();
	SetIndexMapping();
}

void Data::SetIndexMapping()
{
	int num = 0;
	for (int i = 0; i < slice_points.size(); i++) {
		for (int j = 0; j < slice_points[i].size(); j++) {
			index[num] = std::make_pair(i, j);
			index_inv[std::make_pair(i, j)] = num;
			num++;
		}
	}
}

bool Data::IsContour(int i, int j)
{
	
	cv::Point2d p1 = this->slice_points[i][j][0];
	cv::Point2d p2 = this->slice_points[i][j][this->slice_points[i][j].size() - 1];
	//std::cout << i << " " << j << " " << p1 << " " << p2 << std::endl;
	//if (JudgePointEqual(p1, p2)) return true;
	if (std::abs(p1.x - p2.x) <= 2 && std::abs(p1.y - p2.y) <= 2) return true;
 	else return false;
}
