#include "resampling.h"


void RE_SAMPLING::ReSampling(std::string pre_path_file)
{
	this->now_path_file = pre_path_file.substr(0, pre_path_file.size() - 4) + "_resampling.txt";
	std::cout << this->now_path_file << std::endl;
	std::ifstream read;
	std::ofstream write;
	read.open(pre_path_file);
	write.open(this->now_path_file);
	int opp_num, layer_num, point_num;
	double x, y, z, thick;
	read >> opp_num;
	write << opp_num << std::endl;
	for (int k = 0; k < opp_num; k++) {
		read >> layer_num;
		write << layer_num << std::endl;
		for (int i = 0; i < layer_num; i++) {
			read >> point_num;
			std::vector<cv::Point3d> input_path;
			std::vector<cv::Point3d> _input_path;
			std::vector<double> thickness;
			for (int j = 0; j < point_num; j++) {
				read >> x >> y >> z >> thick;
				input_path.push_back(cv::Point3d(x, y, z));
				_input_path.push_back(cv::Point3d(x, y, z));
				thickness.push_back(thick);
			}
			AdaptiveSampling(input_path, 0.001);
			write << input_path.size() << std::endl;
			for (int j = 0; j < input_path.size(); j++) {
				for (int q = 0; q < _input_path.size(); q++) {
					if (IsEqual(_input_path[q], input_path[j])) {
						write << input_path[j].x << " " << input_path[j].y << " " << input_path[j].z  << " " << thickness[q] << std::endl;
						break;
					}
				}
			}
		}
	}
}

void RE_SAMPLING::AdaptiveSampling(std::vector<cv::Point3d>& input_path, double chord_error)
{
	std::vector<cv::Point3d> temp = input_path;
	std::vector<cv::Point3d>().swap(input_path);
	input_path.push_back(temp[0]);

	int start_index = 0;
	for (int i = 1; i < temp.size() - 1; i++)
	{
		int current_index = i;
		for (int j = current_index - 1; j >= start_index + 1; j--)
		{
			double d = Distance_Point_Segment(temp[j], temp[start_index], temp[current_index]);
			if (d > chord_error)
			{
				input_path.push_back(temp[i]);
				start_index = i;
				break;
			}
		}
	}
	input_path.push_back(temp[temp.size() - 1]);
}

double RE_SAMPLING::Distance_Point_Segment(cv::Point3d p, cv::Point3d start, cv::Point3d end)
{
	double res;
	double d1 = Distance_Point_Point(p, start);
	double d2 = Distance_Point_Point(p, end);
	double c1 = Cross(p, start, end);
	double c2 = Cross(p, end, start);
	if (c1 * c2 <= 0) res = std::sqrt(double(std::min(d1, d2)));
	else {
		double d3 = Distance_Point_Point(start, end);
		double s1 = std::sqrt(d1);
		double s2 = std::sqrt(d2);
		double s3 = std::sqrt(d3);
		double c = (s1 + s2 + s3) / 2.0;
		double s = std::sqrt(c * (c - s1) * (c - s2) * (c - s3));
		res = 2 * s / s3;
	}
	return res;
}

double RE_SAMPLING::Distance_Point_Point(cv::Point3d A, cv::Point3d B)
{
	return (A.x - B.x) * (A.x - B.x) + (A.y - B.y) * (A.y - B.y) + (A.z - B.z) * (A.z - B.z);
}

double RE_SAMPLING::Cross(cv::Point3d A, cv::Point3d B, cv::Point3d C)
{
	return (A.x - C.x) * (B.x - C.x) + (A.y - C.y) * (B.y - C.y) + (A.z - C.z) * (B.z - C.z);
}

bool RE_SAMPLING::IsEqual(cv::Point3d A, cv::Point3d B)
{
	int x1 = (int)(A.x * 100);
	int y1 = (int)(A.y * 100);
	int z1 = (int)(A.z * 100);
	int x2 = (int)(B.x * 100);
	int y2 = (int)(B.y * 100);
	int z2 = (int)(B.z * 100);
	if ( (x1 == x2) && (y1 == y2) && (z1 == z2)) return true;
	return false;
}

