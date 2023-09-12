#pragma once
#ifndef DATA_H_
#define DATA_H_


#include       <map>
#include   <fstream>

#include <opencv2/imgproc/imgproc.hpp>

#include "helpers.h"
class Data
{
public:
	void ReadData(const std::string& file_name);

	int total_node_num;
	std::vector<std::vector<std::vector<cv::Point2d>>> slice_points;
	std::vector<std::vector<std::vector<double>>> z_value;
	std::vector<std::vector<bool>> is_contour;
	std::map<int, std::pair<int, int>> index; //1d->2d
	std::map<std::pair<int, int>, int> index_inv; //2d->1d

	std::vector<std::vector<std::vector<std::pair<cv::Point3i, cv::Point3i>>>> adjacent_points;

private:
	void SetIndexMapping();
	bool IsContour(int i, int j);
};

#endif