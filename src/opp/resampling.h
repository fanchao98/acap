#pragma once
#ifndef RE_SAMPLING_
#define RE_SAMPLING_


#include                     <fstream>
#include                    <iostream>
#include <opencv2/imgproc/imgproc.hpp>


class RE_SAMPLING
{
public:
	std::string now_path_file;
	void ReSampling(std::string pre_path_file);
private:
	void AdaptiveSampling(std::vector<cv::Point3d>& input_path, double chord_error);
	double Distance_Point_Segment(cv::Point3d p, cv::Point3d start, cv::Point3d end);
	double Distance_Point_Point(cv::Point3d A, cv::Point3d B);
	double Cross(cv::Point3d A, cv::Point3d B, cv::Point3d C);
	bool IsEqual(cv::Point3d A, cv::Point3d B);
};


#endif // !RE_SAMPLING_