#pragma once
#ifndef POLYGON_H
#define POLYGON_H


#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include "helpers.h"

class Polygon
{
public:
	Polygon(const std::vector<cv::Point2d>& points);
	~Polygon();

	bool JudgePointInside(cv::Point2d p);
	void Draw(int index);
private:
	std::vector<cv::Point2d> points;
	double minx, maxx, miny, maxy;
	int find = 0;
};


#endif // !POLYGON_H
