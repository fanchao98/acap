#pragma once
#ifndef HELPERS_H_
#define HELPERS_H_

#include <math.h>
#include <string>
#include <iostream>

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>

struct nozzle
{
	double upper_surface_r;
	double lowwer_surface_r;
	double nozzle__H_total;
	double nozzle_H_half;
};

const int    maxn = 10000;
const int    MAX_I = 100000000;
const double eps = 1e-10;
const double dependence_offset = 0.5;   //0.5(FDM)     //3.5(ceramic)
const double MAX_D = 1e18;
const double MIN_D = -1e18;
const double dh = 0.2;


//const int direction_samplings = 36;
const int total_directions = 500;
//const int total_directions = (direction_samplings + 1) * (direction_samplings + 1);

//Unit: mm
const double printing_width = 2.5;
//const double support_offset = 0.3 * printing_width;
const double support_offset = 0.5 * printing_width;    //0.3

const std::string file_name = "Results_grail_FDM";
const std::string suffix_stl = ".stl";
const std::string suffix_txt = ".txt";

extern std::vector<std::vector<bool>> is_flatten_area;
extern std::vector<std::pair<int,int>> index_flatten_layer;
inline static cv::Point2d GetNormal(cv::Point2d p1, cv::Point2d p2)
{
	cv::Point2d res;
	res.x = -(p2.y - p1.y);
	res.y = (p2.x - p1.x);
	double norm = std::sqrt(res.dot(res));
	if (norm <= eps) {
		//std::cout << "divide 0 occur !!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
		//std::cout << p1 << " " << p2 << std::endl;
		return res;
	}
	res.x /= norm;
	res.y /= norm;
	return res;
}


inline static bool JudgePointEqual(cv::Point2d p1, cv::Point2d p2)
{
	if (std::abs(p1.x - p2.x) <= eps && std::abs(p1.y - p2.y) <= eps) return true;
	else return false;
}

inline static double Distance2D(cv::Point p1, cv::Point p2) {
	return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}
inline static double Distance3D(cv::Point2d p1, cv::Point2d p2) {
	return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + dh * dh);
}


inline static std::vector<cv::Point2d> ConstructPolygonPoints(const std::vector<cv::Point2d>& points, double offset) {
	cv::Point2d dir;
	std::vector<cv::Point2d> polygon_Points;
	std::vector<cv::Point2d> inside;
	std::vector<cv::Point2d> outside;
	for (int k = 0; k < points.size(); k++) {
		if (k == points.size() - 1) {
			dir = GetNormal(points[points.size() - 2], points[points.size() - 1]);
			inside.push_back(points[points.size() - 1] + dir * offset);
		}
		else {
			dir = GetNormal(points[k], points[k + 1]);
			inside.push_back(points[k] + dir * offset);
		}
	}
	for (int k = points.size() - 1; k >= 0; k--) {
		if (k == 0) {
			dir = GetNormal(points[1], points[0]);
			outside.push_back(points[0] + dir * offset);
		}
		else {
			dir = GetNormal(points[k], points[k - 1]);
			outside.push_back(points[k] + dir * offset);
		}
	}
	polygon_Points.push_back(points[0]);
	for (int k = 0; k < inside.size(); k++) polygon_Points.push_back(inside[k]);
	polygon_Points.push_back(points[points.size() - 1]);
	for (int k = 0; k < outside.size(); k++) polygon_Points.push_back(outside[k]);
	return polygon_Points;
}


inline static std::vector<cv::Point2d> ConstructPolygonPoints_2(std::vector<cv::Point2d>& points, double offset) {
	cv::Point2d dir;
	std::vector<cv::Point2d> polygon_Points;
	std::vector<cv::Point2d> inside;
	std::vector<cv::Point2d> outside;
	for (int k = 0; k < points.size(); k++) {
		if (k == points.size() - 1) {
			dir = GetNormal(points[points.size() - 2], points[points.size() - 1]);
			inside.push_back(points[points.size() - 1] + dir * offset);
		}
		else {
			dir = GetNormal(points[k], points[k + 1]);
			inside.push_back(points[k] + dir * offset);
		}
	}
	for (int k = points.size() - 1; k >= 0; k--) {
		if (k == 0) {
			dir = GetNormal(points[1], points[0]);
			outside.push_back(points[0] + dir * offset);
		}
		else {
			dir = GetNormal(points[k], points[k - 1]);
			outside.push_back(points[k] + dir * offset);
		}
	}
	polygon_Points.push_back(points[0]);
	for (int k = 0; k < inside.size(); k++) polygon_Points.push_back(inside[k]);
	polygon_Points.push_back(points[points.size() - 1]);
	for (int k = 0; k < outside.size(); k++) polygon_Points.push_back(outside[k]);
	return polygon_Points;
}

inline static Eigen::MatrixXd rot(const Eigen::Vector3d& before, const Eigen::Vector3d& after)
{
	Eigen::Vector3d v;
	Eigen::MatrixXd V(3, 3), I(3, 3);
	I.setIdentity();
	v = before.cross(after);
	double s = v.norm();
	double c = before.dot(after);
	if (c == 1.0) return I;
	else if (c == -1.0) return -1 * I;
	V << 0, -v.z(), v.y(),
		v.z(), 0, -v.x(),
		-v.y(), v.x(), 0;
	return (I + V + V * V / (1.0 + c));
}



#endif