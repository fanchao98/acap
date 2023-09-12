#pragma once
#ifndef VISUAL_H_
#define VISUAL_H_

#include "config.h"
#include <vector>

#include <exception>
#include <fstream>


#include "slicer.h"
#include"GeneralMesh.h"
#include<string>
#include <time.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
class Visual
{
public:
    void insert_Line(General_Mesh& mesh, std::vector<Vec3> points, float radius);
    void generateModelForRendering(vector<vector<Vec3>> liens, string file_name);
    void generateModelForRendering_2(const string& file_name);
    void generateModelForRendering_3(vector<vector<Vec3>> the_path, string file_name_txt);
    void generateModelForRendering_4(vector<vector<Vec3>> the_path, string file_name_txt);
    void generateModelForRendering_5(const string& file_name, const string& doc_name);
    void generateModelForRendering_6(std::vector<cv::Point3d> all_points, bool** Dependen_edges);
};
#endif