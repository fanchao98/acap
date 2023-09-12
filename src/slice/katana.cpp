
/*
 * katana, an experimental stl slicer written in C++0x
 *
 * Usage: katana <input.stl> <output.gcode>
 *
 * This program loads a given .stl (stereolithography data, actually triangle data) file
 * and generates a .gcode (RepRap machine instructions) file that can be printed on a RepRap
 * machine.
 *
 * This process is mostly referred as "slicing", as the printers make objects layer by layer,
 * so the 3d model has to be virtually sliced into thin layers. This however is only the first
 * step, after which perimeters and a crosshatch filling pattern are computed for every layer.
 *
 * The .stl file should describe a clean mesh that is a 2-manifold. That means:
 * * every triangle should touch exactly three triangles along its three edges
 * * on every edge, two triangles share exactly two vertices
 * * every triangle has some positive surface area
 *  a triangle may touch further triangles at its vertices.
 *
 *  currently, many checks are disabled to accept objects breaking any of this rules.
 *  the results however are undefined, albeit sometimes usable.
 *
 * in contrast to other slicers, this one does not need:
 * * triangles don't have to carry valid normals
 * * triangle vertices don't need a defined clockwise or counterclockwise order
 *
 * Based on Schlizzer written by Paul Geisler in 2012.
 * https://github.com/dronus/Schlizzer
 */
#include<direct.h>
#include<windows.h> 
#include <stdio.h>
#include <assert.h>
#include <vector>
#include <map>
#include <set>
#include <algorithm>
#include <array>
#include <math.h>
#include <exception>
#include <fstream>

#include "datastructures.h"
#include "config.h"
#include "stl.h"
#include "gcode.h"
#include "slicer.h"
#include "katana.h"
#include"GeneralMesh.h"
#include<string>
#include<ctime>
#include<iomanip>
#include "visual.h"
using namespace std;
using namespace Eigen;
double distance(Vec3 a, Vec3 b);
double distance_2(Vec3 a, Vec3 b);
bool PointCmp(vector<Vec3> layer);
bool compute_angle(Vec3 a, Vec3 b);
void adjust_gcode(vector<vector<vector<Vec3>>>& ALL_OPP);
void connect_by_zigzag_and_spiral(double S,bool choose_spiral,int step, bool is_FDM);


//int main(int argc, const char** argv)
//{
//    argv[1] = "D:\\VS2019 file\\slicer\\Slicer\\Slicer\\stl_files\\12_9_heel.stl";
//    argv[2] = "output.gcode";
//    Katana::Instance().config.loadConfig("config.ini");
//
//    //load the .stl file
//    Katana::Instance().stl.loadStl(argv[1]);
//
//    // create layers and assign touched triangles to them
//    Katana::Instance().slicer.buildLayers();
//
//    // create printable segments for every layer
//    Katana::Instance().slicer.buildSegments();   //�����߶β���������
//
//    //save filled layers in Gcode format
//    Katana::Instance().gcode.write(argv[2]);
//




//
//    //·�����ӻ�
//    //visualizaion VV;
//    //VV.generateModelForRendering(lines,"high_part1");
//
//
//
//    // OPP���ӻ�
//    //visualizaion Vis;
//    //Vis.generateModelForRendering_2();
//    //����gcode
//    connect_by_zigzag_and_spiral();
//
//    return 0;
//}

void change_order(int index_start_point, vector<Vec3> contour, bool is_clockwise)
{

}

vector<vector<int>> choose_connection_point(vector<vector<vector<Vec3>>>& ALL_OPP)
{
    vector<vector<int>> all_optimal_start_index;
    for (int index_OPP = 0;index_OPP < ALL_OPP.size();index_OPP++) {
        std::vector<std::vector<Vec3>> opp = ALL_OPP[index_OPP];
        std::vector<std::vector<Vec3>> choose_points(opp.size());
        for (int i = 0; i < opp.size(); i++) {
            if (distance(opp[i][opp[i].size() - 1], opp[i][0]) < 0.1) {
                choose_points[i] = opp[i];
            }
            else {
                choose_points[i].push_back(opp[i][0]);
                choose_points[i].push_back(opp[i][opp[i].size() - 1]);
            }
        }
        std::vector<std::vector<double>> travel_distance(choose_points.size());
        std::vector<std::vector<int>>  pre_start_num(choose_points.size());

        travel_distance[0].resize(choose_points[0].size(), 0.);
        pre_start_num[0].resize(choose_points[0].size(), -1);
        for (int i = 1; i < choose_points.size(); i++) {
            travel_distance[i].resize(choose_points[i].size(), MAX_D);
            pre_start_num[i].resize(choose_points[i].size());
            for (int j = 0; j < choose_points[i - 1].size(); j++) {
                for (int k = 0; k < choose_points[i].size(); k++) {
                    double dis;
                    if (choose_points[i - 1].size() == 2) dis = distance_2(choose_points[i][k], choose_points[i - 1][!j]);
                    else dis = distance_2(choose_points[i][k], choose_points[i - 1][j]);
                    if (travel_distance[i][k] > travel_distance[i - 1][j] + dis) {
                        travel_distance[i][k] = travel_distance[i - 1][j] + dis;
                        pre_start_num[i][k] = j;
                    }
                }
            }
        }

        int true_optimal_start_index;
        int false_optimal_start_index;
        double _min = MAX_D;
        for (int i = 0; i < choose_points[choose_points.size() - 1].size(); i++) {
            if (_min > travel_distance[choose_points.size() - 1][i]) {
                _min = travel_distance[choose_points.size() - 1][i];
                false_optimal_start_index = i;
                true_optimal_start_index = false_optimal_start_index;
                if (choose_points[choose_points.size() - 1].size() == 2) {
                    true_optimal_start_index = (i == 0 ? 0 : opp[choose_points.size() - 1].size() - 1);
                }
            }
        }
        std::vector<int> optimal_start_index;
        optimal_start_index.push_back(true_optimal_start_index);
        for (int i = choose_points.size() - 1; i >= 1; i--) {
            false_optimal_start_index = pre_start_num[i][false_optimal_start_index];
            true_optimal_start_index = false_optimal_start_index;
            if (choose_points[i - 1].size() == 2) {
                true_optimal_start_index = (false_optimal_start_index == 0 ? 0 : opp[i - 1].size() - 1);
            }
            optimal_start_index.push_back(true_optimal_start_index);
        }
        std::reverse(optimal_start_index.begin(), optimal_start_index.end());

        all_optimal_start_index.push_back(optimal_start_index);
    }
    return all_optimal_start_index;
}

void output_ori_path(std::string file_path, vector<vector<vector<Vec3>>>& ALL_OPP, vector<vector<int>>& all_optimal_start_index, vector<vector<vector<double>>> ALL_H)
{
    ofstream output_ALL_OPPs(file_path);
    output_ALL_OPPs << ALL_OPP.size() << endl;
    for (int i = 0; i < ALL_OPP.size(); i++) {
        output_ALL_OPPs << ALL_OPP[i].size() << endl;
        vector<vector<Vec3>> OPP = ALL_OPP[i];
        for (int j = 0; j < OPP.size(); j++) {
            output_ALL_OPPs << OPP[j].size() << endl;

            int num_next_layer_points = OPP[j].size();
            int num_current_layer_points = OPP[j].size();

            if (distance(OPP[j][num_next_layer_points - 1], OPP[j][0]) < 0.1)
            {
                if (PointCmp(OPP[j]) == true)
                {
                    int k = all_optimal_start_index[i][j];
                    while (1)
                    {
                        output_ALL_OPPs << setiosflags(ios::fixed) << setprecision(3) << OPP[j][k].m_x << " " << OPP[j][k].m_y << " " << OPP[j][k].m_z <<" " <<ALL_H[i][j][k]<< endl;
                        k = (k + 1) % OPP[j].size();
                        if (k == all_optimal_start_index[i][j])
                            break;
                    }
                }
                else
                {
                    int k = all_optimal_start_index[i][j];
                    while (1)
                    {
                        output_ALL_OPPs << setiosflags(ios::fixed) << setprecision(3) << OPP[j][k].m_x << " " << OPP[j][k].m_y << " " << OPP[j][k].m_z << " " << ALL_H[i][j][k] << endl;
                        k = (k - 1 + OPP[j].size()) % OPP[j].size();
                        if (k == all_optimal_start_index[i][j])
                            break;
                    }
                }
            }
            else
            {
                double nozzle_w = 6;
                vector<Vec3> temp_OPP = OPP[j];
                if (all_optimal_start_index[i][j] == 0)  //Print in the original order
                    for (int k = 0; k < OPP[j].size(); k++) {
                        //Increase height when descend
                        /*if (k != 0 && temp_OPP[k].m_z < temp_OPP[k - 1].m_z) {
                            double tan_sita = abs(double(temp_OPP[k].m_z - temp_OPP[k - 1].m_z) / distance_2(temp_OPP[k], temp_OPP[k - 1]));
                            ALL_H[i][j][k] *= (1 - tan_sita);
                            OPP[j][k].m_z += (nozzle_w * tan_sita) / 2;
                            if ((nozzle_w * tan_sita) / 2 > 3)
                                cout << (nozzle_w * tan_sita) / 2 << " ";

                        }*/
                        output_ALL_OPPs << setiosflags(ios::fixed) << setprecision(3) << OPP[j][k].m_x << " " << OPP[j][k].m_y << " " << OPP[j][k].m_z << " " << ALL_H[i][j][k] << endl;
                    }
                        
                else
                    for (int k = OPP[j].size() - 1; k >= 0; k--) {
                        //Increase height when descend
                        /*if (k != OPP[j].size() - 1 && temp_OPP[k].m_z < temp_OPP[k + 1].m_z) {
                            double tan_sita = abs(double(temp_OPP[k].m_z - temp_OPP[k + 1].m_z) / distance_2(temp_OPP[k], temp_OPP[k + 1]));
                            ALL_H[i][j][k] *= (1 - tan_sita);
                            OPP[j][k].m_z += (nozzle_w * tan_sita) / 2;
                            if ((nozzle_w * tan_sita) / 2 > 3)
                                cout << (nozzle_w * tan_sita) / 2 << " ";
                        }*/
                        output_ALL_OPPs << setiosflags(ios::fixed) << setprecision(3) << OPP[j][k].m_x << " " << OPP[j][k].m_y << " " << OPP[j][k].m_z << " " << ALL_H[i][j][k] << endl;
                    }
            }
        }
    }
}

void get_resample_path(vector<vector<vector<Vec3>>>& ALL_OPP, vector<vector<vector<double>>>& ALL_H)
{
    vector<vector<int>> all_optimal_start_index = choose_connection_point(ALL_OPP);
    output_ori_path("../Blocks_files/" + file_name + "_path.txt", ALL_OPP, all_optimal_start_index, ALL_H);
    
    RE_SAMPLING resample_path;
    resample_path.ReSampling("../Blocks_files/" + file_name + "_path.txt");

    ALL_OPP.clear();
    ALL_H.clear();
    int num_OPP;
    ifstream input_ALL_OPPs("../Blocks_files/" + file_name + "_path_resampling.txt");
    input_ALL_OPPs >> num_OPP;
    ALL_OPP.resize(num_OPP);
    ALL_H.resize(num_OPP);
    for (int i = 0;i < num_OPP;i++) {
        int num_layer;
        input_ALL_OPPs >> num_layer;
        ALL_OPP[i].resize(num_layer);
        ALL_H[i].resize(num_layer);
        for (int j = 0;j < num_layer;j++) {
            int num_points;
            input_ALL_OPPs >> num_points;
            ALL_OPP[i][j].resize(num_points);
            ALL_H[i][j].resize(num_points);
            for (int k = 0;k < num_points;k++) {
                input_ALL_OPPs >> ALL_OPP[i][j][k].m_x >> ALL_OPP[i][j][k].m_y >> ALL_OPP[i][j][k].m_z >> ALL_H[i][j][k];
            }
            if (distance(ALL_OPP[i][j][0], ALL_OPP[i][j][ALL_OPP[i][j].size() - 1]) < 1) {
                Vec3 temp_point = ALL_OPP[i][j][0];
                double temp_H = ALL_H[i][j][0];
                ALL_OPP[i][j].push_back(temp_point);
                ALL_H[i][j].push_back(temp_H);
            }
        }
    }
}

void adjust_gcode(vector<vector<vector<Vec3>>>& ALL_OPP,double enlarge_mul)
{
    Vec3 center_point;
    center_point.m_x = center_point.m_y = 0;
    double min_z = 999999;
    int cont_point = 0;
    for (int i = 0; i < ALL_OPP.size(); i++)
        for (int j = 0; j < ALL_OPP[i].size(); j++)
            for (int k = 0; k < ALL_OPP[i][j].size(); k++)
            {
                center_point.m_x += ALL_OPP[i][j][k].m_x;
                center_point.m_y += ALL_OPP[i][j][k].m_y;
                cont_point++;
                if (ALL_OPP[i][j][k].m_z < min_z)
                    min_z = ALL_OPP[i][j][k].m_z;
            }

    center_point.m_x /= cont_point;
    center_point.m_y /= cont_point;

    for (int i = 0; i < ALL_OPP.size(); i++)
        for (int j = 0; j < ALL_OPP[i].size(); j++)
            for (int k = 0; k < ALL_OPP[i][j].size(); k++)
            {
                ALL_OPP[i][j][k].m_x -= center_point.m_x;
                ALL_OPP[i][j][k].m_y -= center_point.m_y;
                ALL_OPP[i][j][k].m_z -= min_z;
            }
    /*for (int i = 0; i < ALL_OPP.size(); i++)
        for (int j = 0; j < ALL_OPP[i].size(); j++)
            for (int k = 0; k < ALL_OPP[i][j].size(); k++)
            {
                double temp = ALL_OPP[i][j][k].m_y;
                ALL_OPP[i][j][k].m_y = ALL_OPP[i][j][k].m_z;
                ALL_OPP[i][j][k].m_z = -temp;
            }*/
    /*for (int i = 0; i < ALL_OPP.size(); i++)
        for (int j = 0; j < ALL_OPP[i].size(); j++)
            for (int k = 0; k < ALL_OPP[i][j].size(); k++)
            {
                ALL_OPP[i][j][k].m_x *= enlarge_mul;
                ALL_OPP[i][j][k].m_y *= enlarge_mul;
                ALL_OPP[i][j][k].m_z *= enlarge_mul;
            }*/
    //for (int i = 0; i < ALL_OPP.size(); i++)
    //    for (int j = 0; j < ALL_OPP[i].size(); j++)
    //        for (int k = 0; k < ALL_OPP[i][j].size(); k++)
    //        {
    //            Vec3 temp_point = ALL_OPP[i][j][k];
    //            ALL_OPP[i][j][k].m_x = (temp_point.m_x * cos(3.14 / 1.6) - temp_point.m_y * sin(3.14 / 1.6));
    //            ALL_OPP[i][j][k].m_y = (temp_point.m_x * sin(3.14 / 1.6) + temp_point.m_y * cos(3.14 / 1.6));
    //            //ALL_OPP[i][j][k].m_z *= enlarge_mul;
    //        }
    for (int i = 0; i < ALL_OPP.size(); i++)
        for (int j = 0; j < ALL_OPP[i].size(); j++)
            for (int k = 0; k < ALL_OPP[i][j].size(); k++)
            {
                ALL_OPP[i][j][k].m_x += 150;
                ALL_OPP[i][j][k].m_y += 150;
                ALL_OPP[i][j][k].m_z += 0.2;
            }

    /*vector<vector<vector<Vec3>>> temp_ALL_OPP = ALL_OPP;
    for (int k = 8; k < 12; k++)
    {
        double offset = 0.5;
        if(k<12 && k>10)
            offset = -offset;
        double kk = (-(temp_ALL_OPP[0][30][k].m_x - temp_ALL_OPP[0][30][k - 1].m_x)/(temp_ALL_OPP[0][30][k].m_y - temp_ALL_OPP[0][30][k - 1].m_y)
            - (temp_ALL_OPP[0][30][k].m_x - temp_ALL_OPP[0][30][k + 1].m_x) / (temp_ALL_OPP[0][30][k].m_y - temp_ALL_OPP[0][30][k + 1].m_y))/2;
        ALL_OPP[0][30][k].m_x += offset * cos(atan(kk));
        ALL_OPP[0][30][k].m_y += offset * sin(atan(kk));
    }
    for (int k = 260; k < 263; k++)
    {
        double offset = 0.5;
        double kk = (-(temp_ALL_OPP[0][30][k].m_x - temp_ALL_OPP[0][30][k - 1].m_x) / (temp_ALL_OPP[0][30][k].m_y - temp_ALL_OPP[0][30][k - 1].m_y)
            - (temp_ALL_OPP[0][30][k].m_x - temp_ALL_OPP[0][30][k + 1].m_x) / (temp_ALL_OPP[0][30][k].m_y - temp_ALL_OPP[0][30][k + 1].m_y)) / 2;
        ALL_OPP[0][30][k].m_x += -offset * cos(atan(kk));
        ALL_OPP[0][30][k].m_y += -offset * sin(atan(kk));
    }*/

    //for (int i = 0; i < ALL_OPP.size(); i++)
    //    for (int j = 0; j < ALL_OPP[i].size(); j++)
    //        for (int k = 0; k < ALL_OPP[i][j].size(); k++)
    //        {
    //            Vec3 temp_point = ALL_OPP[i][j][k];
    //            ALL_OPP[i][j][k].m_z = (temp_point.m_z * cos(3.14 / 4.0) - temp_point.m_x * sin(3.14 / 4.0));
    //            ALL_OPP[i][j][k].m_x = (temp_point.m_z * sin(3.14 / 4.0) + temp_point.m_x * cos(3.14 / 4.0));
    //            //ALL_OPP[i][j][k].m_z *= enlarge_mul;
    //        }


    /////////////////////////compensate//////////////////////
    /*for (int i = 0; i < ALL_OPP[1].size(); i++) {
        for (int k = 0; k < ALL_OPP[1][i].size(); k++) {
            ALL_OPP[1][i][k].m_x -= 0.2;
        }
    }
    for (int i = 0; i < ALL_OPP[2].size(); i++) {
        for (int k = 0; k < ALL_OPP[2][i].size(); k++) {
            ALL_OPP[2][i][k].m_x += 0.3;
        }
    }
    for (int i = 0; i < ALL_OPP[3].size(); i++) {
        for (int k = 0; k < ALL_OPP[3][i].size(); k++) {
            ALL_OPP[3][i][k].m_x += 0.5;
        }
    }
    for (int i = 0; i < ALL_OPP[4].size(); i++) {
        for (int k = 0; k < ALL_OPP[4][i].size(); k++) {
            ALL_OPP[4][i][k].m_x += 0.6;
        }
    }
    for (int i = 0; i < ALL_OPP[5].size(); i++) {
        for (int k = 0; k < ALL_OPP[5][i].size(); k++) {
            ALL_OPP[5][i][k].m_x -= 0.2;
        }
    }
    for (int i = 0; i < ALL_OPP[6].size(); i++) {
        for (int k = 0; k < ALL_OPP[6][i].size(); k++) {
            ALL_OPP[6][i][k].m_x -= 0.6;
        }
    }*/
    /*for (int i = 0; i < ALL_OPP[7].size(); i++) {
        for (int k = 0; k < ALL_OPP[7][i].size(); k++) {
            ALL_OPP[7][i][k].m_x += 0.2;
            ALL_OPP[7][i][k].m_y += 0.2;
        }
    }
    for (int i = 0; i < ALL_OPP[8].size(); i++) {
        for (int k = 0; k < ALL_OPP[8][i].size(); k++) {
            ALL_OPP[8][i][k].m_x -= 0.2;
        }
    }
    for (int i = 0; i < ALL_OPP[9].size(); i++) {
        for (int k = 0; k < ALL_OPP[9][i].size(); k++) {
            ALL_OPP[9][i][k].m_x += 0.3;
        }
    }
    for (int i = 0; i < ALL_OPP[10].size(); i++) {
        for (int k = 0; k < ALL_OPP[10][i].size(); k++) {
            ALL_OPP[10][i][k].m_x += 0.7;
        }
    }
    for (int i = 0; i < ALL_OPP[11].size(); i++) {
        for (int k = 0; k < ALL_OPP[11][i].size(); k++) {
            ALL_OPP[11][i][k].m_x += 0.7;
        }
    }
    for (int i = 0; i < ALL_OPP[12].size(); i++) {
        for (int k = 0; k < ALL_OPP[12][i].size(); k++) {
            ALL_OPP[12][i][k].m_x += 0.7;
        }
    }
    for (int i = 0; i < ALL_OPP[13].size(); i++) {
        for (int k = 0; k < ALL_OPP[13][i].size(); k++) {
            ALL_OPP[13][i][k].m_x += 0.7;
        }
    }
    for (int i = 0; i < ALL_OPP[14].size(); i++) {
        for (int k = 0; k < ALL_OPP[14][i].size(); k++) {
            ALL_OPP[14][i][k].m_x += 0.7;
        }
    }*/
    ////////////////////////////////////////////////////////
}

vector<Vec3> generate_pull_path(int index_relay_point,vector<Vec3>ori_path,int index_terminal_point, bool is_start_layer, vector<bool>& jud_reduce_extrusion)
{
    vector<Vec3> pull_path;
    double offset;
    if (is_start_layer)
        offset = dh / 2;
    else
        offset = -dh / 2;
    if (distance(ori_path[ori_path.size() - 1], ori_path[0]) < 0.1) {
        /*if(is_start_layer == true)
            swap(index_relay_point, index_terminal_point);*/
        double sum_dis_1=0, sum_dis_2=0;
        int k = (index_terminal_point) % ori_path.size();
        while (1) {
            int last = (k + ori_path.size() - 1) % ori_path.size();
            double dis = distance(ori_path[k], ori_path[last]);
            sum_dis_1 += dis;
            k = (k + 1) % ori_path.size();
            if (k == index_relay_point)
                break;
        }
        k = (index_terminal_point + ori_path.size()) % ori_path.size();
        while (1) {
            int last = (k + 1) % ori_path.size();
            double dis = distance(ori_path[k], ori_path[last]);
            sum_dis_2 += dis;
            k = (k + ori_path.size() - 1) % ori_path.size();
            if (k == index_relay_point)
                break;
        }
        if (sum_dis_1 < sum_dis_2) {
            k = (index_terminal_point) % ori_path.size();
            while (1) {
                pull_path.push_back(Vec3(ori_path[k].m_x, ori_path[k].m_y, ori_path[k].m_z + offset));
                if (!is_start_layer)
                    jud_reduce_extrusion[k] = true;
                k = (k + 1) % ori_path.size();
                if (k == index_relay_point)
                    break;
            }
        }
        else {
            k = (index_terminal_point + ori_path.size()) % ori_path.size();
            while (1) {
                pull_path.push_back(Vec3(ori_path[k].m_x, ori_path[k].m_y, ori_path[k].m_z + offset));
                if (!is_start_layer)
                    jud_reduce_extrusion[k] = true;
                k = (k + ori_path.size() - 1) % ori_path.size();
                if (k == index_relay_point)
                    break;
            }
        }
    }
    else {
        if (is_start_layer) {
            if (index_terminal_point == 0) {
                for (int i = ori_path.size() - 1;i >= index_relay_point;i--) {
                    pull_path.push_back(Vec3(ori_path[i].m_x, ori_path[i].m_y, ori_path[i].m_z + offset));
                }
            }
            else {
                for (int i = 0;i <= index_relay_point;i++) {
                    pull_path.push_back(Vec3(ori_path[i].m_x, ori_path[i].m_y, ori_path[i].m_z + offset));
                }
            }
        }
        else {
            if (index_terminal_point == 0) {
                for (int i = 0;i <= index_relay_point;i++) {
                    pull_path.push_back(Vec3(ori_path[i].m_x, ori_path[i].m_y, ori_path[i].m_z + offset));
                    if (!is_start_layer)
                        jud_reduce_extrusion[i] = true;
                }
            }
            else {
                for (int i = ori_path.size() - 1;i >= index_relay_point;i--) {
                    pull_path.push_back(Vec3(ori_path[i].m_x, ori_path[i].m_y, ori_path[i].m_z + offset));
                    if (!is_start_layer)
                        jud_reduce_extrusion[i] = true;
                }
            }
        }
    }
    return pull_path;
}

void spiral_for_contours(vector<vector<Vec3>>& OPP, vector<vector<int>> all_optimal_start_index, int i, int j, bool& jud_spiral,
    vector<vector<vector<double>>>& ALL_H, bool jud_clockwise)
{
    if (j != OPP.size() - 1 && j + 1 < OPP.size() && distance(OPP[j + 1][OPP[j + 1].size() - 1], OPP[j + 1][0]) < 0.1)
    {
        bool jud_faraway = false;
        int k = all_optimal_start_index[i][j];
        while (1)
        {
            //Find the two sampling points in the upper layer that are closest to each other horizontally
            int index_nearest, index_nearest_2;
            double num_nearest = 99999, num_nearest_2 = 99999;
            for (int n = 0; n < OPP[j + 1].size(); n++)
            {
                if (distance_2(OPP[j][k], OPP[j + 1][n]) < num_nearest)
                {
                    num_nearest = distance_2(OPP[j][k], OPP[j + 1][n]);
                    index_nearest = n;
                }
            }
            for (int n = 0; n < OPP[j + 1].size(); n++)
            {
                if (distance_2(OPP[j][k], OPP[j + 1][n]) < num_nearest_2 && n != index_nearest)
                {
                    num_nearest_2 = distance_2(OPP[j][k], OPP[j + 1][n]);
                    index_nearest_2 = n;
                }
            }
            if ((num_nearest + num_nearest_2) / 2 > 4) {
                jud_faraway = true;
                jud_spiral = false;
                break;
            }
            if (jud_clockwise)
                k = (k + 1) % OPP[j].size();
            else
                k = (k - 1 + OPP[j].size()) % OPP[j].size();
            if (k == all_optimal_start_index[i][j])
                break;
        }

        if (jud_faraway == false) {
            double current_mul;
            int k = all_optimal_start_index[i][j];
            int cont_point = 0;
            while (1)
            {
                /*int t = k;
                double distance_connection_point = distance_2(OPP[j][k], OPP[j + 1][all_optimal_start_index[i][j + 1]]);
                int index_same_layer_nearest_1, index_same_layer_nearest_2, real_index_same_layer_nearest;
                while (1) {
                    t = (t + 1) % OPP[j].size();
                    if (abs(distance_connection_point - distance_2(OPP[j][k], OPP[j][t])) < 1)
                        index_same_layer_nearest_1 = t;
                    if (t == k)
                        break;
                }
                while (1) {
                    t = (t - 1 + OPP[j].size()) % OPP[j].size();
                    if (abs(distance_connection_point - distance_2(OPP[j][k], OPP[j][t])) < 1)
                        index_same_layer_nearest_2 = t;
                    if (t == k)
                        break;
                }
                if (distance_2(OPP[j][index_same_layer_nearest_1], OPP[j + 1][all_optimal_start_index[i][j + 1]])
                    < distance_2(OPP[j][index_same_layer_nearest_2], OPP[j + 1][all_optimal_start_index[i][j + 1]])) {
                    real_index_same_layer_nearest = index_same_layer_nearest_1;
                }
                else
                    real_index_same_layer_nearest = index_same_layer_nearest_2;*/
                    //Find the two sampling points in the upper layer that are closest to each other horizontally
                    //int temp_k = real_index_same_layer_nearest;
                int temp_k = k;
                int index_nearest, index_nearest_2;
                double num_nearest = 99999, num_nearest_2 = 99999;
                for (int n = 0; n < OPP[j + 1].size(); n++)
                {
                    if (distance_2(OPP[j][temp_k], OPP[j + 1][n]) < num_nearest)
                    {
                        num_nearest = distance_2(OPP[j][temp_k], OPP[j + 1][n]);
                        index_nearest = n;
                    }
                }
                for (int n = 0; n < OPP[j + 1].size(); n++)
                {
                    if (distance_2(OPP[j][temp_k], OPP[j + 1][n]) < num_nearest_2 && n != index_nearest)
                    {
                        num_nearest_2 = distance_2(OPP[j][temp_k], OPP[j + 1][n]);
                        index_nearest_2 = n;
                    }
                }
                double diff_x = (OPP[j + 1][index_nearest].m_x + OPP[j + 1][index_nearest_2].m_x) / 2 - OPP[j][k].m_x;
                double diff_y = (OPP[j + 1][index_nearest].m_y + OPP[j + 1][index_nearest_2].m_y) / 2 - OPP[j][k].m_y;
                double diff_z = (OPP[j + 1][index_nearest].m_z + OPP[j + 1][index_nearest_2].m_z) / 2 - OPP[j][k].m_z;
                current_mul = cont_point / double(OPP[j].size());
                OPP[j][k].m_x += current_mul * diff_x;
                OPP[j][k].m_y += current_mul * diff_y;
                OPP[j][k].m_z += current_mul * diff_z;
                if (j == 0)
                    ALL_H[i][j][k] = (cont_point / double(OPP[j].size())) * dh / 2 + dh;
                if (jud_clockwise)
                    k = (k + 1) % OPP[j].size();
                else
                    k = (k - 1 + OPP[j].size()) % OPP[j].size();
                temp_k = (temp_k + 1) % OPP[j].size();
                cont_point++;
                if (k == all_optimal_start_index[i][j])
                    break;
            }
            jud_spiral = true;
        }
    }
    if (((j == OPP.size() - 1 && jud_spiral == true) || (j != OPP.size() - 1 && jud_spiral == false)) && distance(OPP[j][OPP[j].size() - 1], OPP[j][0]) < 0.1)
    {
        double dis_up_and_low_z = dh / 2;
        int k = all_optimal_start_index[i][j];
        int cont_point = 0;
        double mul = 0.8;
        while (1) {
            OPP[j][k].m_z += dis_up_and_low_z / (OPP[j].size()) * cont_point;
            ALL_H[i][j][k] = (1 - cont_point / OPP[j].size()) * dis_up_and_low_z + dis_up_and_low_z;
            ALL_H[i][j][k] *= mul;
            if (jud_clockwise)
                k = (k + 1) % OPP[j].size();
            else
                k = (k - 1 + OPP[j].size()) % OPP[j].size();
            cont_point++;
            if (k == all_optimal_start_index[i][j])
                break;
        }
        jud_spiral = false;
    }
    else if (j == 0 && jud_spiral == true && distance(OPP[j][OPP[j].size() - 1], OPP[j][0]) < 0.1)
    {
        //double dis_up_and_low_z = dh / 2;
        int k = all_optimal_start_index[i][j];
        int cont_point = 0;
        double mul = 0.8;
        while (1) {
            //OPP[j][k].m_z -= dis_up_and_low_z - (dis_up_and_low_z / (OPP[j].size()) * cont_point);
            ALL_H[i][j][k] = cont_point / OPP[j].size() * dh + dh;
            ALL_H[i][j][k] *= mul;
            if (jud_clockwise)
                k = (k + 1) % OPP[j].size();
            else
                k = (k - 1 + OPP[j].size()) % OPP[j].size();
            cont_point++;
            if (k == all_optimal_start_index[i][j])
                break;
        }
    }
    else
        jud_spiral = false;
}

void connect_by_zigzag_and_spiral(double S,bool choose_spiral,int step, bool is_FDM)
{
    clock_t start_time, end_time;
    start_time = clock();
    bool open_adjust_z = false;
    //string file_name_txt = "1_optimal";
    int num_block = 0;
    ifstream ifile;
    vector<vector<vector<Vec3>>> ALL_OPP;
    vector<vector<vector<double>>> ALL_H;
    vector<Vec3> last_block_start_point;
    while (true) {
        int cont_step = -1;
        string str_num = to_string(num_block);
        ifile.open("../Blocks_files/" + file_name + "_block" + str_num + ".txt");
        if (!ifile.is_open())
            break;
        num_block++;
        vector<vector<Vec3>> temp_v;
        vector<vector<double>> temp_H;
        ALL_OPP.push_back(temp_v);
        ALL_H.push_back(temp_H);
        int num_seg;
        ifile >> num_seg;
        for (int j = 0; j < num_seg; j++)
        {
            //if (j != num_seg - 1)
                cont_step = (cont_step + 1) % step;
            //else
                //cont_step = 0;
            if (cont_step == 0) {
                vector<Vec3> temp_vv;
                vector<double> temp_hh;
                ALL_OPP[ALL_OPP.size() - 1].push_back(temp_vv);
                ALL_H[ALL_H.size() - 1].push_back(temp_hh);
            }
            int num_points;
            ifile >> num_points;
            for (int k = 0; k < num_points; k++)
            {
                Vec3 the_point;
                double the_H;
                if (cont_step == 0) {
                    ifile >> the_point.m_x >> the_point.m_y >> the_point.m_z >> the_H;
                    ALL_OPP[ALL_OPP.size() - 1][ALL_OPP[ALL_OPP.size() - 1].size() - 1].push_back(the_point);
                    ALL_H[ALL_H.size() - 1][ALL_H[ALL_H.size() - 1].size() - 1].push_back(the_H);
                    if (j == 0 && k == 0)
                        last_block_start_point.push_back(the_point);
                }
                else
                    ifile >> the_point.m_x >> the_point.m_y >> the_point.m_z >> the_H;
            }
        }
        ifile.close();
    }

    //get_resample_path(ALL_OPP, ALL_H);

    vector<vector<int>> all_optimal_start_index = choose_connection_point(ALL_OPP);
    /*all_optimal_start_index[0][0] = 0;
    all_optimal_start_index[0][1] = 72;
    all_optimal_start_index[0][2] = 0;
    all_optimal_start_index[0][3] = 50;
    all_optimal_start_index[0][4] = 0;*/

    //adjust gcode
    double enlarge_mul = 1;
    adjust_gcode(ALL_OPP, enlarge_mul);

        /*for (int j = 0; j < ALL_OPP[10].size(); j++) {
            if (ALL_H[10][j][0] != 0.2) {
                for (int k = 0; k < ALL_OPP[10][j].size(); k++)
                    ALL_OPP[10][j][k].m_z -= 0.2;
                cout << "dd";
            }
        }*/


    //compute highest point
    double highest_point = 0;
    /*for (int i = 0; i < ALL_OPP.size(); i++)
        for (int j = 0;j < ALL_OPP[i].size();j++)
            for (int k = 0;k < ALL_OPP[i][j].size();k++)
                if (ALL_OPP[i][j][k].m_z > highest_point)
                    highest_point = ALL_OPP[i][j][k].m_z;*/

    ofstream gcode("../Blocks_files/" + file_name + "_path.gcode");
    double sum_E = 0;
    double E_para;
    int G1_speed,G0_speed;
    if (is_FDM) {
        E_para = 0.4;  //0.7  //0.4
        G1_speed = 500;  //800  //400  /500
        G0_speed = 833;  //1000 //833
    }
    else {
        E_para = 1.8;
        G1_speed = 1500;
        G0_speed = 2500;
    }
       
    if (is_FDM == true) {
        gcode << "G21" << endl
            << "G90" << endl
            << "M82" << endl
            << "M107" << endl
            << "M140 S40" << endl
            << "M104 T0 S180" << endl
            << "G28 X0 Y0" << endl
            << "M109 T0 S180" << endl
            << "G28 Z0" << endl
            << "G1 F600 X0 Y20" << endl
            << "M104 T0 S180" << endl
            << "G1 Z10.0" << endl
            << "G1 F3600 X10 Y0" << endl
            << "G28 Z0" << endl
            << "G1 Z10.0" << endl
            << "G1 X110 Y0" << endl
            << "M190 S40" << endl
            << "M109 T0 S180" << endl
            << "G1 Z0.2" << endl
            << "T0" << endl
            << "G92 E0" << endl
            << "M117 Printing..." << endl
            << "M104 S180" << endl
            << "G1 F2400 E -4.00000" << endl
            << "G0 F3600 X75.241 Y39.107" << endl
            << "G0 Z0.200" << endl
            << "G1 F2400 E0.00000" << endl;
    }
    else {
        gcode << "G21 ;metric value" << endl
            << "G90;absolute positioning" << endl
            << "M82;set extruder to absolute mode" << endl
            << "G28 X0 Y0;move X / Y to min endstops" << endl
            << "G28 Z0;move Z to min endstops" << endl
            << "G92 E0;zero the extruded length" << endl
            << "G1 F200 E3;extrude 3mm of feed stock" << endl
            << "G92 E0;zero the extruded length again" << endl
            << "M302" << endl;
    }
    
    vector<vector<Vec3>>the_path;
    vector<vector<double>>the_path_H;
    vector<vector<vector<bool>>> jud_reduce_extrusion;
    jud_reduce_extrusion.resize(ALL_OPP.size());
    for (int i = 0; i < ALL_OPP.size(); i++)
    {
            /*if (i == 6|| i ==7) {
                G1_speed = 500;
            }*/
                        
        sum_E += 2.0;
        jud_reduce_extrusion[i].resize(ALL_OPP[i].size());
        vector<Vec3> tmpp;
        vector<double> tmpp_H;
        the_path.push_back(tmpp);
        the_path_H.push_back(tmpp_H);
        vector<vector<Vec3>> OPP = ALL_OPP[i];
        gcode << ";;OPP:" << i << endl;
        gcode << "G0 F"<<G0_speed<<" X" << setiosflags(ios::fixed) << setprecision(3) << OPP[0][all_optimal_start_index[i][0]].m_x << " Y" << OPP[0][all_optimal_start_index[i][0]].m_y << " Z"<< highest_point+50 << endl;
        
        //extrude
        /*if (i != 0) {
            gcode << "G0 F2500 X" << setiosflags(ios::fixed) << setprecision(3) << OPP[0][all_optimal_start_index[i][0]].m_x << " Y" << OPP[0][all_optimal_start_index[i][0]].m_y << " Z" << OPP[0][all_optimal_start_index[i][0]].m_z + 20 << endl;
            sum_E += 800;
            gcode << "G1 F600 " << "E" <<sum_E<< endl;
        }*/
        /////////

        gcode << "G0 F" << G0_speed << " X" << setiosflags(ios::fixed) << setprecision(3) << OPP[0][all_optimal_start_index[i][0]].m_x << " Y" << OPP[0][all_optimal_start_index[i][0]].m_y << " Z" << OPP[0][all_optimal_start_index[i][0]].m_z << endl;
        //gcode << "M204 S50" << endl << "M205 X5 Y5" << endl;
        gcode << "G1 F"<<G1_speed << endl;
        Vec3 last_point;
        Vec3 relay_point_start,relay_point_end;
        int index_relay_point_start, index_relay_point_end;
        bool jud_spiral = false;
        for (int j = 0; j < OPP.size(); j++)
        {
            jud_reduce_extrusion[i][j].resize(OPP[j].size());
            for (int k = 0;k < OPP[j].size();k++)
                jud_reduce_extrusion[i][j][k] = false;
            bool exceed_S = false;
            gcode << ";;Layer:" << j << endl; 
                                     //E_para *= 1.08;
            vector<Vec3> pull_path_1;
            vector<Vec3> pull_path_2;
            pull_path_1.clear();
            pull_path_2.clear();
            /////////////////////////////////when pull distance exceed S/////////////////////////////////
            if (j != 0 && distance(last_point, OPP[j][all_optimal_start_index[i][j]]) > S)
            {
                exceed_S = true;
                //int index_end_connection_point = all_optimal_start_index[i][j];
                double min_dis = MAX_D;
                for (int k = 0;k < OPP[j].size();k++) {
                    double dis = distance(OPP[j][k], last_point);
                    if (dis < min_dis) {
                        min_dis = dis;
                        relay_point_end = OPP[j][k];
                        index_relay_point_end = k;
                    }
                }
                min_dis = MAX_D;
                for (int k = 0;k < OPP[j - 1].size();k++) {
                    double dis = distance(OPP[j - 1][k], relay_point_end);
                    if (dis < min_dis) {
                        min_dis = dis;
                        relay_point_start = OPP[j-1][k];
                        index_relay_point_start = k;
                    }
                }

                pull_path_1 = generate_pull_path(index_relay_point_start, OPP[j - 1], all_optimal_start_index[i][j - 1],true, jud_reduce_extrusion[i][j]);
                pull_path_2 = generate_pull_path(index_relay_point_end, OPP[j], all_optimal_start_index[i][j],false, jud_reduce_extrusion[i][j]);
                std::reverse(pull_path_2.begin(), pull_path_2.end());
                for (int k = 0;k < pull_path_1.size();k++) {
                    if (k == 0)
                        if (is_FDM)
                            sum_E += 0.02;
                        //sum_E += distance_2(OPP[j - 1][all_optimal_start_index[i][j - 1]], pull_path_1[k]) * 1.8 * dh/4;
                        else
                            sum_E += 1;
                    else
                        sum_E += distance_2(pull_path_1[k], pull_path_1[k-1]) * E_para * dh / 3;
                    the_path[i].push_back(pull_path_1[k]);
                    the_path_H[i].push_back(1);
                    gcode << "G1 " << "X" << setiosflags(ios::fixed) << setprecision(3) << pull_path_1[k].m_x << " Y" << pull_path_1[k].m_y << " Z" << pull_path_1[k].m_z << " E" << sum_E << endl;
                }
                    
                for (int k = 0;k < pull_path_2.size();k++) {
                    if (k == 0)
                        if(pull_path_1.size() != 0)
                            sum_E += distance_2(pull_path_1[pull_path_1.size()-1], pull_path_2[k]) * E_para *  2 * dh / 3;
                        else
                            if (is_FDM)
                                sum_E += 0.02;
                            //sum_E += distance_2(OPP[j - 1][all_optimal_start_index[i][j - 1]], pull_path_2[k]) * 1.8 * 3 * dh / 4;
                            else
                                sum_E += 1;
                    else
                        sum_E += distance_2(pull_path_2[k], pull_path_2[k - 1]) * E_para * 2 * dh / 3;
                    the_path[i].push_back(pull_path_2[k]);
                    the_path_H[i].push_back(1);
                    gcode << "G1 " << "X" << setiosflags(ios::fixed) << setprecision(3) << pull_path_2[k].m_x << " Y" << pull_path_2[k].m_y << " Z" << pull_path_2[k].m_z << " E" << sum_E << endl;
                }  
            }
            ///////////////////////////////////////////////////////////////////////////////////////////////////

            bool jud_close;

            int num_next_layer_points = OPP[j].size();
            int num_current_layer_points = OPP[j].size();
            
           // change_order(index_start_point, OPP[j], true);
            if (distance(OPP[j][num_next_layer_points - 1], OPP[j][0]) < 1)
            {
                jud_close = true;
                /*Vec3 tmp = OPP[j][0];
                OPP[j].push_back(tmp);
                double tempp = ALL_H[i][j][0];
                ALL_H[i][j].push_back(tempp);*/
                //counterclockwise sort
                if (PointCmp(OPP[j]) == true)
                {
                    if (choose_spiral == true) {
                        spiral_for_contours(OPP, all_optimal_start_index, i, j, jud_spiral, ALL_H,true);
                    }
                    int k = all_optimal_start_index[i][j];
                    while(1)
                    {
                        double deta_sum_E = 0;
                        if (k == all_optimal_start_index[i][j])
                        {
                            if (j != 0 && distance_2(OPP[j - 1][OPP[j - 1].size() - 1], OPP[j - 1][0]) < 1) {
                                if (j != 0 && PointCmp(OPP[j - 1]) == true) {
                                    if (exceed_S == true) {
                                        Vec3 temp_point = pull_path_2[pull_path_2.size() - 1];
                                        deta_sum_E = distance_2(OPP[j][k], temp_point) * E_para * (ALL_H[i][j][k] + dh) / 2;
                                    }
                                    else {
                                        int k2 = all_optimal_start_index[i][j - 1];
                                        deta_sum_E = distance_2(OPP[j][k], OPP[j - 1][(k2 - 1 + OPP[j - 1].size()) % OPP[j - 1].size()]) * E_para * (ALL_H[i][j][k] + ALL_H[i][j - 1][(k2 - 1 + OPP[j - 1].size()) % OPP[j - 1].size()]) / 2;
                                    }
                                }
                                else if (j != 0 && PointCmp(OPP[j - 1]) == false) {
                                    if (exceed_S == true) {
                                        Vec3 temp_point = pull_path_2[pull_path_2.size() - 1];
                                        deta_sum_E = distance_2(OPP[j][k], temp_point) * E_para * (ALL_H[i][j][k] + dh) / 2;
                                    }
                                    else {
                                        int k2 = all_optimal_start_index[i][j - 1];
                                        deta_sum_E = distance_2(OPP[j][k], OPP[j - 1][(k2 + 1) % OPP[j - 1].size()]) * E_para * (ALL_H[i][j][k] + ALL_H[i][j - 1][(k2 + 1) % OPP[j - 1].size()]) / 2;
                                    }
                                }
                            }
                            else
                                if (is_FDM)
                                    deta_sum_E = 0.02;
                                else
                                    deta_sum_E = 1;
                        }
                        else
                        {
                            deta_sum_E = distance_2(OPP[j][k], OPP[j][(k - 1 + OPP[j].size()) % OPP[j].size()]) * E_para* (ALL_H[i][j][k] + ALL_H[i][j][(k - 1 + OPP[j].size()) % OPP[j].size()]) / 2;
                        }
                        if (jud_reduce_extrusion[i][j][k] == true)
                            deta_sum_E *= (1.0 / 3.0);
                        if (i != 0 && j == 0) {
                            //deta_sum_E *= 0.7;
                            ALL_H[i][j][k] *= 0.7;
                        }
                        sum_E += deta_sum_E;
                        gcode << "G1 X" << setiosflags(ios::fixed) << setprecision(3) << OPP[j][k].m_x << " Y" << OPP[j][k].m_y << " Z" << OPP[j][k].m_z << " E" << sum_E << endl;
                        the_path[i].push_back(Vec3(OPP[j][k].m_x, OPP[j][k].m_y, OPP[j][k].m_z));
                        the_path_H[i].push_back(ALL_H[i][j][k]);
                        k = (k + 1) % OPP[j].size();
                        if (k == all_optimal_start_index[i][j])
                            break;
                    }
                    last_point = OPP[j][(k - 1 + OPP[j].size()) % OPP[j].size()];
                }
                else
                {
                    if (choose_spiral == true) {
                        spiral_for_contours(OPP, all_optimal_start_index, i, j, jud_spiral, ALL_H,false);
                    }
                    int k = all_optimal_start_index[i][j];
                    while (1)
                    {
                        double deta_sum_E = 0;
                        if (k == all_optimal_start_index[i][j])
                        {
                            if (j != 0 && distance_2(OPP[j - 1][OPP[j - 1].size() - 1], OPP[j - 1][0]) < 1) {
                                if (j != 0 && PointCmp(OPP[j - 1]) == true) {
                                    if (exceed_S == true) {
                                        Vec3 temp_point = pull_path_2[pull_path_2.size() - 1];
                                        deta_sum_E = distance_2(OPP[j][k], temp_point) * E_para * (ALL_H[i][j][k] + dh) / 2;
                                    }
                                    else {
                                        int k2 = all_optimal_start_index[i][j - 1];
                                        deta_sum_E = distance_2(OPP[j][k], OPP[j - 1][(k2 - 1 + OPP[j - 1].size()) % OPP[j - 1].size()]) * E_para * (ALL_H[i][j][k] + ALL_H[i][j - 1][(k2 - 1 + OPP[j - 1].size()) % OPP[j - 1].size()]) / 2;
                                    }
                                }
                                else if (j != 0 && PointCmp(OPP[j - 1]) == false) {
                                    if (exceed_S == true) {
                                        Vec3 temp_point = pull_path_2[pull_path_2.size() - 1];
                                        deta_sum_E = distance_2(OPP[j][k], temp_point) * E_para * (ALL_H[i][j][k] + dh) / 2;
                                    }
                                    else {
                                        int k2 = all_optimal_start_index[i][j - 1];
                                        deta_sum_E = distance_2(OPP[j][k], OPP[j - 1][(k2 + 1) % OPP[j - 1].size()]) * E_para * (ALL_H[i][j][k] + ALL_H[i][j - 1][(k2 + 1) % OPP[j - 1].size()]) / 2;
                                    }
                                }
                            }
                            else
                                if(is_FDM)
                                    deta_sum_E = 0.02;
                                else
                                    deta_sum_E = 1;
                        }
                        else
                        {
                            deta_sum_E = distance_2(OPP[j][k], OPP[j][(k + 1) % OPP[j].size()]) * E_para * (ALL_H[i][j][k] + ALL_H[i][j][(k + 1) % OPP[j].size()]) / 2;
                        }
                        if (jud_reduce_extrusion[i][j][k] == true)
                            deta_sum_E *= (1.0 / 3.0);
                        if (i != 0 && j == 0) {
                            //deta_sum_E *= 0.7;
                            ALL_H[i][j][k] *= 0.7;
                        }
                        sum_E += deta_sum_E;
                        gcode << "G1 X" << setiosflags(ios::fixed) << setprecision(3) << OPP[j][k].m_x << " Y" << OPP[j][k].m_y << " Z" << OPP[j][k].m_z << " E" << sum_E << endl;
                        the_path[i].push_back(Vec3(OPP[j][k].m_x, OPP[j][k].m_y, OPP[j][k].m_z));
                        the_path_H[i].push_back(ALL_H[i][j][k]);
                        k = (k - 1 + OPP[j].size()) % OPP[j].size();
                        if (k == all_optimal_start_index[i][j])
                            break;
                    }
                    last_point = OPP[j][(k + 1) % OPP[j].size()];
                }
            }

            //ZigZag
            else
            {
                jud_close = false;
                if (all_optimal_start_index[i][j] == 0)  //Print in the original order
                {
                    double nozzle_w = 1;
                    vector<Vec3> temp_OPP = OPP[j];
                    for (int k = 0; k < OPP[j].size(); k++)
                    {
                        double deta_sum_E = 0;
                        if (k == 0)
                        {
                            if (j != 0 && distance_2(OPP[j - 1][OPP[j - 1].size() - 1], OPP[j - 1][0]) > 1) {
                                if (j != 0 && all_optimal_start_index[i][j - 1] == 0) {
                                    if (exceed_S == true) {
                                        Vec3 temp_point = pull_path_2[pull_path_2.size() - 1];
                                        deta_sum_E = distance_2(OPP[j][k], temp_point) * E_para * (ALL_H[i][j][k] + dh) / 2;
                                    }
                                    else {
                                        int k2 = OPP[j - 1].size() - 1;
                                        deta_sum_E = distance_2(OPP[j][k], OPP[j - 1][k2]) * E_para * (ALL_H[i][j][k] + ALL_H[i][j - 1][k2]) / 2;
                                    }
                                }
                                else if (j != 0 && all_optimal_start_index[i][j - 1] != 0) {
                                    if (exceed_S == true) {
                                        Vec3 temp_point = pull_path_2[pull_path_2.size() - 1];
                                        deta_sum_E = distance_2(OPP[j][k], temp_point) * E_para * (ALL_H[i][j][k] + dh) / 2;
                                    }
                                    else {
                                        int k2 = 0;
                                        deta_sum_E = distance_2(OPP[j][k], OPP[j - 1][k2]) * E_para * (ALL_H[i][j][k] + ALL_H[i][j - 1][k2]) / 2;
                                    }
                                }

                                //reduce extrusion on terminal point of zig-zag
                                deta_sum_E *= 0.2;  //0.5
                            }
                            else
                                if (is_FDM)
                                    deta_sum_E = 0.02;
                                else
                                    deta_sum_E = 1;
                        }
                        else
                        {
                            deta_sum_E = distance_2(OPP[j][k], OPP[j][k - 1]) * E_para * (ALL_H[i][j][k] + ALL_H[i][j][k - 1]) / 2;
                        }
                        if (jud_reduce_extrusion[i][j][k] == true)
                            deta_sum_E *= (1.0 / 3.0);
                        if (i != 0 && j == 0) {
                            //deta_sum_E *= 0.7;
                            ALL_H[i][j][k] *= 0.7;
                        }
                        sum_E += deta_sum_E;

                        if (open_adjust_z == true) {
                            ////////
                            if (k != 0 && temp_OPP[k].m_z < temp_OPP[k - 1].m_z) {
                                double tan_sita = abs(double(temp_OPP[k].m_z - temp_OPP[k - 1].m_z) / distance_2(temp_OPP[k], temp_OPP[k - 1]));
                                ALL_H[i][j][k] *= ((1 - tan_sita*1.2));
                                //OPP[j][k].m_z += (nozzle_w * tan_sita) / 2;
                            }
                            if (k != 0 && temp_OPP[k].m_z > temp_OPP[k - 1].m_z) {
                                double tan_sita = abs(double(temp_OPP[k].m_z - temp_OPP[k - 1].m_z) / distance_2(temp_OPP[k], temp_OPP[k - 1]));
                                ALL_H[i][j][k] *= (1 + tan_sita*1.5);
                                //OPP[j][k].m_z += (nozzle_w * tan_sita) / 2;
                            }
                            ///////
                        }
                        
                        gcode << "G1 X" << setiosflags(ios::fixed) << setprecision(3) << OPP[j][k].m_x << " Y" << OPP[j][k].m_y << " Z" << OPP[j][k].m_z << " E" << sum_E << endl;
                        the_path[i].push_back(Vec3(OPP[j][k].m_x, OPP[j][k].m_y, OPP[j][k].m_z));
                        the_path_H[i].push_back(ALL_H[i][j][k]);
                    }
                    last_point = OPP[j][OPP[j].size() - 1];
                }
                else
                {
                    for (int k = OPP[j].size() - 1; k >= 0; k--)
                    {
                        double deta_sum_E = 0;
                        double nozzle_w = 1;
                        vector<Vec3> temp_OPP = OPP[j];
                        if (k == OPP[j].size() - 1)
                        {
                            if (j != 0 && distance_2(OPP[j - 1][OPP[j - 1].size() - 1], OPP[j - 1][0]) > 1) {
                                if (j != 0 && all_optimal_start_index[i][j - 1] == 0) {
                                    if (exceed_S == true) {
                                        Vec3 temp_point = pull_path_2[pull_path_2.size() - 1];
                                        deta_sum_E = distance_2(OPP[j][k], temp_point) * E_para * (ALL_H[i][j][k] + dh) / 2;
                                    }
                                    else {
                                        int k2 = OPP[j - 1].size() - 1;
                                        deta_sum_E = distance_2(OPP[j][k], OPP[j - 1][k2]) * E_para * (ALL_H[i][j][k] + ALL_H[i][j - 1][k2]) / 2;
                                    }
                                }
                                else if (j != 0 && all_optimal_start_index[i][j - 1] != 0) {
                                    if (exceed_S == true) {
                                        Vec3 temp_point = pull_path_2[pull_path_2.size() - 1];
                                        deta_sum_E = distance_2(OPP[j][k], temp_point) * E_para * (ALL_H[i][j][k] + dh) / 2;
                                    }
                                    else {
                                        int k2 = 0;
                                        deta_sum_E = distance_2(OPP[j][k], OPP[j - 1][k2]) * E_para * (ALL_H[i][j][k] + ALL_H[i][j - 1][k2]) / 2;
                                    }
                                }

                                //reduce extrusion on terminal point of zig-zag
                                deta_sum_E *= 0.5;
                            }
                            else
                                if (is_FDM)
                                    deta_sum_E = 0.02;
                                else
                                    deta_sum_E = 1;
                        }
                        else
                        {
                            deta_sum_E = distance_2(OPP[j][k], OPP[j][k + 1]) * E_para * (ALL_H[i][j][k] + ALL_H[i][j][k + 1]) / 2;
                        }
                        if (jud_reduce_extrusion[i][j][k] == true)
                            deta_sum_E *= (1.0 / 3.0);
                        if (i != 0 && j == 0) {
                            //deta_sum_E *= 0.7;
                            ALL_H[i][j][k] *= 0.7;
                        }
                        sum_E += deta_sum_E;

                        if (open_adjust_z == true) {
                            ////////
                            if (k != OPP[j].size() - 1 && temp_OPP[k].m_z < temp_OPP[k + 1].m_z) {
                                double tan_sita = abs(double(temp_OPP[k].m_z - temp_OPP[k + 1].m_z) / distance_2(temp_OPP[k], temp_OPP[k + 1]));
                                ALL_H[i][j][k] *= ((1 - tan_sita*1.2));
                                //OPP[j][k].m_z += (nozzle_w * tan_sita) / 2;
                            }
                            if (k != OPP[j].size() - 1 && temp_OPP[k].m_z > temp_OPP[k + 1].m_z) {
                                double tan_sita = abs(double(temp_OPP[k].m_z - temp_OPP[k + 1].m_z) / distance_2(temp_OPP[k], temp_OPP[k + 1]));
                                ALL_H[i][j][k] *= (1 + tan_sita*1.5);
                                //OPP[j][k].m_z += (nozzle_w * tan_sita) / 2;
                            }
                            ///////
                        }
                        
                        gcode << "G1 X" << setiosflags(ios::fixed) << setprecision(3) << OPP[j][k].m_x << " Y" << OPP[j][k].m_y << " Z" << OPP[j][k].m_z << " E" << sum_E << endl;
                        the_path[i].push_back(Vec3(OPP[j][k].m_x, OPP[j][k].m_y, OPP[j][k].m_z));
                        the_path_H[i].push_back(ALL_H[i][j][k]);
                    }
                    last_point = OPP[j][0];
                }
            }
        }
        for (int j = 0;j < ALL_OPP[i].size();j++)
            for (int k = 0;k < ALL_OPP[i][j].size();k++)
                if (ALL_OPP[i][j][k].m_z > highest_point)
                    highest_point = ALL_OPP[i][j][k].m_z;

        if (is_FDM) {
            sum_E -= 2.0;
            gcode << "G1 F1500" <<" E" << sum_E << endl;
        }

        //gcode << "M204 S500" << endl << "M205 X50 Y50" << endl;
        gcode << "G0 F" << G0_speed << " X"<< setiosflags(ios::fixed) << setprecision(3) << last_point.m_x << " Y" << last_point.m_y << " Z"<< highest_point + 50 << endl;
    }
    /*double thickness_1 = 0;
    for (int i = 0;i < ALL_OPP[3][43].size();i++) {
        thickness_1 += ALL_H[3][43][i];
    }
    thickness_1 /= ALL_OPP[3][43].size();
    double thickness_2 = 0;
    for (int i = 0;i < ALL_OPP[1][29].size();i++) {
        thickness_2 += ALL_H[1][29][i];
    }
    thickness_2 /= ALL_OPP[1][29].size();*/

    /*vector<vector<Vec3>> temp_path = the_path;
    for (int num = 0; num < the_path.size(); num++) {
        int cont = 0;
        the_path[num].clear();
        for (int i = 0; i < the_path[num].size(); i += 4) {
            the_path[num].push_back(temp_path[num][i]);
            cont++;
        }
    }*/
    Visual VV;
    VV.generateModelForRendering_3(the_path, file_name);

    string out_file = "../Blocks_files/" + file_name + "_ori_path.txt";
    ofstream out_path_txt(out_file);
    int sum_point = 0;
    for (int num = 0;num < the_path.size();num++)
        sum_point += the_path[num].size();
    //sum_point /= 4;
    out_path_txt << sum_point << endl;
    for (int num = 0;num < the_path.size();num++) {
        for (int i = 0;i < the_path[num].size();i++) {
            out_path_txt << the_path[num][i].m_x << " " << the_path[num][i].m_y << " " << the_path[num][i].m_z << " " << the_path_H[num][i] << endl;
            /*if (num == 0 && (abs(the_path[num][i].m_z- 5.8+42.7)<0.01 || abs(the_path[num][i].m_z-6.0 + 42.7)<0.1|| abs(the_path[num][i].m_z-6.2 + 42.7)<0.1|| abs(the_path[num][i].m_z-6.4 + 42.7)<0.1|| abs(the_path[num][i].m_z-6.6 + 42.7)<0.1|| abs(the_path[num][i].m_z-6.8 + 42.7)<0.1))
                out_path_txt << " 1" << endl;
            else
                out_path_txt << " 0" << endl;*/
        }
    }
    out_path_txt.close();

    end_time = clock();
    std::cout << "&&&&&&& path generation time = " << double(end_time - start_time) / CLOCKS_PER_SEC << "s &&&&&&&" << std::endl;
}

double distance(Vec3 a, Vec3 b)
{
    double dis = sqrt(pow(a.m_x - b.m_x, 2) + pow(a.m_y - b.m_y, 2) + pow(a.m_z - b.m_z, 2));
    return dis;
}

double distance_2(Vec3 a, Vec3 b)
{
    double dis = sqrt(pow(a.m_x - b.m_x, 2) + pow(a.m_y - b.m_y, 2));
    return dis;
}

bool PointCmp(vector<Vec3> layer)
{
    double d = 0;
    for (int i = 0; i < layer.size() - 1; i++)
    {
        d += -0.5 * (layer[i + 1].m_y + layer[i].m_y) * (layer[i + 1].m_x - layer[i].m_x);
    }
    if (d > 0)
        return true;
    else
        return false;
}

bool compute_angle(Vec3 a, Vec3 b)
{
    if (abs((b.m_z - a.m_z) / sqrt(pow(a.m_x - b.m_x, 2) + pow(a.m_y - b.m_y, 2))) < 0.176)  //����Ϊ10��
        return true;
    else
        return false;
}




