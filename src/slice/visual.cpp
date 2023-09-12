#include "visual.h"
#include <direct.h>

void Visual::generateModelForRendering_2(const string& _file_name)
{
    //string file_name_txt = "12_9_helmet_2_optimal";
    ifstream ifile2(_file_name + ".txt");
    float radius = 2 * 0.15;
    srand((int)time(0));
    int num_block;
    ifile2 >> num_block;
    for (int i = 0; i < num_block; i++)
    {
        General_Mesh mesh1;
        mesh1.r = rand() / double(RAND_MAX);
        mesh1.g = rand() / double(RAND_MAX);
        mesh1.b = rand() / double(RAND_MAX);
        std::vector<Vec3> points;
        string sub_file_name = "block" + to_string(i);
        int num_seg;
        ifile2 >> num_seg;
        for (int j = 0; j < num_seg; j++)
        {
            int num_points;
            ifile2 >> num_points;
            for (int k = 0; k < num_points; k++)
            {
                Vec3 the_point;
                double temp_h;
                ifile2 >> the_point.m_x >> the_point.m_y >> the_point.m_z;
                temp_h = 1;
                points.push_back(the_point);

            }
            insert_Line(mesh1, points, radius);
            points.clear();
        }
        string derectory = "../segmentation/" + _file_name;
        _mkdir(derectory.c_str());
        std::cout << ("../segmentation/" + _file_name + "/" + sub_file_name + ".obj").c_str() << std::endl;
        mesh1.genResultMesh(("../segmentation/" + _file_name + "/" + sub_file_name + ".obj").c_str());
        points.clear();
        mesh1.~General_Mesh();
    }
}

void Visual::generateModelForRendering_5(const string& _file_name, const string& doc_name)
{
    //string file_name_txt = "12_9_helmet_2_optimal";
    ifstream ifile2(_file_name + ".txt");
    float radius = 1 * 0.3;
    srand((int)time(0));
    int num_block;
    ifile2 >> num_block;
    for (int i = 0; i < num_block; i++)
    {
        General_Mesh mesh1;
        mesh1.r = rand() / double(RAND_MAX);
        mesh1.g = rand() / double(RAND_MAX);
        mesh1.b = rand() / double(RAND_MAX);
        std::vector<Vec3> points;
        string sub_file_name = "block" + to_string(i);
        int num_seg;
        ifile2 >> num_seg;
        for (int j = 0; j < num_seg; j++)
        {
            int num_points;
            ifile2 >> num_points;
            for (int k = 0; k < num_points; k++)
            {
                Vec3 the_point;
                double temp_h;
                ifile2 >> the_point.m_x >> the_point.m_y >> the_point.m_z;
                temp_h = 1;
                points.push_back(the_point);

            }
            insert_Line(mesh1, points, radius);
            points.clear();
        }
        string derectory = "../segmentation/" + doc_name;
        _mkdir(derectory.c_str());
        std::cout << ("../segmentation/" + doc_name + "/" + sub_file_name + ".obj").c_str() << std::endl;
        mesh1.genResultMesh(("../segmentation/" + doc_name + "/" + sub_file_name + ".obj").c_str());
        points.clear();
        mesh1.~General_Mesh();
    }
}
void Visual::insert_Line(General_Mesh& mesh, std::vector<Vec3> points, float radius)
{
    if (points.size() < 2)
        return;

    for (int i = 0; i < points.size() - 1; i++)
    {

        Vec3 v0 = points[i];
        Vec3 v1 = points[i + 1];

        Vec3 t_dir = v1 - v0;

        Vec3 scale(radius, radius, t_dir.Length());
        Vec3 trans = (v0 + v1) / 2;

        vector<Vec3> c_vertex = mesh.meshScale(scale, mesh.cylinder_vertex);
        c_vertex = mesh.meshRotate(t_dir, c_vertex);
        c_vertex = mesh.meshTrans(trans, c_vertex);
        mesh.insert(c_vertex, mesh.cylinder_faces);
    }
}

void Visual::generateModelForRendering(vector<vector<Vec3>> lines, string file_name)
{
    General_Mesh mesh1;
    std::vector<Vec3> points;
    float radius = 0.7;
    ofstream ofile("D:\\CNCProduction\\Release\\geodesic_isoline_usage_shiqing\\test\\" + file_name + ".gcode");
    for (int i = 0; i < lines.size(); i++)
    {
        for (int j = 0; j < lines[i].size(); j++)
        {
            Vec3 v(lines[i][j].m_x, lines[i][j].m_y, lines[i][j].m_z);
            points.push_back(v);
            if (j != 0)
                ofile << "G1 " << "X" << lines[i][j].m_x << " Y" << lines[i][j].m_y << " Z" << lines[i][j].m_z << " E1" << endl;
            else
                ofile << "G0 " << "X" << lines[i][j].m_x << " Y" << lines[i][j].m_y << " Z" << lines[i][j].m_z << endl;
        }
        insert_Line(mesh1, points, radius);
        points.clear();
    }
    mesh1.genResultMesh("visual_path.obj");
    points.clear();
    ofile.close();
}


void Visual::generateModelForRendering_3(vector<vector<Vec3>> the_path, string file_name_txt)
{
    float radius = 2 * 0.15;
    srand((int)time(0));
    int num_block;
    std::vector<Vec3> points;
    for (int i = 0; i < the_path.size(); i++)
    {
        General_Mesh mesh1;
        mesh1.r = rand() / double(RAND_MAX);
        mesh1.g = rand() / double(RAND_MAX);
        mesh1.b = rand() / double(RAND_MAX);
        string file_block = "block" + to_string(i);
        for (int j = 0; j < the_path[i].size(); j++)
        {
            Vec3 the_point;
            the_point.m_x = the_path[i][j].m_x;
            the_point.m_y = the_path[i][j].m_y;
            the_point.m_z = the_path[i][j].m_z;
            points.push_back(the_point);
        }
        insert_Line(mesh1, points, radius);
        points.clear();
        std::cout << ("../Blocks_files/" + file_name_txt + "_" + file_block + ".obj").c_str() << std::endl;
        mesh1.genResultMesh(("../Blocks_files/" + file_name_txt +"_" +file_block + ".obj").c_str());
        points.clear();
        //mesh1.~General_Mesh();

        //output end point
        /*std::ofstream dstream_2("../Blocks_files/" + file_name_txt + "_" + file_block + "_terminal_point.txt");
        dstream_2 << the_path[i][the_path[i].size() - 1].m_x << " "<<the_path[i][the_path[i].size() - 1].m_y<<" " << the_path[i][the_path[i].size() - 1].m_z;
        dstream_2.close();*/
    }

}

void Visual::generateModelForRendering_4(vector<vector<Vec3>> the_path, string file_name)
{
    General_Mesh mesh1;
    std::vector<Vec3> points;
    mesh1.r = 0.8;
    mesh1.b = 0.2;
    mesh1.g = 0.2;
    float radius = 0.5;
    for (int i = 0; i < the_path.size(); i++)
    {
        for (int j = 0; j < the_path[i].size(); j++)
        {
            Vec3 v(the_path[i][j].m_x, the_path[i][j].m_y, the_path[i][j].m_z);
            points.push_back(v);
        }
        insert_Line(mesh1, points, radius);
        points.clear();
    }
    mesh1.genResultMesh(file_name.c_str());
    points.clear();  
}

void Visual::generateModelForRendering_6(std::vector<cv::Point3d> all_points, bool** Dependen_edges)
{
    General_Mesh mesh1;
    std::vector<Vec3> points;
    float radius;
    radius = 0.02;
    for (int i = 0; i < 3000; i++)
    {
        for(int j =0;j<3000;j++)
            if (Dependen_edges[i][j] == true) {
                points.push_back(Vec3(all_points[i].x, all_points[i].y, all_points[i].z));
                points.push_back(Vec3(all_points[j].x, all_points[j].y, all_points[j].z));
                insert_Line(mesh1, points, radius);
                points.clear();
            }
    }
    mesh1.genResultMesh("../base_line/dependency graph.obj");
    points.clear();
}