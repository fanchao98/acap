#pragma once
#include <vector>
#include "Vector3.h"
#include <fstream>
#include <string>

using namespace std;
typedef vector<int> v_int;
typedef Vector3 Vec3;
#define PI 3.1415926543

class General_Mesh
{
public:
	General_Mesh();
	~General_Mesh();




	void genResultMesh(const char* filename);
	void insert(vector<Vec3>& origin_vertexs, vector<v_int>& origin_faces,
		vector<Vec3> vertexs, vector<v_int> faces);
	void insert(vector<Vec3> vertexs, vector<v_int> faces);

	void genCube();
	void genCylinder();

	void genHollowCylinder(float inside_radius, float outside_radius, float height);
	void genHollowCube(float width, float radius, float thick);



	vector<Vec3> meshTrans(Vec3 dire, vector<Vec3> point);
	vector<Vec3> meshScale(Vec3 scale, vector<Vec3> point);
	vector<Vec3> meshRotate(Vec3 z_dire, vector<Vec3> point);
	vector<Vec3> meshRotate(Vec3 x_dire, Vec3 z_dire, vector<Vec3> point);



	vector<v_int> result_faces;
	vector<Vec3> result_vertex;
	vector<v_int> cylinder_faces;
	vector<Vec3> cylinder_vertex;
	vector<v_int> cube_faces;
	vector<Vec3> cube_vertex;
	vector<v_int> hollow_cylinder_faces;
	vector<Vec3> hollow_cylinder_vertex;
	vector<v_int> hollow_cube_faces;
	vector<Vec3> hollow_cube_vertex;
	double r;
	double g;
	double b;

};

