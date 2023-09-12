#include "GeneralMesh.h"



General_Mesh::General_Mesh()
{
	genCylinder();
	genCube();
	genHollowCube(4, 1, 4);
	genHollowCylinder(1, 2, 5);
}


General_Mesh::~General_Mesh()
{
}






void General_Mesh::genCube()
{
	Vec3 temp(-0.5, -0.5, 0.5);
	cube_vertex.push_back(temp);
	temp.m_x = 0.5;
	cube_vertex.push_back(temp);
	temp.m_y = 0.5;
	cube_vertex.push_back(temp);
	temp.m_x = -0.5;
	cube_vertex.push_back(temp);


	temp.m_x = -0.5, temp.m_y = -0.5, temp.m_z = -0.5;
	cube_vertex.push_back(temp);
	temp.m_x = 0.5;
	cube_vertex.push_back(temp);
	temp.m_y = 0.5;
	cube_vertex.push_back(temp);
	temp.m_x = -0.5;
	cube_vertex.push_back(temp);

	v_int v_temp;
	v_temp.push_back(0);
	v_temp.push_back(1);
	v_temp.push_back(2);
	v_temp.push_back(3);
	cube_faces.push_back(v_temp);

	v_temp.clear();
	v_temp.push_back(7);
	v_temp.push_back(6);
	v_temp.push_back(5);
	v_temp.push_back(4);
	cube_faces.push_back(v_temp);


	v_temp.clear();
	v_temp.push_back(1);
	v_temp.push_back(0);
	v_temp.push_back(4);
	v_temp.push_back(5);
	cube_faces.push_back(v_temp);


	v_temp.clear();
	v_temp.push_back(2);
	v_temp.push_back(6);
	v_temp.push_back(7);
	v_temp.push_back(3);
	cube_faces.push_back(v_temp);



	v_temp.clear();
	v_temp.push_back(2);
	v_temp.push_back(1);
	v_temp.push_back(5);
	v_temp.push_back(6);
	cube_faces.push_back(v_temp);

	v_temp.clear();
	v_temp.push_back(0);
	v_temp.push_back(3);
	v_temp.push_back(7);
	v_temp.push_back(4);
	cube_faces.push_back(v_temp);


	ofstream fout;
	fout.open("cube.obj");
	for (int i = 0; i < cube_vertex.size(); i++)
	{
		fout << "v " << cube_vertex[i].m_x << " " << cube_vertex[i].m_y << " " << cube_vertex[i].m_z << endl;
	}
	for (int i = 0; i < cube_faces.size(); i++)
	{
		int facet_count = cube_faces[i].size();
		fout << "f";
		for (int j = 0; j < facet_count; j++)
		{
			fout << " " << cube_faces[i][j] + 1;
		}
		fout << endl;
	}
}

void General_Mesh::genCylinder()
{
	float x[24], y[24];

	for (int i = 0; i < 24; i++)
	{
		x[i] = cos(i * PI / 12);
		y[i] = sin(i * PI / 12);
	}

	v_int v_temp;

	float height;
	for (int k = 0; k < 2; k++)
	{
		height = -0.5 + k;
		for (int i = 0; i < 24; i++)
		{
			Vec3 temp(x[i], y[i], height);
			cylinder_vertex.push_back(temp);
		}
	}

	v_temp.clear();
	for (int i = 24; i > 0; i--)
	{
		v_temp.push_back(i - 1);
	}
	cylinder_faces.push_back(v_temp);


	v_temp.clear();
	for (int i = 25; i <= 48; i++)
	{
		v_temp.push_back(i - 1);
	}
	cylinder_faces.push_back(v_temp);


	int a, b;
	for (int i = 0; i < 24; i++)
	{
		v_temp.clear();
		a = i;
		b = (i + 1) % 24;
		v_temp.push_back(a);
		v_temp.push_back(b);
		v_temp.push_back(b + 24);
		v_temp.push_back(a + 24);

		cylinder_faces.push_back(v_temp);
	}


	ofstream fout;
	fout.open("cylinder.obj");
	for (int i = 0; i < cylinder_vertex.size(); i++)
	{
		fout << "v " << cylinder_vertex[i].m_x << " " << cylinder_vertex[i].m_y << " " << cylinder_vertex[i].m_z << endl;
	}
	for (int i = 0; i < cylinder_faces.size(); i++)
	{
		int facet_count = cylinder_faces[i].size();
		fout << "f";
		for (int j = 0; j < facet_count; j++)
		{
			fout << " " << cylinder_faces[i][j] + 1;
		}
		fout << endl;
	}
	fout.close();
}

void General_Mesh::genHollowCylinder(float inside_radius, float outside_radius, float height)
{
	int segs = 48;
	float xx, yy;
	float r[2] = { inside_radius, outside_radius };
	float h[2] = { height / 2, -height / 2 };
	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < 2; j++)
		{
			for (int k = 0; k < segs; k++)
			{
				Vec3 temp(r[i] * cos(2 * k * PI / segs), r[i] * sin(2 * k * PI / segs), h[j]);
				hollow_cylinder_vertex.push_back(temp);
			}
		}
	}

	v_int v_temp;

	int a, b, start;
	for (int i = 0; i < segs; i++)
	{
		v_temp.clear();
		a = i;
		b = (i + 1) % segs;
		v_temp.push_back(a);
		v_temp.push_back(b);
		v_temp.push_back(b + segs);
		v_temp.push_back(a + segs);
		hollow_cylinder_faces.push_back(v_temp);
	}

	for (int i = 0; i < segs; i++)
	{
		v_temp.clear();
		a = i;
		b = (i + 1) % segs;
		v_temp.push_back(b);
		v_temp.push_back(a);
		v_temp.push_back(a + segs * 2);
		v_temp.push_back(b + segs * 2);
		hollow_cylinder_faces.push_back(v_temp);
	}

	start = segs;
	for (int i = 0; i < segs; i++)
	{
		v_temp.clear();
		a = i;
		b = (i + 1) % segs;
		v_temp.push_back(a + start);
		v_temp.push_back(b + start);
		v_temp.push_back(b + segs * 2 + start);
		v_temp.push_back(a + segs * 2 + start);
		hollow_cylinder_faces.push_back(v_temp);
	}

	start = segs * 2;
	for (int i = 0; i < segs; i++)
	{
		v_temp.clear();
		a = i;
		b = (i + 1) % segs;
		v_temp.push_back(b + start);
		v_temp.push_back(a + start);
		v_temp.push_back(a + segs + start);
		v_temp.push_back(b + segs + start);
		hollow_cylinder_faces.push_back(v_temp);
	}

	ofstream fout;
	fout.open("hollow_cylinder.obj");
	for (int i = 0; i < hollow_cylinder_vertex.size(); i++)
	{
		fout << "v " << hollow_cylinder_vertex[i].m_x << " " << hollow_cylinder_vertex[i].m_y << " " << hollow_cylinder_vertex[i].m_z << endl;
	}
	for (int i = 0; i < hollow_cylinder_faces.size(); i++)
	{
		int facet_count = hollow_cylinder_faces[i].size();
		fout << "f";
		for (int j = 0; j < facet_count; j++)
		{
			fout << " " << hollow_cylinder_faces[i][j] + 1;
		}
		fout << endl;
	}
	fout.close();
}

void General_Mesh::genHollowCube(float width, float radius, float thick)
{
	float y, z;
	Vec3 temp = thick / 2;

	temp.m_y = width / 2, temp.m_z = 0;
	hollow_cube_vertex.push_back(temp);
	temp.m_y = width / 2, temp.m_z = width / 2;
	hollow_cube_vertex.push_back(temp);
	temp.m_y = -width / 2, temp.m_z = width / 2;
	hollow_cube_vertex.push_back(temp);
	temp.m_y = -width / 2, temp.m_z = 0;
	hollow_cube_vertex.push_back(temp);
	temp.m_y = -width / 2, temp.m_z = -width / 2;
	hollow_cube_vertex.push_back(temp);
	temp.m_y = width / 2, temp.m_z = -width / 2;
	hollow_cube_vertex.push_back(temp);



	for (int i = 0; i < 24; i++)
	{
		temp.m_y = radius * cos(i * PI / 12);
		temp.m_z = radius * sin(i * PI / 12);
		hollow_cube_vertex.push_back(temp);
	}

	Vec3 x_dir(1, 0, 0);
	for (int i = 0; i < 30; i++)
	{
		Vec3 t = hollow_cube_vertex[i] - thick * x_dir;
		hollow_cube_vertex.push_back(t);
	}

	v_int v_temp;
	int a, b;
	for (int i = 0; i < 24; i++)
	{
		v_temp.clear();
		a = i + 6;
		b = (i + 1) % 24 + 6;
		v_temp.push_back(a);
		v_temp.push_back(b);
		v_temp.push_back(b + 30);
		v_temp.push_back(a + 30);
		hollow_cube_faces.push_back(v_temp);
	}

	for (int i = 0; i < 6; i++)
	{
		v_temp.clear();
		a = i;
		b = (i + 1) % 6;
		v_temp.push_back(b);
		v_temp.push_back(a);
		v_temp.push_back(a + 30);
		v_temp.push_back(b + 30);
		hollow_cube_faces.push_back(v_temp);
	}

	v_temp.clear();
	for (int i = 0; i < 4; i++)
	{
		v_temp.push_back(i);
	}
	for (int i = 18; i >= 6; i--)
	{
		v_temp.push_back(i);
	}
	hollow_cube_faces.push_back(v_temp);


	v_temp.clear();
	v_temp.push_back(3);
	v_temp.push_back(4);
	v_temp.push_back(5);
	v_temp.push_back(0);
	v_temp.push_back(6);
	for (int i = 29; i >= 18; i--)
	{
		v_temp.push_back(i);
	}
	hollow_cube_faces.push_back(v_temp);



	v_temp.clear();
	v_temp.push_back(33);
	v_temp.push_back(32);
	v_temp.push_back(31);
	v_temp.push_back(30);
	for (int i = 36; i <= 48; i++)
	{
		v_temp.push_back(i);
	}
	hollow_cube_faces.push_back(v_temp);



	v_temp.clear();
	v_temp.push_back(36);
	v_temp.push_back(30);
	v_temp.push_back(35);
	v_temp.push_back(34);
	v_temp.push_back(33);
	for (int i = 48; i <= 59; i++)
	{
		v_temp.push_back(i);
	}
	hollow_cube_faces.push_back(v_temp);




	ofstream fout;
	fout.open("hollow_cube.obj");
	for (int i = 0; i < hollow_cube_vertex.size(); i++)
	{
		fout << "v " << hollow_cube_vertex[i].m_x << " " << hollow_cube_vertex[i].m_y << " " << hollow_cube_vertex[i].m_z << endl;
	}
	for (int i = 0; i < hollow_cube_faces.size(); i++)
	{
		int facet_count = hollow_cube_faces[i].size();
		fout << "f";
		for (int j = 0; j < facet_count; j++)
		{
			fout << " " << hollow_cube_faces[i][j] + 1;
		}
		fout << endl;
	}
	fout.close();
}

//void General_Mesh::genHollowCube_new(float width, float radius, float thick)
//{
//	float r[2] = { radius, width / 2 };
//	float x, y, z;
//
//
//	for (int i = 0; i < 2; i++)
//	{
//		x = (-2 * i + 1)*thick / 2;
//		for (int j = 0; j < 2; j++)
//		{
//			for (int k = 0; k < 24; k++)
//			{
//				y = r[j] * cos(k*PI / 12);
//				z = r[j] * sin(k*PI / 12);
//				Vec3 temp(x, y, z);
//				hollow_cube_vertex.push_back(temp);
//			}
//		}
//	}
//
//
//	v_int v_temp;
//	int a, b;
//	for (int i = 0; i < 24; i++)
//	{
//		v_temp.clear();
//		a = i;
//		b = (i + 1) % 24;
//		v_temp.push_back(a);
//		v_temp.push_back(b);
//		v_temp.push_back(b + 48);
//		v_temp.push_back(a + 48);
//		hollow_cube_faces.push_back(v_temp);
//	}
//
//	for (int i = 2; i < 22; i++)
//	{
//		v_temp.clear();
//		a = i + 24;
//		b = (i + 1) % 24 + 24;
//		v_temp.push_back(b);
//		v_temp.push_back(a);
//		v_temp.push_back(a + 48);
//		v_temp.push_back(b + 48);
//		hollow_cube_faces.push_back(v_temp);
//	}
//
//	v_temp.clear();
//	a = 46, b = 26;
//	v_temp.push_back(b);
//	v_temp.push_back(a);
//	v_temp.push_back(a + 48);
//	v_temp.push_back(b + 48);
//	hollow_cube_faces.push_back(v_temp);
//
//
//	v_temp.clear();
//	for (int i = 30; i <= 42; i++)
//	{
//		v_temp.push_back(i);
//	}
//	for (int i = 18; i >= 6; i--)
//	{
//		v_temp.push_back(i);
//	}
//	hollow_cube_faces.push_back(v_temp);
//
//
//	v_temp.clear();
//	for (int i = 26; i <= 30; i++)
//	{
//		v_temp.push_back(i);
//	}
//	for (int i = 0; i < 13; i++)
//	{
//		int tk = (30 - i) % 24;
//		v_temp.push_back(tk);
//	}
//	for (int i = 42; i <= 46; i++)
//	{
//		v_temp.push_back(i);
//	}
//	hollow_cube_faces.push_back(v_temp);
//
//
//
//
//
//	v_temp.clear();
//	for (int i = 42; i >= 30; i--)
//	{
//		v_temp.push_back(i + 48);
//	}
//	for (int i = 6; i <= 18; i++)
//	{
//		v_temp.push_back(i + 48);
//	}
//	hollow_cube_faces.push_back(v_temp);
//
//
//	v_temp.clear();
//	for (int i = 46; i >= 42; i--)
//	{
//		v_temp.push_back(i + 48);
//	}
//
//	for (int i = 0; i < 13; i++)
//	{
//		int tk = (18 + i) % 24 + 48;
//		v_temp.push_back(tk);
//	}
//	for (int i = 30; i >= 26; i--)
//	{
//		v_temp.push_back(i + 48);
//	}
//	hollow_cube_faces.push_back(v_temp);
//
//
//
//	//ofstream fout;
//	//fout.open("hollow_cube.obj");
//	//for (int i = 0; i < hollow_cube_vertex.size(); i++)
//	//{
//	//	fout << "v " << hollow_cube_vertex[i].m_x << " " << hollow_cube_vertex[i].m_y << " " << hollow_cube_vertex[i].m_z << endl;
//	//}
//	//for (int i = 0; i < hollow_cube_faces.size(); i++)
//	//{
//	//	int facet_count = hollow_cube_faces[i].size();
//	//	fout << "f";
//	//	for (int j = 0; j < facet_count; j++)
//	//	{
//	//		fout << " " << hollow_cube_faces[i][j] + 1;
//	//	}
//	//	fout << endl;
//	//}
//	//fout.close();
//}




void General_Mesh::insert(vector<Vec3>& origin_vertexs, vector<v_int>& origin_faces,
	vector<Vec3> vertexs, vector<v_int> faces)
{
	int origin_vertexs_size = origin_vertexs.size();
	origin_vertexs.insert(origin_vertexs.end(), vertexs.begin(), vertexs.end());

	v_int v_temp;
	for (int i = 0; i < faces.size(); i++)
	{
		v_temp.clear();
		int facet_count = faces[i].size();
		for (int j = 0; j < facet_count; j++)
		{
			v_temp.push_back(faces[i][j] + origin_vertexs_size);
		}
		origin_faces.push_back(v_temp);
	}
}
void General_Mesh::insert(vector<Vec3> vertexs, vector<v_int> faces)
{
	insert(result_vertex, result_faces, vertexs, faces);
}

void General_Mesh::genResultMesh(const char* filename)
{
	ofstream fout;
	fout.open(filename);
	//r = g = b = 0;
	if (r != 0 || g != 0 || b != 0)
		for (int i = 0; i < result_vertex.size(); i++)
		{
			fout << "v " << result_vertex[i].m_x << " " << result_vertex[i].m_y << " " << result_vertex[i].m_z << " " << r << " " << g << " " << b << " " << endl;
		}
	else
		for (int i = 0; i < result_vertex.size(); i++)
		{
			fout << "v " << result_vertex[i].m_x << " " << result_vertex[i].m_y << " " << result_vertex[i].m_z << endl;
		}

	//for (int i = 0; i < 376; i++)
	for (int i = 0; i < result_faces.size(); i++)
	{
		int facet_count = result_faces[i].size();
		fout << "f";
		for (int j = 0; j < facet_count; j++)
		{
			fout << " " << result_faces[i][j] + 1;
		}
		fout << endl;
	}
}

//���¼�����������תƽ�Ʒ�������
vector<Vec3> General_Mesh::meshTrans(Vec3 dire, vector<Vec3> point)
{
	vector<Vec3> result;
	int count = point.size();
	for (int i = 0; i < count; i++)
	{
		result.push_back(point[i] + dire);
	}
	return result;
}

vector<Vec3> General_Mesh::meshScale(Vec3 scale, vector<Vec3> point)
{
	vector<Vec3> result;
	int count = point.size();
	for (int i = 0; i < count; i++)
	{
		result.push_back(point[i] * scale);
	}
	return result;
}

vector<Vec3> General_Mesh::meshRotate(Vec3 z_dire, vector<Vec3> point)
{
	z_dire.Normalized();

	Vec3 x_dire, y_dire;

	float t = powf(z_dire.m_x, 2) + powf(z_dire.m_y, 2);

	if (t == 0)
	{
		x_dire.m_x = z_dire.m_z;
	}
	else
	{
		x_dire.m_x = -z_dire.m_y;
		x_dire.m_y = z_dire.m_x;
		x_dire.Normalized();
	}

	y_dire = Cross(z_dire, x_dire);

	vector<Vec3> result;
	for (int i = 0; i < point.size(); i++)
	{
		Vec3 temp = point[i].m_x * x_dire + point[i].m_y * y_dire + point[i].m_z * z_dire;
		result.push_back(temp);
	}

	return result;
}

vector<Vec3> General_Mesh::meshRotate(Vec3 x_dire, Vec3 z_dire, vector<Vec3> point)
{
	x_dire.Normalized();
	z_dire.Normalized();

	Vec3 y_dire;
	y_dire = Cross(z_dire, x_dire);

	vector<Vec3> result;
	for (int i = 0; i < point.size(); i++)
	{
		Vec3 temp = point[i].m_x * x_dire + point[i].m_y * y_dire + point[i].m_z * z_dire;
		result.push_back(temp);
	}

	return result;
}

