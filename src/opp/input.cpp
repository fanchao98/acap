#include "input.h"

void Input::DefaultPrintingDirectionSlice()
{
	Katana::Instance().config.loadConfig(this->config_path);
	Katana::Instance().stl.loadStl(this->model_path);
	Katana::Instance().slicer.buildLayers();
	Katana::Instance().slicer.buildSegments();
	Katana::Instance().gcode.write("slice_layers.txt");
	this->KanataClear();
}

void Input::SamplingPrintingDirectionSlice()
{
	Katana::Instance().config.loadConfig(this->config_path);
	Katana::Instance().stl.getMultiDirectionStl(this->model_path);
	std::string derectory = "../model/slice_layer/" + file_name;
	_mkdir(derectory.c_str());
	for (int i = 0; i < total_directions; i++) {
		char _model_path[50] = "..\\model\\diff_dir\\";
		strcat_s(_model_path, (file_name + "\\" + std::to_string(i) + suffix_stl).c_str());
		std::cout << _model_path << std::endl;
		Katana::Instance().stl.loadStl(_model_path);
		Katana::Instance().slicer.buildLayers();
		Katana::Instance().slicer.buildSegments();
		std::string result = "..\\model\\slice_layer\\"+ file_name+"\\slice_layers"+ std::to_string(i) + suffix_txt;
		Katana::Instance().gcode.write(result.c_str());

		//clear
		this->KanataClear();
	}

}

int Input::JudgeSupportFallOnGround(const Data& data,int& sum_slice_point)
{
	//generate support area polygon
	std::vector<std::vector<Polygon>> polygons;
	polygons.resize(data.slice_points.size());
	for (int i = 0; i < data.slice_points.size(); i++) {
		for (int j = 0; j < data.slice_points[i].size(); j++) {
			polygons[i].push_back(Polygon(ConstructPolygonPoints(data.slice_points[i][j], support_offset)));
			sum_slice_point += data.slice_points[i][j].size()*2;
		}
	}
	//from top to bottom
	int support_point_num = 0;
	int intersect_layer;
	int admit_error = 60;
	for (int i = data.slice_points.size() - 1; i >= 1; i--) {
		for (int j = 0; j < data.slice_points[i].size(); j++) {
			for (int k = 0; k < data.slice_points[i][j].size(); k++) {
				intersect_layer = -1;
				//three conditions:
				//-1 intersect with ground,add i - 1 support points
				//i - 1 self-supported
				//-1 < < i - 1  not height field  
				bool jud_break = false;
				bool jud_error = false;
				for (int ii = i - 1; ii >= 0; ii--) {
					if (jud_break == true)
						break;
					for (int m = 0; m < data.slice_points[ii].size(); m++) {
						if (polygons[ii][m].JudgePointInside(data.slice_points[i][j][k])) {
							intersect_layer = ii;
							if (ii != i - 1) {
								admit_error--;
								jud_error = true;
								if (admit_error <= 0)
									return -1;
								else {
									jud_break = true;
									break;
								}
									
							}
							else 
								break;
						}
					}
					if (intersect_layer != -1) break;
				}
				if (intersect_layer == -1 && jud_error == false) {
					support_point_num += (i - 1);
					//support_point_num += 1;
				}
			}
   		}
 	}
	return support_point_num;
}
double Input::CalculateFlattenArea(const std::string& path)
{
	double flatten_area = 0.0;
	double sum_area = 0.0;
	STLReader stl;
	stl.loadStl(path.c_str());
	for (int i = 0; i < stl.f_triangles.size(); i++) {
		Triangle t = stl.f_triangles[i];
		double cos = t.normal.dot(Vertex(0, 0, 1));
		/*if (cos >= 0.98481) {
			double a = ((*t.vertices[0]) - (*t.vertices[1])).length();
			double b = ((*t.vertices[0]) - (*t.vertices[2])).length();
			double c = ((*t.vertices[2]) - (*t.vertices[1])).length();
			double p = (a + b + c) / 2.0;
			if ((abs(p - a) <= 0.01) || (abs(p - b) <= 0.01) || (abs(p - c) <= 0.01)) continue;
			double area = sqrt(p * (p - a) * (p - b) * (p - c));
			flatten_area += area;
		}*/
		double a = ((*t.vertices[0]) - (*t.vertices[1])).length();
		double b = ((*t.vertices[0]) - (*t.vertices[2])).length();
		double c = ((*t.vertices[2]) - (*t.vertices[1])).length();
		double p = (a + b + c) / 2.0;
		if ((abs(p - a) <= 0.01) || (abs(p - b) <= 0.01) || (abs(p - c) <= 0.01)) continue;
		double area = sqrt(p * (p - a) * (p - b) * (p - c));
		//if (abs(cos) >= 0.9) {
			flatten_area += area * abs(cos);
		//}
		sum_area += area;
	}
	this->KanataClear();
	flatten_area = flatten_area / sum_area;
	return flatten_area;
}
void Input::KanataClear()
{
	Katana::Instance().triangles.clear();
	Katana::Instance().vertices.clear();
	Katana::Instance().triangles.clear();
	Katana::Instance().vertices.clear();
	Katana::Instance().layers.clear();
	Katana::Instance().min_z = 0;
}
