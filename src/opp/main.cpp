//////////////////////////////////////////////**********************************////////////////////////////////////////
//  In this version, only flat layer module is available, Curvislicer is not necessary.
//////////////////////////////////////////////**********************************////////////////////////////////////////

#include "data.h"
#include "helpers.h"
#include "polygon.h"
#include "layer_graph.h"
#include "opp_graph.h"
#include "input.h"

#include "..\slice\visual.h"
#include "..\slice\sample_on_ball.h"
clock_t start_time, end_time;



int main()
{
	nozzle the_nozzle;
	the_nozzle.upper_surface_r = 4.5; 
	the_nozzle.lowwer_surface_r = 1; 
	the_nozzle.nozzle__H_total = 8;  
	the_nozzle.nozzle_H_half = 2.5; 

	Input input;
	input.config_path = "config.ini";
	input.model_path = "..\\model\\Results_grail_FDM.stl";   //Note that you should also revise the name of model in the helpers.h file.
	
	std::vector<Data> data;

	input.DefaultPrintingDirectionSlice();
	data.resize(1);
	data[0].ReadData("slice_layers.txt");

	Layer_Graph layer_graph(data[0]);
	layer_graph.BuildLayerGraph(the_nozzle);
	layer_graph.GetInitialOPP();

	OPP_Graph opp_graph(data[0], layer_graph.initial_opp_info, layer_graph.temp_edges,layer_graph.cont_normal_dependency_edges);
	opp_graph.BuildOPPGraph_G0();

	opp_graph.MergeOPP_First_ALL_SearchTree();

	std::cout << "&&&&&&& III-OPP merge time = " << double(end_time - start_time) / CLOCKS_PER_SEC << "s &&&&&&&" << std::endl;
	opp_graph.GeneratePath();

	connect_by_zigzag_and_spiral(5, true, 1, true);

	Visual vis;
	//vis.generateModelForRendering_2(file_name + "_initial_opp");
	vis.generateModelForRendering_2(file_name + "_medium_opp");



}
