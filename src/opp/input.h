#pragma once
#ifndef INPUT_H_
#define INPUT_H_

#include <string>
#include <cstring>



#include			"data.h"
#include		 "polygon.h"
#include "..\slice\katana.h"

class Input {
public:
	const char* model_path;
	const char* config_path;

	void DefaultPrintingDirectionSlice();
	void SamplingPrintingDirectionSlice();

	double CalculateFlattenArea(const std::string& path);
	int JudgeSupportFallOnGround(const Data& data, int& sum_slice_point);
private:
	void KanataClear();
};



#endif 