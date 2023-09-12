#pragma once
#ifndef __STL_H__
#define __STL_H__

#include "datastructures.h"
#include "../opp/helpers.h"
#include "sample_on_ball.h"
#include "datastructures.h"

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
#include <direct.h>

#include "config.h"

class STLReader {
private:
public:
    // load an ASCII .stl file
    // fill the vertices and triangle list. the vertices are unified while loading.
    //void loadStl(const char* filename, std::vector<Vertex>& vertices, std::vector<Triangle>& triangles);
    void loadStl(const char* filename);
    void saveStl(std::string filename, const std::vector<Triangle>& triangles);
    void getMultiDirectionStl(const char* filename);

    std::vector<Vertex>   vertices;
    std::vector<Triangle> triangles;

    std::vector<Triangle> f_triangles;
};

#endif //__STL_H__
