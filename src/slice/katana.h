#pragma once
#ifndef __KATANA_H__
#define __KATANA_H__

#include <iostream>
#include <sstream>
#include "config.h"
#include "datastructures.h"
#include "debug.h"
#include "slicer.h"
#include "infill.h"
#include "gcode.h"
#include "stl.h"
#include "resampling.h"
extern void connect_by_zigzag_and_spiral(double S,bool choose_spiral, int step, bool is_FDM);
class Katana
{
public:
    static Katana& Instance()
    {
        // Since it's a static variable, if the class has already been created,
        // It won't be created again.
        // And it **is** thread-safe in C++11.

        static Katana myInstance;
        // Return a reference to our instance.
        return myInstance;
    }

    // delete copy and move constructors and assign operators
    Katana(Katana const&) = delete;             // Copy construct
    Katana(Katana&&) = delete;                  // Move construct
    Katana& operator=(Katana const&) = delete;  // Copy assign
    Katana& operator=(Katana&&) = delete;      // Move assign

    STLReader stl;
    Config config;
    Slicer slicer;
    Infill infill;
    GCodeWriter gcode;
    std::map<Vertex, int> mappint_para_point_index;
    /*void loadConfig(const char *filename) {
      config.loadConfig(filename);
    }*/
    std::vector<Vertex>   vertices;
    std::vector<Triangle> triangles;

    std::vector<Layer> layers;
    float min_z;

protected:
    Katana()
    {
        // Constructor code goes here.
    }

    ~Katana()
    {
        // Destructor code goes here.
    }

    // And any other protected methods.
};


#endif //__KATANA_H__
