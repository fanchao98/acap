#ifndef __GCODE_H__
#define __GCODE_H__

#include "datastructures.h"

class GCodeWriter {
public:
    // save Gcode
    // iterates over the previously generated layers and emit gcode for every segment
    // uses some configuration values to decide when to retract the filament, how much
    // to extrude and so on.
    //void write(const char* filename, std::vector<Layer>& layers, float min_z);
    void write(const char* filename);
};

#endif //__GCODE_H__
