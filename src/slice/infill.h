#ifndef __INFILL_H__
#define __INFILL_H__

#include "datastructures.h"

class Infill {
public:
    // compute 'infill', a hatching pattern to fill the inner area of a layer
    // // it is made by a line grid alternating between +/-45 degree on odd and even layers
    void hatch(int layerIndex, Layer& layer);
};

#endif
