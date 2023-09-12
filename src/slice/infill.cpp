
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

#include "datastructures.h"
#include "config.h"
#include "stl.h"
#include "katana.h"
#include "gcode.h"
#include "slicer.h"

#include "infill.h"

// compute 'infill', a hatching pattern to fill the inner area of a layer
// it is made by a line grid alternating between +/-45 degree on odd and even layers
void Infill::hatch(int layerIndex, Layer& layer)
{

    // make a offset copy of the contour to fill to avoid overlapping the perimeter
    std::vector<Segment> segments = layer.segments;
    // TODO how much should we shrink the contour here?
    // about nozzle_diameter, because the extrusions would exactly touch then ?
    // about nozzle_diameter/2, because the extrusions would definitely merge then?
    // a larger value tends to make gaps in thin walls. try something inbetween now.
    //Katana::Instance().slicer.offsetSegments(segments,-Katana::Instance().config.get("nozzle_diameter")/1.5f);

    std::map<Vertex, std::vector<Segment*>> segmentsByVertex;
    Katana::Instance().slicer.unifySegmentVertices(segments, segmentsByVertex);

    // we compute the infill by using a 'plane sweep'.
    // see http://en.wikipedia.org/wiki/Sweep_line_algorithm
    // for that the vertices are ordered in the fill pattern hatching direction
    // the vertices are then iterated one by one and a heap of active segments is maintained
    // that can then be used to efficiently intersect the pattern lines at the given cut

    // place grid lines by nozzle diameter for 100% infill
    float grid_spacing = Katana::Instance().config.get("nozzle_diameter");

    // 45 degree hatching pattern directions
    Vertex dir = { sqrt(2.f) / 2,sqrt(2.f) / 2,0 };
    Vertex dirOrthogonal = { dir.y,-dir.x,0 };

    // every even layer we swap the directions to get a plywood like 3d pattern
    if (layerIndex % 2 == 0)
        std::swap(dir, dirOrthogonal);

    // initialize two orders that sort vertices along dir or dirOrthogonal
    VertexSweepOrder order(dir);
    VertexSweepOrder orderOrthogonal(dirOrthogonal);

    // create ordered vertex list for the sweep
    std::vector<Vertex> sweepVertices;
    for (std::map<Vertex, std::vector<Segment*>>::iterator i = segmentsByVertex.begin(); i != segmentsByVertex.end(); ++i) {
        assert(i->first.z == layer.z);
        sweepVertices.push_back(i->first);
    }
    std::sort(sweepVertices.begin(), sweepVertices.end(), VertexSweepOrder(dir));

    // the list of infill line segments
    std::vector<Segment> infill;

    // the plane sweep heap.
    // in every sweep step, this is updated to contain the segments that interact with a hatching line
    std::set<Segment*> sweepHeap;

    // the sweep progress distance in direction dir.
    // initialize by the first vertex in dir
    float sweepT = dir.dot(*sweepVertices.begin()) + grid_spacing;

    // ascending index written to the segments to sort them later
    long orderIndex = 0;

    // the plane sweep, hopping from vertex to vertex along dir
    for (std::vector<Vertex>::iterator i = sweepVertices.begin(); i != sweepVertices.end(); ++i)
    {
        const Vertex& v = *i;

        // check if the sweep has passed the next crosshatch line
        // and fill lines until the current sweep vertex is reached
        while (v.dot(dir) > sweepT) {

            // collect intersections
            std::vector<Vertex> intersections;
            for (std::set<Segment*>::iterator j = sweepHeap.begin(); j != sweepHeap.end(); ++j) {
                Segment& s = **j;
                Vertex& a = s.vertices[0], & b = s.vertices[1];

                // compute intersection length on segment
                float aInDir = a.dot(dir);
                float bInDir = b.dot(dir);
                // assert(std::min(aInDir,bInDir)<sweepT+.00001f);
                // assert(sweepT<max(aInDir,bInDir)); // disabled to accept non manifolds
                //float d=bInDir-aInDir;
                float t = (sweepT - aInDir) / (bInDir - aInDir);

                // add intersection
                if (t >= 0 && t <= 1) { // for manifolds this should always be true
                    Vertex intersection = {
                      a.x + t * (b.x - a.x),
                      a.y + t * (b.y - a.y),
                      a.z // z const in layer
                    };
                    intersections.push_back(intersection);
                }
            }
            // assert(intersections.size() % 2 == 0);  // disabled to accept non manifolds

            // sort intersections in dirOrthogonal, perpendicular to the sweep direction
            std::sort(intersections.begin(), intersections.end(), orderOrthogonal);

            // add fill line segments
            // the filling toggles on every intersection, starting with the leftmost outline
            // a pathIndex is used to keep the generated segments ordered by the path taken first
            // otherwise the printer would need to fill the disconnected hatch line using useless travels
            // TODO as pathes are not identifiable, senseless travels occur where pathes appear and vanish
            long pathIndex = 1;
            for (unsigned int j = 0; j < intersections.size(); j++) {
                if (j % 2 == 1 && intersections[j - 1].distance(intersections[j]) >= .5f) {
                    Segment s = {
                      {intersections[j - 1],intersections[j]},
                      {NULL,NULL},
                      pathIndex * 0x10000L + orderIndex // order by path, then by cut index
                    };

                    // add segment
                    infill.push_back(s);

                    // advance mayor sort index for every path
                    pathIndex++;
                }
            }

            sweepT += grid_spacing;
            orderIndex++; // advance minor sort index
        }
        // the filling is on par, now update sweep heap

        std::vector<Segment*>& ss = segmentsByVertex[v]; // the segments touching this sweep point

        // count segments linking the current vertex and already in the heap
        char segments_in_heap = 0; // number of segments already in heap
        char segment_index = -1;   // store segment already there
        // ignore non manifold unconnected segments
        if (ss.size() != 2) continue;
        for (unsigned int j = 0; j < ss.size(); j++) {
            assert(ss[j]->vertices[0] == v || ss[j]->vertices[1] == v);
            if (sweepHeap.count(ss[j]) == 1) {
                segments_in_heap++;
                segment_index = j;
            }
        }

        // assert(segments_in_heap<=2); // disabled to accept non manifolds

        // now we can decide what happens at this sweep coordinate
        if (segments_in_heap == 0) {
            // a new perimeter is encountered. add it to the heap.
            // as perimeters are closed, there are two segments spawning from this new vertex
            sweepHeap.insert(ss[0]);
            sweepHeap.insert(ss[1]);
        }
        else if (segments_in_heap == 1) {
            // an existing perimeter continues, segment_index must be it's index.
            // so remove this old segment and add the new one
            sweepHeap.erase(ss[segment_index]);
            sweepHeap.insert(ss[1 - segment_index]);
        }
        else if (segments_in_heap == 2) {
            // an existing perimeter ends.
            // as perimeters are closed, there are two segments ending here.
            sweepHeap.erase(ss[0]);
            sweepHeap.erase(ss[1]);
        }
    }
    // the sweep is over, if the mesh was manifold the heap should be empty again
    // assert(sweepHeap.size()==0); // disabled to accept non manifolds

    layer.segments.insert(layer.segments.end(), infill.begin(), infill.end());
}

