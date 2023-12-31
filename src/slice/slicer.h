#pragma once
#ifndef __SLICER_H__
#define __SLICER_H__

#include "datastructures.h"
extern float layer_height;
class Slicer {

public:
    // create initialized layers and assign triangles to them
    //void buildLayers(std::vector<Triangle>& triangles, std::vector<Layer>& layers, float &min_z);
    void buildLayers();

    // build segments to be printed for a layer
    // first, the contour gained by intersecting the triangles with it's z plane
    // second, the infill as generated by fill(..)
    //void buildSegments(int layerIndex, Layer& layer);
    void buildSegments(std::vector<Vertex> ori_space_points,std::map<int, int> indices_mapping, std::string mapping_file_name);
    void buildSegments();
    // unify the vertices shared by more than one segment to a map that can be used to find adjacent segments.
    // for manifold geomertry, every vertex mappes to exactly two segments then.
    // however for non manifold geometry, segmentsByVertex can map to any number of segments.
    void unifySegmentVertices(std::vector<Segment>& segments, std::map<Vertex, std::vector<Segment*>>& segmentsByVertex);

    // offset segments by moving them in normal direction and recompute vertices
    // this is used to match an extruded segment of certain width to the outer contour of the model
    // and place the infill inside of the perimeters
    void offsetSegments(std::vector<Segment>& segments, float offset);

    // compute intersection of a segment given by two vertices with a z plane
    Vertex computeIntersection(Vertex& a, Vertex& b, float z);

    // compute intersection of a triangle with a z plane
    // the triangle vertices must be ordered in z
    Segment computeSegment(Triangle& t, float z);
    Vertex calTriNormal(Vertex ver1, Vertex ver2, Vertex ver3);
};

#endif
