#pragma once
#ifndef __DATASTRUSTURES_H__
#define __DATASTRUSTURES_H__

#include <iostream>
#include <sstream>

// data structures and operations

// a 3d vertex
struct Vertex {
    float x, y, z;
    Vertex(){}
    Vertex(float _x, float _y, float _z) {
        x = _x;
        y = _y;
        z = _z;
    }
    bool inline operator==(const Vertex& b) const {
        return this->x == b.x && this->y == b.y && this->z == b.z;
    }

    bool inline operator!=(Vertex& b) const {
        return !(*this == b);
    }

    // dot product
    float inline dot(const Vertex& b) const {
        return this->x * b.x + this->y * b.y + this->z * b.z;
    }

    // distance of two vertices
    float inline distance(Vertex& b) const {
        float dx = this->x - b.x, dy = this->y - b.y, dz = this->z - b.z;
        return sqrt(dx * dx + dy * dy + dz * dz);
    }

    float inline length() const {
        return sqrt(this->dot(*this));
    }

    Vertex inline operator*(const float a) const
    {
        Vertex r = { a * this->x,a * this->y,a * this->z };
        return r;
    }

    Vertex inline operator+(const Vertex& b) const
    {
        Vertex r = { this->x + b.x,this->y + b.y,this->z + b.z };
        return r;
    }

    Vertex inline operator-(const Vertex& b) const
    {
        return *this + (b * -1.f);
    }

    Vertex inline normalize() const {
        float l = this->length();
        if (l == 0)
        {
            Vertex VV;
            VV.x = 0;
            VV.y = 0;
            VV.z = 0;
            return VV;
        }
        return (*this) * (1.f / l);
    }
    Vertex inline Rotate(double rotate_angle, Vertex v_norm, Vertex  v)
    {
        v_norm = v_norm.normalize();
        double cosa = cos(rotate_angle);
        double sina = sin(rotate_angle);

        Vertex v1 = Vertex(cosa + v_norm.x * v_norm.x * (1 - cosa), v_norm.x * v_norm.y * (1 - cosa) - v_norm.z * sina, v_norm.x * v_norm.z * (1 - cosa) + v_norm.y * sina);
        Vertex v2 = Vertex(v_norm.y * v_norm.x * (1 - cosa) + v_norm.z * sina, cosa + v_norm.y * v_norm.y * (1 - cosa), v_norm.y * v_norm.z * (1 - cosa) - v_norm.x * sina);
        Vertex v3 = Vertex(v_norm.z * v_norm.x * (1 - cosa) - v_norm.y * sina, v_norm.z * v_norm.y * (1 - cosa) + v_norm.x * sina, cosa + v_norm.z * v_norm.z * (1 - cosa));
        return Vertex(v1.dot(v), v2.dot(v), v3.dot(v));
    }
    // simple half ordering in z,y,x order
    // automatically used by orderes containers like std::set, std::map
    bool inline operator<(const Vertex& b) const
    {
        return
            (this->z < b.z) ||
            ((this->z == b.z) && (this->y < b.y)) ||
            ((this->z == b.z) && (this->y == b.y) && (this->x < b.x));
    }
};


// directed order for sorting vertices along a given 3d direction
struct VertexSweepOrder
{
    Vertex dir;
    VertexSweepOrder(Vertex _dir) {
        dir = _dir;
    }

    bool operator() (const Vertex& a, const Vertex& b) const
    {
        return a.dot(dir) < b.dot(dir);
    }
};


// a triangle referencing three vertices
struct Triangle
{
    Vertex* vertices[3];
    Vertex _vertices[3];
    Vertex normal;

    void inline sortTriangleVertices()
    {
        Vertex** vs = this->vertices;
        if (vs[0]->z > vs[1]->z) std::swap(vs[0], vs[1]);
        if (vs[0]->z > vs[2]->z) std::swap(vs[0], vs[2]);
        if (vs[1]->z > vs[2]->z) std::swap(vs[1], vs[2]);
    }
};


// helper struct for sorting vertices by some value
struct VertexIndex
{
    float value;
    Triangle* triangle;

    bool inline operator<(const VertexIndex& b) const
    {
        return this->value < b.value;
    };
};


struct adjoin_mapping_indices {
    int begin;
    int end;
    double weight;
};

// a line segment
// usually resulting of the intersection from a triangle with a z plane
struct Segment
{
    std::array<Vertex, 2> vertices;     // the two endpoints of this segment

    // these are used temporary and are invalidated later. do not use them:
    std::array<Segment*, 2> neighbours; // pointers to this segment adjacent ones

    long orderIndex;  // an index generated to order segments for efficient printing

    Vertex normal;    // segment line normal

    std::array<adjoin_mapping_indices, 2> adjoint_mappint_point;
    //int triangle_index;
    std::vector<Vertex> triangle_points;            //������¼��������Ϣ
    bool inline operator<(const Segment& b) const {
        return this->orderIndex < b.orderIndex;
    }
};


// a layer holding the segments build by intersecting the mesh with a z plane
struct Layer
{

    float z; // z plane
    std::vector<Triangle*> triangles;  // triangles touching this layer
    std::vector<Segment> segments;     // segments generated for printing
};

#endif //__DATASTRUSTURES_H__
