#pragma once
#include "vector.h"

class Edge_class{
public:
    Edge_class(Vector &v1, Vector &v2);
    Vector intersect(Vector &vc1, Vector &vc2);
    Vector w_intersect(WeightedVector &vc1, WeightedVector &vc2);
    bool inside(Vector &vertex);
    Vector v1;
    Vector v2;
};
class w_edge {
public:
    w_edge(WeightedVector &v1, WeightedVector &v2);
    WeightedVector v1;
    WeightedVector v2;
    bool inside_voronoi(Vector &vertex);
};