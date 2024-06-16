#pragma once
#include "vector.h"
#include "edge_class.h"

#include <vector>

class Triangle{
public:
    explicit Triangle(std::vector<Vector> vertices = {});
    std::vector<Vector> vertices;

};
class Polygon{
public:
    explicit Polygon(std::vector<Vector> vertices = {});
    std::vector<Vector> vertices;
    std::vector<Edge_class> edges;
    Polygon clip(Polygon &clipPolygon);
    Polygon clip_voronoi(std::vector<WeightedVector> &points, WeightedVector &p_i);
    std::vector<Triangle> triangulate(Vector &main_point);
    double compute_area() const;
    Vector compute_center() const;

};
