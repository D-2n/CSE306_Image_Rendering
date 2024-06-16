#include "polygon.h"
#include "vector.h"
#include "edge_class.h"
#include "iostream"
#include <vector>
#include <cmath>

std::vector<Polygon> power_diag(std::vector<WeightedVector> points) {
    std::vector<Polygon> polys;

    
    Polygon background{
        {Vector(0.0, 0.0, 0.0), Vector(1.0, 0.0,0.0), Vector(1.0,1.0,0.0), Vector(0.0, 1.0,0.0)}
    };

    for (int i = 0; i < points.size(); ++i) {
        Polygon voronoi_polygon = background.clip_voronoi(points, points[i]);
        polys.push_back(voronoi_polygon);
    }
    return polys;
}