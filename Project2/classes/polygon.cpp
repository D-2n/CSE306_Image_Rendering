#include "polygon.h"
#include "vector.h"
#include "edge_class.h"
#include "iostream"
#include <vector>
#include <cmath>
Triangle::Triangle(std::vector<Vector> vertices) : vertices(vertices) {};

Polygon::Polygon(std::vector<Vector> vertices) : vertices(vertices){};

void generate_edges(Polygon *polygon){ //CALL ONLY ON EMPTY POLYGONS
    for (int i = 0; i < polygon->vertices.size(); ++i){
        Vector v1 = polygon->vertices[i];
        Vector v2 = polygon->vertices[(i > 0 ) ? i - 1 : polygon->vertices.size()-1];
        polygon->edges.push_back(Edge_class(v2,v1));
    }
}
/*
Vector intersect(Vector &v1, Vector &v2, Edge_class &edge){
    Vector N(v2[1] - v1[1], v1[0] - v2[0],0.0);
    double t = (v1 - edge.v1).dot(N) / (edge.v2 - edge.v1).dot(N);
    return edge.v1 + (edge.v2 - edge.v1) * t;
}

WeightedVector w_intersect(WeightedVector &v1, WeightedVector &v2, w_edge &edge){//with laguerre diagram implementation please note that!
    Vector M = (v1 + v2) / 2;   
    Vector M_prime = M + (v2-v1)*(v1.weight - v2.weight)/(2*((v2 - v1).norm2()));
    double t = (M_prime - edge.v1).dot(v1 - v2) / (edge.v2 - edge.v1).dot(v1 - v2);
    return edge.v1 + (edge.v2 - edge.v1) * t;
}
*/
Polygon Polygon::clip(Polygon &clipPolygon){
    Polygon subjectPolygon (*this);
    generate_edges(&clipPolygon);
    for (auto edge : clipPolygon.edges){
        Polygon res = Polygon();
        for (int i = 0; i < subjectPolygon.vertices.size(); ++i){

            Vector curr_vertex = subjectPolygon.vertices[i];
            Vector prev_vertex = subjectPolygon.vertices[( i > 0 ) ?  i - 1 : subjectPolygon.vertices.size()-1];
            Vector intersection = edge.intersect(prev_vertex, curr_vertex);

            if (edge.inside(curr_vertex)){
                if (!edge.inside(prev_vertex)){
                    res.vertices.push_back(intersection);
                }
                res.vertices.push_back(curr_vertex);
            }else if(edge.inside(prev_vertex)){
                res.vertices.push_back(intersection);
            }
        }
        subjectPolygon = res;   
    }
    return subjectPolygon;
}
std::vector<Triangle> Polygon::triangulate(Vector &main_point){
    std::vector<Triangle> triang_result;
    for (size_t i = 1; i < vertices.size(); i++) {
        Triangle triangle;
        Vector v1 = vertices[i];
        Vector v2 = vertices[(i + 1) % vertices.size()];
        triangle.vertices.push_back(vertices[0]);
        triangle.vertices.push_back(v1);
        triangle.vertices.push_back(v2);
        triang_result.push_back(triangle);
    }
    
    return triang_result;
}
Polygon Polygon::clip_voronoi(std::vector<WeightedVector> &points, WeightedVector &p_i){
    Polygon subjectPolygon (*this);
    
    #pragma omp parallel for
    for (auto point : points){
        Polygon res = Polygon();
        if (point == p_i){
            continue;
        }
        for (int i = 0; i < subjectPolygon.vertices.size(); ++i){

            Vector curr_vertex = subjectPolygon.vertices[i];
            Vector prev_vertex = subjectPolygon.vertices[( i > 0 ) ?  i - 1 : subjectPolygon.vertices.size()-1];
            Edge_class clipper_edge = Edge_class(prev_vertex, curr_vertex);
            w_edge p_edge = w_edge(p_i, point);

            Vector intersection = clipper_edge.w_intersect(p_i, point);

            if (p_edge.inside_voronoi(curr_vertex)){
                if (!p_edge.inside_voronoi(prev_vertex)){
                    res.vertices.push_back(intersection);
                }
                res.vertices.push_back(curr_vertex);
            }else if(p_edge.inside_voronoi(prev_vertex)){
                res.vertices.push_back(intersection);
            }
        }
        subjectPolygon = res;   
    }
    return subjectPolygon;
}
double Polygon::compute_area() const {
    double area = 0.0;
    int n = vertices.size();

    for (int i = 0; i < n; i++) {
        int j = (i + 1) % n; 
        double x_i = vertices[i].coords[0];
        double y_i = vertices[i].coords[1];
        double x_j = vertices[j].coords[0];
        double y_j = vertices[j].coords[1];
        
        double term = x_i * y_j - x_j * y_i;
        
        area += term;
    }

    double final_area = std::abs(area / double(2.0));
    return final_area;
}
Vector Polygon::compute_center() const { //TODO: vrati ovo u projekat
    double area = this->compute_area();
    double c_x = 0.0;
    double c_y = 0.0;

    for (std::size_t i = 0; i < vertices.size(); ++i) {
        std::size_t index = (i + 1) % vertices.size();
        double cross = vertices[i].cross(vertices[index])[2];
        c_x += (vertices[i][0] + vertices[index][0]) * cross;
        c_y += (vertices[i][1] + vertices[index][1]) * cross;
    }

    c_x /= (6.0 * area);
    c_y /= (6.0 * area);

    return Vector(c_x, c_y);
}