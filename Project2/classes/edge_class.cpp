#include "edge_class.h"

Edge_class::Edge_class(Vector &v1, Vector &v2) : v1(v1), v2(v2) {}

w_edge::w_edge(WeightedVector &v1, WeightedVector &v2) : v1(v1), v2(v2) {}

Vector Edge_class::intersect(Vector &vc1, Vector &vc2) {
    Vector N(vc2[1] - vc1[1], vc1[0] - vc2[0]);
    double t = (vc1 - v1).dot(N) / (v2 - v1).dot(N);
    return v1 + (v2 - v1) * t;
};

Vector Edge_class::w_intersect(WeightedVector &vc1, WeightedVector &vc2) {
    Vector M = (vc1 + vc2) / 2;
    Vector M_prime = M + (vc2 - vc1) * (vc1.weight - vc2.weight) / (2 * (vc2 - vc1).norm2());
    double t = (M_prime - v1).dot(vc1 - vc2) / (v2 - v1).dot(vc1 - vc2);
    return v1 + (v2 - v1) * t;
}
bool Edge_class::inside (Vector &vertex){
    // P-u.norm(N)
    Vector u = (v1 + v2) / 2;

    Vector N(v2[1] - v1[1], v1[0] - v2[0],0.0);
    double res = (vertex - u).dot(N); //define
    return res <= 0;
}
bool w_edge::inside_voronoi (Vector &vertex){ //with laguerre diagram implementation please note that!
    Vector M = (v1 + v2) / 2;
    Vector M_prime = M + (v2-v1)*(v1.weight - v2.weight)/(2*((v2 - v1).norm2()));
    double res = (vertex - M_prime).dot(v2 - v1); 
    return res <= 0;
}
