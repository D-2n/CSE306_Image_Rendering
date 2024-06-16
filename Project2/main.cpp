#include "classes/polygon.h"
#include "classes/vector.h" 
#include "classes/fluid_anim.h" 
#include "functions/save_svg.h"
#include "functions/lbfgs_sample.h"
#include "functions/save_frame.h"
#include <iostream>
#include <mutex>
#include <random>
#include <thread>

Vector generate_random_point(double min, double max) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(min, max);
    return Vector(dis(gen), dis(gen), 0.0);
}
/*
Polygon create_square_around_point(const Vector& point, double side_length) {
    double half_side = side_length / 2.0;
    return Polygon({
        Vector(point.coords[0] - half_side, point.coords[1] - half_side),
        Vector(point.coords[0] + half_side, point.coords[1] - half_side),
        Vector(point.coords[0] + half_side, point.coords[1] + half_side),
        Vector(point.coords[0] - half_side, point.coords[1] + half_side)
    });
}
*/
int main(){
    
    std::vector<Polygon> polygons;
    const int num_points = 1000; //change as needed
    const double min_coord = 0.0;
    const double max_coord = 1.0;
    std::vector<Vector> points;
    /* Sutherland-Hodgman test
    Polygon test_triangle{
        {Vector(0, 0), Vector(0.5, 0.6), Vector(1, 0)}}; //triangle

    Polygon clipper{
        {Vector(0.2, 0.2), Vector(0.7, 0.2), Vector(0.7, 0.7), Vector(0.2, 0.7)}}; //square

        
    Polygon res = test_triangle.clip(clipper);
    polygons.push_back(test_triangle);
    polygons.push_back(res);
    
    save_svg(polygons, "./clip_test.svg", "none");
    */
    /* Voronoi diagram testing
    Polygon background{
        {Vector(0, 0), Vector(1, 0), Vector(1, 1), Vector(0, 1)}};
    
    
    for (int i = 0; i < num_points; ++i) {
        Vector point = generate_random_point(min_coord, max_coord);
        points.push_back(point); 
    }
    
    for (int i = 0; i < num_points; ++i){
        Polygon result = background.clip_voronoi(points, points[i]);
        polygons.push_back(result);

    OR
    
    polygons = power_diag(newpoints);, just remember to include "classes/power_diagram.h"

    save_svg(polygons, "./voronoi_test.svg", "none");
    }*/
    
    /* Semi-discrete optimal transport and L-BFGS test
    std::vector<double> lambdas(points.size());
    Vector C(0.5, 0.5, 0.0); //hard-coded square center
    double lamdas_sum;
    size_t n = points.size();
    for (uint i = 0; i < n; i++) {
         lambdas[i] = std::exp(-std::pow((points[i] - C).norm(), 2) / 0.02);
         tmp += lambdas[i];
    }
     for (int i = 0; i < n; i++) {
         lambdas[i] /= lamdas_sum;
    }

    
    objective_function lbfgs;
    auto polygons1 = lbfgs.run(points,lambdas);
    */
    //Fluid dynamics test
    /*
    fluid_animation fluid_animator(100, 0.7);

    int n_steps = 100;
    auto frames = fluid_animator.run(n_steps);

    for (int i = 0; i < frames.size(); ++i) {
         save_frame(frames[i], "frames/frame", i);
    }
    */
}