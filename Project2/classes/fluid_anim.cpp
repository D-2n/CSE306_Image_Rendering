#include "fluid_anim.h"
#include <random>
Vector generate_random_point1(double min, double max) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(min, max);
    return Vector(dis(gen), dis(gen), 0.0);
}

fluid_animation::fluid_animation(int num_points, double volume) : volume(volume) {
    for (int i = 0; i < num_points; ++i) {
        Vector point = generate_random_point1(0.0, 1.0);
        points.push_back(point); 
    }
    velocities = std::vector<Vector>(num_points, Vector(0, 0));
};

std::vector<std::vector<Polygon>> fluid_animation::run(int n_steps) {
    std::vector<std::vector<Polygon>> frames(n_steps);
    for (int i = 0; i < n_steps; ++i) {
        printf("Step %d: simulated\n", i);
        frames[i] = step();
    }
    return frames;
};
std::vector<Polygon> fluid_animation::step() {
    double eq_lambda = volume / (points.size() - 1);
    std::vector<double> lambdas(points.size(), eq_lambda);
    lambdas[0] = 1 - volume;
    auto polygons = lbfgs.run(points, lambdas);
    for (int i = 0; i < points.size(); ++i) {
        Vector centroid = polygons[i].compute_center();
        Vector F_spring = (centroid - points[i]) / (eps*eps);
        Vector F = F_spring + g * m;

        velocities[i] = velocities[i] + F * (dt / m);
        points[i] = points[i] + velocities[i] * dt;
    }
    return polygons;
};
