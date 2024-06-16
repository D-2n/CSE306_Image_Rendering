#include "../functions/lbfgs_sample.h"
#include "polygon.h"
#include "vector.h"
#include <random>

class fluid_animation {
public:
    fluid_animation(int n, double volume);
    std::vector<std::vector<Polygon>> run(int n_steps);
    std::vector<Polygon> step();

    objective_function lbfgs;
    double volume;
    std::vector<Vector> points;
    std::vector<Vector> velocities;
    double m = 50;
    Vector g = Vector(0, -9.81, 0);
    double eps = 0.004;
    double dt = 0.001;
};