#include <stdio.h>
#include "../lbfgs/lbfgs.h"
#include "../classes/vector.h"
#include "../classes/polygon.h"
#include "../classes/power_diagram.h"
#include "lbfgs_sample.h"
#include <cmath>
#include <iostream>
#include <random>


// Sample taken from the provided github repository: https://github.com/chokkan/liblbfgs/blob/master/sample/sample.cpp


// Constructor
objective_function::objective_function() : m_x(NULL) {}

// Destructor
objective_function::~objective_function() {
    if (m_x != NULL) {
        lbfgs_free(m_x);
        m_x = NULL;
    }
}

std::vector<Polygon> objective_function::run(std::vector<Vector> input_points, std::vector<double> lambdas) {
    this->points = input_points;
    lbfgsfloatval_t fx;
    int n = points.size();
    lbfgsfloatval_t *m_x = lbfgs_malloc(n);
    this->lambdas = lambdas;

    if (m_x == NULL) {
        printf("ERROR: Failed to allocate a memory block for variables.\n");
        return polygons;
    }
    for (size_t i = 0; i < n; ++i) {
        m_x[i] = 0.5; // SAMPLE FOR NOW CHANGE THIS LATER!!!!!
    }

    int ret = lbfgs(n, m_x, &fx, _evaluate, _progress, this, NULL);

    printf("L-BFGS optimization terminated with status code = %d\n", ret);
    printf(lbfgs_strerror(ret));
    printf("\n  fx = %f, x[0] = %f, x[1] = %f\n", fx, m_x[0], m_x[1]);
    return this->polygons;
}

lbfgsfloatval_t objective_function::_evaluate(
    void *instance,
    const lbfgsfloatval_t *x,
    lbfgsfloatval_t *g,
    const int n,
    const lbfgsfloatval_t step
) {
    return reinterpret_cast<objective_function *>(instance)->evaluate(x, g, n, step);
}

lbfgsfloatval_t objective_function::evaluate(
    const lbfgsfloatval_t *x,
    lbfgsfloatval_t *g,
    const int n,
    const lbfgsfloatval_t step) {

    lbfgsfloatval_t fx = 0.0;  

    std::vector<WeightedVector> newpoints(n);
    for (size_t i = 0; i < n; ++i){
        newpoints[i] =  WeightedVector(points[i],x[i]);
    }
    //make lambdas
    

    this->polygons = power_diag(newpoints);
    
    #pragma omp parallel for
    for (int i = 0; i < n; i++) {
        
        Polygon poly = polygons[i];
        double area = poly.compute_area();
        g[i] = area - this->lambdas[i];
        std::vector<Triangle> triangulation = poly.triangulate(points[i]);
        
        double integral_approx = 0;
        for (size_t t = 0; t < triangulation.size(); ++t) {
            double sum = 0;
            Triangle triangle = triangulation[t];
            for (int k = 0; k < 3; k++) {
                for (int l = k; l < 3; l++) {
                    sum += ((triangle.vertices[k] - this->points[i]).dot(triangle.vertices[l] - this->points[i]));
                }
            }
           
            Polygon triangle_poly;
            for (size_t v = 0; v < triangle.vertices.size(); ++v){
                triangle_poly.vertices.push_back(triangle.vertices[v]);
            }
            double T_area = triangle_poly.compute_area();
            
            integral_approx += (T_area / 6) * sum;
        }
        fx += integral_approx - x[i]*area + this->lambdas[i] * x[i];
        //fx += integral_approx;
        //std::cout << fx << "\n";
    }
    //std::cout << "Aa" << fx;
    return -fx;
}
int objective_function::_progress(
        void *instance,
        const lbfgsfloatval_t *x,
        const lbfgsfloatval_t *g,
        const lbfgsfloatval_t fx,
        const lbfgsfloatval_t xnorm,
        const lbfgsfloatval_t gnorm,
        const lbfgsfloatval_t step,
        int n,
        int k,
        int ls
        )
    {
        return reinterpret_cast<objective_function*>(instance)->progress(x, g, fx, xnorm, gnorm, step, n, k, ls);
    }

int objective_function::progress(
        const lbfgsfloatval_t *x,
        const lbfgsfloatval_t *g,
        const lbfgsfloatval_t fx,
        const lbfgsfloatval_t xnorm,
        const lbfgsfloatval_t gnorm,
        const lbfgsfloatval_t step,
        int n,
        int k,
        int ls
        )
    {
        printf("Iteration %d:\n", k);
        printf("  fx = %f, x[0] = %f, x[1] = %f\n", fx, x[0], x[1]);
        printf("  xnorm = %f, gnorm = %f, step = %f\n", xnorm, gnorm, step);
        printf("\n");
        return 0;
    }







