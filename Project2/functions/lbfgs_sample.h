#ifndef OBJECTIVE_FUNCTION_H
#define OBJECTIVE_FUNCTION_H

#include "../lbfgs/lbfgs.h"
#include "../classes/vector.h"
#include "../classes/polygon.h"
#include <vector>

// Sample taken from the provided github repository: https://github.com/chokkan/liblbfgs/blob/master/sample/sample.cpp
class objective_function
{
protected:
    lbfgsfloatval_t *m_x;

public:
    std::vector<Vector> points; //cell points
    std::vector<Polygon> polygons;
    std::vector<double> lambdas;
    objective_function();
    virtual ~objective_function();
    std::vector<Polygon> run(std::vector<Vector> input_points, std::vector<double> lambdas);

protected:
    static lbfgsfloatval_t _evaluate(
        void *instance,
        const lbfgsfloatval_t *x,
        lbfgsfloatval_t *g,
        const int n,
        const lbfgsfloatval_t step
    );

    lbfgsfloatval_t evaluate(
        const lbfgsfloatval_t *x,
        lbfgsfloatval_t *g,
        const int n,
        const lbfgsfloatval_t step
    );
    static int _progress(
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
        );

        int progress(
        const lbfgsfloatval_t *x,
        const lbfgsfloatval_t *g,
        const lbfgsfloatval_t fx,
        const lbfgsfloatval_t xnorm,
        const lbfgsfloatval_t gnorm,
        const lbfgsfloatval_t step,
        int n,
        int k,
        int ls
        );
};

#endif // OBJECTIVE_FUNCTION_H
