#include <iostream>
#include <math.h>

const double ZERO_THRESH = 1e-8;

const double lower_bound = -100000;
const double upper_bound = 100000;
const double increment = 0.1;

typedef double (*FunctionPtr)(double);
 
double func(double x) {
    return pow(x, 4) + 3 * pow(x, 3) - x + 6;
} 

double func1(double x) {
    return pow(x, 4) + 3 * pow(x, 3) - x + 5;
} 
double func2(double x) {
    return pow(x, 4) + 3 * pow(x, 3) - x + 4;
} 
double func3(double x) {
    return pow(x, 4) + 3 * pow(x, 3) - x + 3;
}

/** 
 * SPECIFICATION
 * requires: func(x1) * func(xn) <= 0;
 * returns: the root of func(x) in interval [x1, xn]
 * throws an error if the arguments violate the requirement
 */
double fzero(double (*func)(double), double x1, double xn) {
    // check whether the arguments are legal or not
    if (func(x1) * func(xn) > 0) {
        fprintf(stderr, "fzero() fails: func(x1) * func(xn) >= 0\n");
        exit( 1 );
    }
 
    // solve for the root iteratively
    double res = x1;
    while (fabs(func(res)) > ZERO_THRESH) {
        res = (x1*func(xn) - xn*func(x1)) / (func(xn) - func(x1));
        if (func(res) * func(x1) < 0) xn = res;
        else x1 = res;
    }
    return res;
}

int main() {

    #if 0 // test case 1
    double a = -2.1, b = -1.9;
    printf("func(%.02lf) = %.02lf ;  func(%.02lf) = %.02lf\n", a, func(a), b, func(b));

    printf("The root of func(x) is %lf\n", fzero(func, a, b));
    #endif

    #if 0 // test case 2
    double x1, x2;
    for (x1 = lower_bound; x1 < upper_bound; x1 += increment) {
        x2 = x1 + increment;
        if (func(x1) * func(x2) <= 0)
            printf("find a valid root: %lf\n", fzero(func, x1, x2));
    }
    #endif

    #if 1 // test case 3: mvf represents a multi-valued function
    FunctionPtr mvf[3] = {func1, func2, func3};
    double x1, x2;
    for (x1 = lower_bound; x1 < upper_bound; x1 += increment) {
        x2 = x1 + increment;
        for (int i = 0; i < 3; i ++ )
            if (mvf[i](x1) * mvf[i](x2) <= 0)
                printf("find a valid root (func%d): %lf\n", i + 1, fzero(mvf[i], x1, x2));
    }
    #endif

    return 0;
}