#define IKFAST_HAS_LIBRARY
#include "ikfast.h"

using namespace ikfast;

extern "C" size_t compute_ik_proxy(const IkReal *rotation, const IkReal *translation, IkReal *q) {
    IkSolutionList<IkReal> solutions;

    bool success = ComputeIk(translation, rotation, NULL, solutions);

    if (!success) return 0;

    for(std::size_t i = 0; i < solutions.GetNumSolutions(); ++i) {
        const IkSolutionBase<IkReal> &solution = solutions.GetSolution(i);
        solution.GetSolution(q + i * 6, NULL);
    }

    return solutions.GetNumSolutions();
}
