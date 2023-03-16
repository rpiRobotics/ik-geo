//  time g++ test_timing.cpp -lrt -O2 -DIKFAST_NO_MAIN -include IRB_6640.cpp  -DN=1000000

// #define IKFAST_NO_MAIN // Don't include main() from IKFast

// #include "IRB_6640.cpp"

#include <stdio.h>
#include <stdlib.h>
#include <time.h> // for clock_gettime()

int main(int argc, char** argv)
{
    if( argc != 12+GetNumFreeParameters()+1 ) {
        printf("\nUsage: ./ik r00 r01 r02 t0 r10 r11 r12 t1 r20 r21 r22 t2 free0 ...\n\n"
               "Returns the ik solutions given the transformation of the end effector specified by\n"
               "a 3x3 rotation R (rXX), and a 3x1 translation (tX).\n"
               "There are %d free parameters that have to be specified.\n\n",GetNumFreeParameters());
        return 1;
    }

    IkSolutionList<IkReal> solutions;
    std::vector<IkReal> vfree(GetNumFreeParameters());
    IkReal eerot[9],eetrans[3];
    eerot[0] = atof(argv[1]); eerot[1] = atof(argv[2]); eerot[2] = atof(argv[3]); eetrans[0] = atof(argv[4]);
    eerot[3] = atof(argv[5]); eerot[4] = atof(argv[6]); eerot[5] = atof(argv[7]); eetrans[1] = atof(argv[8]);
    eerot[6] = atof(argv[9]); eerot[7] = atof(argv[10]); eerot[8] = atof(argv[11]); eetrans[2] = atof(argv[12]);
    for(std::size_t i = 0; i < vfree.size(); ++i)
        vfree[i] = atof(argv[13+i]);
    


    volatile bool bSuccess;
    timespec start_time, end_time;
    //int N = 1000000;
    clock_gettime(CLOCK_REALTIME, &start_time); 
    for (int i = 0; i < N; ++i)
    	bSuccess = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);
    clock_gettime(CLOCK_REALTIME, &end_time);
    float total_time = (float)(end_time.tv_sec - start_time.tv_sec) + (float)(end_time.tv_nsec - start_time.tv_nsec)/1000000000;
    printf("\n\n%f seconds total\n", total_time);
    printf("\n\n%f microseconds per IK run\n", total_time/N * 1000000);

    if( !bSuccess ) {
        fprintf(stderr,"Failed to get ik solution\n");
        return -1;
    }

    printf("Found %d ik solutions:\n", (int)solutions.GetNumSolutions());
    std::vector<IkReal> solvalues(GetNumJoints());
    for(std::size_t i = 0; i < solutions.GetNumSolutions(); ++i) {
        const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
        printf("sol%d (free=%d): ", (int)i, (int)sol.GetFree().size());
        std::vector<IkReal> vsolfree(sol.GetFree().size());
        sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
        for( std::size_t j = 0; j < solvalues.size(); ++j)
            printf("%.15f, ", solvalues[j]);
        printf("\n");
    }
    return 0;
}