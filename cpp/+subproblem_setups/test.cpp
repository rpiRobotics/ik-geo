//---------------------------------------------------------------//
// Name: sp_1.h
// Author: Ashton Ropp, Runbin Chen, Amar Maksumic
// Date: 10/15/2022
// Purpose: Port of the subproblem/sp_1.m file
//---------------------------------------------------------------//
// g++ test.cpp -std=c++11 -isystem ~/code/benchmark/include  -L ~/code/benchmark/build/src -lbenchmark -lpthread -o mybenchmark

#include <chrono>
#include <iostream>
#include <benchmark/benchmark.h>
#include "sp_1.cpp"
#include "sp_2.cpp"
#include "sp_2E.cpp"
#include "sp_3.cpp"
#include "sp_4.cpp"

using namespace Eigen;

static void BM_sp1(benchmark::State& state) {
  Eigen::Matrix<double, 3, 1> p1, p2, k;
  double theta;
  sp1_setup(p1, p2, k, theta);
  for (auto _ : state)
    sp1_run(p1, p2, k, theta);
}

BENCHMARK(BM_sp1)->Iterations(1000000);

static void BM_sp1_LS(benchmark::State& state) {
  Eigen::Matrix<double, 3, 1> p1, p2, k;
  double theta;
  sp1_setup_LS(p1, p2, k, theta);
  for (auto _ : state)
    sp1_run(p1, p2, k, theta);
}

BENCHMARK(BM_sp1_LS)->Iterations(1000000);

static void BM_sp2(benchmark::State& state) {
  Eigen::Matrix<double, 3, 1> p1, p2, k1, k2;
  double theta1, theta2;
  sp2_setup(p1, p2, k1, k2, theta1, theta2);
  for (auto _ : state)
  	sp2_run(p1, p2, k1, k2, theta1, theta2);
}

BENCHMARK(BM_sp2)->Iterations(1000000);

static void BM_sp2_LS(benchmark::State& state) {
  Eigen::Matrix<double, 3, 1> p1, p2, k1, k2;
  double theta1, theta2;
  sp2_setup_LS(p1, p2, k1, k2, theta1, theta2);
  for (auto _ : state)
  	sp2_run(p1, p2, k1, k2, theta1, theta2);
}

BENCHMARK(BM_sp2_LS)->Iterations(1000000);

static void BM_sp2E(benchmark::State& state) {
	Eigen::Matrix<double, 3, 1> p0, p1, p2, k1, k2;
  double theta1, theta2;
  sp_2E_setup(p0, p1, p2, k1, k2, theta1, theta2);
  for (auto _ : state)
		sp_2E(p0, p1, p2, k1, k2, theta1, theta2);
}

BENCHMARK(BM_sp2E)->Iterations(1000000);

static void BM_sp3(benchmark::State& state) {
	Eigen::Matrix<double, 3, 1> p1, p2, k;
	double d;
	std::vector<double> theta;
	sp_3_setup(p1, p2, k, d);
  for (auto _ : state)
		sp_3(p1, p2, k, d, theta);
}

BENCHMARK(BM_sp3)->Iterations(1000000);

static void BM_sp4(benchmark::State& state) {
  Eigen::Matrix<double, 3, 1> p, k, h;
  Eigen::Matrix<double, 2, 1> theta;
  double d;
  sp4_setup(p, k, h, d, theta);
  for (auto _ : state)  	  
    sp4_run(p, k, h, d, theta);
}

BENCHMARK(BM_sp4)->Iterations(1000000);

static void BM_sp4_LS(benchmark::State& state) {
  Eigen::Matrix<double, 3, 1> p, k, h;
  Eigen::Matrix<double, 2, 1> theta;
  double d;
  sp4_setup_LS(p, k, h, d, theta);
  for (auto _ : state)  	  
    sp4_run(p, k, h, d, theta);
}

BENCHMARK(BM_sp4_LS)->Iterations(1000000);

BENCHMARK_MAIN();