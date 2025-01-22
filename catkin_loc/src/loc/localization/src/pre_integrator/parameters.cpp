#include "localization/pre_integrator/parameters.h"

double INIT_DEPTH;
double MIN_PARALLAX;
// double ACC_N = 0.08;
// double ACC_W = 0.00004;
// double GYR_N = 0.004;
// double GYR_W = 2.0e-6;
double ACC_N = 2;
double ACC_W = 0.04;
double GYR_N = 0.1;
double GYR_W = 0.08;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
std::string IMU_TOPIC;
double ROW, COL;
double TD, TR;


