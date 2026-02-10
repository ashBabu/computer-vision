#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "utilities.hpp"

namespace ba = bundle_adjustment;

int main(int argc, char** argv) {
  ba::BALProblem data;
  // if (data.LoadFile("problem-16-22106-pre.txt")) {
  if (data.LoadFile("problem-135-90642-pre_dubrovnik.txt")) {
    std::cout << "Loaded " << data.num_cameras << " cameras and "
              << data.num_points << " points." << std::endl;
  }

  ceres::Problem problem;
  for (const auto& obs : data.observations) {
    double* cam_params = data.cameras[obs.camera_index].params;
    double* landmarks = data.points[obs.point_index].xyz;
    double u = obs.u;
    double v = obs.v;

    // Create Cost Function
    ceres::CostFunction* cost_function = ba::ReprojectionError::Create(u, v);

    // Add Residual Block
    problem.AddResidualBlock(
        cost_function,
        new ceres::HuberLoss(1.0),  // Robust loss against outliers
        cam_params, landmarks);
  }
  problem.SetParameterBlockConstant(data.cameras[0].params);

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.num_threads = 16; // Use multiple cores
  options.preconditioner_type = ceres::SCHUR_JACOBI; 
  options.max_num_iterations = 200;

  ceres::Solver::Summary summary;
  std::cout << "Solving..." << std::endl;
  
  data.WriteToPLY("initial_dubrovnik.ply");
  
  ceres::Solve(options, &problem, &summary);

  data.WriteToPLY("optimized_dubrovnik.ply");

  std::cout << summary.FullReport() << "\n";
  return 0;
}