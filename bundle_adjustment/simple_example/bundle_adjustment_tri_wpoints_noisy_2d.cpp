#include <fstream>
#include <iostream>
#include <map>
#include <nlohmann/json.hpp>
#include <random>
#include <string>
#include <vector>

#include "utilities.hpp"

using json = nlohmann::json;
namespace ba = bundle_adjustment;

int main(int argc, char** argv) {
  ceres::Problem problem;

  std::random_device rd;
  std::mt19937 gen(rd());

  // Normal distribution (Mean = 0, Sigma = 1)
  std::normal_distribution<double> normal_dist(0.0, 1.0);

  // Load JSON Data
  std::ifstream json_in("noisy-calibrations-and-world-points.json");
  if (!json_in.is_open()) {
    std::cerr << "Could not open JSON file!" << std::endl;
    return -1;
  }
  json data = json::parse(json_in);

  // Parse Data into Optimizable Structures
  std::map<std::string, ba::CameraParams> cameras;
  std::map<std::string, ba::WorldPoint> points;

  // --- Parse World Points ---
  std::cout << "Parsing World Points......" << std::endl;
  std::map<std::string, std::vector<ba::Observations>> observations;
  Eigen::Matrix<double, 3, 4, Eigen::RowMajor> projection_matrix;

  for (const auto& [camKey, camVal] : data["cameras"].items()) {
    for (const auto& [obsKey, obsVal] : camVal["observations"].items()) {
      projection_matrix = ba::getProjectionMatrix(camVal);

      ba::Observations obs;
      obs.u = obsVal[0].get<double>() + normal_dist(gen);
      obs.v = obsVal[1].get<double>() + normal_dist(gen);
      obs.P = projection_matrix;

      observations[obsKey].push_back(obs);
    }
  }

  ba::WorldPoint wp;
  for (const auto& [obsKey, obsVal] : observations) {
    wp = ba::TriangulateDLT(obsVal);
    points[obsKey] = wp;
  }

  // --- Parse Cameras and Observations ---
  std::cout << "Parsing Cameras..." << std::endl;
  for (const auto& [camKey, camVal] : data["cameras"].items()) {
    ba::CameraParams& cam = cameras[camKey];

    // Parse Intrinsics
    // K_3x3: [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
    // fx  = fy
    double fx = camVal["calib"]["intrinsics"][0][0];
    double cx = camVal["calib"]["intrinsics"][0][2];
    double cy = camVal["calib"]["intrinsics"][1][2];

    cam.params[0] = fx;  // Initial focal length
    cam.cx = cx;         // Constant
    cam.cy = cy;         // Constant

    // Parse Extrinsics
    // T_3x4: [[r00, r01, r02, tx], ...]
    double rotation_matrix[9];

    // Read as column-major rotation matrix
    // Given json input is row-major
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        rotation_matrix[r * 3 + c] = camVal["calib"]["extrinsics"][c][r];
      }
      // Translation vector is the 4th column
      cam.params[4 + r] = camVal["calib"]["extrinsics"][r][3];
    }

    // Convert Rotation Matrix to Angle-Axis for Ceres
    ceres::RotationMatrixToAngleAxis(rotation_matrix, &cam.params[1]);

    // Parse Observations and Build Problem
    for (const auto& [obsKey, obsVal] : camVal["observations"].items()) {
      double u = obsVal[0].get<double>() + normal_dist(gen);
      double v = obsVal[1].get<double>() + normal_dist(gen);

      // Ensure point exists in World Points
      if (points.find(obsKey) == points.end()) continue;

      // Create Cost Function
      ceres::CostFunction* cost_function =
          ba::ReprojectionError::Create(u, v, cam.cx, cam.cy);

      // Add Residual Block
      problem.AddResidualBlock(
          cost_function,
          new ceres::HuberLoss(1.0),  // Robust loss against outliers
          cam.params, points[obsKey].xyz);
    }
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  std::cout << "Solving..." << std::endl;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";

  // Saving to JSON
  json output_data = data;

  ba::json_write(points, cameras, output_data, "answer_tri_wpoints_noisy_2d.json");

  return 0;
}