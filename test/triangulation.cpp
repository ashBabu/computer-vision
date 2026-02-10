#include <map>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>

#include <nlohmann/json.hpp>

#include "utilities.hpp"

using json = nlohmann::json;
namespace ba = bundle_adjustment;


int main(int argc, char** argv) {
  // Load JSON Data
  std::ifstream json_in("noisy-calibrations-and-world-points.json");
  if (!json_in.is_open()) {
    std::cerr << "Could not open JSON file!" << std::endl;
    return -1;
  }
  json data = json::parse(json_in);

  std::map<std::string, std::vector<ba::Observations>> observations;

  for (const auto& [camKey, camVal] : data["cameras"].items()) {
    for (const auto& [obsKey, obsVal] : camVal["observations"].items()) {
      Eigen::Matrix<double, 3, 3, Eigen::RowMajor> K =
          Eigen::Matrix<double, 3, 3, Eigen::RowMajor>::Zero();
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
          K(i, j) = camVal["calib"]["intrinsics"][i][j];

      // Load Extrinsics (3x4)
      Eigen::Matrix<double, 3, 4, Eigen::RowMajor> Rt =
          Eigen::Matrix<double, 3, 4, Eigen::RowMajor>::Zero();
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 4; ++j)
          Rt(i, j) = camVal["calib"]["extrinsics"][i][j];

      Eigen::Matrix<double, 3, 4> projection_matrix = K * Rt;

      ba::Observations obs;
      obs.u = obsVal[0];
      obs.v = obsVal[1];
      obs.P = projection_matrix;

      observations[obsKey].push_back(obs);
    }
  }

  std::map<std::string, ba::WorldPoint> world_points;
  for (const auto& [obsKey, obsVal] : observations) {
    ba::WorldPoint wp = ba::TriangulateDLT(obsVal);
    world_points[obsKey] = wp;
  }
  std::cout << "Done" << std::endl;
}
