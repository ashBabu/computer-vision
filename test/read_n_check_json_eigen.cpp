#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <iomanip>
#include <json.hpp>
#include <Eigen/Dense>

using json = nlohmann::json;

struct ProjectionResult {
    std::string id;
    Eigen::Vector2f groundTruth;
    Eigen::Vector2f projected;
    double error;
};

int main() {
    // 1. Load the JSON file
    std::ifstream file("projection-check.json");
    if (!file.is_open()) {
        std::cerr << "Could not open the file!" << std::endl;
        return 1;
    }

    json data;
    file >> data;

    // 2. Extract Camera 0 parameters
    auto cam0 = data["cameras"]["0"]["calib"];
    
    // Load Intrinsics (3x3)
    Eigen::Matrix3d K = Eigen::Matrix3d::Zero();
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            K(i, j) = cam0["intrinsics"][i][j];

    // Load Extrinsics (3x4)
    Eigen::Matrix<double, 3, 4> Rt = Eigen::Matrix<double, 3, 4>::Zero();
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 4; ++j)
            Rt(i, j) = cam0["extrinsics"][i][j];

    // 3. Process Observations vs World Points
    auto observations = data["cameras"]["0"]["observations"];
    auto worldPoints = data["worldPoints"];

    std::vector<ProjectionResult> results;

    std::cout << std::left << std::setw(5) << "ID" 
              << std::setw(25) << "Ground Truth (u, v)" 
              << std::setw(25) << "Projected (u, v)" 
              << "L2 Error (px)" << std::endl;
    std::cout << std::string(75, '-') << std::endl;

    for (auto& [id, obs] : observations.items()) {
        if (worldPoints.contains(id)) {
            // Get 3D Point [X, Y, Z, 1]
            std::vector<double> wp = worldPoints[id];
            Eigen::Vector4d Pw(wp[0], wp[1], wp[2], 1.0);

            // Project: P_pixel = K * [R|t] * P_world
            Eigen::Vector3d Pc = Rt * Pw; // Transform to camera coords
            Eigen::Vector3d Pp = K * Pc;  // Transform to image plane

            // Perspective division
            double u_proj = Pp(0) / Pp(2);
            double v_proj = Pp(1) / Pp(2);

            // Ground truth from JSON
            double u_gt = obs[0];
            double v_gt = obs[1];

            // Calculate Euclidean Error
            double error = std::sqrt(std::pow(u_proj - u_gt, 2) + std::pow(v_proj - v_gt, 2));

            std::cout << std::left << std::setw(5) << id 
                      << "(" << std::setw(8) << u_gt << "," << std::setw(8) << v_gt << ")  "
                      << "(" << std::setw(8) << std::fixed << std::setprecision(2) << u_proj << "," 
                      << std::setw(8) << v_proj << ")  "
                      << std::setprecision(4) << error << std::endl;
        }
    }

    return 0;
}