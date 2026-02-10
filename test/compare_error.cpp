#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <iomanip>
#include <cmath>
#include <sstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

struct Point3D { double x, y, z; };
struct ProjectionResult { double u, v; };

// Helper to format a tuple (a, b) into a string
std::string format_tuple(double a, double b, int precision, bool scientific = false) {
    std::stringstream ss;
    ss << "(";
    if (scientific) ss << std::scientific;
    else ss << std::fixed;
    ss << std::setprecision(precision) << a << ", " << b << ")";
    return ss.str();
}

ProjectionResult ProjectPoint(const json& camera_data, const Point3D& pt_world) {
    const auto& extrinsics = camera_data["calib"]["extrinsics"];
    const auto& intrinsics = camera_data["calib"]["intrinsics"];

    double r00 = extrinsics[0][0], r01 = extrinsics[0][1], r02 = extrinsics[0][2], tx = extrinsics[0][3];
    double r10 = extrinsics[1][0], r11 = extrinsics[1][1], r12 = extrinsics[1][2], ty = extrinsics[1][3];
    double r20 = extrinsics[2][0], r21 = extrinsics[2][1], r22 = extrinsics[2][2], tz = extrinsics[2][3];

    double xc = r00 * pt_world.x + r01 * pt_world.y + r02 * pt_world.z + tx;
    double yc = r10 * pt_world.x + r11 * pt_world.y + r12 * pt_world.z + ty;
    double zc = r20 * pt_world.x + r21 * pt_world.y + r22 * pt_world.z + tz;

    double fx = intrinsics[0][0], cx = intrinsics[0][2];
    double fy = intrinsics[1][1], cy = intrinsics[1][2];

    if (std::abs(zc) < 1e-9) return {0.0, 0.0};
    return { fx * (xc / zc) + cx, fy * (yc / zc) + cy };
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: ./reprojection_vectors noisy.json answer.json" << std::endl;
        return 1;
    }

    std::ifstream f_init(argv[1]), f_opt(argv[2]);
    json j_init = json::parse(f_init);
    json j_opt = json::parse(f_opt);

    // Header
    std::cout << std::left 
              << std::setw(8)  << "CamID" 
              << std::setw(8)  << "PtID" 
              << std::setw(28) << "Initial Err (du, dv)" 
              << std::setw(32) << "Optimized Err (du, dv)" 
              << "Observed (u, v)" << std::endl;
    std::cout << std::string(95, '-') << std::endl;

    for (auto& [cam_id, cam_init] : j_init["cameras"].items()) {
        if (!j_opt["cameras"].contains(cam_id)) continue;
        auto& cam_opt = j_opt["cameras"][cam_id];

        for (auto& [pt_id, obs_val] : cam_init["observations"].items()) {
            if (!j_init["worldPoints"].contains(pt_id)) continue;

            double u_obs = obs_val[0], v_obs = obs_val[1];

            // Calculate Initial Residuals
            Point3D p_i = { j_init["worldPoints"][pt_id][0], j_init["worldPoints"][pt_id][1], j_init["worldPoints"][pt_id][2] };
            ProjectionResult proj_i = ProjectPoint(cam_init, p_i);
            
            // Calculate Optimized Residuals
            Point3D p_o = { j_opt["worldPoints"][pt_id][0], j_opt["worldPoints"][pt_id][1], j_opt["worldPoints"][pt_id][2] };
            ProjectionResult proj_o = ProjectPoint(cam_opt, p_o);

            // Print Row
            std::cout << std::left 
                      << std::setw(8)  << cam_id 
                      << std::setw(8)  << pt_id 
                      << std::setw(28) << format_tuple(u_obs - proj_i.u, v_obs - proj_i.v, 3)
                      << std::setw(32) << format_tuple(u_obs - proj_o.u, v_obs - proj_o.v, 3, true)
                      << format_tuple(u_obs, v_obs, 2)
                      << std::endl;
        }
    }
    return 0;
}