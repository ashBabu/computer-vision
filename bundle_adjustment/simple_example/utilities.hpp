#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <fstream>

using json = nlohmann::json;

namespace bundle_adjustment {
struct CameraParams {
  /*
  params layout:
  [0] focal_length
  [1] rx (angle-axis)
  [2] ry (angle-axis)
  [3] rz (angle-axis)
  [4] tx (translation)
  [5] ty (translation)
  [6] tz (translation)

  cx, cy are const camera principal point coordinates
  */
  double params[7];
  double cx;
  double cy;
};

struct WorldPoint {
  double xyz[3];
};

struct Observations {
  Eigen::Matrix<double, 3, 4> P;  // projection matrix
  double u;
  double v;
};

struct ReprojectionError {
  ReprojectionError(double observed_x, double observed_y, double cx, double cy)
      : observed_x_(observed_x), observed_y_(observed_y), cx_(cx), cy_(cy) {}

  template <typename T>
  bool operator()(const T* const camera, const T* const point,
                  T* residuals) const {
    const T& focal = camera[0];
    const T* angle_axis = camera + 1;
    const T* translation = camera + 4;

    T p[3];
    ceres::AngleAxisRotatePoint(angle_axis, point, p);

    // Translate the point
    p[0] += translation[0];
    p[1] += translation[1];
    p[2] += translation[2];

    // Project to image plane
    T xp = p[0] / p[2];
    T yp = p[1] / p[2];

    // Apply intrinsics, predicted = K*X
    T predicted_x = focal * xp + T(cx_);
    T predicted_y = focal * yp + T(cy_);

    // 5. Compute residuals
    residuals[0] = predicted_x - T(observed_x_);
    residuals[1] = predicted_y - T(observed_y_);

    return true;
  }

  // Factory to hide the construction of the CostFunction object
  static ceres::CostFunction* Create(double observed_x, double observed_y,
                                     double cx, double cy) {
    return new ceres::AutoDiffCostFunction<ReprojectionError, 2, 7,
                                           3>(  // 2 residuals, 7 camera params,
                                                // 3 point params
        new ReprojectionError(observed_x, observed_y, cx, cy));
  }

  double observed_x_;
  double observed_y_;
  double cx_;
  double cy_;
};

Eigen::Matrix<double, 3, 4, Eigen::RowMajor> getProjectionMatrix(
    const nlohmann::json_abi_v3_11_3::json& camVal) {
  // Get Intrinsics
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> K =
      Eigen::Matrix<double, 3, 3, Eigen::RowMajor>::Zero();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      K(i, j) = camVal["calib"]["intrinsics"][i][j];
    }
  }

  // Get Extrinsics (3x4)
  Eigen::Matrix<double, 3, 4, Eigen::RowMajor> Rt =
      Eigen::Matrix<double, 3, 4, Eigen::RowMajor>::Zero();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 4; ++j) {
      Rt(i, j) = camVal["calib"]["extrinsics"][i][j];
    }
  }

  return K * Rt;
}

/*
Triangulation using Direct Linear Transformation
*/
WorldPoint TriangulateDLT(const std::vector<Observations>& obs) {
  Eigen::MatrixXd A(2 * obs.size(), 4);

  for (size_t i = 0; i < obs.size(); ++i) {
    const auto P = obs[i].P;
    double u = obs[i].u;
    double v = obs[i].v;

    // Row 1: u * P3 - P1
    A.row(2 * i) = u * P.row(2) - P.row(0);
    // Row 2: v * P3 - P2
    A.row(2 * i + 1) = v * P.row(2) - P.row(1);
  }

  // Solve A*X = 0 using SVD
  // The solution is the eigenvector corresponding to the smallest eigenvalue
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
  Eigen::Vector4d X_homo = svd.matrixV().col(3);

  // Convert from homogeneous to Euclidean
  WorldPoint world_pts;
  Eigen::Vector3d wp(X_homo.head<3>() / X_homo(3));
  world_pts.xyz[0] = wp(0);
  world_pts.xyz[1] = wp(1);
  world_pts.xyz[2] = wp(2);

  return world_pts;
}

void json_write(const std::map<std::string, WorldPoint>& points,
                const std::map<std::string, CameraParams>& cameras,
                json& output_data,
                const std::string& file_name = "answer.json") {
  // Overwrite World Points
  for (auto const& [id, point] : points) {
    output_data["worldPoints"][id] = {point.xyz[0], point.xyz[1], point.xyz[2]};
  }

  // Overwrite Camera Calibrations
  for (auto const& [id, cam] : cameras) {
    // Update Intrinsics (focal length updated, cx/cy kept same)
    output_data["cameras"][id]["calib"]["intrinsics"][0][0] =
        cam.params[0];  // fx
    output_data["cameras"][id]["calib"]["intrinsics"][1][1] =
        cam.params[0];  // fy = fx

    // Update Extrinsics
    // Convert Angle-Axis back to Rotation Matrix
    double optimized_rotation[9];
    ceres::AngleAxisToRotationMatrix(&cam.params[1], optimized_rotation);

    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        output_data["cameras"][id]["calib"]["extrinsics"][c][r] =
            optimized_rotation[r * 3 + c];
      }
      output_data["cameras"][id]["calib"]["extrinsics"][r][3] =
          cam.params[4 + r];
    }
  }

  // Write to file
  std::ofstream output_json(file_name);
  output_json << std::setw(4) << output_data << std::endl;
  std::cout << "Done." << std::endl;
}

}  // namespace bundle_adjustment