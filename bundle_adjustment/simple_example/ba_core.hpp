#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <Eigen/Dense>
#include <fstream>
#include <map>
#include <nlohmann/json.hpp>
#include <string>

using json = nlohmann::json;

namespace ba {

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
        const nlohmann::json_abi_v3_11_3::json& camVal);

    /*
    Triangulation using Direct Linear Transformation
    */
    WorldPoint TriangulateDLT(const std::vector<Observations>& obs);

    void json_write(const std::map<std::string, WorldPoint>& points,
                    const std::map<std::string, CameraParams>& cameras,
                    json& output_data,
                    const std::string& file_name = "answer.json");


    json read_json(const std::string & filename);

    void parseWorldPoints(const json & data, std::map<std::string, WorldPoint> & points);

    void getCameraMap(const nlohmann::json &data, std::map<std::string, CameraParams>& cameras);
    
    void getRotationMatrix(const nlohmann::json &camVal, double *rotation_matrix);
    
    struct Observation2D {
        double u, v;  // 2D observations
        std::string cameraId, pointId;
    };

    // Lightweight container for the problem data (observations only)
    struct ProblemData {
        std::vector<Observation2D> observations;
    };

    ProblemData loadObservations(const json & data);

    // Build Ceres problem from current camera and point parameters
    ceres::Problem buildProblem(const ProblemData& data,
                                std::map<std::string, CameraParams>& cameras,
                                std::map<std::string, WorldPoint>& points,
                                double loss_threshold = 1.0);

    // Run solver with standard options
    void solveAndSave(ceres::Problem& problem,
                    const std::map<std::string, CameraParams>& cameras,
                    const std::map<std::string, WorldPoint>& points,
                    const nlohmann::json &data, 
                    const std::string& output_filename);

}  // namespace ba