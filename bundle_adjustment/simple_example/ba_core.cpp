#include "ba_core.hpp"

// namespace bau = bundle_adjustment;

namespace ba {
    Eigen::Matrix<double, 3, 4, Eigen::RowMajor> getProjectionMatrix(
        const nlohmann::json_abi_v3_11_3::json& camVal) {
    // Get Intrinsics
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> K =
        Eigen::Matrix<double, 3, 3, Eigen::RowMajor>::Zero();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
        K(i, j) = camVal["calib"]["intrinsics"][i][j];
        }
    }// Get Extrinsics (3x4)
    Eigen::Matrix<double, 3, 4, Eigen::RowMajor> Rt =
        Eigen::Matrix<double, 3, 4, Eigen::RowMajor>::Zero();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
        Rt(i, j) = camVal["calib"]["extrinsics"][i][j];
        }
    }

    return K * Rt;
    }

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

    json read_json(const std::string & filename){
        std::ifstream json_in(filename);
        if (!json_in.is_open()) {
            std::cerr << "Could not open JSON file!" << std::endl;
            return -1;
        }
        json data = json::parse(json_in);
        return data;
    }

    void parseWorldPoints(const json & data, std::map<std::string, WorldPoint> & points){
        std::cout << "Parsing World Points........" << std::endl;
        for (const auto& [key, val] : data["worldPoints"].items()) {
            points[key].xyz[0] = val[0];
            points[key].xyz[1] = val[1];
            points[key].xyz[2] = val[2];
        }
    }
    void getRotationMatrix(const nlohmann::json &camVal, double *rotation_matrix) {
    try {
        const auto& extrinsics = camVal.at("calib").at("extrinsics");
        
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                // Use [c][r] for column-major reading required for Ceres
                rotation_matrix[r * 3 + c] = extrinsics.at(c).at(r).get<double>();
            }
        }
    } catch (const std::exception& e) {
        // Handle cases where JSON path doesn't exist or isn't 3x3
        std::cerr << "JSON Error: " << e.what() << std::endl;
    }
    }

    ProblemData loadObservations(const json & data) {
        ProblemData result;
        Observation2D ob2d;
        for (const auto& [camId, camVal] : data["cameras"].items()) {
            ob2d.cameraId = camId;
            for (const auto& [obsKey, obsVal] : camVal["observations"].items()) {
                ob2d.u = obsVal[0];
                ob2d.v = obsVal[1];
                ob2d.pointId = obsKey;

                result.observations.push_back(ob2d);
            }
        }
        return result;
    }

    void getCameraMap(const nlohmann::json &data, std::map<std::string, CameraParams>& cameras){
        std::cout << "Parsing Cameras..." << std::endl;
        for (const auto& [camKey, camVal] : data["cameras"].items()) {

            ba::CameraParams& cam = cameras[camKey];

            // Parse Intrinsics
            // K_3x3: [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
            // fx  = fy = cam.params[0]
            cam.params[0] = camVal["calib"]["intrinsics"][0][0];
            cam.cx = camVal["calib"]["intrinsics"][0][2];  // Constant
            cam.cy = camVal["calib"]["intrinsics"][1][2];  // Constant

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
            ceres::RotationMatrixToAngleAxis(rotation_matrix, &cam.params[1]);
        }
    }

    ceres::Problem buildProblem(const ProblemData& data,
                                std::map<std::string, CameraParams>& cameras,
                                std::map<std::string, WorldPoint>& points,
                                double loss_threshold) {
        ceres::Problem problem;
        ceres::LossFunction* loss = new ceres::HuberLoss(loss_threshold);

        for (const auto& obs : data.observations) {
            auto& cam = cameras.at(obs.cameraId);
            auto& pt  = points.at(obs.pointId);
            ceres::CostFunction* cost =
                ReprojectionError::Create(obs.u, obs.v, cam.cx, cam.cy);
            problem.AddResidualBlock(cost, loss, cam.params, pt.xyz);
        }
        return problem;
    }

    void solveAndSave(ceres::Problem& problem,
                    const std::map<std::string, CameraParams>& cameras,
                    const std::map<std::string, WorldPoint>& points,
                    nlohmann::json &data, 
                    const std::string& output_filename) {
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.minimizer_progress_to_stdout = true;
        // ... other options

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        std::cout << summary.FullReport() << "\n";

        // ... or adapt json_write to take the maps and a filename
        json_write(points, cameras, data, "test.json");
    }

} // namespace ba

int main(int, char**){
    json jdata = ba::read_json("bare_minimal.json");
    std::map<std::string, ba::CameraParams> cameras;
    std::map<std::string, ba::WorldPoint> points;

    // --- Parse Cameras and Observations ---
    ba::getCameraMap(jdata, cameras);
    // --- Parse World Points ---
    ba::parseWorldPoints(jdata, points);

    ba::ProblemData data = ba::loadObservations(jdata);

    ceres::Problem problem = ba::buildProblem(data, cameras, points);

    json output_data = jdata;
    ba::solveAndSave(problem, cameras, points, output_data, "test_bare_minimal.json");

    json answer = ba::read_json("answer_bare_minimal.json");

    if (output_data == answer) {
        std::cout << "The files are equal!" << std::endl;
    } else {
        std::cout << "The files are different." << std::endl;
    }

    return 0;
}