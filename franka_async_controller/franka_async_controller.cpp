// franka_async_controller.cpp
// Implementation of Franka robot kinematics: IK solver, FK, Jacobian

#include "franka_async_controller.h"
#include <iostream>
#include <cmath>
#include <franka/exception.h>

// For IK computation
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include <cstdlib>

#ifdef USE_PINOCCHIO
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/frames.hpp>
#endif

namespace franka_control {

// Constructor with robot IP
FrankaKinematics::FrankaKinematics(const std::string& robot_ip,
                                   const KinematicsConfig& config)
    : robot_ip_(robot_ip), config_(config), has_robot_ip_(true) {
}

// Constructor without robot IP (offline / Pinocchio only)
FrankaKinematics::FrankaKinematics(const KinematicsConfig& config)
    : config_(config), has_robot_ip_(false) {
}

FrankaKinematics::~FrankaKinematics() = default;

bool FrankaKinematics::initialize() {
    if (initialized_) {
        if (config_.verbose) {
            std::cout << "[Kinematics] Already initialized" << std::endl;
        }
        return true;
    }

    // Connect to robot if IP was provided
    if (has_robot_ip_) {
        try {
            if (config_.verbose) {
                std::cout << "[Kinematics] Connecting to robot at " << robot_ip_ << std::endl;
            }
            robot_ = std::make_shared<franka::Robot>(robot_ip_, franka::RealtimeConfig::kIgnore);
        } catch (const franka::Exception& e) {
            std::cerr << "[Kinematics] Franka connection failed: " << e.what() << std::endl;
            std::cerr << "[Kinematics] Continuing without robot connection (Pinocchio only)" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "[Kinematics] Connection failed: " << e.what() << std::endl;
            std::cerr << "[Kinematics] Continuing without robot connection (Pinocchio only)" << std::endl;
        }
    }

#ifdef USE_PINOCCHIO
    if (config_.verbose) {
        std::cout << "[Kinematics] Initializing Pinocchio model..." << std::endl;
    }

    if (initializePinocchioModel()) {
        pinocchio_initialized_ = true;
        if (config_.verbose) {
            std::cout << "[Kinematics] Pinocchio IK/FK/Jacobian solver ready" << std::endl;
        }
    } else {
        std::cerr << "[Kinematics] Warning: Pinocchio init failed" << std::endl;
        if (!robot_) {
            std::cerr << "[Kinematics] Error: No robot connection and no Pinocchio. Cannot compute kinematics." << std::endl;
            return false;
        }
        std::cerr << "[Kinematics] Will use libfranka (requires robot connection)" << std::endl;
    }
#else
    if (!robot_) {
        std::cerr << "[Kinematics] Error: Pinocchio not compiled in and no robot connection. Cannot compute kinematics." << std::endl;
        return false;
    }
    if (config_.verbose) {
        std::cout << "[Kinematics] Using libfranka for FK/Jacobian (Pinocchio not available)" << std::endl;
    }
#endif

    initialized_ = true;

    if (config_.verbose) {
        std::cout << "[Kinematics] Initialization complete";
#ifdef USE_PINOCCHIO
        if (pinocchio_initialized_) std::cout << " (Pinocchio)";
#endif
        if (robot_) std::cout << " (libfranka)";
        std::cout << std::endl;
    }

    return true;
}

std::optional<std::array<double, 7>> FrankaKinematics::readJointPositions() const {
    if (!robot_) {
        return std::nullopt;
    }

    try {
        auto state = robot_->readOnce();
        return state.q;
    } catch (const std::exception& e) {
        if (config_.verbose) {
            std::cerr << "[Kinematics] Failed to read joint positions: " << e.what() << std::endl;
        }
        return std::nullopt;
    }
}

std::optional<Eigen::Matrix4d> FrankaKinematics::readCurrentPose() const {
    if (!robot_) {
        return std::nullopt;
    }

    try {
        auto state = robot_->readOnce();
        Eigen::Matrix4d pose;
        for (int col = 0; col < 4; ++col) {
            for (int row = 0; row < 4; ++row) {
                pose(row, col) = state.O_T_EE[col * 4 + row];
            }
        }
        return pose;
    } catch (const std::exception& e) {
        if (config_.verbose) {
            std::cerr << "[Kinematics] Failed to read current pose: " << e.what() << std::endl;
        }
        return std::nullopt;
    }
}

// ============================================================
// Forward Kinematics
// ============================================================

Eigen::Matrix4d FrankaKinematics::computeForwardKinematics(
    const std::array<double, 7>& joint_positions) const
{
#ifdef USE_PINOCCHIO
    if (pinocchio_initialized_) {
        return computeFK_Pinocchio(joint_positions);
    }
#endif

    if (robot_) {
        return computeFK_libfranka(joint_positions);
    }

    throw std::runtime_error("[FK] No backend available. Initialize Pinocchio or connect to robot first.");
}

Eigen::Matrix4d FrankaKinematics::computeFK_libfranka(
    const std::array<double, 7>& joint_positions) const
{
    franka::Model model = robot_->loadModel();
    franka::RobotState state;
    state.q = joint_positions;

    std::array<double, 16> pose_array = model.pose(franka::Frame::kEndEffector, state);

    Eigen::Matrix4d pose;
    for (int col = 0; col < 4; ++col) {
        for (int row = 0; row < 4; ++row) {
            pose(row, col) = pose_array[col * 4 + row];
        }
    }

    return pose;
}

#ifdef USE_PINOCCHIO
Eigen::Matrix4d FrankaKinematics::computeFK_Pinocchio(
    const std::array<double, 7>& joint_positions) const
{
    Eigen::VectorXd q(7);
    for (int i = 0; i < 7; ++i) {
        q(i) = joint_positions[i];
    }

    // Need a mutable copy of data for Pinocchio algorithms
    pinocchio::Data data(*pinocchio_model_);
    pinocchio::framesForwardKinematics(*pinocchio_model_, data, q);

    const pinocchio::SE3& oMf = data.oMf[ee_frame_id_];

    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose.block<3, 3>(0, 0) = oMf.rotation();
    pose.block<3, 1>(0, 3) = oMf.translation();

    return pose;
}
#endif

// ============================================================
// Jacobian
// ============================================================

Eigen::Matrix<double, 6, 7> FrankaKinematics::computeJacobian(
    const std::array<double, 7>& joint_positions) const
{
#ifdef USE_PINOCCHIO
    if (pinocchio_initialized_) {
        return computeJacobian_Pinocchio(joint_positions);
    }
#endif

    if (robot_) {
        return computeJacobian_libfranka(joint_positions);
    }

    throw std::runtime_error("[Jacobian] No backend available. Initialize Pinocchio or connect to robot first.");
}

Eigen::Matrix<double, 6, 7> FrankaKinematics::computeJacobian_libfranka(
    const std::array<double, 7>& joint_positions) const
{
    franka::Model model = robot_->loadModel();
    franka::RobotState state;
    state.q = joint_positions;
    state.dq = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    state.O_T_EE = model.pose(franka::Frame::kEndEffector, state);

    std::array<double, 42> jacobian_array =
        model.zeroJacobian(franka::Frame::kEndEffector, state);

    Eigen::Matrix<double, 6, 7> J;
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 7; ++j) {
            J(i, j) = jacobian_array[i * 7 + j];
        }
    }

    return J;
}

#ifdef USE_PINOCCHIO
Eigen::Matrix<double, 6, 7> FrankaKinematics::computeJacobian_Pinocchio(
    const std::array<double, 7>& joint_positions) const
{
    Eigen::VectorXd q(7);
    for (int i = 0; i < 7; ++i) {
        q(i) = joint_positions[i];
    }

    pinocchio::Data data(*pinocchio_model_);
    pinocchio::computeFrameJacobian(*pinocchio_model_, data, q,
                                     ee_frame_id_,
                                     pinocchio::LOCAL_WORLD_ALIGNED,
                                     data.J);

    // data.J is nv x nv, but computeFrameJacobian writes 6 x nv
    // We need to extract the 6x7 block
    pinocchio::Data::Matrix6x J_full(6, pinocchio_model_->nv);
    J_full.setZero();
    pinocchio::computeFrameJacobian(*pinocchio_model_, data, q,
                                     ee_frame_id_,
                                     pinocchio::LOCAL_WORLD_ALIGNED,
                                     J_full);

    Eigen::Matrix<double, 6, 7> J = J_full.leftCols<7>();
    return J;
}
#endif

// ============================================================
// IK Solver
// ============================================================

std::optional<std::array<double, 7>> FrankaKinematics::solveIK(
    const Eigen::Matrix4d& target_pose,
    const std::array<double, 7>& seed) const
{
#ifdef USE_PINOCCHIO
    if (pinocchio_initialized_) {
        return solveIK_Pinocchio(target_pose, seed);
    }
#endif

    return solveIK_Jacobian(target_pose, seed);
}

#ifdef USE_PINOCCHIO
bool FrankaKinematics::initializePinocchioModel() {
    try {
        pinocchio_model_ = std::make_unique<pinocchio::Model>();

        std::string urdf_path = config_.urdf_path;

        // If not specified, try default locations
        if (urdf_path.empty()) {
            std::vector<std::string> search_paths = {
                "./fr3_robot.urdf",
                "../fr3_robot.urdf",
                "/home/ani/ExDex/data_collection/franka_ws/async_pos_ctrl_cpp/franka_async_controller/fr3_robot.urdf",
                std::string(getenv("HOME") ? getenv("HOME") : "") + "/.local/share/franka_async_controller/fr3_robot.urdf"
            };

            for (const auto& path : search_paths) {
                std::ifstream f(path);
                if (f.good()) {
                    urdf_path = path;
                    break;
                }
            }
        }

        if (urdf_path.empty()) {
            std::cerr << "[Pinocchio] Could not find fr3_robot.urdf" << std::endl;
            std::cerr << "[Pinocchio] Please set config.urdf_path or place fr3_robot.urdf in current directory" << std::endl;
            return false;
        }

        if (config_.verbose) {
            std::cout << "[Pinocchio] Loading URDF from: " << urdf_path << std::endl;
        }

        pinocchio::urdf::buildModel(urdf_path, *pinocchio_model_);
        pinocchio_data_ = std::make_unique<pinocchio::Data>(*pinocchio_model_);

        if (!pinocchio_model_->existFrame("fr3_hand_tcp")) {
            std::cerr << "[Pinocchio] Frame 'fr3_hand_tcp' not found in URDF" << std::endl;
            return false;
        }

        ee_frame_id_ = pinocchio_model_->getFrameId("fr3_hand_tcp");

        if (config_.verbose) {
            std::cout << "[Pinocchio] Model loaded: " << pinocchio_model_->nq
                      << " DOF, " << pinocchio_model_->nframes << " frames" << std::endl;
        }

        return true;

    } catch (const std::exception& e) {
        std::cerr << "[Pinocchio] Failed to initialize: " << e.what() << std::endl;
        return false;
    }
}

std::optional<std::array<double, 7>> FrankaKinematics::solveIK_Pinocchio(
    const Eigen::Matrix4d& target_pose,
    const std::array<double, 7>& seed) const
{
    Eigen::VectorXd q(7);
    for (int i = 0; i < 7; ++i) {
        q(i) = seed[i];
    }

    pinocchio::SE3 oMdes(target_pose.block<3, 3>(0, 0), target_pose.block<3, 1>(0, 3));

    if (config_.verbose) {
        std::cout << "Target pose:\nR=\n"
            << target_pose.block<3,3>(0,0)
            << "\np=\n"
            << target_pose.block<3,1>(0,3).transpose()
            << std::endl;
    }

    const double eps = config_.ik_tolerance;
    const int max_iter = config_.ik_max_iterations;
    const double dt = 0.1;
    const double damp = 1e-6;

    // Use a mutable copy of data
    pinocchio::Data data(*pinocchio_model_);

    for (int iter = 0; iter < max_iter; ++iter) {
        pinocchio::framesForwardKinematics(*pinocchio_model_, data, q);

        const pinocchio::SE3& oMi = data.oMf[ee_frame_id_];

        pinocchio::SE3 iMd = oMi.actInv(oMdes);
        Eigen::Matrix<double, 6, 1> err = pinocchio::log6(iMd).toVector();

        double pos_error = err.head<3>().norm();
        double rot_error = err.tail<3>().norm();

        if (pos_error < eps && rot_error < config_.ik_rotation_tolerance) {
            std::array<double, 7> result;
            for (int i = 0; i < 7; ++i) {
                result[i] = q(i);
            }

            if (config_.verbose) {
                std::cout << "[Pinocchio IK] Converged in " << iter
                          << " iterations (pos_err=" << pos_error * 1000.0
                          << "mm, rot_err=" << rot_error * 180.0 / M_PI << "deg)" << std::endl;
            }

            return result;
        }

        pinocchio::Data::Matrix6x J(6, pinocchio_model_->nv);
        J.setZero();
        pinocchio::computeFrameJacobian(
                *pinocchio_model_, data, q,
                ee_frame_id_,
                pinocchio::LOCAL,
                J
            );

        pinocchio::Data::Matrix6 Jlog;
        pinocchio::Jlog6(iMd.inverse(), Jlog);
        J = -Jlog * J;

        Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();
        JJt.diagonal().array() += damp;

        Eigen::VectorXd v = -J.transpose() * JJt.ldlt().solve(err);

        q = pinocchio::integrate(*pinocchio_model_, q, v * dt);

        q = q.cwiseMax(pinocchio_model_->lowerPositionLimit)
             .cwiseMin(pinocchio_model_->upperPositionLimit);
    }

    if (config_.verbose) {
        std::cout << "[Pinocchio IK] Did not converge after "
                  << max_iter << " iterations" << std::endl;
    }

    return std::nullopt;
}
#endif  // USE_PINOCCHIO

std::optional<std::array<double, 7>> FrankaKinematics::solveIK_Jacobian(
    const Eigen::Matrix4d& target_pose,
    const std::array<double, 7>& seed) const
{
    if (!robot_) {
        throw std::runtime_error("[IK Jacobian] Requires robot connection (no Pinocchio available)");
    }

    Eigen::Matrix<double, 7, 1> q;
    for (int i = 0; i < 7; ++i) {
        q(i) = seed[i];
    }

    const double lambda = 0.01;
    const double step_size = 0.5;

    for (int iter = 0; iter < config_.ik_max_iterations; ++iter) {
        Eigen::Matrix4d current_pose = computeForwardKinematics(
            {q(0), q(1), q(2), q(3), q(4), q(5), q(6)});

        Eigen::Matrix<double, 6, 1> error = computePoseError(current_pose, target_pose);

        double pos_error = error.head<3>().norm();
        double rot_error = error.tail<3>().norm();

        if (pos_error < config_.ik_tolerance &&
            rot_error < config_.ik_rotation_tolerance) {
            std::array<double, 7> result;
            for (int i = 0; i < 7; ++i) {
                result[i] = q(i);
            }
            return result;
        }

        Eigen::Matrix<double, 6, 7> J = computeJacobian(
            {q(0), q(1), q(2), q(3), q(4), q(5), q(6)});

        Eigen::Matrix<double, 7, 7> I = Eigen::Matrix<double, 7, 7>::Identity();
        Eigen::Matrix<double, 7, 6> J_pinv =
            (J.transpose() * J + lambda * lambda * I).inverse() * J.transpose();

        Eigen::Matrix<double, 7, 1> dq = J_pinv * error;
        q += step_size * dq;

        // Clamp to joint limits (FR3 limits)
        const double joint_limits[7][2] = {
            {-2.8973, 2.8973},
            {-1.7628, 1.7628},
            {-2.8973, 2.8973},
            {-3.0718, -0.0698},
            {-2.8973, 2.8973},
            {-0.0175, 3.7525},
            {-2.8973, 2.8973}
        };

        for (int i = 0; i < 7; ++i) {
            q(i) = std::max(joint_limits[i][0],
                           std::min(joint_limits[i][1], q(i)));
        }
    }

    if (config_.verbose) {
        std::cout << "[IK] Did not converge after "
                  << config_.ik_max_iterations << " iterations" << std::endl;
    }
    return std::nullopt;
}

// ============================================================
// Validation & Error
// ============================================================

bool FrankaKinematics::validateIKSolution(
    const std::array<double, 7>& joint_positions,
    const Eigen::Matrix4d& target_pose) const
{
    Eigen::Matrix4d achieved_pose = computeForwardKinematics(joint_positions);
    Eigen::Matrix<double, 6, 1> error = computePoseError(achieved_pose, target_pose);

    double pos_error = error.head<3>().norm();
    double rot_error = error.tail<3>().norm();

    if (config_.verbose) {
        std::cout << "[IK Validation] Position error: " << pos_error * 1000.0 << " mm, "
                  << "Rotation error: " << rot_error * 180.0 / M_PI << " deg" << std::endl;
    }

    return (pos_error < config_.ik_tolerance * 10.0 &&
            rot_error < config_.ik_rotation_tolerance * 10.0);
}

Eigen::Matrix<double, 6, 1> FrankaKinematics::computePoseError(
    const Eigen::Matrix4d& current,
    const Eigen::Matrix4d& target) const
{
    Eigen::Matrix<double, 6, 1> error;

    // Position error
    error.head<3>() = target.block<3, 1>(0, 3) - current.block<3, 1>(0, 3);

    // Orientation error (axis-angle representation)
    Eigen::Matrix3d R_current = current.block<3, 3>(0, 0);
    Eigen::Matrix3d R_target = target.block<3, 3>(0, 0);
    Eigen::Matrix3d R_error = R_target * R_current.transpose();

    Eigen::AngleAxisd aa(R_error);
    error.tail<3>() = aa.angle() * aa.axis();

    return error;
}

} // namespace franka_control
