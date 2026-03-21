// franka_async_controller.h
// Franka robot kinematics: IK solver, FK, and Jacobian computation

#pragma once

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <stdexcept>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <franka/robot.h>
#include <franka/model.h>

#ifdef USE_PINOCCHIO
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/se3.hpp>
#endif

namespace franka_control {

// Cartesian pose representation (4x4 homogeneous transformation)
using CartesianPose = Eigen::Matrix4d;

struct KinematicsConfig {
    // IK solver parameters
    int ik_max_iterations = 100;
    double ik_tolerance = 1e-4;  // Position error tolerance (m)
    double ik_rotation_tolerance = 1e-3;  // Rotation error tolerance (rad)
    bool enable_ik_validation = true;  // Validate IK solutions
    std::string urdf_path = "";  // Path to URDF (empty = use embedded path)
    bool verbose = false;
};

class FrankaKinematics {
public:
    /**
     * Constructor - with robot IP (can connect to robot for readOnce / libfranka FK)
     * @param robot_ip IP address of the Franka robot
     * @param config Kinematics configuration
     */
    FrankaKinematics(const std::string& robot_ip,
                     const KinematicsConfig& config = KinematicsConfig());

    /**
     * Constructor - offline mode (Pinocchio only, no robot connection)
     * @param config Kinematics configuration (urdf_path should be set)
     */
    explicit FrankaKinematics(const KinematicsConfig& config);

    ~FrankaKinematics();

    // Disable copy and move
    FrankaKinematics(const FrankaKinematics&) = delete;
    FrankaKinematics& operator=(const FrankaKinematics&) = delete;

    /**
     * Initialize the kinematics model
     * If robot_ip was provided, also connects to robot.
     * If only config was provided, initializes Pinocchio only.
     * @return true if successful
     */
    bool initialize();

    /**
     * Check if the kinematics model is initialized
     */
    bool isInitialized() const { return initialized_; }

    // IK solver - tries Pinocchio first, falls back to Jacobian method
    std::optional<std::array<double, 7>> solveIK(
        const Eigen::Matrix4d& target_pose,
        const std::array<double, 7>& seed) const;

#ifdef USE_PINOCCHIO
    // Pinocchio-based IK solver (faster and more robust)
    std::optional<std::array<double, 7>> solveIK_Pinocchio(
        const Eigen::Matrix4d& target_pose,
        const std::array<double, 7>& seed) const;
#endif

    // Fallback Jacobian-based IK solver (requires robot connection)
    std::optional<std::array<double, 7>> solveIK_Jacobian(
        const Eigen::Matrix4d& target_pose,
        const std::array<double, 7>& seed) const;

    // Forward kinematics (uses Pinocchio if available, else libfranka)
    Eigen::Matrix4d computeForwardKinematics(
        const std::array<double, 7>& joint_positions) const;

    // Validate IK solution
    bool validateIKSolution(
        const std::array<double, 7>& joint_positions,
        const Eigen::Matrix4d& target_pose) const;

    // Compute pose error (6D: position + rotation)
    Eigen::Matrix<double, 6, 1> computePoseError(
        const Eigen::Matrix4d& current,
        const Eigen::Matrix4d& target) const;

    // Compute Jacobian matrix (uses Pinocchio if available, else libfranka)
    Eigen::Matrix<double, 6, 7> computeJacobian(
        const std::array<double, 7>& joint_positions) const;

    /**
     * Read current joint positions from robot (requires robot connection)
     * @return 7-DOF joint positions if available
     */
    std::optional<std::array<double, 7>> readJointPositions() const;

    /**
     * Read current end-effector pose from robot (requires robot connection)
     * @return 4x4 transformation matrix if available
     */
    std::optional<Eigen::Matrix4d> readCurrentPose() const;

private:
    // Robot connection
    std::string robot_ip_;
    std::shared_ptr<franka::Robot> robot_;
    bool initialized_{false};
    bool has_robot_ip_{false};

    // Configuration
    KinematicsConfig config_;

    // libfranka FK/Jacobian (requires robot connection)
    Eigen::Matrix4d computeFK_libfranka(
        const std::array<double, 7>& joint_positions) const;
    Eigen::Matrix<double, 6, 7> computeJacobian_libfranka(
        const std::array<double, 7>& joint_positions) const;

#ifdef USE_PINOCCHIO
    // Pinocchio model for IK/FK/Jacobian
    std::unique_ptr<pinocchio::Model> pinocchio_model_;
    std::unique_ptr<pinocchio::Data> pinocchio_data_;
    pinocchio::FrameIndex ee_frame_id_;
    bool pinocchio_initialized_{false};

    // Initialize Pinocchio model
    bool initializePinocchioModel();

    // Pinocchio FK/Jacobian (no robot needed)
    Eigen::Matrix4d computeFK_Pinocchio(
        const std::array<double, 7>& joint_positions) const;
    Eigen::Matrix<double, 6, 7> computeJacobian_Pinocchio(
        const std::array<double, 7>& joint_positions) const;
#endif
};

} // namespace franka_control
