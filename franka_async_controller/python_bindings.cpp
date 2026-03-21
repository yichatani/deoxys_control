// python_bindings.cpp
// Python bindings for FrankaKinematics using pybind11

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>

#include "franka_async_controller.h"

namespace py = pybind11;
using namespace franka_control;

PYBIND11_MODULE(franka_controller, m) {
    m.doc() = "Python bindings for Franka Robot Kinematics (IK/FK/Jacobian)";

    // Bind KinematicsConfig struct
    py::class_<KinematicsConfig>(m, "KinematicsConfig")
        .def(py::init<>())
        .def_readwrite("ik_max_iterations", &KinematicsConfig::ik_max_iterations,
                      "Maximum IK solver iterations (default: 100)")
        .def_readwrite("ik_tolerance", &KinematicsConfig::ik_tolerance,
                      "IK position error tolerance in meters (default: 1e-4 = 0.1mm)")
        .def_readwrite("ik_rotation_tolerance", &KinematicsConfig::ik_rotation_tolerance,
                      "IK rotation error tolerance in radians (default: 1e-3 ~= 0.06 deg)")
        .def_readwrite("enable_ik_validation", &KinematicsConfig::enable_ik_validation,
                      "Validate IK solutions with forward kinematics (default: True)")
        .def_readwrite("urdf_path", &KinematicsConfig::urdf_path,
                      "Path to URDF file (empty = auto-search, for Pinocchio IK)")
        .def_readwrite("verbose", &KinematicsConfig::verbose,
                      "Enable verbose logging")
        .def("__repr__", [](const KinematicsConfig& self) {
            return "<KinematicsConfig ik_max_iterations=" +
                   std::to_string(self.ik_max_iterations) + ">";
        });

    // Bind the main kinematics class
    py::class_<FrankaKinematics>(m, "FrankaKinematics")
        .def(py::init<const std::string&, const KinematicsConfig&>(),
             py::arg("robot_ip"),
             py::arg("config") = KinematicsConfig(),
             "Initialize the Franka kinematics module with robot connection\n\n"
             "Args:\n"
             "    robot_ip: IP address of the Franka robot\n"
             "    config: Kinematics configuration (optional)")

        .def(py::init<const KinematicsConfig&>(),
             py::arg("config"),
             "Initialize the Franka kinematics module in offline mode (Pinocchio only)\n\n"
             "Args:\n"
             "    config: Kinematics configuration (set urdf_path!)\n\n"
             "Example:\n"
             "    config = fc.KinematicsConfig()\n"
             "    config.urdf_path = '/path/to/fr3_robot.urdf'\n"
             "    kin = fc.FrankaKinematics(config)\n"
             "    kin.initialize()")

        .def("initialize", &FrankaKinematics::initialize,
             "Connect to robot and initialize kinematics model\n\n"
             "Returns:\n"
             "    bool: True if successful")

        .def("is_initialized", &FrankaKinematics::isInitialized,
             "Check if kinematics model is initialized\n\n"
             "Returns:\n"
             "    bool: True if initialized")

        // read_joint_positions
        .def("read_joint_positions",
             [](const FrankaKinematics& self) -> py::object {
                 auto q = self.readJointPositions();
                 if (q.has_value()) {
                     py::array_t<double> arr({7}, {sizeof(double)}, q.value().data());
                     return py::object(arr);
                 }
                 return py::object(py::none());
             },
             "Read current joint positions from robot\n\n"
             "Returns:\n"
             "    7-DOF joint positions (numpy array) or None")

        // read_current_pose
        .def("read_current_pose",
             [](const FrankaKinematics& self) -> py::object {
                 auto pose = self.readCurrentPose();
                 if (pose.has_value()) {
                     py::array_t<double> result({4, 4});
                     auto r = result.mutable_unchecked<2>();
                     for (size_t i = 0; i < 4; i++)
                         for (size_t j = 0; j < 4; j++)
                             r(i, j) = pose.value()(i, j);
                     return py::object(result);
                 }
                 return py::object(py::none());
             },
             "Read current end-effector pose from robot\n\n"
             "Returns:\n"
             "    4x4 numpy array or None")

        // ======================================================================
        // IK Solver / FK / Jacobian API
        // ======================================================================

        // solve_ik: 4x4 pose matrix + 7-DOF seed -> joint positions
        .def("solve_ik",
             [](const FrankaKinematics& self,
                py::array_t<double> target_pose,
                py::array_t<double> seed) -> py::object {
                 if (target_pose.ndim() != 2 || target_pose.shape(0) != 4 || target_pose.shape(1) != 4) {
                     throw std::invalid_argument("target_pose must be a 4x4 matrix");
                 }
                 if (seed.size() != 7) {
                     throw std::invalid_argument("seed must have 7 elements");
                 }

                 Eigen::Matrix4d pose_matrix;
                 auto p = target_pose.unchecked<2>();
                 for (size_t i = 0; i < 4; i++)
                     for (size_t j = 0; j < 4; j++)
                         pose_matrix(i, j) = p(i, j);

                 std::array<double, 7> seed_array;
                 auto s = seed.unchecked<1>();
                 for (size_t i = 0; i < 7; i++)
                     seed_array[i] = s(i);

                 auto result = self.solveIK(pose_matrix, seed_array);
                 if (result.has_value()) {
                     py::array_t<double> arr({7}, {sizeof(double)}, result.value().data());
                     return py::object(arr);
                 }
                 return py::object(py::none());
             },
             py::arg("target_pose"),
             py::arg("seed"),
             "Solve inverse kinematics for a target pose\n\n"
             "Uses Pinocchio if available, otherwise falls back to Jacobian method.\n\n"
             "Args:\n"
             "    target_pose: 4x4 homogeneous transformation matrix (numpy array)\n"
             "    seed: 7-DOF initial joint positions for IK solver (numpy array)\n\n"
             "Returns:\n"
             "    7-DOF joint positions (numpy array) or None if IK failed")

        // solve_ik: position + quaternion + seed -> joint positions
        .def("solve_ik",
             [](const FrankaKinematics& self,
                py::array_t<double> position,
                py::array_t<double> quaternion,
                py::array_t<double> seed) -> py::object {
                 if (position.size() != 3) {
                     throw std::invalid_argument("position must have 3 elements [x, y, z]");
                 }
                 if (quaternion.size() != 4) {
                     throw std::invalid_argument("quaternion must have 4 elements [w, x, y, z]");
                 }
                 if (seed.size() != 7) {
                     throw std::invalid_argument("seed must have 7 elements");
                 }

                 auto p = position.unchecked<1>();
                 auto q = quaternion.unchecked<1>();

                 Eigen::Quaterniond quat(q(0), q(1), q(2), q(3));
                 quat.normalize();

                 Eigen::Matrix4d pose_matrix = Eigen::Matrix4d::Identity();
                 pose_matrix.block<3, 3>(0, 0) = quat.toRotationMatrix();
                 pose_matrix(0, 3) = p(0);
                 pose_matrix(1, 3) = p(1);
                 pose_matrix(2, 3) = p(2);

                 std::array<double, 7> seed_array;
                 auto s = seed.unchecked<1>();
                 for (size_t i = 0; i < 7; i++)
                     seed_array[i] = s(i);

                 auto result = self.solveIK(pose_matrix, seed_array);
                 if (result.has_value()) {
                     py::array_t<double> arr({7}, {sizeof(double)}, result.value().data());
                     return py::object(arr);
                 }
                 return py::object(py::none());
             },
             py::arg("position"),
             py::arg("quaternion"),
             py::arg("seed"),
             "Solve IK from position + quaternion\n\n"
             "Args:\n"
             "    position: [x, y, z] in meters\n"
             "    quaternion: [w, x, y, z]\n"
             "    seed: 7-DOF initial joint positions\n\n"
             "Returns:\n"
             "    7-DOF joint positions (numpy array) or None if IK failed")

        // solve_ik: 4x4 pose matrix only (auto seed from current state)
        .def("solve_ik",
             [](const FrankaKinematics& self,
                py::array_t<double> target_pose) -> py::object {
                 if (target_pose.ndim() != 2 || target_pose.shape(0) != 4 || target_pose.shape(1) != 4) {
                     throw std::invalid_argument("target_pose must be a 4x4 matrix");
                 }

                 auto q = self.readJointPositions();
                 if (!q.has_value()) {
                     throw std::runtime_error("Cannot read current joint positions for IK seed. Is robot connected?");
                 }

                 Eigen::Matrix4d pose_matrix;
                 auto p = target_pose.unchecked<2>();
                 for (size_t i = 0; i < 4; i++)
                     for (size_t j = 0; j < 4; j++)
                         pose_matrix(i, j) = p(i, j);

                 auto result = self.solveIK(pose_matrix, q.value());
                 if (result.has_value()) {
                     py::array_t<double> arr({7}, {sizeof(double)}, result.value().data());
                     return py::object(arr);
                 }
                 return py::object(py::none());
             },
             py::arg("target_pose"),
             "Solve IK using current joint positions as seed\n\n"
             "Args:\n"
             "    target_pose: 4x4 homogeneous transformation matrix\n\n"
             "Returns:\n"
             "    7-DOF joint positions or None")

        // compute_fk: 7-DOF joint positions -> 4x4 pose
        .def("compute_fk",
             [](const FrankaKinematics& self, py::array_t<double> joint_positions) -> py::array_t<double> {
                 if (joint_positions.size() != 7) {
                     throw std::invalid_argument("joint_positions must have 7 elements");
                 }

                 std::array<double, 7> q_array;
                 auto q = joint_positions.unchecked<1>();
                 for (size_t i = 0; i < 7; i++)
                     q_array[i] = q(i);

                 Eigen::Matrix4d pose = self.computeForwardKinematics(q_array);

                 py::array_t<double> result({4, 4});
                 auto r = result.mutable_unchecked<2>();
                 for (size_t i = 0; i < 4; i++)
                     for (size_t j = 0; j < 4; j++)
                         r(i, j) = pose(i, j);
                 return result;
             },
             py::arg("joint_positions"),
             "Compute forward kinematics\n\n"
             "Args:\n"
             "    joint_positions: 7-DOF joint positions\n\n"
             "Returns:\n"
             "    4x4 end-effector pose matrix")

        // compute_jacobian: 7-DOF joint positions -> 6x7 Jacobian
        .def("compute_jacobian",
             [](const FrankaKinematics& self, py::array_t<double> joint_positions) -> py::array_t<double> {
                 if (joint_positions.size() != 7) {
                     throw std::invalid_argument("joint_positions must have 7 elements");
                 }

                 std::array<double, 7> q_array;
                 auto q = joint_positions.unchecked<1>();
                 for (size_t i = 0; i < 7; i++)
                     q_array[i] = q(i);

                 Eigen::Matrix<double, 6, 7> J = self.computeJacobian(q_array);

                 py::array_t<double> result({6, 7});
                 auto r = result.mutable_unchecked<2>();
                 for (size_t i = 0; i < 6; i++)
                     for (size_t j = 0; j < 7; j++)
                         r(i, j) = J(i, j);
                 return result;
             },
             py::arg("joint_positions"),
             "Compute the 6x7 geometric Jacobian at given joint configuration\n\n"
             "Args:\n"
             "    joint_positions: 7-DOF joint positions\n\n"
             "Returns:\n"
             "    6x7 Jacobian matrix (first 3 rows: linear, last 3: angular)")

        // compute_pose_error: two 4x4 poses -> 6D error
        .def("compute_pose_error",
             [](const FrankaKinematics& self,
                py::array_t<double> current_pose,
                py::array_t<double> target_pose) -> py::array_t<double> {
                 if (current_pose.ndim() != 2 || current_pose.shape(0) != 4 || current_pose.shape(1) != 4) {
                     throw std::invalid_argument("current_pose must be a 4x4 matrix");
                 }
                 if (target_pose.ndim() != 2 || target_pose.shape(0) != 4 || target_pose.shape(1) != 4) {
                     throw std::invalid_argument("target_pose must be a 4x4 matrix");
                 }

                 Eigen::Matrix4d current_mat, target_mat;
                 auto c = current_pose.unchecked<2>();
                 auto t = target_pose.unchecked<2>();
                 for (size_t i = 0; i < 4; i++) {
                     for (size_t j = 0; j < 4; j++) {
                         current_mat(i, j) = c(i, j);
                         target_mat(i, j) = t(i, j);
                     }
                 }

                 Eigen::Matrix<double, 6, 1> error = self.computePoseError(current_mat, target_mat);

                 py::array_t<double> result(6);
                 auto r = result.mutable_unchecked<1>();
                 for (size_t i = 0; i < 6; i++)
                     r(i) = error(i);
                 return result;
             },
             py::arg("current_pose"),
             py::arg("target_pose"),
             "Compute 6D pose error between two poses\n\n"
             "Args:\n"
             "    current_pose: 4x4 current pose matrix\n"
             "    target_pose: 4x4 target pose matrix\n\n"
             "Returns:\n"
             "    6D error vector [dx, dy, dz, rx, ry, rz]")

        // validate_ik_solution
        .def("validate_ik_solution",
             [](const FrankaKinematics& self,
                py::array_t<double> joint_positions,
                py::array_t<double> target_pose) -> py::dict {
                 if (joint_positions.size() != 7) {
                     throw std::invalid_argument("joint_positions must have 7 elements");
                 }
                 if (target_pose.ndim() != 2 || target_pose.shape(0) != 4 || target_pose.shape(1) != 4) {
                     throw std::invalid_argument("target_pose must be a 4x4 matrix");
                 }

                 std::array<double, 7> q_array;
                 auto q = joint_positions.unchecked<1>();
                 for (size_t i = 0; i < 7; i++)
                     q_array[i] = q(i);

                 Eigen::Matrix4d target_mat;
                 auto t = target_pose.unchecked<2>();
                 for (size_t i = 0; i < 4; i++)
                     for (size_t j = 0; j < 4; j++)
                         target_mat(i, j) = t(i, j);

                 Eigen::Matrix4d achieved_pose = self.computeForwardKinematics(q_array);
                 Eigen::Matrix<double, 6, 1> error = self.computePoseError(achieved_pose, target_mat);

                 double pos_error = error.head<3>().norm();
                 double rot_error = error.tail<3>().norm();
                 bool valid = self.validateIKSolution(q_array, target_mat);

                 py::dict result;
                 result["valid"] = valid;
                 result["position_error"] = pos_error;
                 result["rotation_error"] = rot_error;
                 result["position_error_mm"] = pos_error * 1000.0;
                 result["rotation_error_deg"] = rot_error * 180.0 / M_PI;
                 return result;
             },
             py::arg("joint_positions"),
             py::arg("target_pose"),
             "Validate an IK solution by checking FK error\n\n"
             "Args:\n"
             "    joint_positions: 7-DOF IK solution to validate\n"
             "    target_pose: 4x4 target pose that was used for IK\n\n"
             "Returns:\n"
             "    dict with keys: valid, position_error, rotation_error, "
             "position_error_mm, rotation_error_deg")

        .def("__repr__",
             [](const FrankaKinematics& self) {
                 std::string status = self.isInitialized() ? "initialized" : "not initialized";
                 return "<FrankaKinematics status=" + status + ">";
             });

    // Module-level info
    m.attr("__version__") = "2.0.0";

#ifdef USE_PINOCCHIO
    m.attr("HAS_PINOCCHIO") = true;
    m.def("has_pinocchio", []() { return true; },
          "Check if Pinocchio IK solver is available");
#else
    m.attr("HAS_PINOCCHIO") = false;
    m.def("has_pinocchio", []() { return false; },
          "Check if Pinocchio IK solver is available");
#endif

    // Utility functions
    m.def("quaternion_to_matrix",
        [](py::array_t<double> quaternion) -> py::array_t<double> {
            if (quaternion.size() != 4) {
                throw std::invalid_argument("quaternion must have 4 elements [w, x, y, z]");
            }

            auto q = quaternion.unchecked<1>();
            Eigen::Quaterniond quat(q(0), q(1), q(2), q(3));
            quat.normalize();

            Eigen::Matrix3d rot = quat.toRotationMatrix();

            py::array_t<double> result({3, 3});
            auto r = result.mutable_unchecked<2>();
            for (size_t i = 0; i < 3; i++)
                for (size_t j = 0; j < 3; j++)
                    r(i, j) = rot(i, j);
            return result;
        },
        py::arg("quaternion"),
        "Convert quaternion [w, x, y, z] to 3x3 rotation matrix");

    m.def("matrix_to_quaternion",
        [](py::array_t<double> matrix) -> py::array_t<double> {
            if (matrix.ndim() != 2 || matrix.shape(0) != 3 || matrix.shape(1) != 3) {
                throw std::invalid_argument("matrix must be 3x3");
            }

            Eigen::Matrix3d rot;
            auto m = matrix.unchecked<2>();
            for (size_t i = 0; i < 3; i++)
                for (size_t j = 0; j < 3; j++)
                    rot(i, j) = m(i, j);

            Eigen::Quaterniond quat(rot);
            quat.normalize();

            py::array_t<double> result(4);
            auto r = result.mutable_unchecked<1>();
            r(0) = quat.w();
            r(1) = quat.x();
            r(2) = quat.y();
            r(3) = quat.z();

            return result;
        },
        py::arg("matrix"),
        "Convert 3x3 rotation matrix to quaternion [w, x, y, z]");

    m.def("create_pose_matrix",
        [](py::array_t<double> position, py::array_t<double> quaternion) -> py::array_t<double> {
            if (position.size() != 3) {
                throw std::invalid_argument("position must have 3 elements");
            }
            if (quaternion.size() != 4) {
                throw std::invalid_argument("quaternion must have 4 elements");
            }

            auto p = position.unchecked<1>();
            auto q = quaternion.unchecked<1>();

            Eigen::Quaterniond quat(q(0), q(1), q(2), q(3));
            quat.normalize();

            Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
            pose.block<3, 3>(0, 0) = quat.toRotationMatrix();
            pose(0, 3) = p(0);
            pose(1, 3) = p(1);
            pose(2, 3) = p(2);

            py::array_t<double> result({4, 4});
            auto r = result.mutable_unchecked<2>();
            for (size_t i = 0; i < 4; i++)
                for (size_t j = 0; j < 4; j++)
                    r(i, j) = pose(i, j);
            return result;
        },
        py::arg("position"),
        py::arg("quaternion"),
        "Create 4x4 pose matrix from position and quaternion");
}
