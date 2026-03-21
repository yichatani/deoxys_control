cmake_minimum_required(VERSION 3.10)
project(franka_async_controller)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(Franka REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)

# Try to find Pinocchio
find_package(pinocchio QUIET)
if(pinocchio_FOUND)
    message(STATUS "✓ Found Pinocchio: ${pinocchio_VERSION}")
    message(STATUS "✓ Pinocchio IK solver will be used (faster and more robust)")
    add_definitions(-DUSE_PINOCCHIO)
else()
    message(WARNING "✗ Pinocchio not found. Will use fallback Jacobian IK solver.")
    message(WARNING "For better IK performance, install Pinocchio:")
    message(WARNING "  sudo apt install robotpkg-pinocchio")
endif()

# C++ library (can be used standalone)
add_library(franka_async_controller_lib SHARED
    franka_async_controller.cpp
)

target_include_directories(franka_async_controller_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(franka_async_controller_lib
    Franka::Franka
    Eigen3::Eigen
    pthread
)

if(pinocchio_FOUND)
    target_link_libraries(franka_async_controller_lib pinocchio::pinocchio)
endif()

# Python module
pybind11_add_module(franka_controller
    python_bindings.cpp
    franka_async_controller.cpp
)

target_link_libraries(franka_controller PRIVATE
    Franka::Franka
    Eigen3::Eigen
    pthread
)

if(pinocchio_FOUND)
    target_link_libraries(franka_controller PRIVATE pinocchio::pinocchio)
endif()

# Install Python module
install(TARGETS franka_controller
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages
)

# Install C++ library
install(TARGETS franka_async_controller_lib
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

install(FILES franka_async_controller.h
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)

# Install URDF file
install(FILES fr3_robot.urdf
    DESTINATION ${CMAKE_INSTALL_PREFIX}/share/franka_async_controller
)
