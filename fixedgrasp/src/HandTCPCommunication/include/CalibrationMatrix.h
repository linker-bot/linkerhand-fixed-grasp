#ifndef LINKERHAND_CALIBRATION_MATRIX_H
#define LINKERHAND_CALIBRATION_MATRIX_H
#include <Eigen/Dense>
#include <string>

namespace calibration {
// Tolerances accommodate the six-decimal rotation entries in the bundled file.
constexpr double RotationTolerance = 1e-5;
constexpr double HomogeneousRowTolerance = 1e-9;

bool validateRigidTransform(const Eigen::Matrix4d &matrix, std::string &error);
// Read 16 whitespace-delimited values in row-major order. Output is unchanged on failure.
bool readRigidTransform(const std::string &path, Eigen::Matrix4d &output, std::string &error);
} // namespace calibration
#endif
