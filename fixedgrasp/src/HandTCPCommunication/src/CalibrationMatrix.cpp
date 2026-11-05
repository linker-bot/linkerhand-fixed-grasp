#include "CalibrationMatrix.h"
#include <cmath>
#include <fstream>
#include <limits>
#include <locale>
#include <sstream>

namespace calibration {
bool validateRigidTransform(const Eigen::Matrix4d &matrix, std::string &error) {
    if (!matrix.allFinite()) {
        error = "matrix contains a non-finite value";
        return false;
    }
    const Eigen::RowVector4d bottom(0, 0, 0, 1);
    if ((matrix.row(3) - bottom).cwiseAbs().maxCoeff() > HomogeneousRowTolerance) {
        error = "homogeneous row must be [0, 0, 0, 1]";
        return false;
    }
    const Eigen::Matrix3d rotation = matrix.block<3, 3>(0, 0);
    const Eigen::Matrix3d residual = rotation.transpose() * rotation - Eigen::Matrix3d::Identity();
    if (!residual.allFinite() || residual.cwiseAbs().maxCoeff() > RotationTolerance) {
        error = "rotation must be orthonormal";
        return false;
    }
    const double determinant = rotation.determinant();
    if (!std::isfinite(determinant) || std::abs(determinant - 1) > RotationTolerance) {
        error = "rotation determinant must be +1";
        return false;
    }
    if (matrix.block<3, 1>(0, 3).cwiseAbs().maxCoeff() > std::numeric_limits<float>::max()) {
        error = "translation exceeds the SDK float range";
        return false;
    }
    error.clear();
    return true;
}

bool readRigidTransform(const std::string &path, Eigen::Matrix4d &output, std::string &error) {
    const auto fail = [&](const std::string &message) {
        error = "Invalid hand-eye calibration file '" + path + "': " + message;
        return false;
    };
    if (path.empty() || path.front() != '/' || path.find('\0') != std::string::npos)
        return fail("path must be absolute and contain no NUL");
    std::ifstream file(path);
    file.imbue(std::locale::classic());
    if (!file.is_open()) return fail("cannot open file");
    Eigen::Matrix4d candidate;
    for (int index = 0; index < 16; ++index) {
        std::string token;
        if (!(file >> token)) return fail("expected 16 values; missing component " + std::to_string(index));
        std::istringstream number(token);
        number.imbue(std::locale::classic());
        double value;
        if (!(number >> std::noskipws >> value) || number.peek() != std::char_traits<char>::eof() || !std::isfinite(value))
            return fail("invalid numeric component " + std::to_string(index));
        candidate(index / 4, index % 4) = value;
    }
    std::string extra;
    if (file >> extra) return fail("expected exactly 16 values; trailing data found");
    if (file.bad()) return fail("file read failed");
    std::string matrix_error;
    if (!validateRigidTransform(candidate, matrix_error)) return fail(matrix_error);
    output = candidate;
    error.clear();
    return true;
}
} // namespace calibration
