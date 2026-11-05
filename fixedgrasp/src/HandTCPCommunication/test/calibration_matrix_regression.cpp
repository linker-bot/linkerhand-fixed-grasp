#include "CalibrationMatrix.h"
#include <gtest/gtest.h>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <unistd.h>

namespace calibration {
class CalibrationFileTest : public testing::Test {
protected:
    std::string path, error;
    Eigen::Matrix4d output = Eigen::Matrix4d::Constant(7);
    void SetUp() override {
        char name[] = "/tmp/linkerhand-calibration-XXXXXX";
        const int fd = ::mkstemp(name); ASSERT_GE(fd, 0); ::close(fd); path = name;
    }
    void TearDown() override { ::unlink(path.c_str()); }
    void write(const Eigen::Matrix4d &matrix) {
        std::ofstream file(path); file << std::setprecision(17);
        for (int row = 0; row < 4; ++row)
            for (int column = 0; column < 4; ++column) file << matrix(row, column) << '\n';
    }
    bool load() { return readRigidTransform(path, output, error); }
};
TEST_F(CalibrationFileTest, AcceptsIdentityWithoutTreatingZeroTranslationAsMissing) {
    write(Eigen::Matrix4d::Identity()); ASSERT_TRUE(load()) << error;
    EXPECT_TRUE(output.isIdentity());
}
TEST_F(CalibrationFileTest, PreservesNonIdentityRotationWithZeroTranslation) {
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    matrix.block<3, 3>(0, 0) = Eigen::AngleAxisd(.5, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    write(matrix); ASSERT_TRUE(load()) << error; EXPECT_TRUE(output.isApprox(matrix));
    EXPECT_FALSE(output.isIdentity());
}
TEST_F(CalibrationFileTest, ReadsRowMajorRotationAndTranslation) {
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    matrix.block<3, 3>(0, 0) = Eigen::AngleAxisd(.4, Eigen::Vector3d::UnitY()).toRotationMatrix();
    matrix(0, 3) = .12; matrix(1, 3) = -.34; matrix(2, 3) = .56;
    write(matrix); ASSERT_TRUE(load()) << error; EXPECT_TRUE(output.isApprox(matrix));
    EXPECT_DOUBLE_EQ(.12, output(0, 3)); EXPECT_DOUBLE_EQ(0, output(3, 0));
}
TEST_F(CalibrationFileTest, AcceptsRowsWhitespaceCrLfAndScientificNotation) {
    std::ofstream(path) << "\r\n+1e0\t0 0 1.25e-2\r\n0 1 0 -2.5E-2\n0 0 1 0\n0 0 0 1\n  \t";
    ASSERT_TRUE(load()) << error; EXPECT_DOUBLE_EQ(.0125, output(0, 3));
    EXPECT_DOUBLE_EQ(-.025, output(1, 3));
}
TEST_F(CalibrationFileTest, RejectsMissingRelativeAndEmbeddedNulPaths) {
    EXPECT_FALSE(readRigidTransform(path + ".missing", output, error));
    EXPECT_FALSE(readRigidTransform("calibration.txt", output, error));
    EXPECT_FALSE(readRigidTransform("", output, error));
    EXPECT_FALSE(readRigidTransform(path + std::string("\0tail", 5), output, error));
    EXPECT_TRUE(output.isConstant(7));
}
TEST_F(CalibrationFileTest, RequiresExactlySixteenValues) {
    for (int count : {0, 1, 15, 17, 256, 1024}) {
        { std::ofstream file(path); for (int i = 0; i < count; ++i) file << "1\n"; }
        EXPECT_FALSE(load()) << count; EXPECT_TRUE(output.isConstant(7)); EXPECT_FALSE(error.empty());
    }
}
TEST_F(CalibrationFileTest, RejectsMalformedOrPartiallyNumericComponents) {
    for (const auto &token : {"text", "1junk", "0x1", "1,0", "--1", "1e", "#"}) {
        std::ofstream(path) << token << " 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1";
        EXPECT_FALSE(load()) << token; EXPECT_TRUE(output.isConstant(7));
    }
}
TEST_F(CalibrationFileTest, RejectsNonFiniteAndOverflowTokens) {
    for (const auto &token : {"nan", "NaN", "inf", "-inf", "Infinity", "1e309", "-1e309"}) {
        std::ofstream(path) << token << " 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1";
        EXPECT_FALSE(load()) << token; EXPECT_TRUE(output.isConstant(7));
    }
}
TEST_F(CalibrationFileTest, RejectsTrailingNumbersTextAndComments) {
    for (const auto &extra : {"0", "garbage", "# comment"}) {
        write(Eigen::Matrix4d::Identity()); std::ofstream(path, std::ios::app) << extra;
        EXPECT_FALSE(load()) << extra; EXPECT_TRUE(output.isConstant(7));
    }
}
TEST_F(CalibrationFileTest, RejectsNonHomogeneousBottomRows) {
    for (int column = 0; column < 4; ++column) {
        Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity(); matrix(3, column) += .01;
        write(matrix); EXPECT_FALSE(load()) << column;
    }
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity(); matrix(3, 3) = 0;
    write(matrix); EXPECT_FALSE(load());
}
TEST_F(CalibrationFileTest, RejectsScaledShearedAndSingularRotations) {
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity(); matrix(0, 0) = 2;
    write(matrix); EXPECT_FALSE(load());
    matrix.setIdentity(); matrix(0, 1) = .01; write(matrix); EXPECT_FALSE(load());
    matrix.block<3, 3>(0, 0).setZero(); write(matrix); EXPECT_FALSE(load());
}
TEST_F(CalibrationFileTest, RejectsReflectionsEvenWhenOrthogonal) {
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity(); matrix(0, 0) = -1;
    write(matrix); EXPECT_FALSE(load()); EXPECT_NE(std::string::npos, error.find("determinant"));
}
TEST_F(CalibrationFileTest, RejectsHugeRotationAndUnrepresentableTranslation) {
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity(); matrix(0, 0) = 1e200;
    write(matrix); EXPECT_FALSE(load());
    matrix.setIdentity(); matrix(0, 3) = 1e100; write(matrix); EXPECT_FALSE(load());
}
TEST_F(CalibrationFileTest, UsesDocumentedRotationAndHomogeneousTolerances) {
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity(); matrix(0, 0) += 1e-6;
    matrix(3, 0) = 1e-10;
    write(matrix); EXPECT_TRUE(load()) << error;
    matrix(0, 0) += 1e-4; write(matrix); EXPECT_FALSE(load());
    matrix.setIdentity(); matrix(3, 0) = 1e-8; write(matrix); EXPECT_FALSE(load());
}
TEST_F(CalibrationFileTest, PreservesOutputOnFailureAndClearsErrorOnSuccess) {
    std::ofstream(path) << "invalid";
    EXPECT_FALSE(load()); EXPECT_TRUE(output.isConstant(7)); EXPECT_FALSE(error.empty());
    write(Eigen::Matrix4d::Identity()); ASSERT_TRUE(load()) << error; EXPECT_TRUE(error.empty());
}
TEST(CalibrationMatrix, RejectsNonFiniteValuesInEveryMatrixPosition) {
    std::string error;
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            for (double value : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
                Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity(); matrix(row, column) = value;
                EXPECT_FALSE(validateRigidTransform(matrix, error));
            }
        }
    }
}
} // namespace calibration
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
