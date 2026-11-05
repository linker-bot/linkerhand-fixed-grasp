
#ifndef COORDINATE_TRANSFORM
#define COORDINATE_TRANSFORM

#include "GeneralFunction.h"
#include <Eigen/Dense> // 确保包含 Eigen 库
#include "rm_service.h"

using Matrix4d = Eigen::Matrix4d;
using Matrix3d = Eigen::Matrix3d;

namespace AIMOCoordinate
// 定义矩阵类型

{
    class CoordinateTransform
    {
    public:
        OXYZ_Pose ReadX_CloudPointToBaseOXYZ(OXYZ_Pose &camera, PoseMatrix &calibrationX);
        OXYZ_Pose ReadX_CloudPointToBaseOXYZ_EyeInHand(OXYZ_Pose &camera, OXYZ_Pose &robot, PoseMatrix &calibrationX);
        Matrix4d createTransformationMatrix(Pose armpose);
        Pose calculateObjectPoseInBase(const Matrix4d &ArmCurrentMat, const Matrix4d &CaliMat, const Matrix4d &ThInCamMat);
        Pose GetGraspPose(Pose CarmerRecPose, Pose ArmRecPose, const Matrix4d &CaliMat);
        OXYZ_Pose ReadX_CameraMatrixToBaseOXYZ(PoseMatrix &camera, PoseMatrix &calibrationX);

        int TF_Point_CameraToBase(std::array<float, 6> camera_data, std::array<float, 6> calibration_data, std::array<float, 6> &robot_base_data);
        int TF_Point_CameraToTool(std::array<float, 6> camera_data, std::array<float, 6> calibration_data, std::array<float, 6> robot_base_data, std::array<float, 6> &robot_tool_data);
        ArrayPosData RobotGetCartMoveTargetPosition(ArrayPosData &camera, ArrayPosData robot); // 获取空间运动的目标点位
        ArrayPosData ReadX_CameraToBase(ArrayPosData &camera);                                 // 获取空间运动的目标点位
        ArrayPosData CameraToBase(ArrayPosData camera_basc_x, ArrayPosData camera);            // 获取空间运动的目标点位

        ArrayPosData Calibration_Camera_RobotBasic_X;
        std::array<float, 6> TF_Calibration_X;

    private:
        general::GeneralFunction coor_generalfun;
    };
} // namespace AIMOCoordinate

#endif
