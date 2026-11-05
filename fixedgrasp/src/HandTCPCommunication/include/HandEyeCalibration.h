
/*
    手眼标定
*/

#ifndef HAND_EYE_CALIBRATION_FUNCTION
#define HAND_EYE_CALIBRATION_FUNCTION

#include "handeyesolver.h"

namespace calibration
{
    class HandEyeCalibration
    {
    public:
        HandEyeCalibration();
        ~HandEyeCalibration();

        PoseMatrix ReadCalibrationResultXFile_EyeInHand(void);

        PoseMatrix GetCalibrationResult_EyeInHand(void);

        PoseMatrix HandEyeCalibrationMove(void);
        PoseMatrix CalibrationValidity(void);
        PoseMatrix CalibrationValidity_EyeInHand(void);

        uint8_t CalibrationProcess = 0U; // 标定进度

    private:
        PoseMatrix CalibrationValidity_4method(void);

        std::array<std::array<float, 6>, CalibrationSize> ReadJson_Point;
        ArrayPointData Calibration_EndPoint;

    private:
        general::GeneralFunction calibration_generalfun;

    private:
        ArrayPointData Calibration_MovePoint;
        OXYZ_Pose Calibration_RobotCurrentPose;
        Eigen::Matrix3d Calibration_Robot_matrix3x3;
        PoseMatrix Calibration_Robot_matrix;
        PoseMatrix Calibration_Camera_matrix;

        bool EyeInHand = false; // 判断是否眼在手上 true is eyeinhand
        bool CalibrationMoveCtrl;
        uint8_t CalibrationNumi = 0U;

        PoseMatrix CalibrationResultX_LeftArm;
        PoseMatrix ReadCalibrationResultX_LeftArm;
        PoseMatrix CalibrationResultX;
        PoseMatrix ReadCalibrationResultX;
        PoseMatrix CalibrationResultX_EyeInHand;
        PoseMatrix ReadCalibrationResultX_EyeInHand;

        std::vector<PoseMatrix> CalibrationHandData;
        std::vector<PoseMatrix> CalibrationEyeData;
        std::vector<std::pair<PoseMatrix, PoseMatrix>> CalibrationResultData;
    };
} // namespace calibration

#endif
