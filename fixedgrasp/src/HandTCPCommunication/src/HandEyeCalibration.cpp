
#include "HandEyeCalibration.h"

namespace calibration
{
    HandEyeCalibration::HandEyeCalibration()
    {
        CalibrationMoveCtrl = true;
        ReadCalibrationResultXFile_EyeInHand();
    }
    HandEyeCalibration::~HandEyeCalibration() {}

    //////////////////////////手眼标定-读取函数////////////////////////////

    PoseMatrix HandEyeCalibration::ReadCalibrationResultXFile_EyeInHand(void)
    {
        PoseMatrix result;
        std::string path_eyeinhand = "/CalibrateMatrix_EyeInHand.txt";
        ReadCalibrationResultX_EyeInHand << calibration_generalfun.CalibrationEyetohandXRead_path(path_eyeinhand);
        // std::cout << "Init ReadCalibrationResultX_EyeInHand = " << std::endl;
        // std::cout << ReadCalibrationResultX_EyeInHand << std::endl;
        result << ReadCalibrationResultX_EyeInHand;
        return result;
    }
    PoseMatrix HandEyeCalibration::GetCalibrationResult_EyeInHand(void)
    {
        PoseMatrix result;
        result << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
        if ((ReadCalibrationResultX_EyeInHand(0, 3) == 0) && (ReadCalibrationResultX_EyeInHand(1, 3) == 0) && (ReadCalibrationResultX_EyeInHand(2, 3) == 0))
        {
            std::cout << "Device don't have calibration result, need to calibraition" << std::endl;
        }
        else
        {
            result << ReadCalibrationResultX_EyeInHand;
        }
        return result;
    }

    //////////////////////////手眼标定-运动函数和处理函数////////////////////////////
    // PoseMatrix HandEyeCalibration::HandEyeCalibrationMove(void)
    // {
    //     std::cout << "标定板 标定运动过程" << std::endl;
    //     ArrayPointData Calibration_EndPoint_Rad = calibration_generalfun.PointDegreeToRad(Calibration_EndPoint);

    //     CalibrationMoveCtrl = true;
    //     OXYZ_Pose calibration_tcp;
    //     calibration_tcp.x = 0;
    //     calibration_tcp.y = 0;
    //     calibration_tcp.z = 0;
    //     calibration_tcp.rx = 0;
    //     calibration_tcp.ry = 0;
    //     calibration_tcp.rz = 0;
    //     float TCPmass_centerr[3] = {0.13777, 0.01991, 0.04589};
    //     CalibrationGetControl()->SetRobotTCPLoad(0.375, TCPmass_centerr);
    //     CalibrationGetControl()->UserSetTCP_Offset(calibration_tcp);
    //     sleep(2);

    //     while (true)
    //     {
    //         if (CalibrationMoveCtrl == true)
    //         {
    //             switch (CalibrationNumi)
    //             {
    //             case 0:
    //                 Calibration_MovePoint = calibration_generalfun.PointDegreeToRad(ReadJson_Point.at(0));
    //                 break;
    //             case 1:
    //                 Calibration_MovePoint = calibration_generalfun.PointDegreeToRad(ReadJson_Point.at(1));
    //                 break;
    //             case 2:
    //                 Calibration_MovePoint = calibration_generalfun.PointDegreeToRad(ReadJson_Point.at(2));
    //                 break;
    //             case 3:
    //                 Calibration_MovePoint = calibration_generalfun.PointDegreeToRad(ReadJson_Point.at(3));
    //                 break;
    //             case 4:
    //                 Calibration_MovePoint = calibration_generalfun.PointDegreeToRad(ReadJson_Point.at(4));
    //                 break;
    //             case 5:
    //                 Calibration_MovePoint = calibration_generalfun.PointDegreeToRad(ReadJson_Point.at(5));
    //                 break;
    //             case 6:
    //                 Calibration_MovePoint = calibration_generalfun.PointDegreeToRad(ReadJson_Point.at(6));
    //                 break;
    //             case 7:
    //                 Calibration_MovePoint = calibration_generalfun.PointDegreeToRad(ReadJson_Point.at(7));
    //                 break;
    //             case 8:
    //                 Calibration_MovePoint = calibration_generalfun.PointDegreeToRad(ReadJson_Point.at(8));
    //                 break;
    //             case 9:
    //                 Calibration_MovePoint = calibration_generalfun.PointDegreeToRad(ReadJson_Point.at(9));
    //                 break;
    //             case 10:
    //                 Calibration_MovePoint = calibration_generalfun.PointDegreeToRad(ReadJson_Point.at(10));
    //                 break;
    //             case 11:
    //                 Calibration_MovePoint = calibration_generalfun.PointDegreeToRad(ReadJson_Point.at(11));
    //                 break;
    //             default:
    //                 break;
    //             }

    //             CalibrationGetControl()->JointMove_WorkPoint(Calibration_MovePoint, true);
    //             CalibrationProcess = (CalibrationNumi + 1) * 100 / (CalibrationSize + 1);
    //             std::cout << "运动至第" << +(CalibrationNumi + 1) << "个检测点" << std::endl;
    //             sleep(8);

    //             // 获取机械臂数据
    //             Calibration_RobotCurrentPose = CalibrationGetControl()->GetCartPose();
    //             Calibration_Robot_matrix3x3 = calibration_generalfun.RPY2Rotation(Calibration_RobotCurrentPose.rx, Calibration_RobotCurrentPose.ry, Calibration_RobotCurrentPose.rz);
    //             std::cout << "Calibration_Robot_matrix3x3 = " << std::endl;
    //             std::cout << Calibration_Robot_matrix3x3 << std::endl;
    //             Calibration_Robot_matrix << Calibration_Robot_matrix3x3(0, 0), Calibration_Robot_matrix3x3(0, 1), Calibration_Robot_matrix3x3(0, 2), Calibration_RobotCurrentPose.x,
    //                 Calibration_Robot_matrix3x3(1, 0), Calibration_Robot_matrix3x3(1, 1), Calibration_Robot_matrix3x3(1, 2), Calibration_RobotCurrentPose.y, Calibration_Robot_matrix3x3(2, 0),
    //                 Calibration_Robot_matrix3x3(2, 1), Calibration_Robot_matrix3x3(2, 2), Calibration_RobotCurrentPose.z, 0, 0, 0, 1;
    //             // 获取相机数据
    //             Calibration_Camera_matrix = GetMark()->GetEyeMarkData();
    //             CalibrationNumi = CalibrationNumi + 1;
    //             // 计算X的储存数据
    //             CalibrationHandData.push_back(Calibration_Robot_matrix);
    //             CalibrationEyeData.push_back(Calibration_Camera_matrix);
    //         }

    //         if (CalibrationNumi == CalibrationSize)
    //         {
    //             CalibrationNumi = 0;
    //             CalibrationMoveCtrl = false;
    //             CalibrationGetControl()->JointMove_WorkPoint(Calibration_EndPoint_Rad, true);
    //             CalibrationProcess = 100;
    //             std::cout << "运动至标定结束后的位置" << std::endl;
    //             break;
    //         }
    //     }

    //     static algorithm::calibration::HandEyeSolver solver;
    //     std::vector<algorithm::calibration::HandEyeMethod> calibration_methods;
    //     if (EyeInHand == false) // eye isnt in hande
    //     {
    //         // calibration_methods.push_back(algorithm::calibration::HandEyeMethod::DualQuaternion);
    //         // calibration_methods.push_back(algorithm::calibration::HandEyeMethod::KroneckerProduct);
    //         // calibration_methods.push_back(algorithm::calibration::HandEyeMethod::MinCovirance);
    //         calibration_methods.push_back(algorithm::calibration::HandEyeMethod::Iterative);
    //         std::cout << "eye not in hand!" << std::endl;
    //     }
    //     else
    //     {
    //         calibration_methods.push_back(algorithm::calibration::HandEyeMethod::Iterative_eyeinhand);
    //         std::cout << "eye in hand!" << std::endl;
    //     }
    //     std::cout << "CalibrationHandDataSize = " << CalibrationHandData.size() << std::endl;
    //     std::cout << "CalibrationEyeDataSize = " << CalibrationEyeData.size() << std::endl;

    //     if ((CalibrationHandData.size() <= CalibrationSize) && (CalibrationEyeData.size() <= CalibrationSize))
    //     {
    //         CalibrationResultData = solver.CalculateX(CalibrationHandData, CalibrationEyeData, calibration_methods);
    //         std::cout << "CalibrationSize is ok!" << std::endl;
    //     }
    //     // calibration_generalfun.CalibrationEyetohandResultWrite(CalibrationResultData);

    //     float TCPmass_centerr_default[3] = {0, 0, 0};
    //     CalibrationGetControl()->SetRobotTCPLoad(0.0, TCPmass_centerr_default);
    //     sleep(5);
    //     PoseMatrix result_x;
    //     if (EyeInHand == false)
    //     {
    //         result_x = CalibrationValidity();
    //         std::cout << "CalibrationValidity" << std::endl;
    //     }
    //     else
    //     {
    //         result_x = CalibrationValidity_EyeInHand();
    //         std::cout << "CalibrationValidity_EyeInHand" << std::endl;
    //     }

    //     usleep(100 * 1000);
    //     CalibrationHandData.clear();
    //     CalibrationEyeData.clear();
    //     CalibrationResultData.clear();
    //     std::cout << "result_x = " << std::endl
    //               << result_x << std::endl;
    //     return result_x;
    // }

    // PoseMatrix HandEyeCalibration::CalibrationValidity(void) // 验证手眼标定
    // {
    //     // 判断X有效性
    //     PoseMatrix calibration_read_X, calibration_read_Y;
    //     double method_dis_y, method_dis_x;
    //     size_t num_x = 0;

    //     Calibration_Camera_matrix = GetMark()->GetEyeMarkData();

    //     calibration_read_X = CalibrationResultData[0].first;
    //     calibration_read_Y = CalibrationResultData[0].second;
    //     std::cout << "旋转矩阵X:" << calibration_read_X << std::endl;
    //     std::cout << "旋转矩阵Y:" << calibration_read_Y << std::endl;

    //     PoseMatrix camera_post_treat = calibration_read_X.inverse() * Calibration_Camera_matrix;
    //     Eigen::Vector3d method_posttreatData(camera_post_treat(0, 3), camera_post_treat(1, 3), camera_post_treat(2, 3));

    //     // 获取机械臂数据
    //     OXYZ_Pose rob_pos = CalibrationGetControl()->GetCartPose();
    //     Eigen::Matrix3d rob_matrix3x3 = calibration_generalfun.RPY2Rotation(rob_pos.rx, rob_pos.ry, rob_pos.rz);
    //     PoseMatrix rob_matrix;
    //     rob_matrix << rob_matrix3x3(0, 0), rob_matrix3x3(0, 1), rob_matrix3x3(0, 2), rob_pos.x, rob_matrix3x3(1, 0), rob_matrix3x3(1, 1), rob_matrix3x3(1, 2), rob_pos.y, rob_matrix3x3(2, 0),
    //         rob_matrix3x3(2, 1), rob_matrix3x3(2, 2), rob_pos.z, 0, 0, 0, 1;
    //     PoseMatrix robot_post_treat = rob_matrix * calibration_read_Y;
    //     Eigen::Vector3d robot_pos(robot_post_treat(0, 3), robot_post_treat(1, 3), robot_post_treat(2, 3));

    //     Eigen::Vector3d pos_dis = method_posttreatData - robot_pos;
    //     method_dis_x = pos_dis.norm();

    //     PoseMatrix x_validity_result;
    //     x_validity_result = calibration_read_X;
    //     std::cout << "X,Y,Z偏差：" << std::endl
    //               << pos_dis << std::endl
    //               << "距离" << method_dis_x << std::endl;

    //     if ((method_dis_x > 0.01) || ((x_validity_result(0, 3) == 0) && (x_validity_result(1, 3) == 0) && (x_validity_result(2, 3) == 0)))
    //     {
    //         x_validity_result << -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0;
    //         std::cout << "结果无效，需要重新标定！" << std::endl;
    //     }

    //     CalibrationResultX = x_validity_result;
    //     std::string resultWrite_path = "/CalibrationResultX.txt";
    //     calibration_generalfun.CalibrationEyetohandXWrite_path(CalibrationResultX, resultWrite_path);
    //     sleep(1);

    //     ReadCalibrationResultX << calibration_generalfun.CalibrationEyetohandXRead_path(resultWrite_path);
    //     std::cout << "x_validity_result = " << x_validity_result << std::endl;
    //     return x_validity_result;
    // }
    // PoseMatrix HandEyeCalibration::CalibrationValidity_EyeInHand(void) // 验证手眼标定
    // {
    //     // 判断X有效性
    //     PoseMatrix calibration_read_X, calibration_read_Y;
    //     double method_dis_y, method_dis_x;
    //     size_t num_x = 0;

    //     Calibration_Camera_matrix = GetMark()->GetEyeMarkData();

    //     calibration_read_X = CalibrationResultData[0].first;
    //     calibration_read_Y = CalibrationResultData[0].second;
    //     std::cout << "旋转矩阵X:" << calibration_read_X << std::endl;
    //     std::cout << "旋转矩阵Y:" << calibration_read_Y << std::endl;

    //     // 获取机械臂数据
    //     OXYZ_Pose rob_pos = CalibrationGetControl()->GetCartPose();
    //     Eigen::Matrix3d rob_matrix3x3 = calibration_generalfun.RPY2Rotation(rob_pos.rx, rob_pos.ry, rob_pos.rz);
    //     PoseMatrix rob_matrix;
    //     rob_matrix << rob_matrix3x3(0, 0), rob_matrix3x3(0, 1), rob_matrix3x3(0, 2), rob_pos.x, rob_matrix3x3(1, 0), rob_matrix3x3(1, 1), rob_matrix3x3(1, 2), rob_pos.y, rob_matrix3x3(2, 0),
    //         rob_matrix3x3(2, 1), rob_matrix3x3(2, 2), rob_pos.z, 0, 0, 0, 1;

    //     PoseMatrix camera_post_treat = rob_matrix * calibration_read_Y * Calibration_Camera_matrix;
    //     Eigen::Vector3d method_posttreatData(camera_post_treat(0, 3), camera_post_treat(1, 3), camera_post_treat(2, 3));
    //     std::cout << "method_posttreatData : " << method_posttreatData << std::endl;

    //     PoseMatrix robot_post_treat_inverse = calibration_read_X.inverse();
    //     Eigen::Vector3d robot_pos_inverse(robot_post_treat_inverse(0, 3), robot_post_treat_inverse(1, 3), robot_post_treat_inverse(2, 3));
    //     std::cout << "robot_pos_inverse : " << robot_pos_inverse << std::endl;

    //     Eigen::Vector3d pos_dis = method_posttreatData - robot_pos_inverse;
    //     method_dis_x = pos_dis.norm();

    //     PoseMatrix x_validity_result;
    //     x_validity_result = calibration_read_Y;
    //     std::cout << "X,Y,Z偏差：" << std::endl
    //               << pos_dis << std::endl
    //               << "距离" << method_dis_x << std::endl;

    //     if ((method_dis_x > 0.01) || ((x_validity_result(0, 3) == 0) && (x_validity_result(1, 3) == 0) && (x_validity_result(2, 3) == 0)))
    //     {
    //         x_validity_result << -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0;
    //         std::cout << "结果无效，需要重新标定！" << std::endl;
    //     }

    //     CalibrationResultX = x_validity_result;
    //     std::string resultWrite_path = "/CalibrationResultX_EyeInHand.txt";
    //     calibration_generalfun.CalibrationEyetohandXWrite_path(CalibrationResultX, resultWrite_path);
    //     sleep(1);

    //     ReadCalibrationResultX_EyeInHand << calibration_generalfun.CalibrationEyetohandXRead_path(resultWrite_path);
    //     std::cout << "x_validity_result = " << x_validity_result << std::endl;
    //     return x_validity_result;
    // }
    // PoseMatrix HandEyeCalibration::CalibrationValidity_4method(void) // 验证手眼标定
    // {
    //     // 判断X有效性
    //     std::vector<PoseMatrix> calibration_read_X, calibration_read_Y;
    //     double method_dis_y[3], method_dis_x[3];
    //     size_t num_x = 0;

    //     Calibration_Camera_matrix = GetMark()->GetEyeMarkData();

    //     std::vector<std::pair<PoseMatrix, PoseMatrix>> calibration_result = calibration_generalfun.CalibrationEyetohandResultRead();
    //     for (size_t i = 0; i < 4; i++)
    //     {
    //         calibration_read_X.push_back(calibration_result[i].first);
    //         calibration_read_Y.push_back(calibration_result[i].second);
    //     }
    //     std::cout << "旋转矩阵X1:" << calibration_read_X[0] << std::endl;
    //     std::cout << "旋转矩阵X2:" << calibration_read_X[1] << std::endl;
    //     std::cout << "旋转矩阵X3:" << calibration_read_X[2] << std::endl;
    //     std::cout << "旋转矩阵X4:" << calibration_read_X[3] << std::endl;
    //     std::cout << "旋转矩阵Y1:" << calibration_read_Y[0] << std::endl;
    //     std::cout << "旋转矩阵Y2:" << calibration_read_Y[1] << std::endl;
    //     std::cout << "旋转矩阵Y3:" << calibration_read_Y[2] << std::endl;
    //     std::cout << "旋转矩阵Y4:" << calibration_read_Y[3] << std::endl;
    //     PoseMatrix camera_post_treat_1 = calibration_read_X[0].inverse() * Calibration_Camera_matrix;
    //     PoseMatrix camera_post_treat_2 = calibration_read_X[1].inverse() * Calibration_Camera_matrix;
    //     PoseMatrix camera_post_treat_3 = calibration_read_X[2].inverse() * Calibration_Camera_matrix;
    //     PoseMatrix camera_post_treat_4 = calibration_read_X[3].inverse() * Calibration_Camera_matrix;
    //     Eigen::Vector3d method1_posttreatData(camera_post_treat_1(0, 3), camera_post_treat_1(1, 3), camera_post_treat_1(2, 3));
    //     Eigen::Vector3d method2_posttreatData(camera_post_treat_2(0, 3), camera_post_treat_2(1, 3), camera_post_treat_2(2, 3));
    //     Eigen::Vector3d method3_posttreatData(camera_post_treat_3(0, 3), camera_post_treat_3(1, 3), camera_post_treat_3(2, 3));
    //     Eigen::Vector3d method4_posttreatData(camera_post_treat_4(0, 3), camera_post_treat_4(1, 3), camera_post_treat_4(2, 3));

    //     PoseMatrix y_pos_1 = calibration_read_Y[0];
    //     PoseMatrix y_pos_2 = calibration_read_Y[1];
    //     PoseMatrix y_pos_3 = calibration_read_Y[2];
    //     PoseMatrix y_pos_4 = calibration_read_Y[3];
    //     OXYZ_Pose tcp1, tcp2, tcp3, tcp4;
    //     tcp1.x = y_pos_1(0, 3);
    //     tcp1.y = y_pos_1(1, 3);
    //     tcp1.z = y_pos_1(2, 3);
    //     tcp2.x = y_pos_2(0, 3);
    //     tcp2.y = y_pos_2(1, 3);
    //     tcp2.z = y_pos_2(2, 3);
    //     tcp3.x = y_pos_3(0, 3);
    //     tcp3.y = y_pos_3(1, 3);
    //     tcp3.z = y_pos_3(2, 3);
    //     tcp4.x = y_pos_4(0, 3);
    //     tcp4.y = y_pos_4(1, 3);
    //     tcp4.z = y_pos_4(2, 3);
    //     tcp1.rx = 0;
    //     tcp1.ry = 0;
    //     tcp1.rz = 0;
    //     tcp2.rx = 0;
    //     tcp2.ry = 0;
    //     tcp2.rz = 0;
    //     tcp3.rx = 0;
    //     tcp3.ry = 0;
    //     tcp3.rz = 0;
    //     tcp4.rx = 0;
    //     tcp4.ry = 0;
    //     tcp4.rz = 0;

    //     CalibrationGetControl()->UserSetTCP_Offset(tcp1);
    //     OXYZ_Pose tcp_rob_pos1 = CalibrationGetControl()->GetCartPose();
    //     Eigen::Vector3d robot_pos1(tcp_rob_pos1.x, tcp_rob_pos1.y, tcp_rob_pos1.z);
    //     std::cout << "robot data setcoor1: " << robot_pos1 << std::endl;
    //     Eigen::Vector3d pos_dis1 = method1_posttreatData - robot_pos1;

    //     CalibrationGetControl()->UserSetTCP_Offset(tcp2);
    //     OXYZ_Pose tcp_rob_pos2 = CalibrationGetControl()->GetCartPose();
    //     Eigen::Vector3d robot_pos2(tcp_rob_pos2.x, tcp_rob_pos2.y, tcp_rob_pos2.z);
    //     std::cout << "robot data setcoor2: " << robot_pos2 << std::endl;
    //     Eigen::Vector3d pos_dis2 = method2_posttreatData - robot_pos2;

    //     CalibrationGetControl()->UserSetTCP_Offset(tcp3);
    //     OXYZ_Pose tcp_rob_pos3 = CalibrationGetControl()->GetCartPose();
    //     Eigen::Vector3d robot_pos3(tcp_rob_pos3.x, tcp_rob_pos3.y, tcp_rob_pos3.z);
    //     std::cout << "robot data setcoor3: " << robot_pos3 << std::endl;
    //     Eigen::Vector3d pos_dis3 = method3_posttreatData - robot_pos3;

    //     CalibrationGetControl()->UserSetTCP_Offset(tcp4);
    //     OXYZ_Pose tcp_rob_pos4 = CalibrationGetControl()->GetCartPose();
    //     Eigen::Vector3d robot_pos4(tcp_rob_pos4.x, tcp_rob_pos4.y, tcp_rob_pos4.z);
    //     std::cout << "robot data setcoor4: " << robot_pos4 << std::endl;
    //     Eigen::Vector3d pos_dis4 = method4_posttreatData - robot_pos4;

    //     method_dis_x[0] = pos_dis1.norm();
    //     method_dis_x[1] = pos_dis2.norm();
    //     method_dis_x[2] = pos_dis4.norm();
    //     num_x = calibration_generalfun.MinData3NumChoose(method_dis_x);

    //     PoseMatrix x_validity_result;
    //     x_validity_result = calibration_read_X[num_x];
    //     calibration_generalfun.CalibrationEyetohandXWrite(x_validity_result);

    //     std::cout << "X 方法1：X,Y,Z偏差：" << std::endl
    //               << pos_dis1 << std::endl
    //               << "距离" << method_dis_x[0] << std::endl;
    //     std::cout << "X 方法2：X,Y,Z偏差：" << std::endl
    //               << pos_dis2 << std::endl
    //               << "距离" << method_dis_x[1] << std::endl;
    //     std::cout << "X 方法3：X,Y,Z偏差：" << std::endl
    //               << pos_dis3 << std::endl;
    //     std::cout << "X 方法4：X,Y,Z偏差：" << std::endl
    //               << pos_dis4 << std::endl
    //               << "距离" << method_dis_x[2] << std::endl
    //               << std::endl;
    //     std::cout << "距离最近的方法：" << +(num_x + 1) << "; " << std::endl
    //               << "距离值：" << method_dis_x[num_x] << std::endl;

    //     if ((method_dis_x[num_x] > 0.01) || ((x_validity_result(0, 3) == 0) && (x_validity_result(1, 3) == 0) && (x_validity_result(2, 3) == 0)))
    //     {
    //         x_validity_result << -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0;
    //         std::cout << "结果无效，需要重新标定！" << std::endl;
    //     }

    //     CalibrationResultX = x_validity_result;
    //     std::cout << "x_validity_result = " << x_validity_result << std::endl;
    //     calibration_read_X.clear();
    //     calibration_read_Y.clear();
    //     return x_validity_result;
    // }

} // namespace calibration
