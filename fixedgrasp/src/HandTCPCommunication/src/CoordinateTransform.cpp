/*
   Robot Move control and planning and tracking
*/

#include "CoordinateTransform.h"
using namespace Eigen;
namespace AIMOCoordinate
{
    // 相机坐标系下的点转换到基坐标系
    //  param:
    //  输入：（输入）camera  相机数据（输入） 4x4 array
    //       （输入）robot   基坐标系机械臂数据 4x4 array
    //  输出：基坐标系机械臂数据
    //  注意：1. array 顺序依次是：（rx1 rx2 rx3 x ry1 ry2 ry3 y rz1 rz2 rz3 z）;
    //       2. x y z 单位 m；
    OXYZ_Pose CoordinateTransform::ReadX_CameraMatrixToBaseOXYZ(PoseMatrix &camera, PoseMatrix &calibrationX) // 获取空间运动的目标点位
    {
        // std::cout << "handEye_CalibrationX" << calibrationX << std::endl;
        PoseMatrix track_matrixdata = calibrationX.inverse() * camera; // 转化坐标系(绕Z轴旋转)
        // std::cout << "track_matrixdata " << track_matrixdata << std::endl;

        Eigen::Matrix3d angle_matrix;
        angle_matrix << track_matrixdata(0, 0), track_matrixdata(0, 1), track_matrixdata(0, 2), track_matrixdata(1, 0), track_matrixdata(1, 1), track_matrixdata(1, 2), track_matrixdata(2, 0),
            track_matrixdata(2, 1), track_matrixdata(2, 2);
        Eigen::Vector3d angle = coor_generalfun.Rotation2RPY(angle_matrix);
        OXYZ_Pose result_data;
        result_data.x = track_matrixdata(0, 3);
        result_data.y = track_matrixdata(1, 3);
        result_data.z = track_matrixdata(2, 3);
        result_data.rx = angle(0);
        result_data.ry = angle(1);
        result_data.rz = angle(2);
        return result_data;
    }
    OXYZ_Pose CoordinateTransform::ReadX_CloudPointToBaseOXYZ(OXYZ_Pose &camera, PoseMatrix &calibrationX) // 获取空间运动的目标点位
    {
        OXYZ_Pose result_data;
        // std::cout << "handEye_CalibrationX" << calibrationX << std::endl;

        Eigen::Matrix<double, 4, 1> camera_pos, camera_angle;
        camera_pos << camera.x, camera.y, camera.z, 1;
        camera_angle << camera.rx, camera.ry, camera.rz, 1;

        Eigen::Matrix<double, 4, 1> track_pos = calibrationX.inverse() * camera_pos;
        Eigen::Matrix<double, 4, 1> track_angletemp = calibrationX.inverse() * camera_angle;

        Eigen::Matrix<double, 3, 1> track_angle;
        track_angle << track_angletemp(0), track_angletemp(1), track_angletemp(2);
        track_angle.normalize();
        // std::cout << "track_pos " << track_pos << std::endl;
        // std::cout << "track_angle " << track_angle << std::endl;
        result_data.x = track_pos(0);
        result_data.y = track_pos(1);
        result_data.z = track_pos(2);
        result_data.rx = track_angle(0);
        result_data.ry = track_angle(1);
        result_data.rz = track_angle(2);

        return result_data;
    }
    OXYZ_Pose CoordinateTransform::ReadX_CloudPointToBaseOXYZ_EyeInHand(OXYZ_Pose &camera, OXYZ_Pose &robot, PoseMatrix &calibrationX) // 获取空间运动的目标点位
    {

        OXYZ_Pose result_data;
        // std::cout << "handEye_CalibrationX" << calibrationX << std::endl;

        Eigen::Matrix<double, 4, 1> camera_pos, camera_angle;
        camera_pos << camera.x, camera.y, camera.z, 1;
        camera_angle << camera.rx, camera.ry, camera.rz, 1;

        Eigen::Matrix<double, 4, 4> robot_temp;
        Eigen::Matrix3d robot_Rxyz;
        robot_Rxyz = coor_generalfun.RPY2Rotation(robot.rx, robot.ry, robot.rz);
        robot_temp << robot_Rxyz(0, 0), robot_Rxyz(0, 1), robot_Rxyz(0, 2), robot.x, robot_Rxyz(1, 0), robot_Rxyz(1, 1), robot_Rxyz(1, 2), robot.y, robot_Rxyz(2, 0), robot_Rxyz(2, 1),
            robot_Rxyz(2, 2), robot.z, 0, 0, 0, 1;

        Eigen::Matrix<double, 4, 1> track_pos = robot_temp * calibrationX * camera_pos;
        Eigen::Matrix<double, 4, 1> track_angletemp = robot_temp * calibrationX * camera_angle;

        Eigen::Matrix<double, 3, 1> track_angle;
        track_angle << track_angletemp(0), track_angletemp(1), track_angletemp(2);
        track_angle.normalize();
        // std::cout << "track_pos " << track_pos << std::endl;
        // std::cout << "track_angle " << track_angle << std::endl;
        result_data.x = track_pos(0);
        result_data.y = track_pos(1);
        result_data.z = track_pos(2);
        result_data.rx = track_angle(0);
        result_data.ry = track_angle(1);
        result_data.rz = track_angle(2);

        return result_data;
    }

    Matrix4d CoordinateTransform::createTransformationMatrix(Pose armpose)
    {
        double x, y, z, rx, ry, rz;

        x = armpose.position.x;
        y = armpose.position.y;
        z = armpose.position.z;
        rx = armpose.euler.rx;
        ry = armpose.euler.ry;
        rz = armpose.euler.rz;

        // 创建平移矩阵
        Eigen::Translation3d translation(x, y, z);

        // 创建每个轴的旋转矩阵
        Eigen::Matrix3d rotationX;
        rotationX << 1, 0, 0,
            0, cos(rx), -sin(rx),
            0, sin(rx), cos(rx);

        Eigen::Matrix3d rotationY;
        rotationY << cos(ry), 0, sin(ry),
            0, 1, 0,
            -sin(ry), 0, cos(ry);

        Eigen::Matrix3d rotationZ;
        rotationZ << cos(rz), -sin(rz), 0,
            sin(rz), cos(rz), 0,
            0, 0, 1;

        // 按XYZ顺序依次相乘
        Eigen::Matrix3d rotation = rotationZ * rotationY * rotationX;

        // 创建齐次变换矩阵
        Matrix4d transform = Matrix4d::Identity();
        transform.block<3, 3>(0, 0) = rotation;
        transform.block<3, 1>(0, 3) = translation.vector();

        return transform;
    }
    // 计算物体在机械臂基坐标系下的位姿，输入分别为机械臂当前位姿矩阵、手眼标定矩阵、识别到的物体在相机下的抓取位姿
    Pose CoordinateTransform::calculateObjectPoseInBase(const Matrix4d &ArmCurrentMat, const Matrix4d &CaliMat, const Matrix4d &ThInCamMat)
    {
        // Matrix4d Cam_To_GraspTool;
        // Cam_To_GraspTool << 0, 0, 1, 0,
        //     0, 1, 0, 0,
        //     -1, 0, 0, 0,
        //     0, 0, 0, 1;
        // // 计算物体在机械臂基坐标系下的位姿
        // Matrix4d T_obj_in_base = ArmCurrentMat * CaliMat * ThInCamMat * Cam_To_GraspTool;
        // 确保矩阵乘法的顺序正确，通常情况下，需要先应用手眼标定，再应用机械臂当前位姿
        Matrix4d T_obj_in_base = ArmCurrentMat * CaliMat * ThInCamMat;

        // 提取平移部分
        Pose pose;
        Eigen::Vector3d position = T_obj_in_base.block<3, 1>(0, 3);
        Eigen::Matrix3d rotationMatrix = T_obj_in_base.block<3, 3>(0, 0);

        // 计算欧拉角 (rx, ry, rz)，注意这里的旋转顺序可能需要调整
        pose.position.x = position.x();
        pose.position.y = position.y();
        pose.position.z = position.z();

        // 注意：如果旋转顺序是XYZ，你可以使用Eigen的eulerAngles方法来计算
        Eigen::Vector3d euler_angles = rotationMatrix.eulerAngles(0, 1, 2); // XYZ 顺序
        pose.euler.rx = euler_angles.x();
        pose.euler.ry = euler_angles.y();
        pose.euler.rz = euler_angles.z();

        return pose;
    }
    Pose CoordinateTransform::GetGraspPose(Pose CarmerRecPose, Pose ArmRecPose, const Matrix4d &CaliMat)
    {
        Pose GraspPose;

        Matrix4d MatrixCamera = createTransformationMatrix(CarmerRecPose);
        Matrix4d MatrixArm = createTransformationMatrix(ArmRecPose);
        std::cout << "MatrixCamera: " << MatrixCamera << std::endl;
        std::cout << "MatrixArm: " << MatrixArm << std::endl;
        // std::cout << "CaliMat: " << CaliMat << std::endl;
        // 机械臂抓取姿态
        GraspPose = calculateObjectPoseInBase(MatrixArm, CaliMat, MatrixCamera);

        return GraspPose;
    }

    // 相机坐标系中的点转换到基坐标系
    // param:
    // 输入：（输入）camera_data 相机数据
    //      （输入）calibration_data 手眼标定或其他标定方法标定的camera和robot-base之间的关系 AX=XB的X
    //      （输出）robot_base_data 基坐标系机械臂数据（外部使用）
    // 输出：0 正确；其他数据 表示错误
    // 注意：1. 三组数据的array 顺序依次是：（x y z rx ry rz）;
    //      2. x y z 单位 m； rx ry rz 单位 rad
    int CoordinateTransform::TF_Point_CameraToBase(std::array<float, 6> camera_data, std::array<float, 6> calibration_data, std::array<float, 6> &robot_base_data)
    {
        tf2_ros::TransformBroadcaster broadcaster;
        geometry_msgs::TransformStamped transformStamped;
        geometry_msgs::TransformStamped transformStamped1;
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        tf2::Quaternion q;

        // 发布base->camera
        //  设置header
        transformStamped.header.stamp = ros::Time(0);
        transformStamped.header.frame_id = "base";
        transformStamped.child_frame_id = "camera";

        // 设置transform分量
        transformStamped.transform.translation.x = calibration_data[0];
        transformStamped.transform.translation.y = calibration_data[1];
        transformStamped.transform.translation.z = calibration_data[2];

        q.setRPY(calibration_data[3], calibration_data[4], calibration_data[5]); // base->camera

        // 为坐标转换消息赋值，得到四元数的四个分量，为消息的rotation的四个分量赋值
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        // point->camera
        transformStamped1.header.stamp = ros::Time(0);
        transformStamped1.header.frame_id = "camera";
        transformStamped1.child_frame_id = "point";
        transformStamped1.transform.translation.x = camera_data[0];
        transformStamped1.transform.translation.y = camera_data[1];
        transformStamped1.transform.translation.z = camera_data[2];
        q.setRPY(camera_data[3], camera_data[4], camera_data[5]);
        transformStamped1.transform.rotation.x = q.x();
        transformStamped1.transform.rotation.y = q.y();
        transformStamped1.transform.rotation.z = q.z();
        transformStamped1.transform.rotation.w = q.w();

        // 发布坐标变换
        broadcaster.sendTransform({transformStamped, transformStamped1});

        while (1)
        {
            try
            {
                transformStamped1 = tfBuffer.lookupTransform("base", "point", ros::Time(0));
                float x1 = transformStamped1.transform.translation.x;
                float y1 = transformStamped1.transform.translation.y;
                float z1 = transformStamped1.transform.translation.z;

                tf::Quaternion q1;
                tf::quaternionMsgToTF(transformStamped1.transform.rotation, q1);
                double roll, pitch, yaw;
                tf::Matrix3x3(q1).getRPY(roll, pitch, yaw);
                robot_base_data[0] = x1;
                robot_base_data[1] = y1;
                robot_base_data[2] = z1;
                robot_base_data[3] = roll * 180 / 3.14;
                robot_base_data[4] = pitch * 180 / 3.14;
                robot_base_data[5] = yaw * 180 / 3.14;

                // ROS_INFO("TF-robotbase %f %f %f %f %f %f", robot_base_data[0], robot_base_data[1], robot_base_data[2], robot_base_data[3], robot_base_data[4], robot_base_data[5]);

                break;
            }
            catch (tf2::TransformException &ex)
            {
                // ROS_WARN("%s", ex.what());
                //  ros::Duration(1.0).sleep();
                continue;
            }
        }
        return 0;
    }

    // 相机坐标系下的点转换到工具坐标系
    //  param:
    //  输入：（输入）camera_data 相机数据（输入）
    //       （输入）calibration_data 手眼标定或其他标定方法标定的camera和robot-base之间的关系 AX=XB的X
    //       （输入）robot_base_data 基坐标系机械臂数据
    //       （输出）robot_tool_data TCP坐标系机械臂数据（外部使用）
    //  输出：0 正确；其他数据 表示错误
    //  注意：1. 三组数据的array 顺序依次是：（x y z rx ry rz）;
    //       2. x y z 单位 m； rx ry rz 单位 rad
    int CoordinateTransform::TF_Point_CameraToTool(std::array<float, 6> camera_data, std::array<float, 6> calibration_data, std::array<float, 6> robot_base_data, std::array<float, 6> &robot_tool_data)
    {
        tf2_ros::TransformBroadcaster broadcaster;
        geometry_msgs::TransformStamped transformStamped;
        geometry_msgs::TransformStamped transformStamped1;
        geometry_msgs::TransformStamped transformStamped2;
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        tf2::Quaternion q;
        float tx, ty, tz, troll, tpitch, tyaw;

        // 发布base->camera
        //  设置header
        transformStamped.header.stamp = ros::Time(0);
        transformStamped.header.frame_id = "base";
        transformStamped.child_frame_id = "camera";

        // 设置transform分量
        transformStamped.transform.translation.x = calibration_data[0];
        transformStamped.transform.translation.y = calibration_data[1];
        transformStamped.transform.translation.z = calibration_data[2];

        q.setRPY(calibration_data[3], calibration_data[4], calibration_data[5]); // base->camera
        // q.setRPY(0, 0, 0);//base->camera
        // 为坐标转换消息赋值，得到四元数的四个分量，为消息的rotation的四个分量赋值
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        // camera->point
        transformStamped1.header.stamp = ros::Time(0);
        transformStamped1.header.frame_id = "camera";
        transformStamped1.child_frame_id = "point";
        transformStamped1.transform.translation.x = camera_data[0];
        transformStamped1.transform.translation.y = camera_data[1];
        transformStamped1.transform.translation.z = camera_data[2];
        q.setRPY(camera_data[3], camera_data[4], camera_data[5]);
        transformStamped1.transform.rotation.x = q.x();
        transformStamped1.transform.rotation.y = q.y();
        transformStamped1.transform.rotation.z = q.z();
        transformStamped1.transform.rotation.w = q.w();

        // base->tool
        tx = robot_base_data[0];
        ty = robot_base_data[1];
        tz = robot_base_data[2];
        troll = robot_base_data[3];
        tpitch = robot_base_data[4];
        tyaw = robot_base_data[5];
        troll = troll * 3.14 / 180;
        tpitch = tpitch * 3.14 / 180;
        tyaw = tyaw * 3.14 / 180;

        transformStamped2.header.stamp = ros::Time(0);
        transformStamped2.header.frame_id = "base";
        transformStamped2.child_frame_id = "tool";
        transformStamped2.transform.translation.x = tx;
        transformStamped2.transform.translation.y = ty;
        transformStamped2.transform.translation.z = tz;
        q.setRPY(troll, tpitch, tyaw);
        transformStamped2.transform.rotation.x = q.x();
        transformStamped2.transform.rotation.y = q.y();
        transformStamped2.transform.rotation.z = q.z();
        transformStamped2.transform.rotation.w = q.w();

        // 发布坐标变换
        // static_broadcaster.sendTransform({transformStamped,transformStamped1,transformStamped2});
        broadcaster.sendTransform({transformStamped, transformStamped1, transformStamped2});

        while (1)
        {
            try
            {
                transformStamped1 = tfBuffer.lookupTransform("tool", "point", ros::Time(0));
                float x1 = transformStamped1.transform.translation.x;
                float y1 = transformStamped1.transform.translation.y;
                float z1 = transformStamped1.transform.translation.z;

                tf::Quaternion q1;
                tf::quaternionMsgToTF(transformStamped1.transform.rotation, q1);
                double roll, pitch, yaw;
                tf::Matrix3x3(q1).getRPY(roll, pitch, yaw);
                robot_tool_data[0] = x1;
                robot_tool_data[1] = y1;
                robot_tool_data[2] = z1;
                robot_tool_data[3] = roll * 180 / 3.14;
                robot_tool_data[4] = pitch * 180 / 3.14;
                robot_tool_data[5] = yaw * 180 / 3.14;

                ROS_INFO("TF-robottool %f %f %f %f %f %f", robot_tool_data[0], robot_tool_data[1], robot_tool_data[2], robot_tool_data[3], robot_tool_data[4], robot_tool_data[5]);
                break;
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
                // ros::Duration(1.0).sleep();
                continue;
            }
        }
        return 0;
    }

    // 相机坐标系下的点转换到基坐标系
    //  param:
    //  输入：（输入）camera  相机数据（输入） 4x4 array
    //       （输入）robot   基坐标系机械臂数据 4x4 array
    //  输出：基坐标系机械臂数据
    //  注意：1. array 顺序依次是：（rx1 rx2 rx3 x ry1 ry2 ry3 y rz1 rz2 rz3 z）;
    //       2. x y z 单位 m；
    // ArrayPosData CoordinateTransform::RobotGetCartMoveTargetPosition(ArrayPosData &camera, ArrayPosData robot) // 获取空间运动的目标点位
    // {
    //     ArrayPosData result_data;
    //     PoseMatrix track_matrixdata, handEye_CalibrationX, head_pose;
    //     Eigen::Vector3f robot_pos_temp, eye_pos_temp, rob_eye_distance_temp;
    //     bool get_position = true;

    //     // 结构光相机姿态旋转矩阵的调整
    //     float angle_x = 0.5;
    //     float angle_y = 1;
    //     float angle_z = 1;
    //     PoseMatrix translation_angle_x, translation_angle_y, translation_angle_z;
    //     translation_angle_z << cos(Pi * angle_z), sin(Pi * angle_z), 0, 0, -sin(Pi * angle_z), cos(Pi * angle_z), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    //     translation_angle_y << cos(Pi * angle_y), 0, -sin(Pi * angle_y), 0, 0, 1, 0, 0, sin(Pi * angle_y), 0, cos(Pi * angle_y), 0, 0, 0, 0, 1;
    //     translation_angle_x << 1, 0, 0, 0, 0, cos(Pi * angle_x), sin(Pi * angle_x), 0, 0, -sin(Pi * angle_x), cos(Pi * angle_x), 0, 0, 0, 0, 1;

    //     handEye_CalibrationX << coor_generalfun.CalibrationEyetohandXRead();
    //     // std::cout << "handEye_CalibrationX" << handEye_CalibrationX << std::endl;

    //     // 结构光相机的靶点法向量的调整
    //     while (get_position)
    //     {
    //         if ((camera[3] == 0.0) && (camera[7] == 0.0) && (camera[11] == 0.0))
    //         {
    //             get_position = true;
    //             std::cout << "camera 获取数据失败" << std::endl;
    //         }
    //         else
    //         {
    //             get_position = false;
    //             break;
    //         }
    //     }
    //     std::cout << "RGB camera head_pose" << head_pose << std::endl;
    //     track_matrixdata = handEye_CalibrationX.inverse() * head_pose * translation_angle_z; // 转化坐标系(绕Z轴旋转)

    //     std::cout << "track_matrixdata " << track_matrixdata << std::endl;

    //     result_data = coor_generalfun.PoseMatrixToArray(track_matrixdata);
    //     eye_pos_temp[0] = result_data[3];
    //     eye_pos_temp[1] = result_data[7];
    //     eye_pos_temp[2] = result_data[11];
    //     std::cout << "目标位置数据:" << eye_pos_temp << std::endl;

    //     robot_pos_temp[0] = robot[3];
    //     robot_pos_temp[1] = robot[7];
    //     robot_pos_temp[2] = robot[11];
    //     std::cout << "Robot 初始位置数据:" << robot_pos_temp << std::endl;

    //     rob_eye_distance_temp = eye_pos_temp - robot_pos_temp;
    //     float rob_eye_disdata = rob_eye_distance_temp.norm();
    //     std::cout << "初始 X偏差：" << abs(rob_eye_distance_temp[0] * 1000) << "mm ; "
    //               << "初始 Y偏差：" << abs(rob_eye_distance_temp[1] * 1000) << "mm ; "
    //               << "初始 Z偏差：" << abs(rob_eye_distance_temp[2] * 1000) << "mm ; " << std::endl
    //               << "初始 distance:" << rob_eye_disdata * 1000 << "mm ; " << std::endl;

    //     return result_data;
    // }

    // 相机坐标系下的点转换到基坐标系
    //  param:
    //  输入：（输入）camera  相机数据（输入） 4x4 array
    //       （输入）robot   基坐标系机械臂数据 4x4 array
    //  输出：基坐标系机械臂数据
    //  注意：1. array 顺序依次是：（rx1 rx2 rx3 x ry1 ry2 ry3 y rz1 rz2 rz3 z）;
    //       2. x y z 单位 m；
    // ArrayPosData CoordinateTransform::ReadX_CameraToBase(ArrayPosData &camera) // 获取空间运动的目标点位
    // {
    //     PoseMatrix handEye_CalibrationX;

    //     // 结构光相机姿态旋转矩阵的调整
    //     // float angle_z = 1;
    //     // PoseMatrix translation_angle_x, translation_angle_y, translation_angle_z;
    //     // translation_angle_z << cos(Pi * angle_z), sin(Pi * angle_z), 0, 0, -sin(Pi * angle_z), cos(Pi * angle_z), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    //     // translation_angle_y << cos(Pi * angle_y), 0, -sin(Pi * angle_y), 0, 0, 1, 0, 0, sin(Pi * angle_y), 0, cos(Pi * angle_y), 0, 0, 0, 0, 1;
    //     // translation_angle_x << 1, 0, 0, 0, 0, cos(Pi * angle_x), sin(Pi * angle_x), 0, 0, -sin(Pi * angle_x), cos(Pi * angle_x), 0, 0, 0, 0, 1;

    //     handEye_CalibrationX << coor_generalfun.CalibrationEyetohandXRead();
    //     // std::cout << "handEye_CalibrationX" << handEye_CalibrationX << std::endl;
    //     PoseMatrix camera_matrix = coor_generalfun.ArrayToPoseMatrix(camera);
    //     PoseMatrix track_matrixdata = handEye_CalibrationX.inverse() * camera_matrix; // 转化坐标系(绕Z轴旋转)
    //     std::cout << "track_matrixdata " << track_matrixdata << std::endl;

    //     ArrayPosData result_data = coor_generalfun.PoseMatrixToArray(track_matrixdata);
    //     return result_data;
    // }

    // ArrayPosData CoordinateTransform::CameraToBase(ArrayPosData camera_basc_x, ArrayPosData camera) // 获取空间运动的目标点位
    // {
    //     PoseMatrix camera_matrix = coor_generalfun.ArrayToPoseMatrix(camera);
    //     PoseMatrix camera_basc_x_matrix = coor_generalfun.ArrayToPoseMatrix(camera_basc_x);
    //     PoseMatrix track_matrixdata = camera_basc_x_matrix.inverse() * camera_matrix; // 转化坐标系(绕Z轴旋转)

    //     ArrayPosData result_data = coor_generalfun.PoseMatrixToArray(track_matrixdata);
    //     return result_data;
    // }

} // namespace AIMOCoordinate
