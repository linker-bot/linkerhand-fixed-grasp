
#ifndef HAND_GENERAL_FUNCTION
#define HAND_GENERAL_FUNCTION

#include <eigen3/Eigen/Dense>
#include <stdio.h>
#include <fcntl.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <string.h>
#include <unistd.h>
#include <array>
#include <sys/time.h>
#include <time.h>
#include <vector>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int32MultiArray.h>

#include "stdint.h"
#include <cmath>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <json/json.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <boost/asio.hpp>
#include <sys/socket.h>
#include <sys/select.h>
#include <arpa/inet.h>

#include <cstring>
#include <termios.h>
#include <serial/serial.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>

#include "sensor_msgs/JointState.h"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

const float Pi = 3.141596f;
const float DegToRad = Pi / 180.0;
const float RadToDeg = 180.0 / Pi;
const float HandPosToRad = Pi / 255;

const uint8_t CheckNum = 6;
const uint8_t CalibrationSize = 12;

using namespace sensor_msgs;
using namespace std;

typedef std::array<std::array<double, 16>, CalibrationSize> TwoDimensionArray;
typedef std::array<float, 16> ArrayPosData;  // 笛卡尔空间坐标矩阵
typedef std::array<float, 6> ArrayPointData; // 关节角度
typedef std::array<float, 6> ArrayRobotData; // X Y Z RX RY RZ

typedef Eigen::Matrix<double, 4, 4> PoseMatrix;

struct HumanHandParam
{
    float Joint1; // 拇指1 弯曲
    float Joint2; // 拇指2 横摆
    float Joint3; // 食指
    float Joint4; // 中指
    float Joint5; // 无名指
    float Joint6; // 小拇指
    float Joint7; // 食指和中指
    float Joint8; // 中指和无名指
    float Joint9; // 无名指和小拇指

    float Joint1_rotation; // 拇指旋转 —— 10自由度多余部分

    float Joint3_raw; // 中指左右运动 —— 16自由度多余部分
    float Joint1_tip; // 拇指末端弯曲度
    float Joint2_tip; // 食指末端弯曲度
    float Joint3_tip; // 中指末端弯曲度
    float Joint4_tip; // 无名指末端弯曲度
    float Joint5_tip; // 小拇指末端弯曲度

    // 默认构造函数
    HumanHandParam()
        : Joint1(250), Joint2(250), Joint3(250), Joint4(250), Joint5(250), Joint6(250),
          Joint7(250), Joint8(250), Joint9(250), Joint1_rotation(250), Joint3_raw(250),
          Joint1_tip(250), Joint2_tip(250), Joint3_tip(250), Joint4_tip(250), Joint5_tip(250) {}
};
struct HandForceParam
{
    float Joint1; // 拇指
    float Joint2; // 食指
    float Joint3; // 中指
    float Joint4; // 无名指
    float Joint5; // 小拇指
};

struct ForceSensorData
{
    int SensorData1;
    int SensorData2;
    int SensorData3;
    int SensorData4;
    int SensorData5;
    int SensorData6;
};
struct OXYZ_Pose
{
    float x;
    float y;
    float z;
    float rx;
    float ry;
    float rz;
};

namespace general
{
    class GeneralFunction
    {
    public:
        GeneralFunction() {};

        bool TCP_Connect_error_deal(const int connect_sockfd, const int connect_status)
        {
            if (connect_status < 0)
            {
                if (errno == EINPROGRESS || errno == EALREADY)
                {
                    fd_set write_fds; // 等待套接字变为可写
                    FD_ZERO(&write_fds);
                    FD_SET(connect_sockfd, &write_fds);

                    timeval timeout;
                    timeout.tv_sec = 5; // 设置超时时间为5秒
                    timeout.tv_usec = 0;

                    int select_status = select(connect_sockfd + 1, NULL, &write_fds, NULL, &timeout);
                    if (select_status < 0)
                    {
                        std::cerr << "Error in select" << std::endl;
                        return false;
                    }
                    else if (select_status == 0)
                    {
                        std::cerr << "Connection timed out" << std::endl;
                        return false;
                    }
                    else
                    {
                        int error_code; // 检查套接字上的错误状态
                        socklen_t error_code_size = sizeof(error_code);
                        getsockopt(connect_sockfd, SOL_SOCKET, SO_ERROR, &error_code, &error_code_size);
                        if (error_code == 0)
                        {
                            std::cout << "Connection successful" << std::endl; // 连接成功，可以进行数据发送和接收
                        }
                        else
                        {
                            std::cerr << "Connection failed: " << strerror(error_code) << std::endl;
                            return false;
                        }
                    }
                }
                else
                {
                    if ((errno != EINPROGRESS) && (errno != EALREADY))
                        std::cerr << "EINPROGRESS EALREADY Failed to connect" << std::endl;
                    false;
                }
            }
            else
            {
                std::cout << "Connection successful" << std::endl;
            }
            return true;
        };
        int TCP_Blocking_deal(const int connect_sockfd, unsigned char *send_buf, const uint16_t send_len, unsigned char *recv_buf, const uint16_t recv_len)
        {
            if ((send(connect_sockfd, send_buf, send_len, 0)) < 0)
            {
                ROS_WARN("TCP send TCP_Blocking_deal error!");
                return -1;
            }
            if ((recv(connect_sockfd, recv_buf, recv_len, 0)) < 0)
            {
                ROS_WARN("TCP recv TCP_Blocking_deal error!");
                return -1;
            }
            return 0;
        };
        int TCP_nonBlocking_deal(const int connect_sockfd, unsigned char *send_buf, const uint16_t send_len, unsigned char *recv_buf, const uint16_t recv_len)
        {
            unsigned char recv_buf_temp[recv_len];
            if ((send(connect_sockfd, send_buf, send_len, 0)) < 0)
            {
                // ROS_WARN("TCP send TCP_nonBlocking_deal error!");
                return -1;
            }
            int byte_recv = recv(connect_sockfd, recv_buf_temp, recv_len, 0);
            if ((byte_recv) < 0)
            {
                if (errno == EWOULDBLOCK) // 没有可用数据
                {
                    // ROS_ERROR("receive data is EWOULDBLOCK");
                    return -1;
                }
                else if (errno == EAGAIN) // 没有可用数据
                {
                    // ROS_ERROR("receive data is EAGAIN");
                    return -1;
                }
                else
                {
                    ROS_ERROR("Failed to receive data");
                    return -1;
                }
            }
            else if (byte_recv == 0) // 连接关闭
            {
                ROS_ERROR("connect is close");
                return -1;
            }
            for (int i = 0; i < recv_len; i++)
            {
                recv_buf[i] = recv_buf_temp[i];
            }
            return 0;
        };
        Eigen::Vector3d Rotation2RPY(Eigen::Matrix3d &aux_rotation)
        {
            Eigen::Vector3d eulerAngle;
            if (abs(aux_rotation(2, 0) - 1.0) < 1.0e-15)
            {
                eulerAngle(0) = 0.0;
                eulerAngle(1) = -M_PI / 2.0;
                eulerAngle(2) = atan2(-aux_rotation(0, 1), -aux_rotation(0, 2));
            }
            else if (abs(abs(aux_rotation(2, 0) + 1.0) < 1.0e-15))
            {
                eulerAngle(0) = 0.0;
                eulerAngle(1) = M_PI / 2.0;
                eulerAngle(2) = atan2(-aux_rotation(0, 1), aux_rotation(0, 2));
            }
            else
            {
                eulerAngle(0) = atan2(aux_rotation(2, 1), aux_rotation(2, 2));
                eulerAngle(2) = atan2(aux_rotation(1, 0), aux_rotation(0, 0));
                if (abs(cos(eulerAngle(2))) > abs(sin(eulerAngle(2))))
                {
                    eulerAngle(1) = atan2(-aux_rotation(2, 0), aux_rotation(0, 0) / cos(eulerAngle(2)));
                }
                else
                {
                    eulerAngle(1) = atan2(-aux_rotation(2, 0), aux_rotation(1, 0) / sin(eulerAngle(2)));
                }
            }

            // eulerAngle = aux_rotation.eulerAngles(0, 1, 2);         //row , pitch , yaw
            // eulerAngle(0) = eulerAngle(0) * 180 / Pi;
            // eulerAngle(1) = eulerAngle(1) * 180 / Pi;
            // eulerAngle(2) = eulerAngle(2) * 180 / Pi;
            return eulerAngle;
        }
        Eigen::Matrix3d RPY2Rotation(float &roll, float &pitch, float &yaw) // roll , pitch , yaw
        {
            /// send EulerAngles and transform into RotationMatrix

            Eigen::Matrix3d R, Rx, Ry, Rz;

            Rx << 1.0, 0.0, 0.0, 0.0, cos(roll), -sin(roll), 0.0, sin(roll), cos(roll);

            Ry << cos(pitch), 0.0, sin(pitch), 0.0, 1.0, 0.0, -sin(pitch), 0.0, cos(pitch);

            Rz << cos(yaw), -sin(yaw), 0.0, sin(yaw), cos(yaw), 0.0, 0.0, 0.0, 1.0;

            R = Rz * Ry * Rx;

            return R;
        }
        ArrayPosData PoseMatrixToArray(PoseMatrix &matrix_data)
        {
            ArrayPosData matrix_to_array_data;
            matrix_to_array_data = {{(float)matrix_data(0, 0), (float)matrix_data(0, 1), (float)matrix_data(0, 2), (float)matrix_data(0, 3), (float)matrix_data(1, 0), (float)matrix_data(1, 1),
                                     (float)matrix_data(1, 2), (float)matrix_data(1, 3), (float)matrix_data(2, 0), (float)matrix_data(2, 1), (float)matrix_data(2, 2), (float)matrix_data(2, 3),
                                     (float)matrix_data(3, 0), (float)matrix_data(3, 1), (float)matrix_data(3, 2), (float)matrix_data(3, 3)}};
            return matrix_to_array_data;
        }
        PoseMatrix floatToPoseMatrix(const float float_data[16])
        {
            PoseMatrix float_to_matrix_data;
            float_to_matrix_data << (double)float_data[0], (double)float_data[1], (double)float_data[2], (double)float_data[3], (double)float_data[4], (double)float_data[5], (double)float_data[6],
                (double)float_data[7], (double)float_data[8], (double)float_data[9], (double)float_data[10], (double)float_data[11], (double)float_data[12], (double)float_data[13], (double)float_data[14],
                (double)float_data[15];
            return float_to_matrix_data;
        }
        PoseMatrix ArrayToPoseMatrix(const ArrayPosData &array_data)
        {
            PoseMatrix array_to_matrix_data;
            array_to_matrix_data << array_data[0], array_data[1], array_data[2], array_data[3], array_data[4], array_data[5], array_data[6], array_data[7], array_data[8], array_data[9], array_data[10],
                array_data[11], array_data[12], array_data[13], array_data[14], array_data[15];
            return array_to_matrix_data;
        }
        PoseMatrix CalibrationEyetohandXRead_path(std::string path)
        {
            float eyetohand_x[16];
            std::string readline_temp;

            std::string resultX_path = ros::package::getPath("HandTCPCommunication");
            resultX_path = resultX_path + path;
            std::ifstream calibration_x_read(resultX_path);
            uint8_t read_x = 0;
            while (getline(calibration_x_read, readline_temp))
            {
                eyetohand_x[read_x] = atof(readline_temp.c_str());
                read_x++;
            }
            return floatToPoseMatrix(eyetohand_x);
        }
    };
} // namespace general

#endif
