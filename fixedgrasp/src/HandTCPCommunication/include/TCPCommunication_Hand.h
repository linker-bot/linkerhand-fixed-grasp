/*
  通讯处理类：Json处理数据 机械手
*/

#ifndef HAND_TCP_COMMUNICATION_H
#define HAND_TCP_COMMUNICATION_H

#include "GeneralFunction.h"
#include "TcpStream.h"
#include "ControllerConfiguration.h"
#include "VisualPointCloud.h"
#include <json/json.h>
#include "rm_service.h"
#include <std_msgs/Float32MultiArray.h>
#include "HandEyeCalibration.h"
#include "CoordinateTransform.h"
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/point_cloud2_iterator.h>
#include <map>
#include <sensor_msgs/PointField.h>
#include <iostream>
#include <vector>
#include <cstring>
#include <cmath>
#include <HandTCPCommunication/OrderDetectResult.h>

struct Hand_SendStruct
{
    uint8_t PowerOn;
    uint8_t Work_Mode;
    uint8_t Control_Mode;
    uint8_t Move_Control;

    uint8_t Hand_Freedom; // 机械手自由度
    uint8_t Retain1;
    uint8_t Retain2;
    uint8_t Retain3;

    HumanHandParam Set_Point_RightHand;
    HumanHandParam Set_Speed_RightHand;
    HumanHandParam Set_Point_LeftHand;
    HumanHandParam Set_Speed_LeftHand;
    HandForceParam Set_Force_RightHand;
    HandForceParam Set_Force_LeftHand;
};
struct Hand_RecvStruct
{
    uint8_t PowerOn_State;
    uint8_t Work_State;
    uint8_t Control_State;
    uint8_t Retain;

    uint8_t RightHand_MoveState;
    uint8_t LeftHand_MoveState;
    uint16_t Error_State;

    HumanHandParam Get_CurrentPoint_RightHand;
    HumanHandParam Get_CurrentPoint_LeftHand;

    HandForceParam Get_CurrentForce_RightHand;
    HandForceParam Get_CurrentForce_LeftHand;

    HumanHandParam Get_CurrentPoint_RightGlove;
    HumanHandParam Get_CurrentPoint_LeftGlove;
};

struct TCP_Hand_SendData
{
    uint8_t PowerOn;
    uint8_t Work_Mode;
    uint8_t Control_Mode;
    uint8_t Move_Control;

    uint8_t Hand_Freedom; // 机械手自由度
    uint8_t Retain1;
    uint8_t Retain2;
    uint8_t Retain3;

    HumanHandParam Set_Point_RightHand;
    HumanHandParam Set_Speed_RightHand;
    HumanHandParam Set_Point_LeftHand;
    HumanHandParam Set_Speed_LeftHand;
    HandForceParam Set_Force_RightHand;
    HandForceParam Set_Force_LeftHand;
};
struct TCP_Hand_RecvData
{
    uint8_t PowerOn_State;
    std::string Work_State;
    std::string Control_State;

    std::string RightHand_MoveState;
    std::string LeftHand_MoveState;
    uint16_t Error_State;

    HumanHandParam Get_CurrentPoint_RightHand;
    HumanHandParam Get_CurrentPoint_LeftHand;

    HandForceParam Get_CurrentForce_RightHand;
    HandForceParam Get_CurrentForce_LeftHand;

    HumanHandParam Get_CurrentPoint_RightGlove;
    HumanHandParam Get_CurrentPoint_LeftGlove;
};

namespace AIMOcommunicate
{
    class TCPCommunication_Hand
    {
    public:
        // Tests can skip transport setup and override the arm commands.
        explicit TCPCommunication_Hand(bool initialize_hardware = true);
        virtual ~TCPCommunication_Hand();
        void Hand_CrawlPoses(int tool_id);
        bool Hand_TCPCommunication_connect(const ros::NodeHandle &ros_TCP_nh);

        bool initTargetPoses();
        void GetVisualData_subCallback(const sensor_msgs::PointCloud2ConstPtr &GridData);
        void orderDetectCallback(const HandTCPCommunication::OrderDetectResult::ConstPtr& msg);
        int getFieldOffset(const std::vector<sensor_msgs::PointField> &fields, const std::string &field_name);
        void motionResultCallback(const std_msgs::Bool::ConstPtr &msg);
        void RightHandCtrl_subCallback(const sensor_msgs::JointState::ConstPtr &msg);
        void LeftHandCtrl_subCallback(const sensor_msgs::JointState::ConstPtr &msg);
        void RightHand_pubFunction(void);
        void LeftHand_pubFunction(void);
        void RightGlove_pubFunction(void);
        void LeftGlove_pubFunction(void);
        void grep_init();
        void grep_run();
        void show_left();
        void show_right();

    private:
        void Hand_InitDataSend(void);
        void Hand_TCPCommunication_stop(void);
        void Hand_TCPCommunication_Json_Launch_Deal();

    public:
        TCP_Hand_RecvData HandClient_Recv{};
        TCP_Hand_SendData HandClient_Send{};

    private: // 通讯连接变量
        TcpStream hand_stream;
        int m_server_hand_port = 20008;
        int hand_connect_timeout_ms = 250;
        int hand_io_timeout_ms = 100;
        int hand_reconnect_delay_ms = 1000;
        TcpStream::Deadline next_hand_connect{};
        bool hand_feedback_valid = false;
        ControllerConfiguration arm_configuration;
        bool configuration_valid = false;
        int show_step = 0;
        int show_count = 0;
        int show_count_obj = 0;
        std::vector<Grap_Obj> obj_list;
        std::map<std::string, Grap_Obj> fixed_grasp_targets;
        Grap_Obj pass_obj{};

    private:
        general::GeneralFunction hand_TCPcommunicate_generalfun;

        ros::NodeHandle m_hand_tcp_nh;
        std::string m_server_hand_ip;
        bool handEnable = false;
        int Hand_Freedom;
        int JointVelocity;
        int JointEffort;

        ros::Subscriber sub_right_hand;
        ros::Subscriber sub_left_hand;
        ros::Publisher pub_right_hand;
        ros::Publisher pub_left_hand;
        ros::Publisher pub_right_glove;
        ros::Publisher pub_left_glove;

        ros::WallTimer m_hand_timer;
        bool m_hand_isConnected = false;

        /////////////
        ros::Subscriber result_sub;
        ros::Subscriber detection_sub;
        void connectVisual(ros::NodeHandle node);
        friend struct GraspTestAccess;
        friend struct TcpTestAccess;
        friend struct ConfigurationTestAccess;
        friend struct VisualCloudTestAccess;
        ros::Publisher pose_pub;
        ros::Publisher arm_reached_pub;
        ros::Publisher get_Visual_pub;
        std::vector<geometry_msgs::Pose> target_poses;
        std::vector<OXYZ_Pose> Tools_Tcp;
        std::vector<ObjectData> object_data;
        OXYZ_Pose movej_p_pose;
        OXYZ_Pose Move_pose{};
        size_t current_index;
        Json::Reader reader; // JSON 解析器
        Json::Value value;   // 存储解析结果
        bool if_complite = false;
        bool if_send = false;
        std_msgs::String msg;
        PoseMatrix Calibration_Result_EyeInHand = PoseMatrix::Constant(std::numeric_limits<double>::quiet_NaN());
        AIMOCoordinate::CoordinateTransform coorTrans;
        std::vector<HumanHandParam> hand_poses;
        std_msgs::String Visual_msg;

    private:
        void createTimer(int seconds)
        {
            signal(SIGALRM, [](int) {});
            alarm(seconds);
        }
        void Hand_TCPSendRecvMessage(const ros::WallTimerEvent &event);

        // 结构体通讯
        int Hand_DataDeal_Struct();
        void Hand_SendStructDataDeal(TCP_Hand_SendData handData, Hand_SendStruct &sendData);
        void Hand_RecvStructDataDeal(Hand_RecvStruct recvData, TCP_Hand_RecvData &handData);
        Hand_SendStruct TCP_SendStruct_Data{};
        Hand_RecvStruct TCP_RecvStruct_Data{};

        std::string Hand_RecvDataStr, Hand_SendDataStr;
        HumanHandParam hand_angle_left_test;

        // 机械臂相关
    public:
        RM_Service *m_pApi = nullptr;
        SOCKHANDLE m_sockhand = -1;
        std::vector<std::string> tools_farmeName;

    protected:
        virtual int initializeArmApi();
        virtual SOCKHANDLE connectArmSocket(const std::string &ip, int port, int timeout_ms);
        virtual int armSocketState(SOCKHANDLE socket);
        virtual int moveArmJoints(const float *joints, int speed_percent);
        virtual int moveArmPose(const Pose &pose, int speed_percent);
    public:
        bool InitArm();
        void Realseaemsocket();
        virtual bool armmove_j(geometry_msgs::Pose);
        virtual bool armmovej_p(OXYZ_Pose move_pose);
        virtual bool Change_Tool_Frame(const int id);
        // 手眼标定
        PoseMatrix GetHandEyeCalibrationResultX_EyeInHand();
        float Getry_pose(float vec_x, float vec_y, float vec_z);
    };
} // namespace AIMOcommunicate
#endif
