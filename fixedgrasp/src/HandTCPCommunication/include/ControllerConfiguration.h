#ifndef LINKERHAND_CONTROLLER_CONFIGURATION_H
#define LINKERHAND_CONTROLLER_CONFIGURATION_H
#include "GeneralFunction.h"
#include "GraspTypes.h"
#include <map>
#include <limits>

namespace AIMOcommunicate {
struct ArmSettings {
    std::string ip = "192.168.1.18";
    int port = 8080;
    int timeout_ms = 5000;
    int speed_percent = 20;
};
struct ControllerConfiguration {
    ArmSettings arm;
    std::string target_pose_file, tools_tcp_file, hand_eye_calibration_file;
    Eigen::Matrix4d hand_eye_calibration = Eigen::Matrix4d::Constant(std::numeric_limits<double>::quiet_NaN());
    std::vector<geometry_msgs::Pose> joint_poses;
    std::vector<OXYZ_Pose> tool_offsets;
    std::vector<std::string> tool_frames;
    std::map<std::string, Grap_Obj> fixed_targets;
    std::vector<std::pair<float, float>> joint_limits_deg, workspace_bounds_m;
};
// On failure output is unchanged; error identifies the invalid field or file.
bool loadControllerConfiguration(const ros::NodeHandle &node, const std::string &package_path,
                                 ControllerConfiguration &output, std::string &error);
bool validJointTarget(const geometry_msgs::Pose &pose, const ControllerConfiguration &config);
bool validCartesianTarget(const OXYZ_Pose &pose, const ControllerConfiguration &config);
} // namespace AIMOcommunicate
#endif
