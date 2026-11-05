#include "TCPCommunication_Hand.h"
#include "LoopbackServer.h"
#include <gtest/gtest.h>
#include <fstream>
#include <limits>
#include <cstdio>

namespace AIMOcommunicate {
struct ConfigurationTestAccess {
    static void enableHand(TCPCommunication_Hand &h) { h.handEnable = true; h.m_server_hand_ip = "127.0.0.1"; }
};
class ConfigurationTest : public testing::Test {
protected:
    ros::NodeHandle node{"~"};
    std::string directory, pose_path, tool_path, calibration_path, error;
    Json::Value poses, tools;
    ControllerConfiguration config;
    const std::vector<std::string> parameters = {"arm_ip", "arm_port", "arm_timeout_ms", "arm_speed_percent",
        "target_pose_file", "tools_tcp_file", "fixed_grasp_targets", "joint_limits_deg", "workspace_bounds_m", "hand_port", "hand_eye_calibration_file"};
    void SetUp() override {
        for (const auto &name : parameters) node.deleteParam(name);
        char pattern[] = "/tmp/linkerhand-configuration-XXXXXX";
        char *path = ::mkdtemp(pattern); ASSERT_NE(nullptr, path);
        directory = path; calibration_path = directory + "/calibration.txt"; pose_path = directory + "/poses.json"; tool_path = directory + "/tools.json";
        const auto package = ros::package::getPath("HandTCPCommunication");
        Json::Reader reader;
        std::ifstream pose_file(package + "/target_pose.json"), tool_file(package + "/tools_Tcp.json");
        ASSERT_TRUE(reader.parse(pose_file, poses)); ASSERT_TRUE(reader.parse(tool_file, tools));
        write(); node.setParam("target_pose_file", pose_path); node.setParam("tools_tcp_file", tool_path);
    }
    void TearDown() override {
        for (const auto &name : parameters) node.deleteParam(name);
        ::unlink(calibration_path.c_str());
        ::unlink(pose_path.c_str()); ::unlink(tool_path.c_str()); ::rmdir(directory.c_str());
    }
    void write() {
        std::ofstream(pose_path) << poses;
        std::ofstream(tool_path) << tools;
    }
    bool load() {
        return loadControllerConfiguration(node, ros::package::getPath("HandTCPCommunication"), config, error);
    }
    XmlRpc::XmlRpcValue targets(int count = 1) {
        XmlRpc::XmlRpcValue result; result.setSize(count);
        for (int i = 0; i < count; ++i) {
            result[i]["label"] = std::string("object") + std::to_string(i);
            result[i]["id"] = i + 1; result[i]["pose"].setSize(6);
            for (int j = 0; j < 6; ++j) result[i]["pose"][j] = j < 3 ? .1 : 0.0;
        }
        return result;
    }
    XmlRpc::XmlRpcValue bounds(int count, double min, double max) {
        XmlRpc::XmlRpcValue result; result.setSize(count);
        for (int i = 0; i < count; ++i) {
            result[i].setSize(2); result[i][0] = min; result[i][1] = max;
        }
        return result;
    }
};
TEST_F(ConfigurationTest, LoadsDefaultsAndOrdersToolsByPresetId) {
    ASSERT_TRUE(load()) << error;
    EXPECT_EQ("192.168.1.18", config.arm.ip); EXPECT_EQ(8080, config.arm.port);
    EXPECT_EQ(5000, config.arm.timeout_ms); EXPECT_EQ(20, config.arm.speed_percent);
    ASSERT_EQ(5u, config.joint_poses.size()); ASSERT_EQ(8u, config.tool_offsets.size());
    EXPECT_FLOAT_EQ(tools["Crawl_tool"]["Silk"]["Tcp"][0].asFloat(), config.tool_offsets[0].x);
    EXPECT_FLOAT_EQ(tools["Crawl_tool"]["bottle"]["Tcp"][0].asFloat(), config.tool_offsets[6].x);
    EXPECT_FLOAT_EQ(tools["Crawl_tool"]["Bolt"]["Tcp"][0].asFloat(), config.tool_offsets[7].x);
}
TEST_F(ConfigurationTest, AcceptsEndpointAndSpeedOverrides) {
    node.setParam("arm_ip", "127.0.0.2"); node.setParam("arm_port", 18080);
    node.setParam("arm_timeout_ms", 1234); node.setParam("arm_speed_percent", 35);
    ASSERT_TRUE(load()) << error;
    EXPECT_EQ("127.0.0.2", config.arm.ip); EXPECT_EQ(18080, config.arm.port);
    EXPECT_EQ(1234, config.arm.timeout_ms); EXPECT_EQ(35, config.arm.speed_percent);
    EXPECT_EQ(pose_path, config.target_pose_file); EXPECT_EQ(tool_path, config.tools_tcp_file);
}
TEST_F(ConfigurationTest, RejectsMalformedAddressAndIncorrectParameterTypes) {
    for (const std::string &ip : {"", " ", "arm.local", "192.168.1.999", "127.0.0.1:8080"}) {
        node.setParam("arm_ip", ip); EXPECT_FALSE(load()) << ip;
    }
    node.setParam("arm_ip", 1); EXPECT_FALSE(load()); node.deleteParam("arm_ip");
    for (const auto &name : {"arm_port", "arm_timeout_ms", "arm_speed_percent"}) {
        node.setParam(name, "20"); EXPECT_FALSE(load()) << name;
        node.setParam(name, 20.0); EXPECT_FALSE(load()) << name;
        node.setParam(name, true); EXPECT_FALSE(load()) << name;
        node.deleteParam(name);
    }
}
TEST_F(ConfigurationTest, RejectsOutOfRangeEndpointTimeoutAndSpeed) {
    for (const auto &name : {"arm_port", "arm_timeout_ms", "arm_speed_percent"}) {
        for (int value : {-1, 0, 65536}) { node.setParam(name, value); EXPECT_FALSE(load()) << name; }
        node.deleteParam(name);
    }
    node.setParam("arm_timeout_ms", 60001); EXPECT_FALSE(load()); node.deleteParam("arm_timeout_ms");
    node.setParam("arm_speed_percent", 101); EXPECT_FALSE(load());
}
TEST_F(ConfigurationTest, RejectsMissingMalformedAndRelativeFiles) {
    node.setParam("target_pose_file", directory + "/missing.json"); EXPECT_FALSE(load());
    node.setParam("target_pose_file", "poses.json"); EXPECT_FALSE(load());
    node.setParam("target_pose_file", pose_path); std::ofstream(pose_path) << "{broken"; EXPECT_FALSE(load());
    write(); node.setParam("tools_tcp_file", directory + "/missing.json"); EXPECT_FALSE(load());
    node.setParam("tools_tcp_file", tool_path); std::ofstream(tool_path) << "[]"; EXPECT_FALSE(load());
}
TEST_F(ConfigurationTest, RejectsTrailingJsonAndDuplicateKeys) {
    for (const auto &suffix : {" trailing", " garbage}", " null", " {}"}) {
        std::ofstream(pose_path) << poses << suffix; EXPECT_FALSE(load()) << suffix;
    }
    poses["note"] = "Braces { } and quotes \" inside strings"; write(); ASSERT_TRUE(load()) << error;
    std::ofstream(pose_path) << "{\"target_poses\":[],\"target_poses\":" << poses["target_poses"] << "}";
    EXPECT_FALSE(load());
}
TEST_F(ConfigurationTest, RequiresExactlyFiveJointTargets) {
    const auto original = poses;
    for (int count : {0, 4, 6}) {
        poses = original; poses["target_poses"].resize(count); write(); EXPECT_FALSE(load()) << count;
    }
    poses["target_poses"] = "invalid"; write(); EXPECT_FALSE(load());
}
TEST_F(ConfigurationTest, RequiresEveryJointComponentToBeNumeric) {
    const auto original = poses;
    for (const auto &group : {"position", "orientation"}) {
        for (const auto &axis : {"x", "y", "z"}) {
            poses = original; poses["target_poses"][0][group].removeMember(axis); write(); EXPECT_FALSE(load());
            for (const Json::Value &bad : {Json::Value("0"), Json::Value(true), Json::Value(), Json::Value(Json::arrayValue)}) {
                poses = original; poses["target_poses"][0][group][axis] = bad; write(); EXPECT_FALSE(load());
            }
        }
    }
}
TEST_F(ConfigurationTest, RejectsOverflowAndInvalidOptionalLegacyW) {
    poses["target_poses"][0]["position"]["x"] = std::numeric_limits<double>::max();
    write(); EXPECT_FALSE(load());
    poses["target_poses"][0]["position"]["x"] = 1e-300; write(); EXPECT_FALSE(load());
    poses["target_poses"][0]["position"]["x"] = 0;
    poses["target_poses"][0]["orientation"]["w"] = "invalid"; write(); EXPECT_FALSE(load());
    poses["target_poses"][0]["orientation"].removeMember("w"); write(); EXPECT_TRUE(load()) << error;
}
TEST_F(ConfigurationTest, RejectsMissingOrMalformedToolOffsets) {
    const auto original = tools;
    tools["Crawl_tool"].removeMember("Bolt"); write(); EXPECT_FALSE(load());
    for (int count : {0, 3, 7}) {
        tools = original; tools["Crawl_tool"]["Silk"]["Tcp"].resize(count); write(); EXPECT_FALSE(load());
    }
    for (int i = 0; i < 6; ++i) {
        tools = original; tools["Crawl_tool"]["Silk"]["Tcp"][i] = "0"; write(); EXPECT_FALSE(load());
    }
    tools = original; tools["Crawl_tool"]["Silk"]["Tcp"][5] = std::numeric_limits<double>::max();
    write(); EXPECT_FALSE(load());
}
TEST_F(ConfigurationTest, DoesNotCommitPartialConfigurationOnFailure) {
    ASSERT_TRUE(load()); const auto previous_ip = config.arm.ip;
    const auto previous_joint = config.joint_poses[0].position.x;
    node.setParam("arm_ip", "127.0.0.3"); poses["target_poses"][0]["position"]["x"] = 12;
    tools["Crawl_tool"].removeMember("Bolt"); write();
    EXPECT_FALSE(load()); EXPECT_EQ(previous_ip, config.arm.ip);
    EXPECT_EQ(previous_joint, config.joint_poses[0].position.x); EXPECT_FALSE(error.empty());
}
TEST_F(ConfigurationTest, ValidatesFixedTargetLabelsIdsAndNumericTypes) {
    auto value = targets(2); value[1]["id"] = 1; node.setParam("fixed_grasp_targets", value); EXPECT_FALSE(load());
    value = targets(2); value[1]["label"] = "object0"; node.setParam("fixed_grasp_targets", value); EXPECT_FALSE(load());
    value = targets(); value[0]["label"] = " "; node.setParam("fixed_grasp_targets", value); EXPECT_FALSE(load());
    value = targets(); value[0]["pose"][0] = "0"; node.setParam("fixed_grasp_targets", value); EXPECT_FALSE(load());
    value = targets(); value[0]["pose"][0] = std::numeric_limits<double>::max();
    node.setParam("fixed_grasp_targets", value); EXPECT_FALSE(load());
    node.setParam("fixed_grasp_targets", targets()); EXPECT_TRUE(load()) << error;
}
TEST_F(ConfigurationTest, RejectsNonFiniteFixedTargetAndLimitValues) {
    auto value = targets(); value[0]["pose"][0] = std::numeric_limits<double>::quiet_NaN();
    node.setParam("fixed_grasp_targets", value); EXPECT_FALSE(load()); node.deleteParam("fixed_grasp_targets");
    value = bounds(6, -180, 180); value[0][1] = std::numeric_limits<double>::infinity();
    node.setParam("joint_limits_deg", value); EXPECT_FALSE(load());
}
TEST_F(ConfigurationTest, ValidatesJointLimitsAndRejectsTargetsOutsideThem) {
    node.setParam("joint_limits_deg", bounds(6, -180, 180)); ASSERT_TRUE(load()) << error;
    node.setParam("joint_limits_deg", bounds(5, -180, 180)); EXPECT_FALSE(load());
    node.setParam("joint_limits_deg", bounds(6, 180, -180)); EXPECT_FALSE(load());
    node.setParam("joint_limits_deg", bounds(6, -1, 1)); EXPECT_FALSE(load());
}
TEST_F(ConfigurationTest, ValidatesWorkspaceBoundsAndFixedTargets) {
    node.setParam("fixed_grasp_targets", targets()); node.setParam("workspace_bounds_m", bounds(3, -1, 1));
    ASSERT_TRUE(load()) << error;
    node.setParam("workspace_bounds_m", bounds(3, -.01, .01)); EXPECT_FALSE(load());
    node.setParam("workspace_bounds_m", bounds(2, -1, 1)); EXPECT_FALSE(load());
    node.setParam("workspace_bounds_m", bounds(3, 1, -1)); EXPECT_FALSE(load());
}

class FakeArm : public TCPCommunication_Hand {
public:
    FakeArm() : TCPCommunication_Hand(false) {}
    int initializations = 0, connects = 0, checks = 0, joint_moves = 0, pose_moves = 0, speed = 0;
    int api_result = 0, state_result = 0;
    SOCKHANDLE socket_result = 7;
    std::string ip;
    int port = 0, timeout = 0;
    Pose last_pose{};
protected:
    int initializeArmApi() override { ++initializations; return api_result; }
    SOCKHANDLE connectArmSocket(const std::string &address, int value, int wait) override {
        ++connects; ip = address; port = value; timeout = wait; return socket_result;
    }
    int armSocketState(SOCKHANDLE) override { ++checks; return state_result; }
    int moveArmJoints(const float *, int value) override { ++joint_moves; speed = value; return 0; }
    int moveArmPose(const Pose &pose, int value) override { ++pose_moves; speed = value; last_pose = pose; return 0; }
};
TEST_F(ConfigurationTest, ForwardsConfiguredEndpointAndSpeedToArmBackend) {
    node.setParam("arm_ip", "127.0.0.4"); node.setParam("arm_port", 18081);
    node.setParam("arm_timeout_ms", 2345); node.setParam("arm_speed_percent", 37);
    FakeArm arm; ASSERT_TRUE(arm.InitArm());
    EXPECT_EQ("127.0.0.4", arm.ip); EXPECT_EQ(18081, arm.port); EXPECT_EQ(2345, arm.timeout);
    geometry_msgs::Pose joints; ASSERT_TRUE(arm.armmove_j(joints)); EXPECT_EQ(37, arm.speed);
    OXYZ_Pose pose{.1f, .2f, .3f, 0, 0, 0};
    ASSERT_TRUE(arm.armmovej_p(pose)); EXPECT_EQ(37, arm.speed);
    EXPECT_FLOAT_EQ(1, arm.last_pose.quaternion.w); EXPECT_FLOAT_EQ(0, arm.last_pose.quaternion.x);
    pose.rx = 1.57079632679f; ASSERT_TRUE(arm.armmovej_p(pose));
    EXPECT_NEAR(std::sqrt(.5), arm.last_pose.quaternion.x, 1e-6);
    EXPECT_NEAR(std::sqrt(.5), arm.last_pose.quaternion.w, 1e-6);
}
TEST_F(ConfigurationTest, InvalidConfigurationPreventsArmInitializationAndCommands) {
    FakeArm arm; node.setParam("arm_port", 0);
    EXPECT_FALSE(arm.InitArm()); EXPECT_EQ(0, arm.initializations); EXPECT_EQ(0, arm.connects);
    EXPECT_FALSE(arm.armmove_j(geometry_msgs::Pose())); EXPECT_FALSE(arm.armmovej_p(OXYZ_Pose{}));
    EXPECT_EQ(0, arm.checks); EXPECT_EQ(0, arm.joint_moves); EXPECT_EQ(0, arm.pose_moves);
}
TEST_F(ConfigurationTest, InvalidPoseFilePreventsHandConnectionAsWellAsArmInitialization) {
    test_tcp::Server server;
    FakeArm arm; ConfigurationTestAccess::enableHand(arm); node.setParam("hand_port", server.port);
    poses["target_poses"][0]["position"].removeMember("x"); write();
    EXPECT_FALSE(arm.Hand_TCPCommunication_connect(ros::NodeHandle()));
    pollfd listener{server.socket.fd, POLLIN, 0};
    EXPECT_EQ(0, ::poll(&listener, 1, 20));
    EXPECT_FALSE(arm.InitArm()); EXPECT_EQ(0, arm.initializations); EXPECT_EQ(0, arm.connects);
}
TEST_F(ConfigurationTest, FailedReloadDisablesPreviouslyLoadedMotionTargets) {
    FakeArm arm; ASSERT_TRUE(arm.InitArm());
    tools["Crawl_tool"].removeMember("Bolt"); write(); EXPECT_FALSE(arm.initTargetPoses());
    const int checks = arm.checks;
    EXPECT_FALSE(arm.armmove_j(geometry_msgs::Pose())); EXPECT_FALSE(arm.armmovej_p(OXYZ_Pose{}));
    EXPECT_EQ(checks, arm.checks);
}
TEST_F(ConfigurationTest, RuntimeValidationRejectsNonFiniteAndOutOfBoundsMotion) {
    node.setParam("joint_limits_deg", bounds(6, -180, 180));
    node.setParam("workspace_bounds_m", bounds(3, -1, 1));
    FakeArm arm; ASSERT_TRUE(arm.InitArm()); const int checks = arm.checks;
    geometry_msgs::Pose joint; joint.position.x = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(arm.armmove_j(joint)); joint.position.x = 181; EXPECT_FALSE(arm.armmove_j(joint));
    OXYZ_Pose pose{2, 0, 0, 0, 0, 0}; EXPECT_FALSE(arm.armmovej_p(pose));
    pose.x = 0; pose.rx = std::numeric_limits<float>::quiet_NaN(); EXPECT_FALSE(arm.armmovej_p(pose));
    EXPECT_EQ(checks, arm.checks); EXPECT_EQ(0, arm.joint_moves); EXPECT_EQ(0, arm.pose_moves);
}
TEST_F(ConfigurationTest, TreatsApiSocketAndStateFailuresAsInitializationFailures) {
    FakeArm api; api.api_result = -1; EXPECT_FALSE(api.InitArm()); EXPECT_EQ(0, api.connects);
    for (int socket : {-1, 0}) {
        FakeArm arm; arm.socket_result = socket; EXPECT_FALSE(arm.InitArm()); EXPECT_EQ(0, arm.checks);
    }
    FakeArm state; state.state_result = -1; EXPECT_FALSE(state.InitArm());
}
TEST_F(ConfigurationTest, UsesConfiguredCalibrationPathAndPreservesZeroTranslationRotation) {
    std::ofstream(calibration_path) << "0 -1 0 0\n1 0 0 0\n0 0 1 0\n0 0 0 1\n";
    node.setParam("hand_eye_calibration_file", calibration_path);
    ASSERT_TRUE(load()) << error;
    EXPECT_EQ(calibration_path, config.hand_eye_calibration_file);
    EXPECT_DOUBLE_EQ(-1, config.hand_eye_calibration(0, 1));
    EXPECT_DOUBLE_EQ(0, config.hand_eye_calibration(0, 3));
    FakeArm arm; ASSERT_TRUE(arm.InitArm());
    EXPECT_TRUE(arm.GetHandEyeCalibrationResultX_EyeInHand().isApprox(config.hand_eye_calibration));
}
TEST_F(ConfigurationTest, InvalidCalibrationPreventsHardwareConnectionsAndInvalidatesGetter) {
    node.setParam("hand_eye_calibration_file", calibration_path + ".missing");
    test_tcp::Server server; node.setParam("hand_port", server.port);
    FakeArm arm; ConfigurationTestAccess::enableHand(arm);
    EXPECT_FALSE(arm.Hand_TCPCommunication_connect(ros::NodeHandle()));
    pollfd listener{server.socket.fd, POLLIN, 0}; EXPECT_EQ(0, ::poll(&listener, 1, 20));
    EXPECT_FALSE(arm.InitArm()); EXPECT_EQ(0, arm.initializations); EXPECT_EQ(0, arm.connects);
    EXPECT_THROW(arm.GetHandEyeCalibrationResultX_EyeInHand(), std::logic_error);
}
TEST_F(ConfigurationTest, RejectsCalibrationParameterTypesAndRelativePaths) {
    node.setParam("hand_eye_calibration_file", 3); EXPECT_FALSE(load());
    node.setParam("hand_eye_calibration_file", ""); EXPECT_FALSE(load());
    node.setParam("hand_eye_calibration_file", "calibration.txt"); EXPECT_FALSE(load());
}
TEST_F(ConfigurationTest, CalibrationFailureDoesNotCommitOrReuseOldControllerMatrix) {
    FakeArm arm; ASSERT_TRUE(arm.InitArm()); ASSERT_TRUE(load());
    const auto previous = config.hand_eye_calibration;
    node.setParam("hand_eye_calibration_file", calibration_path + ".missing");
    EXPECT_FALSE(load()); EXPECT_TRUE(config.hand_eye_calibration.isApprox(previous));
    EXPECT_FALSE(arm.initTargetPoses());
    EXPECT_THROW(arm.GetHandEyeCalibrationResultX_EyeInHand(), std::logic_error);
    const int checks = arm.checks; EXPECT_FALSE(arm.armmove_j(geometry_msgs::Pose())); EXPECT_EQ(checks, arm.checks);
}
TEST_F(ConfigurationTest, CalibrationClassRejectsUnloadedOrFailedResultsWithoutIdentityFallback) {
    calibration::HandEyeCalibration eye;
    EXPECT_THROW(eye.GetCalibrationResult_EyeInHand(), std::logic_error);
    std::ofstream(calibration_path) << "0 -1 0 0\n1 0 0 0\n0 0 1 0\n0 0 0 1\n";
    ASSERT_TRUE(eye.LoadEyeInHand(calibration_path, error)) << error;
    EXPECT_DOUBLE_EQ(-1, eye.GetCalibrationResult_EyeInHand()(0, 1));
    EXPECT_FALSE(eye.LoadEyeInHand(calibration_path + ".missing", error));
    EXPECT_THROW(eye.GetCalibrationResult_EyeInHand(), std::logic_error);
}
TEST_F(ConfigurationTest, LegacyReadersUseStrictLoaderAndConfiguredPath) {
    std::ofstream(calibration_path) << "1 0 0 .1\n0 1 0 .2\n0 0 1 .3\n0 0 0 1\n";
    node.setParam("hand_eye_calibration_file", calibration_path);
    calibration::HandEyeCalibration eye;
    EXPECT_DOUBLE_EQ(.1, eye.ReadCalibrationResultXFile_EyeInHand()(0, 3));
    general::GeneralFunction helper;
    EXPECT_DOUBLE_EQ(.2, helper.CalibrationEyetohandXRead_path(calibration_path)(1, 3));
    EXPECT_THROW(helper.CalibrationEyetohandXRead_path("relative.txt"), std::runtime_error);
    node.setParam("hand_eye_calibration_file", false);
    EXPECT_THROW(eye.ReadCalibrationResultXFile_EyeInHand(), std::runtime_error);
    EXPECT_THROW(eye.GetCalibrationResult_EyeInHand(), std::logic_error);
}
} // namespace AIMOcommunicate
int main(int argc, char **argv) {
    ros::init(argc, argv, "configuration_regression");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
