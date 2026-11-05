#include "TCPCommunication_Hand.h"
#include <gtest/gtest.h>
#include <limits>

namespace AIMOcommunicate {
struct GraspTestAccess {
    static int step(const TCPCommunication_Hand &h) { return h.show_step; }
    static void step(TCPCommunication_Hand &h, int s) { h.show_step = s; }
    static void tick(TCPCommunication_Hand &h) { h.show_count_obj = 0; h.grep_run(); }
    static void configure(TCPCommunication_Hand &h) {
        h.target_poses.resize(5);
        h.tools_farmeName.resize(8, "tool");
        h.connectVisual(ros::NodeHandle());
        h.fixed_grasp_targets["bottle"] = Grap_Obj{8, 0, .1f, .2f, .3f, 0, 0, 0};
        h.fixed_grasp_targets["ball"] = Grap_Obj{5, 0, .4f, .5f, .6f, 0, 0, 0};
    }
    static std::vector<Grap_Obj> &tasks(TCPCommunication_Hand &h) { return h.obj_list; }
    static bool ready(TCPCommunication_Hand &h) { return h.if_complite; }
    static void ready(TCPCommunication_Hand &h, bool ready) { h.if_complite = ready; }
    static void clearTargets(TCPCommunication_Hand &h) { h.fixed_grasp_targets.clear(); }
    static void clearPoses(TCPCommunication_Hand &h) { h.target_poses.clear(); }
};
class FakeHand : public TCPCommunication_Hand {
public:
    FakeHand() : TCPCommunication_Hand(false) { GraspTestAccess::configure(*this); }
    bool motion_ok = true, tool_ok = true;
    int joint_moves = 0, pose_moves = 0;
    bool armmove_j(geometry_msgs::Pose) override { ++joint_moves; return motion_ok; }
    bool armmovej_p(OXYZ_Pose) override { ++pose_moves; return motion_ok; }
    bool Change_Tool_Frame(int) override { return tool_ok; }
};
using Access = GraspTestAccess;
HandTCPCommunication::OrderDetectResult::Ptr detection(std::vector<std::string> labels) {
    HandTCPCommunication::OrderDetectResult::Ptr msg(new HandTCPCommunication::OrderDetectResult);
    msg->labels = labels;
    for (size_t i = 0; i < labels.size(); ++i)
        msg->bboxes.insert(msg->bboxes.end(), {0, 0, 10, 20});
    return msg;
}
TEST(GraspSequence, InitialMoveDoesNotSkipSelection) {
    FakeHand h;
    Access::tick(h);
    EXPECT_EQ(1, Access::step(h));
    EXPECT_EQ(1, h.joint_moves);
    EXPECT_EQ(0, h.pose_moves);
    Access::tick(h);
    EXPECT_EQ(1, Access::step(h));
}
TEST(GraspSequence, EmptyAndExhaustedQueuesFinishWithoutIndexing) {
    FakeHand h;
    Access::step(h, 1); Access::ready(h, true);
    Access::tick(h); EXPECT_EQ(8, Access::step(h));
    Access::tick(h); EXPECT_EQ(9, Access::step(h));
    Access::step(h, 1);
    Access::tasks(h).push_back(Grap_Obj{8, 3, 0, 0, 0, 0, 0, 0});
    Access::tick(h); EXPECT_EQ(8, Access::step(h));
    EXPECT_TRUE(Access::tasks(h).empty());
}
TEST(GraspSequence, MotionFailuresStopWithoutFurtherCommands) {
    for (int step : {0, 3, 5, 6}) {
        FakeHand h; h.motion_ok = false; Access::step(h, step);
        Access::tick(h); EXPECT_EQ(9, Access::step(h));
        const int calls = h.joint_moves + h.pose_moves;
        Access::tick(h); EXPECT_EQ(calls, h.joint_moves + h.pose_moves);
    }
}
TEST(GraspSequence, ToolFailureStopsBeforeGraspMotion) {
    FakeHand h;
    Access::tick(h); h.orderDetectCallback(detection({"bottle"}));
    Access::tick(h); EXPECT_EQ(2, Access::step(h));
    h.tool_ok = false; Access::tick(h);
    EXPECT_EQ(9, Access::step(h)); EXPECT_EQ(0, h.pose_moves);
}
TEST(GraspSequence, ProcessesEachConfiguredObjectOnce) {
    FakeHand h;
    Access::tick(h); h.orderDetectCallback(detection({"ball", "bottle", "bottle"}));
    ASSERT_EQ(2u, Access::tasks(h).size());
    for (int i = 0; i < 30; ++i) Access::tick(h);
    EXPECT_EQ(9, Access::step(h)); EXPECT_TRUE(Access::tasks(h).empty());
    EXPECT_EQ(2, h.pose_moves); EXPECT_EQ(5, h.joint_moves);
}
TEST(GraspSequence, MissingPosesPreventMotion) {
    FakeHand h; Access::clearPoses(h); Access::tick(h);
    EXPECT_EQ(0, h.joint_moves); EXPECT_EQ(0, h.pose_moves);
}
TEST(Vision, RejectsMalformedBoxesWithoutCompletingSnapshot) {
    FakeHand h; Access::step(h, 1);
    auto msg = detection({"bottle"}); msg->bboxes.pop_back();
    h.orderDetectCallback(msg); EXPECT_FALSE(Access::ready(h));
    msg = detection({"bottle"}); msg->bboxes[0] = std::numeric_limits<float>::quiet_NaN();
    h.orderDetectCallback(msg); EXPECT_FALSE(Access::ready(h));
    msg = detection({"bottle"}); msg->bboxes[2] = -1;
    h.orderDetectCallback(msg); EXPECT_FALSE(Access::ready(h));
    EXPECT_TRUE(Access::tasks(h).empty());
}
TEST(Vision, EmptyAndUnknownLabelsProduceNoGraspTasks) {
    for (const auto &labels : {std::vector<std::string>{}, std::vector<std::string>{"unknown"}}) {
        FakeHand h; Access::step(h, 1); h.orderDetectCallback(detection(labels));
        EXPECT_TRUE(Access::ready(h)); EXPECT_TRUE(Access::tasks(h).empty());
    }
}
TEST(Vision, PixelsDoNotBecomeUnconfiguredRobotPoses) {
    FakeHand h; Access::step(h, 1); Access::clearTargets(h);
    h.orderDetectCallback(detection({"bottle"}));
    EXPECT_FALSE(Access::ready(h)); EXPECT_TRUE(Access::tasks(h).empty());
}
TEST(Vision, SubscriberRemainsActiveAfterSetupReturns) {
    FakeHand h; Access::step(h, 1);
    ros::NodeHandle node;
    auto pub = node.advertise<HandTCPCommunication::OrderDetectResult>(
        "/aimo/robot_ai/orderDetectobj_result", 1, true);
    const auto deadline = ros::WallTime::now() + ros::WallDuration(5);
    while (pub.getNumSubscribers() == 0 && ros::WallTime::now() < deadline) {
        ros::spinOnce(); ros::WallDuration(.01).sleep();
    }
    ASSERT_GT(pub.getNumSubscribers(), 0u);
    pub.publish(detection({"bottle"}));
    while (!Access::ready(h) && ros::WallTime::now() < deadline) {
        ros::spinOnce(); ros::WallDuration(.01).sleep();
    }
    ASSERT_TRUE(Access::ready(h)); ASSERT_EQ(1u, Access::tasks(h).size());
    EXPECT_FLOAT_EQ(.1f, Access::tasks(h)[0].x);
    EXPECT_FLOAT_EQ(.3f, Access::tasks(h)[0].z);
}
TEST(Configuration, LoadsInstalledResourcesAndRejectsInvalidTargetIds) {
    FakeHand h; ros::NodeHandle node("~"); node.deleteParam("fixed_grasp_targets");
    ASSERT_TRUE(h.initTargetPoses());
    XmlRpc::XmlRpcValue targets; targets.setSize(1);
    targets[0]["label"] = "bottle"; targets[0]["id"] = 9;
    targets[0]["pose"].setSize(6);
    for (int i = 0; i < 6; ++i) targets[0]["pose"][i] = 0.0;
    node.setParam("fixed_grasp_targets", targets);
    EXPECT_FALSE(h.initTargetPoses());
    targets[0]["id"] = 8; node.setParam("fixed_grasp_targets", targets);
    EXPECT_TRUE(h.initTargetPoses());
    node.deleteParam("fixed_grasp_targets");
}
} // namespace AIMOcommunicate
int main(int argc, char **argv) {
    ros::init(argc, argv, "grasp_regression");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
