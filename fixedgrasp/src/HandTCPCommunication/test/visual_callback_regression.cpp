#include "TCPCommunication_Hand.h"
#include "VisualCloudFixture.h"
#include <gtest/gtest.h>
#include <memory>

namespace AIMOcommunicate {
struct VisualCloudTestAccess {
    static void waiting(TCPCommunication_Hand &h) { h.show_step = 1; h.if_complite = false; }
    static bool ready(TCPCommunication_Hand &h) { return h.if_complite; }
    static void ready(TCPCommunication_Hand &h, bool value) { h.if_complite = value; }
    static void step(TCPCommunication_Hand &h, int value) { h.show_step = value; }
    static void invalidate(TCPCommunication_Hand &h) { h.configuration_valid = false; }
    static void invalidMatrix(TCPCommunication_Hand &h) { h.Calibration_Result_EyeInHand.setZero(); }
    static std::vector<Grap_Obj> &tasks(TCPCommunication_Hand &h) { return h.obj_list; }
    static std::vector<ObjectData> &objects(TCPCommunication_Hand &h) { return h.object_data; }
    static Grap_Obj &current(TCPCommunication_Hand &h) { return h.pass_obj; }
    static void bounds(TCPCommunication_Hand &h) { h.arm_configuration.workspace_bounds_m = {{0, 0}, {0, 0}, {0, 0}}; }
};
class VisualCallbackTest : public testing::Test {
protected:
    std::unique_ptr<TCPCommunication_Hand> hand;
    using Access = VisualCloudTestAccess;
    void SetUp() override {
        hand.reset(new TCPCommunication_Hand(false)); ASSERT_TRUE(hand->initTargetPoses()); Access::waiting(*hand);
        Access::tasks(*hand).push_back(Grap_Obj{8, 2, 1, 2, 3, 4, 5, 6});
        ObjectData old; old.id = 8; Access::objects(*hand).push_back(old);
    }
    void callback(sensor_msgs::PointCloud2 cloud) {
        sensor_msgs::PointCloud2::Ptr msg(new sensor_msgs::PointCloud2(std::move(cloud)));
        hand->GetVisualData_subCallback(msg);
    }
    void unchanged() {
        ASSERT_EQ(1u, Access::tasks(*hand).size()); EXPECT_EQ(8, Access::tasks(*hand)[0].ID);
        EXPECT_EQ(2, Access::tasks(*hand)[0].count); EXPECT_FLOAT_EQ(1, Access::tasks(*hand)[0].x);
        ASSERT_EQ(1u, Access::objects(*hand).size()); EXPECT_EQ(8, Access::objects(*hand)[0].id);
    }
};
TEST_F(VisualCallbackTest, AcceptsSparseIdsAndFullyInitializedTasksWithoutChangingCurrentObject) {
    Access::current(*hand) = Grap_Obj{6, 1, 0, 0, 0, 0, 0, .7f};
    callback(test_cloud::make({8, 2}, true)); ASSERT_TRUE(Access::ready(*hand));
    const auto &tasks = Access::tasks(*hand); ASSERT_EQ(2u, tasks.size());
    EXPECT_EQ(8, tasks[0].ID); EXPECT_EQ(2, tasks[0].count);
    EXPECT_EQ(2, tasks[1].ID); EXPECT_EQ(0, tasks[1].count);
    for (const auto &task : tasks) {
        EXPECT_TRUE(std::isfinite(task.x)); EXPECT_TRUE(std::isfinite(task.y)); EXPECT_TRUE(std::isfinite(task.z));
        EXPECT_TRUE(std::isfinite(task.rx)); EXPECT_TRUE(std::isfinite(task.ry)); EXPECT_TRUE(std::isfinite(task.rz));
    }
    EXPECT_EQ(6, Access::current(*hand).ID); EXPECT_FLOAT_EQ(.7f, Access::current(*hand).rz);
}
TEST_F(VisualCallbackTest, MissingFieldsLeaveExistingSnapshotUnchangedAndAllowRetry) {
    auto cloud = test_cloud::make({1}); cloud.fields.erase(cloud.fields.begin());
    callback(cloud); EXPECT_FALSE(Access::ready(*hand)); unchanged();
    callback(test_cloud::make({2})); EXPECT_TRUE(Access::ready(*hand));
    ASSERT_EQ(1u, Access::tasks(*hand).size()); EXPECT_EQ(2, Access::tasks(*hand)[0].ID);
}
TEST_F(VisualCallbackTest, InvalidSecondObjectDoesNotPartiallyReplaceSnapshot) {
    auto cloud = test_cloud::make({1, 2});
    test_cloud::storeFloat(cloud, cloud.point_step + 4, std::numeric_limits<float>::quiet_NaN());
    callback(cloud); EXPECT_FALSE(Access::ready(*hand)); unchanged();
}
TEST_F(VisualCallbackTest, NullAndTruncatedMessagesDoNotCompleteButValidEmptyCloudClearsBatch) {
    hand->GetVisualData_subCallback(sensor_msgs::PointCloud2ConstPtr());
    EXPECT_FALSE(Access::ready(*hand)); unchanged();
    auto cloud = test_cloud::make({1}); cloud.data.clear(); callback(cloud);
    EXPECT_FALSE(Access::ready(*hand)); unchanged();
    callback(sensor_msgs::PointCloud2()); EXPECT_TRUE(Access::ready(*hand));
    EXPECT_TRUE(Access::tasks(*hand).empty()); EXPECT_TRUE(Access::objects(*hand).empty());
}
TEST_F(VisualCallbackTest, IgnoresSnapshotsOutsideWaitingStateOrWithoutValidConfiguration) {
    for (int step : {0, 2, 3, 9}) {
        Access::step(*hand, step); callback(test_cloud::make({1})); unchanged(); EXPECT_FALSE(Access::ready(*hand));
    }
    Access::waiting(*hand); Access::ready(*hand, true); callback(test_cloud::make({1})); unchanged();
    Access::waiting(*hand); Access::invalidate(*hand); callback(test_cloud::make({1})); unchanged();
}
TEST_F(VisualCallbackTest, RejectsTransformedPositionsOutsideWorkspace) {
    Access::bounds(*hand); callback(test_cloud::make({1})); EXPECT_FALSE(Access::ready(*hand)); unchanged();
}
TEST_F(VisualCallbackTest, RejectsInvalidCalibrationWithoutCommittingTasks) {
    Access::invalidMatrix(*hand); callback(test_cloud::make({1})); EXPECT_FALSE(Access::ready(*hand)); unchanged();
}
} // namespace AIMOcommunicate
int main(int argc, char **argv) {
    ros::init(argc, argv, "visual_callback_regression");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
