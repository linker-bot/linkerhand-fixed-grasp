#include "VisualPointCloud.h"
#include "VisualCloudFixture.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace AIMOcommunicate {
class CloudDecoderTest : public testing::Test {
protected:
    std::vector<ObjectData> objects;
    std::string error;
    bool decode(const sensor_msgs::PointCloud2 &cloud) { return decodeVisualPointCloud(cloud, objects, error); }
};
TEST_F(CloudDecoderTest, DecodesSparseIdsAndInitializesEveryObjectField) {
    ASSERT_TRUE(decode(test_cloud::make({8, 2}))) << error;
    ASSERT_EQ(2u, objects.size()); EXPECT_EQ(8, objects[0].id); EXPECT_EQ(2, objects[1].id);
    EXPECT_FLOAT_EQ(.1f, objects[0].cen_x); EXPECT_FLOAT_EQ(.2f, objects[0].cen_y);
    EXPECT_FLOAT_EQ(.3f, objects[0].cen_z); EXPECT_FLOAT_EQ(1, objects[0].vec_x);
    EXPECT_FLOAT_EQ(0, objects[0].vec_y); EXPECT_FLOAT_EQ(0, objects[0].vec_z);
    EXPECT_FLOAT_EQ(2, objects[0].a); EXPECT_FLOAT_EQ(3, objects[0].b);
    EXPECT_FLOAT_EQ(4, objects[0].c); EXPECT_FLOAT_EQ(5, objects[0].d); EXPECT_EQ(0, objects[0].count);
}
TEST_F(CloudDecoderTest, SupportsBothByteOrdersAndPaddedOrganizedRows) {
    for (bool big : {false, true}) {
        ASSERT_TRUE(decode(test_cloud::make({1, 8, 2, 7}, big, 2, 9))) << error;
        ASSERT_EQ(4u, objects.size()); EXPECT_EQ(2, objects[2].id);
        EXPECT_EQ(7, objects[3].id); EXPECT_FLOAT_EQ(.3f, objects[3].cen_z);
    }
}
TEST_F(CloudDecoderTest, SupportsUnalignedOffsetsAndReorderedFields) {
    auto cloud = test_cloud::make({5});
    cloud.data.insert(cloud.data.begin(), 0xab); ++cloud.point_step; ++cloud.row_step;
    for (auto &field : cloud.fields) ++field.offset;
    std::reverse(cloud.fields.begin(), cloud.fields.end());
    ASSERT_TRUE(decode(cloud)) << error; EXPECT_EQ(5, objects[0].id); EXPECT_FLOAT_EQ(.1f, objects[0].cen_x);
}
TEST_F(CloudDecoderTest, AllowsValidExtraFieldsAndTrailingPointPadding) {
    auto cloud = test_cloud::make({1});
    sensor_msgs::PointField field; field.name = "extra"; field.offset = 44;
    field.datatype = sensor_msgs::PointField::UINT8; field.count = 3;
    cloud.fields.push_back(field); cloud.point_step = cloud.row_step = 48; cloud.data.resize(48);
    EXPECT_TRUE(decode(cloud)) << error;
}
TEST_F(CloudDecoderTest, RejectsEveryMissingRequiredField) {
    for (size_t i = 0; i < 11; ++i) {
        auto cloud = test_cloud::make({1}); cloud.fields.erase(cloud.fields.begin() + i);
        EXPECT_FALSE(decode(cloud)) << i; EXPECT_TRUE(objects.empty());
    }
}
TEST_F(CloudDecoderTest, RejectsDuplicateNamesAndOverlappingFields) {
    auto cloud = test_cloud::make({1}); cloud.fields.push_back(cloud.fields.front()); EXPECT_FALSE(decode(cloud));
    cloud = test_cloud::make({1}); cloud.fields[1].offset = 2; EXPECT_FALSE(decode(cloud));
    cloud = test_cloud::make({1}); cloud.fields[0].name = ""; EXPECT_FALSE(decode(cloud));
}
TEST_F(CloudDecoderTest, RejectsWrongRequiredTypesAndCounts) {
    for (size_t i = 0; i < 11; ++i) {
        auto cloud = test_cloud::make({1}); cloud.fields[i].datatype = sensor_msgs::PointField::UINT32;
        EXPECT_FALSE(decode(cloud));
        cloud = test_cloud::make({1}); cloud.fields[i].count = 0; EXPECT_FALSE(decode(cloud));
        cloud = test_cloud::make({1}); cloud.fields[i].count = 2; EXPECT_FALSE(decode(cloud));
    }
}
TEST_F(CloudDecoderTest, RejectsOutOfRangeFieldSpansAndUnknownTypes) {
    auto cloud = test_cloud::make({1}); cloud.fields[0].offset = std::numeric_limits<uint32_t>::max(); EXPECT_FALSE(decode(cloud));
    cloud = test_cloud::make({1}); cloud.fields[0].offset = 43; EXPECT_FALSE(decode(cloud));
    cloud = test_cloud::make({1}); cloud.fields[0].count = std::numeric_limits<uint32_t>::max(); EXPECT_FALSE(decode(cloud));
    cloud = test_cloud::make({1}); cloud.fields[0].datatype = 99; EXPECT_FALSE(decode(cloud));
}
TEST_F(CloudDecoderTest, RejectsInvalidStridesDimensionsAndPayloadSizes) {
    auto original = test_cloud::make({1});
    auto cloud = original; cloud.point_step = 0; EXPECT_FALSE(decode(cloud));
    cloud = original; cloud.row_step = 43; EXPECT_FALSE(decode(cloud));
    cloud = original; cloud.height = 0; EXPECT_FALSE(decode(cloud));
    cloud = original; cloud.width = std::numeric_limits<uint32_t>::max(); EXPECT_FALSE(decode(cloud));
    cloud = original; cloud.height = std::numeric_limits<uint32_t>::max(); EXPECT_FALSE(decode(cloud));
    cloud = original; cloud.data.clear(); EXPECT_FALSE(decode(cloud));
    cloud = original; cloud.data.pop_back(); EXPECT_FALSE(decode(cloud));
    cloud = original; cloud.data.push_back(0); EXPECT_FALSE(decode(cloud));
}
TEST_F(CloudDecoderTest, AcceptsCanonicalEmptySnapshotsAndRejectsInconsistentEmptyLayouts) {
    ObjectData old; old.id = 8; objects.push_back(old);
    ASSERT_TRUE(decode(sensor_msgs::PointCloud2())) << error; EXPECT_TRUE(objects.empty());
    ASSERT_TRUE(decode(test_cloud::make({}))) << error; EXPECT_TRUE(objects.empty());
    auto cloud = test_cloud::make({}); cloud.height = 2; EXPECT_FALSE(decode(cloud));
    cloud = test_cloud::make({}); cloud.row_step = 1; cloud.data.push_back(0); EXPECT_FALSE(decode(cloud));
}
TEST_F(CloudDecoderTest, RejectsNonFiniteValuesEvenWhenCloudIsNotDense) {
    for (size_t i = 0; i < 11; ++i) {
        for (float value : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()}) {
            auto cloud = test_cloud::make({1}); cloud.is_dense = false;
            test_cloud::storeFloat(cloud, i * 4, value); EXPECT_FALSE(decode(cloud)) << i;
        }
    }
}
TEST_F(CloudDecoderTest, RejectsFractionalOutOfRangeAndDuplicatePresetIds) {
    for (float cls : {0.f, -1.f, 1.5f, 9.f, std::numeric_limits<float>::max()}) {
        auto cloud = test_cloud::make({1}); test_cloud::storeFloat(cloud, 0, cls); EXPECT_FALSE(decode(cloud));
    }
    EXPECT_FALSE(decode(test_cloud::make({3, 3})));
    EXPECT_FALSE(decode(test_cloud::make({1, 2, 3, 4, 5, 6, 7, 8, 1})));
}
TEST_F(CloudDecoderTest, RejectsZeroDirectionsAndPreservesOutputOnFailure) {
    ASSERT_TRUE(decode(test_cloud::make({8}))); const auto previous = objects[0];
    auto cloud = test_cloud::make({1}); test_cloud::storeFloat(cloud, 16, 0);
    EXPECT_FALSE(decode(cloud)); ASSERT_EQ(1u, objects.size()); EXPECT_EQ(previous.id, objects[0].id);
    EXPECT_FALSE(error.empty()); ASSERT_TRUE(decode(test_cloud::make({2}))); EXPECT_TRUE(error.empty());
}

class CloudTasksTest : public testing::Test {
protected:
    std::string error;
    std::vector<Grap_Obj> tasks, previous;
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    std::vector<std::pair<float, float>> bounds;
    ObjectData object(int id = 8) {
        ObjectData value; value.id = id; value.cen_x = 1; value.vec_x = 1; return value;
    }
    bool build(std::vector<ObjectData> objects) {
        return buildVisualGraspTasks(objects, transform, bounds, previous, tasks, error);
    }
};
TEST_F(CloudTasksTest, TransformsPositionsAndDirectionsAndInitializesEveryTaskField) {
    transform.block<3, 3>(0, 0) = Eigen::AngleAxisd(1.5707963267948966, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    transform.block<3, 1>(0, 3) = Eigen::Vector3d(1, 2, 3);
    ASSERT_TRUE(build({object()})) << error; ASSERT_EQ(1u, tasks.size());
    EXPECT_EQ(8, tasks[0].ID); EXPECT_EQ(0, tasks[0].count);
    EXPECT_NEAR(1, tasks[0].x, 1e-6); EXPECT_NEAR(3, tasks[0].y, 1e-6); EXPECT_NEAR(3, tasks[0].z, 1e-6);
    const auto angles = transform.block<3, 3>(0, 0).eulerAngles(0, 1, 2);
    EXPECT_NEAR(angles.x(), tasks[0].rx, 1e-6); EXPECT_NEAR(angles.y(), tasks[0].ry, 1e-6);
    EXPECT_NEAR(1.5707963267948966, tasks[0].rz, 1e-6);
}
TEST_F(CloudTasksTest, PreservesAttemptCountsAndRemovesObjectsAbsentFromSnapshot) {
    previous = {Grap_Obj{1, 2, 0, 0, 0, 0, 0, 0}, Grap_Obj{8, 1, 0, 0, 0, 0, 0, 0}};
    ASSERT_TRUE(build({object(8)})) << error; ASSERT_EQ(1u, tasks.size()); EXPECT_EQ(1, tasks[0].count);
    previous[1].count = -1; ASSERT_TRUE(build({object(8)})); EXPECT_EQ(0, tasks[0].count);
}
TEST_F(CloudTasksTest, FoldsDirectionYawIntoGraspAxisRangeIncludingPi) {
    auto value = object(); value.vec_x = -1;
    ASSERT_TRUE(build({value})); EXPECT_NEAR(0, tasks[0].rz, 1e-6);
    value.vec_y = 1; ASSERT_TRUE(build({value})); EXPECT_NEAR(-.7853981633974483, tasks[0].rz, 1e-6);
    value.vec_y = -1; ASSERT_TRUE(build({value})); EXPECT_NEAR(.7853981633974483, tasks[0].rz, 1e-6);
}
TEST_F(CloudTasksTest, RejectsInvalidTransformsAndNonPlanarDirections) {
    transform(0, 0) = -1; EXPECT_FALSE(build({object()}));
    transform.setIdentity(); auto value = object(); value.vec_x = 0; value.vec_z = 1;
    EXPECT_FALSE(build({value})); value.vec_x = 1e-10f; EXPECT_FALSE(build({value}));
    value.vec_z = 0; value.vec_x = 0; EXPECT_FALSE(build({value}));
}
TEST_F(CloudTasksTest, RejectsOverflowAndPositionsOutsideValidatedWorkspace) {
    bounds = {{-1, 1}, {-1, 1}, {-1, 1}};
    auto value = object(); value.cen_x = 2; EXPECT_FALSE(build({value}));
    value.cen_x = 1; ASSERT_TRUE(build({value}));
    bounds[0] = {1, -1}; EXPECT_FALSE(build({value}));
    bounds = {{-1, 1}}; EXPECT_FALSE(build({value})); bounds.clear();
    transform(0, 3) = std::numeric_limits<float>::max(); value.cen_x = std::numeric_limits<float>::max();
    EXPECT_FALSE(build({value}));
}
TEST_F(CloudTasksTest, DoesNotCommitPartialSnapshotsAndClearsOnValidEmptyInput) {
    ASSERT_TRUE(build({object()})); const auto previous_task = tasks[0];
    auto bad = object(2); bad.cen_x = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(build({object(1), bad})); ASSERT_EQ(1u, tasks.size()); EXPECT_EQ(previous_task.ID, tasks[0].ID);
    EXPECT_FALSE(build({object(1), object(1)})); EXPECT_EQ(previous_task.ID, tasks[0].ID);
    ASSERT_TRUE(build({})); EXPECT_TRUE(tasks.empty()); EXPECT_TRUE(error.empty());
}
} // namespace AIMOcommunicate
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
