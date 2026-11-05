#ifndef LINKERHAND_TEST_VISUAL_CLOUD_FIXTURE_H
#define LINKERHAND_TEST_VISUAL_CLOUD_FIXTURE_H
#include <sensor_msgs/PointCloud2.h>
#include <cstring>
#include <vector>

namespace test_cloud {
inline void storeFloat(sensor_msgs::PointCloud2 &cloud, size_t offset, float value) {
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(value));
    for (size_t i = 0; i < 4; ++i)
        cloud.data[offset + i] = static_cast<uint8_t>(bits >> (cloud.is_bigendian ? 8 * (3 - i) : 8 * i));
}
inline sensor_msgs::PointCloud2 make(const std::vector<int> &ids, bool big_endian = false,
                                   uint32_t columns = 0, uint32_t row_padding = 0) {
    sensor_msgs::PointCloud2 cloud;
    const char *names[] = {"cls", "x1", "y1", "z1", "x2", "y2", "z2", "a", "b", "c", "d"};
    for (uint32_t i = 0; i < 11; ++i) {
        sensor_msgs::PointField field;
        field.name = names[i]; field.offset = 4 * i;
        field.datatype = sensor_msgs::PointField::FLOAT32; field.count = 1;
        cloud.fields.push_back(field);
    }
    cloud.width = columns ? columns : ids.size();
    cloud.height = cloud.width ? ids.size() / cloud.width : 1;
    cloud.point_step = 44; cloud.row_step = cloud.width * cloud.point_step + row_padding;
    cloud.is_bigendian = big_endian; cloud.is_dense = true;
    cloud.data.resize(static_cast<size_t>(cloud.row_step) * cloud.height, 0xcd);
    for (size_t i = 0; i < ids.size(); ++i) {
        const float values[] = {static_cast<float>(ids[i]), .1f, .2f, .3f, 1, 0, 0, 2, 3, 4, 5};
        const size_t start = (i / cloud.width) * cloud.row_step + (i % cloud.width) * cloud.point_step;
        for (size_t j = 0; j < 11; ++j) storeFloat(cloud, start + j * 4, values[j]);
    }
    return cloud;
}
} // namespace test_cloud
#endif
