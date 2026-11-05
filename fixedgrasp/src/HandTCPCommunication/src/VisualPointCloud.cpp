#include "VisualPointCloud.h"
#include "CalibrationMatrix.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <set>

namespace AIMOcommunicate {
namespace {
const std::array<std::string, 11> names{{"cls", "x1", "y1", "z1", "x2", "y2", "z2", "a", "b", "c", "d"}};
size_t fieldSize(uint8_t type) {
    using sensor_msgs::PointField;
    switch (type) {
        case PointField::INT8: case PointField::UINT8: return 1;
        case PointField::INT16: case PointField::UINT16: return 2;
        case PointField::INT32: case PointField::UINT32: case PointField::FLOAT32: return 4;
        case PointField::FLOAT64: return 8;
        default: return 0;
    }
}
float readFloat(const uint8_t *bytes, bool big_endian) {
    static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
                  "PointCloud2 FLOAT32 requires IEEE-754 floats");
    uint32_t bits = 0;
    for (size_t i = 0; i < 4; ++i) {
        const size_t shift = big_endian ? 8 * (3 - i) : 8 * i;
        bits |= static_cast<uint32_t>(bytes[i]) << shift;
    }
    float value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}
bool finiteFloat(double value) {
    return std::isfinite(value) && std::abs(value) <= std::numeric_limits<float>::max();
}
bool validObject(const ObjectData &object) {
    const float values[] = {object.cen_x, object.cen_y, object.cen_z,
        object.vec_x, object.vec_y, object.vec_z, object.a, object.b, object.c, object.d};
    for (float value : values) if (!std::isfinite(value)) return false;
    return object.id >= 1 && object.id <= 8 &&
        (object.vec_x != 0 || object.vec_y != 0 || object.vec_z != 0);
}
} // namespace

bool decodeVisualPointCloud(const sensor_msgs::PointCloud2 &cloud,
                            std::vector<ObjectData> &output, std::string &error) {
    const auto fail = [&](const std::string &message) { error = "Invalid visual point cloud: " + message; return false; };
    if (cloud.width == 0) {
        if (cloud.height > 1 || cloud.row_step != 0 || !cloud.data.empty()) return fail("inconsistent empty layout");
        if (cloud.fields.empty() && cloud.point_step == 0) { output.clear(); error.clear(); return true; }
    } else if (cloud.height == 0) return fail("nonzero width requires nonzero height");
    if (cloud.point_step == 0 || cloud.width > cloud.row_step / cloud.point_step)
        return fail("point_step or row_step cannot hold the declared points");
    if (cloud.height != 0 && cloud.row_step > std::numeric_limits<size_t>::max() / cloud.height)
        return fail("row_step times height overflows size_t");
    if (static_cast<size_t>(cloud.row_step) * cloud.height != cloud.data.size())
        return fail("data length does not match row_step times height");
    if (cloud.height != 0 && cloud.width > 8u / cloud.height)
        return fail("at most eight distinct preset IDs are supported");

    std::map<std::string, sensor_msgs::PointField> fields;
    std::vector<std::pair<size_t, size_t>> ranges;
    for (const auto &field : cloud.fields) {
        const auto size = fieldSize(field.datatype);
        if (field.name.empty() || !fields.emplace(field.name, field).second) return fail("empty or duplicate field name");
        if (size == 0 || field.count == 0 || field.offset > cloud.point_step ||
            field.count > (cloud.point_step - field.offset) / size)
            return fail("invalid field span or datatype: " + field.name);
        ranges.emplace_back(field.offset, field.offset + static_cast<size_t>(field.count) * size);
    }
    std::sort(ranges.begin(), ranges.end());
    for (size_t i = 1; i < ranges.size(); ++i)
        if (ranges[i].first < ranges[i - 1].second) return fail("overlapping fields");
    std::array<size_t, 11> offsets{};
    for (size_t i = 0; i < names.size(); ++i) {
        const auto it = fields.find(names[i]);
        if (it == fields.end() || it->second.datatype != sensor_msgs::PointField::FLOAT32 || it->second.count != 1)
            return fail("required field must be one FLOAT32: " + names[i]);
        offsets[i] = it->second.offset;
    }
    std::vector<ObjectData> candidate;
    std::set<int> ids;
    for (size_t row = 0; row < cloud.height; ++row) {
        for (size_t column = 0; column < cloud.width; ++column) {
            const size_t start = row * cloud.row_step + column * cloud.point_step;
            std::array<float, 11> values{};
            for (size_t i = 0; i < names.size(); ++i) {
                values[i] = readFloat(cloud.data.data() + start + offsets[i], cloud.is_bigendian);
                if (!std::isfinite(values[i])) return fail("non-finite field: " + names[i]);
            }
            const float cls = values[0];
            if (cls < 1 || cls > 8 || std::floor(cls) != cls) return fail("cls must be an integer preset ID in 1-8");
            ObjectData object{};
            object.id = static_cast<int>(cls);
            if (!ids.insert(object.id).second) return fail("duplicate preset ID");
            object.cen_x = values[1]; object.cen_y = values[2]; object.cen_z = values[3];
            object.vec_x = values[4]; object.vec_y = values[5]; object.vec_z = values[6];
            object.a = values[7]; object.b = values[8]; object.c = values[9]; object.d = values[10];
            if (!validObject(object)) return fail("invalid object or zero direction vector");
            candidate.push_back(object);
        }
    }
    output.swap(candidate);
    error.clear();
    return true;
}

bool buildVisualGraspTasks(const std::vector<ObjectData> &objects,
                          const Eigen::Matrix4d &base_from_camera,
                          const std::vector<std::pair<float, float>> &workspace_bounds,
                          const std::vector<Grap_Obj> &previous,
                          std::vector<Grap_Obj> &output, std::string &error) {
    const auto fail = [&](const std::string &message) { error = "Invalid visual grasp snapshot: " + message; return false; };
    std::string transform_error;
    if (!calibration::validateRigidTransform(base_from_camera, transform_error)) return fail(transform_error);
    if (!workspace_bounds.empty() && workspace_bounds.size() != 3) return fail("workspace requires three bounds");
    for (const auto &bound : workspace_bounds)
        if (!finiteFloat(bound.first) || !finiteFloat(bound.second) || bound.first > bound.second) return fail("invalid workspace bounds");
    if (objects.size() > 8) return fail("too many objects");
    const Eigen::Matrix3d rotation = base_from_camera.block<3, 3>(0, 0);
    const Eigen::Vector3d angles = rotation.eulerAngles(0, 1, 2);
    constexpr double pi = 3.14159265358979323846;
    std::set<int> ids;
    std::vector<Grap_Obj> candidate;
    for (const auto &object : objects) {
        if (!validObject(object) || !ids.insert(object.id).second) return fail("invalid or duplicate object");
        const Eigen::Vector4d point = base_from_camera * Eigen::Vector4d(object.cen_x, object.cen_y, object.cen_z, 1);
        const Eigen::Vector3d direction = rotation * Eigen::Vector3d(object.vec_x, object.vec_y, object.vec_z);
        if (!point.allFinite() || !direction.allFinite() ||
            std::hypot(direction.x(), direction.y()) <= direction.norm() * 1e-8)
            return fail("direction has no stable projection onto the base XY plane");
        for (size_t axis = 0; axis < 3; ++axis) {
            if (!finiteFloat(point[axis])) return fail("transformed position exceeds float range");
            if (!workspace_bounds.empty() && (point[axis] < workspace_bounds[axis].first || point[axis] > workspace_bounds[axis].second))
                return fail("transformed position exceeds workspace bounds");
        }
        Grap_Obj task{};
        task.ID = object.id;
        task.x = point.x(); task.y = point.y(); task.z = point.z();
        task.rx = angles.x(); task.ry = angles.y();
        double yaw = std::atan2(direction.y(), direction.x());
        if (yaw > pi / 2) yaw -= pi;
        else if (yaw < -pi / 2) yaw += pi;
        task.rz = yaw;
        for (const auto &old : previous) if (old.ID == task.ID) { task.count = std::max(0, old.count); break; }
        candidate.push_back(task);
    }
    output.swap(candidate);
    error.clear();
    return true;
}
} // namespace AIMOcommunicate
