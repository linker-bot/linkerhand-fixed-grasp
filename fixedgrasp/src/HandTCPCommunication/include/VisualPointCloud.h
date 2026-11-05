#ifndef LINKERHAND_VISUAL_POINT_CLOUD_H
#define LINKERHAND_VISUAL_POINT_CLOUD_H
#include "GraspTypes.h"
#include <Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <utility>
#include <vector>

namespace AIMOcommunicate {
// Invalid input leaves output unchanged. A valid empty cloud clears output.
bool decodeVisualPointCloud(const sensor_msgs::PointCloud2 &cloud,
                            std::vector<ObjectData> &output, std::string &error);
bool buildVisualGraspTasks(const std::vector<ObjectData> &objects,
                          const Eigen::Matrix4d &base_from_camera,
                          const std::vector<std::pair<float, float>> &workspace_bounds,
                          const std::vector<Grap_Obj> &previous,
                          std::vector<Grap_Obj> &output, std::string &error);
} // namespace AIMOcommunicate
#endif
