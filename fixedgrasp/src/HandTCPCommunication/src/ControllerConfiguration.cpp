#include "ControllerConfiguration.h"
#include <xmlrpcpp/XmlRpcException.h>
#include <limits>
#include <set>
#include <stdexcept>
#include <memory>

namespace AIMOcommunicate {
namespace {
void require(bool valid, const std::string &field) {
    if (!valid) throw std::runtime_error("Invalid configuration: " + field);
}
bool finiteFloat(double value) {
    return std::isfinite(value) && std::abs(value) <= std::numeric_limits<float>::max() &&
        (value == 0 || std::abs(value) >= std::numeric_limits<float>::denorm_min());
}
bool cleanString(const std::string &s) {
    return !s.empty() && s.find_first_not_of(" \t\r\n") != std::string::npos &&
        s.find('\0') == std::string::npos;
}
std::string stringParam(const ros::NodeHandle &node, const std::string &key, const std::string &fallback) {
    XmlRpc::XmlRpcValue value;
    if (!node.getParam(key, value)) return fallback;
    require(value.getType() == XmlRpc::XmlRpcValue::TypeString, key + " must be a string");
    const auto text = static_cast<std::string>(value);
    require(cleanString(text), key + " must not be blank or contain NUL");
    return text;
}
int intParam(const ros::NodeHandle &node, const std::string &key, int fallback, int min, int max) {
    XmlRpc::XmlRpcValue value;
    if (!node.getParam(key, value)) return fallback;
    require(value.getType() == XmlRpc::XmlRpcValue::TypeInt, key + " must be an integer");
    const int number = static_cast<int>(value);
    require(number >= min && number <= max, key + " is out of range");
    return number;
}
float xmlNumber(XmlRpc::XmlRpcValue &value, const std::string &field) {
    double number = 0;
    if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) number = static_cast<int>(value);
    else if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) number = static_cast<double>(value);
    else require(false, field + " must be numeric");
    require(finiteFloat(number), field + " must be finite and representable as float");
    return static_cast<float>(number);
}
float jsonNumber(const Json::Value &value, const std::string &field) {
    require(value.isNumeric(), field + " must be numeric");
    const double number = value.asDouble();
    require(finiteFloat(number), field + " must be finite and representable as float");
    return static_cast<float>(number);
}
// jsoncpp 1.7 accepts an invalid token after a complete root object even with
// failIfExtra enabled. Check the root boundary; jsoncpp validates its grammar.
void requireObjectBoundary(const std::string &text, const std::string &path) {
    const auto first = text.find_first_not_of(" \t\r\n");
    require(first != std::string::npos && text[first] == '{', path + " must contain an object");
    size_t depth = 0;
    bool quoted = false, escaped = false;
    for (size_t i = first; i < text.size(); ++i) {
        const char c = text[i];
        if (quoted) {
            if (escaped) escaped = false;
            else if (c == '\\') escaped = true;
            else if (c == '"') quoted = false;
            continue;
        }
        if (c == '"') quoted = true;
        else if (c == '{' || c == '[') ++depth;
        else if (c == '}' || c == ']') {
            require(depth > 0, "unbalanced JSON in " + path);
            if (--depth == 0) {
                require(text.find_first_not_of(" \t\r\n", i + 1) == std::string::npos,
                        "trailing JSON data in " + path);
                return;
            }
        }
    }
    require(false, "incomplete JSON in " + path);
}
Json::Value readJson(const std::string &path) {
    require(cleanString(path) && path.front() == '/', "configuration file path must be absolute");
    std::ifstream file(path);
    require(file.is_open(), "cannot open " + path);
    Json::CharReaderBuilder reader;
    reader["collectComments"] = false;
    reader["allowComments"] = false;
    reader["failIfExtra"] = true;
    reader["rejectDupKeys"] = true;
    reader["allowSpecialFloats"] = false;
    std::ostringstream content;
    content << file.rdbuf();
    require(!file.bad(), "cannot read " + path);
    const auto text = content.str();
    requireObjectBoundary(text, path);
    std::unique_ptr<Json::CharReader> parser(reader.newCharReader());
    Json::Value root;
    std::string errors;
    const bool parsed = parser->parse(text.data(), text.data() + text.size(), &root, &errors);
    require(parsed, "cannot parse " + path + ": " + errors);
    require(root.isObject(), path + " must contain an object");
    return root;
}
std::vector<std::pair<float, float>> limitsParam(const ros::NodeHandle &node,
                                                const std::string &name, int count) {
    XmlRpc::XmlRpcValue value;
    if (!node.getParam(name, value)) return {};
    require(value.getType() == XmlRpc::XmlRpcValue::TypeArray && value.size() == count,
            name + " has an invalid axis count");
    std::vector<std::pair<float, float>> limits;
    for (int i = 0; i < count; ++i) {
        require(value[i].getType() == XmlRpc::XmlRpcValue::TypeArray && value[i].size() == 2,
                name + " requires [min, max] for each axis");
        const auto min = xmlNumber(value[i][0], name);
        const auto max = xmlNumber(value[i][1], name);
        require(min <= max, name + " bounds are reversed");
        limits.emplace_back(min, max);
    }
    return limits;
}
bool within(const double *values, size_t count, const std::vector<std::pair<float, float>> &limits) {
    if (!limits.empty() && limits.size() != count) return false;
    for (size_t i = 0; i < count; ++i) {
        if (!finiteFloat(values[i])) return false;
        if (!limits.empty() && (values[i] < limits[i].first || values[i] > limits[i].second)) return false;
    }
    return true;
}
} // namespace

bool validJointTarget(const geometry_msgs::Pose &pose, const ControllerConfiguration &config) {
    const double joints[] = {pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.x, pose.orientation.y, pose.orientation.z};
    return within(joints, 6, config.joint_limits_deg);
}
bool validCartesianTarget(const OXYZ_Pose &pose, const ControllerConfiguration &config) {
    const double position[] = {pose.x, pose.y, pose.z};
    const double angles[] = {pose.rx, pose.ry, pose.rz};
    return within(position, 3, config.workspace_bounds_m) && within(angles, 3, {});
}

bool loadControllerConfiguration(const ros::NodeHandle &node, const std::string &package_path,
                                 ControllerConfiguration &output, std::string &error) {
    try {
        ControllerConfiguration next;
        next.arm.ip = stringParam(node, "arm_ip", next.arm.ip);
        in_addr address{};
        require(::inet_pton(AF_INET, next.arm.ip.c_str(), &address) == 1, "arm_ip must be a numeric IPv4 address");
        next.arm.port = intParam(node, "arm_port", next.arm.port, 1, 65535);
        next.arm.timeout_ms = intParam(node, "arm_timeout_ms", next.arm.timeout_ms, 1, 60000);
        next.arm.speed_percent = intParam(node, "arm_speed_percent", next.arm.speed_percent, 1, 100);
        next.target_pose_file = stringParam(node, "target_pose_file", package_path + "/target_pose.json");
        next.tools_tcp_file = stringParam(node, "tools_tcp_file", package_path + "/tools_Tcp.json");
        next.joint_limits_deg = limitsParam(node, "joint_limits_deg", 6);
        next.workspace_bounds_m = limitsParam(node, "workspace_bounds_m", 3);

        const auto poses = readJson(next.target_pose_file);
        require(poses["target_poses"].isArray() && poses["target_poses"].size() == 5,
                "target_poses must contain exactly five six-axis joint targets");
        for (Json::ArrayIndex i = 0; i < poses["target_poses"].size(); ++i) {
            const auto &value = poses["target_poses"][i];
            const auto field = "target_poses[" + std::to_string(i) + "]";
            require(value.isObject() && value["position"].isObject() && value["orientation"].isObject(), field);
            geometry_msgs::Pose pose;
            pose.position.x = jsonNumber(value["position"]["x"], field + ".position.x");
            pose.position.y = jsonNumber(value["position"]["y"], field + ".position.y");
            pose.position.z = jsonNumber(value["position"]["z"], field + ".position.z");
            pose.orientation.x = jsonNumber(value["orientation"]["x"], field + ".orientation.x");
            pose.orientation.y = jsonNumber(value["orientation"]["y"], field + ".orientation.y");
            pose.orientation.z = jsonNumber(value["orientation"]["z"], field + ".orientation.z");
            pose.orientation.w = value["orientation"].isMember("w") ?
                jsonNumber(value["orientation"]["w"], field + ".orientation.w") : 1.0;
            require(validJointTarget(pose, next), field + " exceeds joint_limits_deg");
            next.joint_poses.push_back(pose);
        }

        const auto tools = readJson(next.tools_tcp_file);
        const auto &entries = tools["Crawl_tool"];
        require(entries.isObject(), "Crawl_tool must be an object");
        for (const auto &key : entries.getMemberNames()) {
            const auto &entry = entries[key];
            require(entry.isObject() && entry["Tcp"].isArray() && entry["Tcp"].size() == 6,
                    "Crawl_tool." + key + ".Tcp must contain six numbers");
            for (Json::ArrayIndex i = 0; i < 6; ++i) jsonNumber(entry["Tcp"][i], "Crawl_tool." + key + ".Tcp");
        }
        const std::vector<std::string> keys = {"Silk", "Eggs", "Battery", "pills", "Ball", "Screwdriver", "bottle", "Bolt"};
        for (const auto &key : keys) {
            require(entries.isMember(key), "missing required tool: " + key);
            const auto &tcp = entries[key]["Tcp"];
            OXYZ_Pose pose{};
            pose.x = tcp[0].asFloat(); pose.y = tcp[1].asFloat(); pose.z = tcp[2].asFloat();
            pose.rx = tcp[3].asFloat(); pose.ry = tcp[4].asFloat(); pose.rz = tcp[5].asFloat();
            next.tool_offsets.push_back(pose);
        }
        next.tool_frames = {"silk", "Eggs", "Battery", "pills", "Ball", "Screwdriver", "bottle", "Bolt"};

        XmlRpc::XmlRpcValue targets;
        if (node.getParam("fixed_grasp_targets", targets)) {
            require(targets.getType() == XmlRpc::XmlRpcValue::TypeArray, "fixed_grasp_targets must be an array");
            std::set<int> ids;
            for (int i = 0; i < targets.size(); ++i) {
                auto &entry = targets[i];
                const auto field = "fixed_grasp_targets[" + std::to_string(i) + "]";
                require(entry.getType() == XmlRpc::XmlRpcValue::TypeStruct &&
                    entry.hasMember("label") && entry.hasMember("id") && entry.hasMember("pose"), field);
                require(entry["label"].getType() == XmlRpc::XmlRpcValue::TypeString &&
                    entry["id"].getType() == XmlRpc::XmlRpcValue::TypeInt &&
                    entry["pose"].getType() == XmlRpc::XmlRpcValue::TypeArray && entry["pose"].size() == 6, field);
                const auto label = static_cast<std::string>(entry["label"]);
                Grap_Obj target{};
                target.ID = static_cast<int>(entry["id"]);
                require(cleanString(label) && next.fixed_targets.count(label) == 0, field + " has a blank or duplicate label");
                require(target.ID >= 1 && target.ID <= 8 && ids.insert(target.ID).second, field + " has an invalid or duplicate preset id");
                float *values[] = {&target.x, &target.y, &target.z, &target.rx, &target.ry, &target.rz};
                for (int j = 0; j < 6; ++j) *values[j] = xmlNumber(entry["pose"][j], field + ".pose");
                OXYZ_Pose pose{target.x, target.y, target.z, target.rx, target.ry, target.rz};
                require(validCartesianTarget(pose, next), field + " exceeds workspace_bounds_m");
                next.fixed_targets.emplace(label, target);
            }
        }
        output = std::move(next);
        error.clear();
        return true;
    } catch (const std::exception &exception) {
        error = exception.what();
        return false;
    } catch (const XmlRpc::XmlRpcException &exception) {
        error = exception.getMessage();
        return false;
    }
}
} // namespace AIMOcommunicate
