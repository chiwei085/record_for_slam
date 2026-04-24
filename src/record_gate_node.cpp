/**
 * record_gate — waits for all required sensor topics to produce valid data,
 * then starts an in-process rosbag2 recorder.
 *
 * Replaces the fragile fixed-delay approach: recording only starts once the
 * camera streams, IMU filter, and static TF are all confirmed live and sane.
 *
 * Parameters
 * ----------
 * bag_output           (string)          Output path for the bag directory.
 * bag_storage          (string, "mcap")  rosbag2 storage plugin.
 * qos_override_path    (string, "")      Path to QoS override YAML.
 *                                        Optional — flag omitted if empty.
 * record_topics        (string[])        Topics to pass to rosbag2 recorder.
 * rgb_image_topic      (string)          RGB image topic to gate on.
 * rgb_camera_info_topic(string)          RGB camera info topic to gate on.
 * depth_image_topic    (string)          Depth image topic to gate on.
 * depth_camera_info_topic (string)       Optional depth camera info topic.
 * point_cloud_topic    (string)          Optional point cloud topic.
 * imu_topic            (string)          Filtered IMU topic to gate on.
 * use_point_cloud      (bool, false)     Gate on point cloud_topic if true.
 * approximate_sync     (bool, true)      Logged for diagnostics.
 * depth_registered_to_color (bool, true) Logged for diagnostics.
 * require_registered_depth (bool, true)  Warn if false registration conflicts.
 * require_imu          (bool, true)      Gate on imu_topic if true.
 * qos_profile          (string, "sensor_data") Subscriber QoS preset.
 * required_tf_frames   (string[], [])    child_frame_id values that must
 *                                        appear in /tf_static.  If empty,
 *                                        any non-empty TFMessage passes.
 *
 * Gate conditions (all must pass before recording starts)
 * -------------------------------------------------------
 *   rgb_image_topic                        — width/height/encoding/frame_id ok
 *   depth_image_topic                      — same checks
 *   rgb_camera_info_topic                  — width/height/K diagonal non-zero
 *   depth_camera_info_topic                — optional
 *   point_cloud_topic                      — optional
 *   imu_topic                              — frame_id set, orientation valid
 *   /tf_static                             — non-empty, required frames present
 */

#include <rclcpp/rclcpp.hpp>
#include <rmw/rmw.h>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_transport/record_options.hpp>
#include <rosbag2_transport/recorder.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{
enum class Gate : std::size_t
{
    Color = 0,
    Depth,
    RgbCamInfo,
    DepthCamInfo,
    PointCloud,
    Imu,
    TfStatic,
    Count,
};

constexpr std::size_t kNumGates = static_cast<std::size_t>(Gate::Count);

constexpr std::array<const char*, kNumGates> kGateNames = {
    "rgb image",   "depth image",  "rgb camera_info", "depth camera_info",
    "point cloud", "filtered IMU", "tf_static",
};

constexpr std::size_t gate_idx(Gate g) {
    return static_cast<std::size_t>(g);
}

std::string to_lower(std::string value) {
    std::transform(
        value.begin(), value.end(), value.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

rclcpp::ReliabilityPolicy parse_reliability(const YAML::Node& node) {
    const auto value = to_lower(node.as<std::string>());
    if (value == "best_effort") {
        return rclcpp::ReliabilityPolicy::BestEffort;
    }
    if (value == "reliable") {
        return rclcpp::ReliabilityPolicy::Reliable;
    }
    throw std::runtime_error("unsupported reliability: " + value);
}

rclcpp::DurabilityPolicy parse_durability(const YAML::Node& node) {
    const auto value = to_lower(node.as<std::string>());
    if (value == "volatile") {
        return rclcpp::DurabilityPolicy::Volatile;
    }
    if (value == "transient_local") {
        return rclcpp::DurabilityPolicy::TransientLocal;
    }
    throw std::runtime_error("unsupported durability: " + value);
}

rclcpp::HistoryPolicy parse_history(const YAML::Node& node) {
    const auto value = to_lower(node.as<std::string>());
    if (value == "keep_last") {
        return rclcpp::HistoryPolicy::KeepLast;
    }
    if (value == "keep_all") {
        return rclcpp::HistoryPolicy::KeepAll;
    }
    throw std::runtime_error("unsupported history: " + value);
}

rclcpp::QoS parse_qos_profile(const YAML::Node& node) {
    if (!node.IsMap()) {
        throw std::runtime_error("QoS profile must be a map");
    }

    const auto history = node["history"] ? parse_history(node["history"])
                                         : rclcpp::HistoryPolicy::KeepLast;
    const auto depth = node["depth"] ? node["depth"].as<std::size_t>() : 10U;

    rclcpp::QoS qos = (history == rclcpp::HistoryPolicy::KeepAll)
                          ? rclcpp::QoS(rclcpp::KeepAll())
                          : rclcpp::QoS(rclcpp::KeepLast(depth));

    if (node["reliability"]) {
        qos.reliability(parse_reliability(node["reliability"]));
    }
    if (node["durability"]) {
        qos.durability(parse_durability(node["durability"]));
    }
    if (node["history"]) {
        qos.history(history);
    }
    return qos;
}

std::unordered_map<std::string, rclcpp::QoS> load_qos_overrides(
    const std::string& path) {
    std::unordered_map<std::string, rclcpp::QoS> overrides;
    if (path.empty()) {
        return overrides;
    }

    const auto root = YAML::LoadFile(path);
    if (!root.IsMap()) {
        throw std::runtime_error("QoS override file must contain a topic map");
    }

    for (const auto& entry : root) {
        if (!entry.first.IsScalar()) {
            throw std::runtime_error("QoS override topic name must be scalar");
        }
        overrides.emplace(entry.first.as<std::string>(),
                          parse_qos_profile(entry.second));
    }
    return overrides;
}

rclcpp::QoS make_subscription_qos(const std::string& profile_name) {
    const auto value = to_lower(profile_name);
    if (value.empty() || value == "sensor_data") {
        return rclcpp::SensorDataQoS();
    }
    if (value == "default" || value == "system_default") {
        return rclcpp::QoS(10);
    }
    throw std::runtime_error("unsupported qos_profile: " + profile_name);
}
}  // namespace

class RecordGate : public rclcpp::Node
{
public:
    using NodeCallback = std::function<void(const rclcpp::Node::SharedPtr&)>;

    RecordGate(NodeCallback add_node, NodeCallback remove_node)
        : Node("record_gate"),
          flags_{},
          launched_(false),
          start_time_(std::chrono::steady_clock::now()),
          add_node_(std::move(add_node)),
          remove_node_(std::move(remove_node)) {
        declare_parameter("bag_output", "");
        declare_parameter("bag_storage", "mcap");
        declare_parameter("qos_override_path", "");
        declare_parameter("record_topics", std::vector<std::string>{});
        declare_parameter("rgb_image_topic", "/camera/color/image_raw");
        declare_parameter("rgb_camera_info_topic", "/camera/color/camera_info");
        declare_parameter("depth_image_topic", "/camera/depth/image_raw");
        declare_parameter("depth_camera_info_topic",
                          "/camera/depth/camera_info");
        declare_parameter("point_cloud_topic", "");
        declare_parameter("imu_topic", "/imu/filtered");
        declare_parameter("use_point_cloud", false);
        declare_parameter("approximate_sync", true);
        declare_parameter("depth_registered_to_color", false);
        declare_parameter("require_registered_depth", false);
        declare_parameter("require_imu", true);
        declare_parameter("qos_profile", "sensor_data");
        declare_parameter("required_tf_frames", std::vector<std::string>{});

        using Image = sensor_msgs::msg::Image;
        using CameraInfo = sensor_msgs::msg::CameraInfo;
        using Imu = sensor_msgs::msg::Imu;
        using PointCloud2 = sensor_msgs::msg::PointCloud2;
        using TFMessage = tf2_msgs::msg::TFMessage;

        const auto rgb_topic = get_parameter("rgb_image_topic").as_string();
        const auto rgb_info_topic =
            get_parameter("rgb_camera_info_topic").as_string();
        const auto depth_topic = get_parameter("depth_image_topic").as_string();
        const auto depth_info_topic =
            get_parameter("depth_camera_info_topic").as_string();
        const auto point_cloud_topic =
            get_parameter("point_cloud_topic").as_string();
        const auto imu_topic = get_parameter("imu_topic").as_string();
        const auto use_point_cloud = get_parameter("use_point_cloud").as_bool();
        const auto approximate_sync =
            get_parameter("approximate_sync").as_bool();
        const auto depth_registered_to_color =
            get_parameter("depth_registered_to_color").as_bool();
        const auto require_registered_depth =
            get_parameter("require_registered_depth").as_bool();
        const auto require_imu = get_parameter("require_imu").as_bool();
        const auto qos_profile = get_parameter("qos_profile").as_string();

        required_.fill(false);
        try {
            sensor_qos_ = make_subscription_qos(qos_profile);
        }
        catch (const std::exception& ex) {
            RCLCPP_WARN(get_logger(), "%s; falling back to sensor_data",
                        ex.what());
            sensor_qos_ = rclcpp::SensorDataQoS();
        }

        auto reliable_qos = rclcpp::QoS(100).reliable();
        // transient_local: subscriber receives the latched message immediately
        // on connect, so tf_static is typically the first gate to clear.
        auto static_qos = rclcpp::QoS(10).reliable().transient_local();

        required_[gate_idx(Gate::Color)] = !rgb_topic.empty();
        required_[gate_idx(Gate::Depth)] = !depth_topic.empty();
        required_[gate_idx(Gate::RgbCamInfo)] = !rgb_info_topic.empty();
        required_[gate_idx(Gate::DepthCamInfo)] = !depth_info_topic.empty();
        required_[gate_idx(Gate::PointCloud)] =
            use_point_cloud && !point_cloud_topic.empty();
        required_[gate_idx(Gate::Imu)] = require_imu && !imu_topic.empty();
        required_[gate_idx(Gate::TfStatic)] = true;

        if (required_[gate_idx(Gate::Color)]) {
            color_sub_ = create_subscription<Image>(
                rgb_topic, sensor_qos_, [this](Image::ConstSharedPtr msg) {
                    if (msg->width == 0 || msg->height == 0 ||
                        msg->encoding.empty() || msg->header.frame_id.empty()) {
                        return;
                    }
                    set_flag(Gate::Color);
                });
        }

        if (required_[gate_idx(Gate::Depth)]) {
            depth_sub_ = create_subscription<Image>(
                depth_topic, sensor_qos_, [this](Image::ConstSharedPtr msg) {
                    if (msg->width == 0 || msg->height == 0 ||
                        msg->encoding.empty() || msg->header.frame_id.empty()) {
                        return;
                    }
                    set_flag(Gate::Depth);
                });
        }

        if (required_[gate_idx(Gate::RgbCamInfo)]) {
            rgb_caminfo_sub_ = create_subscription<CameraInfo>(
                rgb_info_topic, sensor_qos_,
                [this](CameraInfo::ConstSharedPtr msg) {
                    if (msg->width == 0 || msg->height == 0 ||
                        msg->k[0] == 0.0 || msg->k[4] == 0.0 ||
                        msg->header.frame_id.empty()) {
                        return;
                    }
                    set_flag(Gate::RgbCamInfo);
                });
        }

        if (required_[gate_idx(Gate::DepthCamInfo)]) {
            depth_caminfo_sub_ = create_subscription<CameraInfo>(
                depth_info_topic, sensor_qos_,
                [this](CameraInfo::ConstSharedPtr msg) {
                    if (msg->width == 0 || msg->height == 0 ||
                        msg->k[0] == 0.0 || msg->k[4] == 0.0 ||
                        msg->header.frame_id.empty()) {
                        return;
                    }
                    set_flag(Gate::DepthCamInfo);
                });
        }

        if (required_[gate_idx(Gate::PointCloud)]) {
            point_cloud_sub_ = create_subscription<PointCloud2>(
                point_cloud_topic, sensor_qos_,
                [this](PointCloud2::ConstSharedPtr msg) {
                    if (msg->header.frame_id.empty()) {
                        return;
                    }
                    set_flag(Gate::PointCloud);
                });
        }

        if (required_[gate_idx(Gate::Imu)]) {
            imu_sub_ = create_subscription<Imu>(
                imu_topic, reliable_qos, [this](Imu::ConstSharedPtr msg) {
                    if (msg->header.frame_id.empty()) {
                        return;
                    }
                    // orientation_covariance[0] == -1 signals unknown
                    // orientation
                    if (msg->orientation_covariance[0] == -1.0) {
                        return;
                    }
                    set_flag(Gate::Imu);
                });
        }

        tf_static_sub_ = create_subscription<TFMessage>(
            "/tf_static", static_qos, [this](TFMessage::ConstSharedPtr msg) {
                if (msg->transforms.empty()) {
                    return;
                }
                const auto required =
                    get_parameter("required_tf_frames").as_string_array();
                for (const auto& req : required) {
                    bool found = false;
                    for (const auto& t : msg->transforms) {
                        if (t.child_frame_id == req ||
                            t.header.frame_id == req) {
                            found = true;
                            break;
                        }
                    }
                    if (!found) {
                        RCLCPP_DEBUG(get_logger(),
                                     "tf_static received but required frame "
                                     "'%s' not yet present",
                                     req.c_str());
                        return;
                    }
                }
                set_flag(Gate::TfStatic);
            });

        status_timer_ = create_wall_timer(std::chrono::seconds(5),
                                          [this]() { log_pending(); });

        RCLCPP_INFO(get_logger(), "rgb_image_topic: %s",
                    rgb_topic.empty() ? "<disabled>" : rgb_topic.c_str());
        RCLCPP_INFO(
            get_logger(), "rgb_camera_info_topic: %s",
            rgb_info_topic.empty() ? "<disabled>" : rgb_info_topic.c_str());
        RCLCPP_INFO(get_logger(), "depth_image_topic: %s",
                    depth_topic.empty() ? "<disabled>" : depth_topic.c_str());
        RCLCPP_INFO(
            get_logger(), "depth_camera_info_topic: %s",
            depth_info_topic.empty() ? "<disabled>" : depth_info_topic.c_str());
        RCLCPP_INFO(get_logger(), "point_cloud_topic: %s",
                    point_cloud_topic.empty() ? "<disabled>"
                                              : point_cloud_topic.c_str());
        RCLCPP_INFO(get_logger(), "imu_topic: %s",
                    imu_topic.empty() ? "<disabled>" : imu_topic.c_str());
        RCLCPP_INFO(get_logger(), "require_imu: %s",
                    require_imu ? "true" : "false");
        RCLCPP_INFO(get_logger(), "approximate_sync: %s",
                    approximate_sync ? "true" : "false");
        RCLCPP_INFO(get_logger(), "depth_registered_to_color: %s",
                    depth_registered_to_color ? "true" : "false");
        RCLCPP_INFO(get_logger(), "qos_profile: %s", qos_profile.c_str());

        if (!depth_registered_to_color && require_registered_depth) {
            RCLCPP_WARN(get_logger(),
                        "depth_registered_to_color=false, but this node "
                        "assumes color-depth pixel alignment");
        }

        log_required_gates();
    }

    ~RecordGate() override { stop_recording(); }

    void stop_recording() {
        if (!recorder_) {
            return;
        }

        try {
            recorder_->stop();
        }
        catch (const std::exception& ex) {
            RCLCPP_WARN(get_logger(), "Failed to stop recorder cleanly: %s",
                        ex.what());
        }

        if (remove_node_) {
            remove_node_(recorder_);
        }
        recorder_.reset();
    }

private:
    using Clock = std::chrono::steady_clock;

    double elapsed_s() const {
        return std::chrono::duration<double>(Clock::now() - start_time_)
            .count();
    }

    void set_flag(Gate gate) {
        const std::size_t idx = gate_idx(gate);
        if (!required_[idx]) {
            return;
        }
        if (flags_[idx].exchange(true)) {
            return;  // already set by an earlier message
        }

        int ready = 0;
        int total = 0;
        for (std::size_t i = 0; i < kNumGates; ++i) {
            if (!required_[i]) {
                continue;
            }
            ++total;
            ready += static_cast<int>(flags_[i].load());
        }
        RCLCPP_INFO(get_logger(), "  [%d/%zu] %s ready (+%.1fs)", ready,
                    static_cast<std::size_t>(total), kGateNames[idx],
                    elapsed_s());

        if (ready == total) {
            launch_recording();
        }
    }

    void log_pending() {
        if (launched_.load()) {
            status_timer_->cancel();
            return;
        }
        std::string pending;
        for (std::size_t i = 0; i < kNumGates; ++i) {
            if (required_[i] && !flags_[i].load()) {
                if (!pending.empty()) {
                    pending += ", ";
                }
                pending += kGateNames[i];
            }
        }
        if (!pending.empty()) {
            RCLCPP_WARN(get_logger(), "Still waiting (+%.1fs): %s", elapsed_s(),
                        pending.c_str());
        }
    }

    void log_required_gates() {
        std::string waiting_for;
        for (std::size_t i = 0; i < kNumGates; ++i) {
            if (!required_[i]) {
                continue;
            }
            if (!waiting_for.empty()) {
                waiting_for += " | ";
            }
            waiting_for += kGateNames[i];
        }

        RCLCPP_INFO(get_logger(), "Waiting for: %s", waiting_for.c_str());
    }

    void launch_recording() {
        if (launched_.exchange(true)) {
            return;  // guard against concurrent callbacks on multi-threaded
                     // exec
        }
        status_timer_->cancel();

        const auto output = get_parameter("bag_output").as_string();
        const auto storage = get_parameter("bag_storage").as_string();
        const auto qos_path = get_parameter("qos_override_path").as_string();
        const auto topics = get_parameter("record_topics").as_string_array();

        if (output.empty()) {
            RCLCPP_ERROR(get_logger(),
                         "bag_output parameter is empty — aborting");
            rclcpp::shutdown();
            return;
        }
        if (topics.empty()) {
            RCLCPP_ERROR(get_logger(),
                         "record_topics parameter is empty — aborting");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(get_logger(),
                    "All gates cleared (+%.1fs) — starting rosbag2 recorder "
                    "in-process → %s",
                    elapsed_s(), output.c_str());

        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = output;
        storage_options.storage_id = storage;

        rosbag2_transport::RecordOptions record_options;
        record_options.topics = topics;
        record_options.rmw_serialization_format =
            rmw_get_serialization_format();

        try {
            record_options.topic_qos_profile_overrides =
                load_qos_overrides(qos_path);

            recorder_ = std::make_shared<rosbag2_transport::Recorder>(
                std::make_shared<rosbag2_cpp::Writer>(), storage_options,
                record_options, "record_gate_recorder");

            if (add_node_) {
                add_node_(recorder_);
            }

            recorder_->record();
            RCLCPP_INFO(get_logger(),
                        "rosbag2 recorder is active and will stop with this "
                        "node → %s",
                        output.c_str());
        }
        catch (const std::exception& ex) {
            if (recorder_ && remove_node_) {
                remove_node_(recorder_);
            }
            recorder_.reset();
            launched_.store(false);
            RCLCPP_ERROR(get_logger(),
                         "Failed to start rosbag2 recorder: %s"
                         " — bag is NOT recording",
                         ex.what());
            rclcpp::shutdown();
        }
    }

    std::array<std::atomic<bool>, kNumGates> flags_;
    std::array<bool, kNumGates> required_;
    std::atomic<bool> launched_;
    Clock::time_point start_time_;
    NodeCallback add_node_;
    NodeCallback remove_node_;
    std::shared_ptr<rosbag2_transport::Recorder> recorder_;
    rclcpp::QoS sensor_qos_{rclcpp::SensorDataQoS()};

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
        rgb_caminfo_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
        depth_caminfo_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
        point_cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto executor =
        std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    auto gate = std::make_shared<RecordGate>(
        [executor](const rclcpp::Node::SharedPtr& node) {
            executor->add_node(node);
        },
        [executor](const rclcpp::Node::SharedPtr& node) {
            executor->remove_node(node);
        });

    executor->add_node(gate);
    executor->spin();
    gate->stop_recording();
    executor->remove_node(gate);

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return 0;
}
