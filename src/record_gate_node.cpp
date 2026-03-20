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
 * required_tf_frames   (string[], [])    child_frame_id values that must
 *                                        appear in /tf_static.  If empty,
 *                                        any non-empty TFMessage passes.
 *
 * Gate conditions (all must pass before recording starts)
 * -------------------------------------------------------
 *   /camera/color/image_raw                — width/height/encoding/frame_id ok
 *   /camera/aligned_depth_to_color/image_raw — same checks
 *   /camera/color/camera_info              — width/height/K diagonal non-zero
 *   /imu/filtered                          — frame_id set, orientation valid
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
    CamInfo,
    Imu,
    TfStatic,
    Count,
};

constexpr std::size_t kNumGates = static_cast<std::size_t>(Gate::Count);

constexpr std::array<const char*, kNumGates> kGateNames = {
    "color image", "aligned depth", "camera_info", "filtered IMU", "tf_static",
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
        declare_parameter("required_tf_frames", std::vector<std::string>{});

        using Image = sensor_msgs::msg::Image;
        using CameraInfo = sensor_msgs::msg::CameraInfo;
        using Imu = sensor_msgs::msg::Imu;
        using TFMessage = tf2_msgs::msg::TFMessage;

        auto sensor_qos = rclcpp::SensorDataQoS();
        auto reliable_qos = rclcpp::QoS(100).reliable();
        // transient_local: subscriber receives the latched message immediately
        // on connect, so tf_static is typically the first gate to clear.
        auto static_qos = rclcpp::QoS(10).reliable().transient_local();

        color_sub_ = create_subscription<Image>(
            "/camera/color/image_raw", sensor_qos,
            [this](Image::ConstSharedPtr msg) {
                if (msg->width == 0 || msg->height == 0 ||
                    msg->encoding.empty() || msg->header.frame_id.empty()) {
                    return;
                }
                set_flag(Gate::Color);
            });

        depth_sub_ = create_subscription<Image>(
            "/camera/aligned_depth_to_color/image_raw", sensor_qos,
            [this](Image::ConstSharedPtr msg) {
                if (msg->width == 0 || msg->height == 0 ||
                    msg->encoding.empty() || msg->header.frame_id.empty()) {
                    return;
                }
                set_flag(Gate::Depth);
            });

        caminfo_sub_ = create_subscription<CameraInfo>(
            "/camera/color/camera_info", sensor_qos,
            [this](CameraInfo::ConstSharedPtr msg) {
                if (msg->width == 0 || msg->height == 0 || msg->k[0] == 0.0 ||
                    msg->k[4] == 0.0 || msg->header.frame_id.empty()) {
                    return;
                }
                set_flag(Gate::CamInfo);
            });

        imu_sub_ = create_subscription<Imu>(
            "/imu/filtered", reliable_qos, [this](Imu::ConstSharedPtr msg) {
                if (msg->header.frame_id.empty()) {
                    return;
                }
                // orientation_covariance[0] == -1 signals unknown orientation
                if (msg->orientation_covariance[0] == -1.0) {
                    return;
                }
                set_flag(Gate::Imu);
            });

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

        RCLCPP_INFO(get_logger(),
                    "Waiting for: color image | aligned depth | "
                    "camera_info | filtered IMU | tf_static");
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
        if (flags_[idx].exchange(true)) {
            return;  // already set by an earlier message
        }

        int ready = 0;
        for (const auto& f : flags_) {
            ready += static_cast<int>(f.load());
        }
        RCLCPP_INFO(get_logger(), "  [%d/%zu] %s ready (+%.1fs)", ready,
                    kNumGates, kGateNames[idx], elapsed_s());

        if (ready == static_cast<int>(kNumGates)) {
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
            if (!flags_[i].load()) {
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
    std::atomic<bool> launched_;
    Clock::time_point start_time_;
    NodeCallback add_node_;
    NodeCallback remove_node_;
    std::shared_ptr<rosbag2_transport::Recorder> recorder_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub_;
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
