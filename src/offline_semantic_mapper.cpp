#include "rtabmap_semantic_mapping/offline_semantic_mapper.hpp"

#include <filesystem>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#ifdef RTABMAP_SEMANTIC_MAPPING_HAS_YOLO_CPP
#include "yolo/core/error.hpp"
#include "yolo/core/image.hpp"
#include "yolo/core/model_spec.hpp"
#include "yolo/core/session_options.hpp"
#include "yolo/tasks/detection.hpp"
#endif

namespace rtabmap_semantic_mapping
{
namespace
{

// ---- YOLO helpers ----------------------------------------------------------

const std::vector<std::string> kCocoLabels = {
    "person",        "bicycle",      "car",
    "motorcycle",    "airplane",     "bus",
    "train",         "truck",        "boat",
    "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench",        "bird",
    "cat",           "dog",          "horse",
    "sheep",         "cow",          "elephant",
    "bear",          "zebra",        "giraffe",
    "backpack",      "umbrella",     "handbag",
    "tie",           "suitcase",     "frisbee",
    "skis",          "snowboard",    "sports ball",
    "kite",          "baseball bat", "baseball glove",
    "skateboard",    "surfboard",    "tennis racket",
    "bottle",        "wine glass",   "cup",
    "fork",          "knife",        "spoon",
    "bowl",          "banana",       "apple",
    "sandwich",      "orange",       "broccoli",
    "carrot",        "hot dog",      "pizza",
    "donut",         "cake",         "chair",
    "couch",         "potted plant", "bed",
    "dining table",  "toilet",       "tv",
    "laptop",        "mouse",        "remote",
    "keyboard",      "cell phone",   "microwave",
    "oven",          "toaster",      "sink",
    "refrigerator",  "book",         "clock",
    "vase",          "scissors",     "teddy bear",
    "hair drier",    "toothbrush"};

#ifdef RTABMAP_SEMANTIC_MAPPING_HAS_YOLO_CPP
struct DetectorState
{
    std::unique_ptr<yolo::Detector> detector;
};

std::string detector_cache_key(const std::string& model_path,
                               const YoloDetectorConfig& config) {
    std::ostringstream stream;
    stream << model_path << '\n'
           << config.confidence_threshold << '\n'
           << config.nms_iou_threshold << '\n'
           << config.max_detections << '\n'
           << config.class_agnostic_nms << '\n';
    for (const auto& label : config.allowed_labels) {
        stream << label << '\n';
    }
    return stream.str();
}

DetectorState& get_detector_state(const std::string& model_path,
                                  const YoloDetectorConfig& config) {
    static std::mutex mutex;
    static std::unordered_map<std::string, std::shared_ptr<DetectorState>>
        cache;

    const std::string cache_key = detector_cache_key(model_path, config);
    std::lock_guard<std::mutex> lock(mutex);
    const auto it = cache.find(cache_key);
    if (it != cache.end()) {
        return *it->second;
    }

    yolo::ModelSpec spec;
    spec.path = model_path;
    spec.task = yolo::TaskKind::detect;
    spec.labels = kCocoLabels;

    yolo::SessionOptions session;
    yolo::DetectionOptions options;
    options.confidence_threshold = config.confidence_threshold;
    options.nms_iou_threshold = config.nms_iou_threshold;
    options.max_detections = static_cast<std::size_t>(config.max_detections);
    options.class_agnostic_nms = config.class_agnostic_nms;

    auto state = std::make_shared<DetectorState>();
    state->detector = yolo::create_detector(std::move(spec), std::move(session),
                                            std::move(options));
    if (!state->detector) {
        throw std::runtime_error("YOLO-CPP returned a null detector.");
    }

    cache.emplace(cache_key, state);
    return *state;
}

yolo::ImageView to_image_view(const cv::Mat& rgb) {
    const auto* bytes = reinterpret_cast<const std::byte*>(rgb.data);
    const std::size_t total_bytes = rgb.total() * rgb.elemSize();

    yolo::ImageView image;
    image.bytes = std::span<const std::byte>(bytes, total_bytes);
    image.size = yolo::Size2i{rgb.cols, rgb.rows};
    image.stride_bytes = static_cast<std::ptrdiff_t>(rgb.step[0]);
    image.format = rgb.channels() == 3 ? yolo::PixelFormat::bgr8
                                       : yolo::PixelFormat::gray8;
    return image;
}

bool class_is_allowed(const std::string& class_name,
                      const std::unordered_set<std::string>& allowed) {
    return allowed.find(class_name) != allowed.end();
}

Detection2D to_detection(const yolo::Detection& source) {
    Detection2D detection;
    detection.class_id = static_cast<int>(source.class_id);
    detection.class_name = source.label.value_or(
        detection.class_id >= 0 &&
                detection.class_id < static_cast<int>(kCocoLabels.size())
            ? kCocoLabels[static_cast<std::size_t>(detection.class_id)]
            : "unknown");
    detection.confidence = source.score;
    detection.bbox = cv::Rect(static_cast<int>(std::round(source.bbox.x)),
                              static_cast<int>(std::round(source.bbox.y)),
                              static_cast<int>(std::round(source.bbox.width)),
                              static_cast<int>(std::round(source.bbox.height)));
    return detection;
}
#endif

// ---- OfflineSemanticMapper helpers -----------------------------------------

DetectionSummary summarize_detections(
    const std::vector<FrameDetections>& frame_detections) {
    DetectionSummary summary;
    summary.total_frames = frame_detections.size();

    for (const auto& frame_detection : frame_detections) {
        if (!frame_detection.detections.empty()) {
            ++summary.frames_with_detections;
        }

        summary.total_detections += frame_detection.detections.size();
        for (const auto& detection : frame_detection.detections) {
            ++summary.detections_per_class[detection.class_name];
        }
    }

    return summary;
}

}  // namespace

// ---- YoloDetector ----------------------------------------------------------

YoloDetector::YoloDetector(std::string model_path, YoloDetectorConfig config)
    : model_path_(std::move(model_path)), config_(std::move(config)) {
    if (model_path_.empty()) {
        throw std::invalid_argument("YOLO model path is empty.");
    }
    if (!std::filesystem::exists(model_path_)) {
        throw std::runtime_error("YOLO model path does not exist: " +
                                 model_path_);
    }

#ifdef RTABMAP_SEMANTIC_MAPPING_HAS_YOLO_CPP
    static_cast<void>(get_detector_state(model_path_, config_));
#endif
}

std::vector<Detection2D> YoloDetector::infer(const cv::Mat& rgb) const {
    if (rgb.empty()) {
        return {};
    }

#ifdef RTABMAP_SEMANTIC_MAPPING_HAS_YOLO_CPP
    DetectorState& state = get_detector_state(model_path_, config_);
    const yolo::DetectionResult result =
        state.detector->run(to_image_view(rgb));
    yolo::throw_if_error(result.error);

    const std::unordered_set<std::string> allowed_set(
        config_.allowed_labels.begin(), config_.allowed_labels.end());

    std::vector<Detection2D> detections;
    detections.reserve(result.detections.size());
    for (const auto& raw_detection : result.detections) {
        Detection2D detection = to_detection(raw_detection);
        if (!class_is_allowed(detection.class_name, allowed_set)) {
            continue;
        }
        if (detection.bbox.width <= 0 || detection.bbox.height <= 0) {
            continue;
        }
        detections.push_back(std::move(detection));
    }
    return detections;
#else
    throw std::runtime_error(
        "YOLO-CPP support is not available in this build. Install/build "
        "onnxruntime first, "
        "then rebuild the package to enable YOLO inference.");
#endif
}

// ---- OfflineSemanticMapper -------------------------------------------------

OfflineSemanticMapper::OfflineSemanticMapper(OfflineSemanticMapperConfig config)
    : config_(std::move(config)),
      database_reader_(config_.database_path),
      detector_(config_.model_path, config_.detector),
      projector_(config_.projector),
      fusion_(config_.fusion),
      map_cloud_exporter_(config_.map_cloud),
      exporter_(config_.output_dir) {}

OfflineSemanticMapperResult OfflineSemanticMapper::run() {
    OfflineSemanticMapperResult result;
    const LoadedDataset loaded_dataset = database_reader_.load();
    result.dataset_summary = loaded_dataset.summary;
    result.frames = loaded_dataset.frames;
    exporter_.export_dataset_debug(result.dataset_summary, result.frames);

    result.frame_detections.reserve(result.frames.size());
    result.frame_observations.reserve(result.frames.size());
    for (std::size_t frame_index = 0; frame_index < result.frames.size();
         ++frame_index) {
        const auto& frame = result.frames[frame_index];

        FrameDetections frame_detections;
        frame_detections.node_id = frame.node_id;
        frame_detections.stamp_sec = frame.stamp_sec;
        frame_detections.detections = detector_.infer(frame.rgb);

        exporter_.export_detection_debug(frame, frame_detections.detections,
                                         frame_index);
        auto observations =
            projector_.project(frame, frame_detections.detections);
        exporter_.export_observation_debug(frame, frame_detections.detections,
                                           observations, frame_index,
                                           config_.projector.bbox_center_ratio);
        result.frame_detections.push_back(std::move(frame_detections));
        result.frame_observations.push_back(std::move(observations));
    }

    result.detection_summary = summarize_detections(result.frame_detections);
    result.projection_summary = projector_.summary();
    for (const auto& observations : result.frame_observations) {
        fusion_.update(observations);
    }
    result.fusion_summary = fusion_.summary();
    result.semantic_objects = fusion_.filtered_objects();
    const auto own_map_cloud = map_cloud_exporter_.reconstruct(result.frames);
    map_cloud_exporter_.save_pcd(
        own_map_cloud,
        (std::filesystem::path(exporter_.output_dir()) / "own_map_cloud.pcd")
            .string());
    exporter_.export_detection_summary(result.detection_summary);
    exporter_.export_observation_summary(result.projection_summary);
    exporter_.export_fusion_debug(fusion_.objects(), result.semantic_objects,
                                  result.fusion_summary, fusion_.assignments(),
                                  config_.fusion.min_seen_count_to_keep);
    return result;
}

}  // namespace rtabmap_semantic_mapping
