#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "third_party/rknn_api_shim.h"

namespace rk3588::modules {

struct YoloDetection {
    int class_id = -1;
    std::string class_name;
    float confidence = 0.0F;
    float distance_m = -1.0F;
    int left = 0;
    int top = 0;
    int right = 0;
    int bottom = 0;
};

class RKNNRunner {
public:
    RKNNRunner() = default;
    ~RKNNRunner();

    bool init(const std::string& model_path, int input_w, int input_h,
              const std::string& labels_path = "models/coco_80_labels_list.txt");
    bool inferRgb(const std::uint8_t* rgb_data, std::size_t rgb_size,
                  int src_w, int src_h, std::vector<YoloDetection>* detections);
    void destroy();

    [[nodiscard]] bool isInitialized() const { return initialized_; }
    [[nodiscard]] std::uint32_t outputCount() const { return io_num_.n_output; }

private:
    bool loadLabels(const std::string& labels_path);

    rknn_context ctx_ = 0;
    rknn_input_output_num io_num_ {};
    bool initialized_ = false;
    int input_w_ = 0;
    int input_h_ = 0;
    std::vector<std::string> labels_;
};

}  // namespace rk3588::modules
