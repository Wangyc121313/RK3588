#pragma once

#include "pipeline/perception_pipeline.hpp"
#include "pipeline/streaming_pipeline.hpp"

namespace rk3588::modules {

class PipelineFactory {
public:
    static PerceptionPipeline makePerception(const AppConfig& config);
    static StreamingPipeline makeStreaming(const AppConfig& config);
};

}  // namespace rk3588::modules
