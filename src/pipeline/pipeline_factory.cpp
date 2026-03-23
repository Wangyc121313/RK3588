#include "pipeline/pipeline_factory.hpp"

namespace rk3588::modules {

PerceptionPipeline PipelineFactory::makePerception(const AppConfig& config) {
    return PerceptionPipeline(config);
}

StreamingPipeline PipelineFactory::makeStreaming(const AppConfig& config) {
    return StreamingPipeline(config);
}

}  // namespace rk3588::modules
