#include "pipeline/streaming_pipeline.hpp"

#include <utility>

namespace rk3588::modules {

StreamingPipeline::StreamingPipeline(AppConfig config) : config_(std::move(config)) {}
int StreamingPipeline::run() { return 0; }

}  // namespace rk3588::modules
