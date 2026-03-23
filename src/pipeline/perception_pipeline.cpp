#include "pipeline/perception_pipeline.hpp"

#include <utility>

namespace rk3588::modules {

PerceptionPipeline::PerceptionPipeline(AppConfig config) : config_(std::move(config)) {}
int PerceptionPipeline::run() { return 0; }

}  // namespace rk3588::modules
