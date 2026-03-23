#pragma once

#include "pipeline/app_config.hpp"

namespace rk3588::modules {

class PerceptionPipeline {
public:
    explicit PerceptionPipeline(AppConfig config);
    int run();

private:
    AppConfig config_;
};

}  // namespace rk3588::modules
