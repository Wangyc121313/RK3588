#pragma once

#include "pipeline/app_config.hpp"

namespace rk3588::modules {

class StreamingPipeline {
public:
    explicit StreamingPipeline(AppConfig config);
    int run();

private:
    AppConfig config_;
};

}  // namespace rk3588::modules
