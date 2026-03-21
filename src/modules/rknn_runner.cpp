#include "modules/rknn_runner.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <numeric>
#include <set>

namespace rk3588::modules {

namespace {

constexpr int kObjClassNum = 80;
constexpr float kConfThresh = 0.25F;
constexpr float kNmsThresh = 0.45F;

std::vector<std::uint8_t> readBinaryFile(const std::string& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        return {};
    }

    input.seekg(0, std::ios::end);
    const auto size = static_cast<std::size_t>(input.tellg());
    input.seekg(0, std::ios::beg);

    std::vector<std::uint8_t> data(size);
    if (size > 0) {
        input.read(reinterpret_cast<char*>(data.data()), static_cast<std::streamsize>(size));
    }
    return data;
}

float clampf(float v, float low, float high) {
    return std::max(low, std::min(v, high));
}

float iouXYWH(const YoloDetection& a, const YoloDetection& b) {
    const float ax1 = static_cast<float>(a.left);
    const float ay1 = static_cast<float>(a.top);
    const float ax2 = static_cast<float>(a.right);
    const float ay2 = static_cast<float>(a.bottom);

    const float bx1 = static_cast<float>(b.left);
    const float by1 = static_cast<float>(b.top);
    const float bx2 = static_cast<float>(b.right);
    const float by2 = static_cast<float>(b.bottom);

    const float inter_w = std::max(0.0F, std::min(ax2, bx2) - std::max(ax1, bx1) + 1.0F);
    const float inter_h = std::max(0.0F, std::min(ay2, by2) - std::max(ay1, by1) + 1.0F);
    const float inter = inter_w * inter_h;
    const float area_a = std::max(0.0F, ax2 - ax1 + 1.0F) * std::max(0.0F, ay2 - ay1 + 1.0F);
    const float area_b = std::max(0.0F, bx2 - bx1 + 1.0F) * std::max(0.0F, by2 - by1 + 1.0F);
    const float denom = area_a + area_b - inter;
    return denom <= 0.0F ? 0.0F : (inter / denom);
}

void computeDfl(const float* tensor, int dfl_len, float* box) {
    for (int b = 0; b < 4; ++b) {
        std::vector<float> exp_t(static_cast<std::size_t>(dfl_len));
        float exp_sum = 0.0F;
        float acc_sum = 0.0F;
        for (int i = 0; i < dfl_len; ++i) {
            exp_t[static_cast<std::size_t>(i)] = std::exp(tensor[i + b * dfl_len]);
            exp_sum += exp_t[static_cast<std::size_t>(i)];
        }
        if (exp_sum <= 0.0F) {
            box[b] = 0.0F;
            continue;
        }
        for (int i = 0; i < dfl_len; ++i) {
            acc_sum += exp_t[static_cast<std::size_t>(i)] / exp_sum * static_cast<float>(i);
        }
        box[b] = acc_sum;
    }
}

struct BranchLayout {
    int box_idx = -1;
    int score_idx = -1;
    int score_sum_idx = -1;
    int grid = 0;
    int stride = 0;
};

std::vector<BranchLayout> inferBranchLayout(const std::vector<rknn_output>& outputs, int input_h) {
    std::vector<BranchLayout> branches;
    if (outputs.size() != 9) {
        return branches;
    }

    for (int b = 0; b < 3; ++b) {
        const int base = b * 3;
        int score_sum_idx = base + 2;
        int score_idx = base + 1;
        int box_idx = base;

        const auto score_sum_elem = outputs[static_cast<std::size_t>(score_sum_idx)].size / sizeof(float);
        const int grid = static_cast<int>(std::sqrt(static_cast<double>(score_sum_elem)));
        if (grid <= 0 || grid * grid != static_cast<int>(score_sum_elem)) {
            return {};
        }

        BranchLayout layout;
        layout.box_idx = box_idx;
        layout.score_idx = score_idx;
        layout.score_sum_idx = score_sum_idx;
        layout.grid = grid;
        layout.stride = input_h / grid;
        branches.push_back(layout);
    }

    std::sort(branches.begin(), branches.end(), [](const BranchLayout& a, const BranchLayout& b) {
        return a.grid > b.grid;
    });
    return branches;
}

}  // namespace

RKNNRunner::~RKNNRunner() {
    destroy();
}

bool RKNNRunner::init(const std::string& model_path, int input_w, int input_h, const std::string& labels_path) {
    if (input_w <= 0 || input_h <= 0) {
        std::cerr << "invalid RKNN input shape" << '\n';
        return false;
    }

    auto model_data = readBinaryFile(model_path);
    if (model_data.empty()) {
        std::cerr << "failed to read rknn model: " << model_path << '\n';
        return false;
    }

    const int ret = rknn_init(&ctx_, model_data.data(), static_cast<uint32_t>(model_data.size()), 0, nullptr);
    if (ret != RKNN_SUCC) {
        std::cerr << "rknn_init failed ret=" << ret << '\n';
        ctx_ = 0;
        return false;
    }

    io_num_ = {};
    const int query_ret = rknn_query(ctx_, RKNN_QUERY_IN_OUT_NUM, &io_num_, sizeof(io_num_));
    if (query_ret != RKNN_SUCC) {
        std::cerr << "rknn_query(RKNN_QUERY_IN_OUT_NUM) failed ret=" << query_ret << '\n';
        destroy();
        return false;
    }

    input_w_ = input_w;
    input_h_ = input_h;
    initialized_ = true;

    if (!loadLabels(labels_path)) {
        labels_.clear();
    }

    std::cout << "RKNN init ok: n_input=" << io_num_.n_input << " n_output=" << io_num_.n_output << '\n';
    return true;
}

bool RKNNRunner::inferRgb(const std::uint8_t* rgb_data, std::size_t rgb_size,
                          int src_w, int src_h, std::vector<YoloDetection>* detections) {
    if (!initialized_ || rgb_data == nullptr) {
        return false;
    }
    if (detections == nullptr || src_w <= 0 || src_h <= 0) {
        return false;
    }
    detections->clear();

    const std::size_t expected_size = static_cast<std::size_t>(input_w_) * input_h_ * 3;
    if (rgb_size < expected_size || io_num_.n_input == 0) {
        return false;
    }

    rknn_input input {};
    input.index = 0;
    input.buf = const_cast<std::uint8_t*>(rgb_data);
    input.size = static_cast<uint32_t>(expected_size);
    input.pass_through = 0;
    input.type = RKNN_TENSOR_UINT8;
    input.fmt = RKNN_TENSOR_NHWC;

    int ret = rknn_inputs_set(ctx_, 1, &input);
    if (ret != RKNN_SUCC) {
        std::cerr << "rknn_inputs_set failed ret=" << ret << '\n';
        return false;
    }

    ret = rknn_run(ctx_, nullptr);
    if (ret != RKNN_SUCC) {
        std::cerr << "rknn_run failed ret=" << ret << '\n';
        return false;
    }

    std::vector<rknn_output> outputs(io_num_.n_output);
    for (std::uint32_t i = 0; i < io_num_.n_output; ++i) {
        outputs[i].want_float = 1;
        outputs[i].is_prealloc = 0;
        outputs[i].index = i;
    }

    ret = rknn_outputs_get(ctx_, io_num_.n_output, outputs.data(), nullptr);
    if (ret != RKNN_SUCC) {
        std::cerr << "rknn_outputs_get failed ret=" << ret << '\n';
        return false;
    }

    std::size_t total_output_bytes = 0;
    for (const auto& out : outputs) {
        total_output_bytes += out.size;
    }

    auto branches = inferBranchLayout(outputs, input_h_);
    if (branches.size() != 3) {
        std::cerr << "unsupported output layout, expected 9 outputs in YOLOv8 branch order" << '\n';
        (void)rknn_outputs_release(ctx_, io_num_.n_output, outputs.data());
        return false;
    }

    struct Candidate {
        float x = 0.0F;
        float y = 0.0F;
        float w = 0.0F;
        float h = 0.0F;
        float score = 0.0F;
        int class_id = -1;
    };

    std::vector<Candidate> candidates;

    for (const auto& branch : branches) {
        const auto* box = static_cast<const float*>(outputs[static_cast<std::size_t>(branch.box_idx)].buf);
        const auto* score = static_cast<const float*>(outputs[static_cast<std::size_t>(branch.score_idx)].buf);
        const auto* score_sum = static_cast<const float*>(outputs[static_cast<std::size_t>(branch.score_sum_idx)].buf);
        if (box == nullptr || score == nullptr || score_sum == nullptr) {
            continue;
        }

        const int grid_h = branch.grid;
        const int grid_w = branch.grid;
        const int grid_len = grid_h * grid_w;
        const int dfl_len = static_cast<int>(outputs[static_cast<std::size_t>(branch.box_idx)].size / sizeof(float)) / (4 * grid_len);
        if (dfl_len <= 0) {
            continue;
        }

        for (int i = 0; i < grid_h; ++i) {
            for (int j = 0; j < grid_w; ++j) {
                const int pos = i * grid_w + j;
                if (score_sum[pos] < kConfThresh) {
                    continue;
                }

                float max_score = 0.0F;
                int max_class_id = -1;
                int offset = pos;
                for (int c = 0; c < kObjClassNum; ++c) {
                    const float s = score[offset];
                    if (s > kConfThresh && s > max_score) {
                        max_score = s;
                        max_class_id = c;
                    }
                    offset += grid_len;
                }
                if (max_class_id < 0) {
                    continue;
                }

                std::vector<float> before_dfl(static_cast<std::size_t>(dfl_len * 4));
                offset = pos;
                for (int k = 0; k < dfl_len * 4; ++k) {
                    before_dfl[static_cast<std::size_t>(k)] = box[offset];
                    offset += grid_len;
                }

                float box_decoded[4] = {0.0F, 0.0F, 0.0F, 0.0F};
                computeDfl(before_dfl.data(), dfl_len, box_decoded);

                Candidate c;
                const float x1 = (-box_decoded[0] + static_cast<float>(j) + 0.5F) * static_cast<float>(branch.stride);
                const float y1 = (-box_decoded[1] + static_cast<float>(i) + 0.5F) * static_cast<float>(branch.stride);
                const float x2 = (box_decoded[2] + static_cast<float>(j) + 0.5F) * static_cast<float>(branch.stride);
                const float y2 = (box_decoded[3] + static_cast<float>(i) + 0.5F) * static_cast<float>(branch.stride);
                c.x = x1;
                c.y = y1;
                c.w = x2 - x1;
                c.h = y2 - y1;
                c.score = max_score;
                c.class_id = max_class_id;
                candidates.push_back(c);
            }
        }
    }

    std::vector<int> order(candidates.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        return candidates[static_cast<std::size_t>(a)].score > candidates[static_cast<std::size_t>(b)].score;
    });

    std::set<int> classes;
    for (const auto& c : candidates) {
        classes.insert(c.class_id);
    }

    std::vector<bool> suppressed(candidates.size(), false);
    for (int cls : classes) {
        for (std::size_t oi = 0; oi < order.size(); ++oi) {
            const int idx_i = order[oi];
            if (idx_i < 0 || suppressed[static_cast<std::size_t>(idx_i)] || candidates[static_cast<std::size_t>(idx_i)].class_id != cls) {
                continue;
            }

            YoloDetection det_i;
            det_i.class_id = cls;
            det_i.confidence = candidates[static_cast<std::size_t>(idx_i)].score;

            for (std::size_t oj = oi + 1; oj < order.size(); ++oj) {
                const int idx_j = order[oj];
                if (idx_j < 0 || suppressed[static_cast<std::size_t>(idx_j)] || candidates[static_cast<std::size_t>(idx_j)].class_id != cls) {
                    continue;
                }

                YoloDetection a;
                a.left = static_cast<int>(candidates[static_cast<std::size_t>(idx_i)].x);
                a.top = static_cast<int>(candidates[static_cast<std::size_t>(idx_i)].y);
                a.right = static_cast<int>(candidates[static_cast<std::size_t>(idx_i)].x + candidates[static_cast<std::size_t>(idx_i)].w);
                a.bottom = static_cast<int>(candidates[static_cast<std::size_t>(idx_i)].y + candidates[static_cast<std::size_t>(idx_i)].h);

                YoloDetection b;
                b.left = static_cast<int>(candidates[static_cast<std::size_t>(idx_j)].x);
                b.top = static_cast<int>(candidates[static_cast<std::size_t>(idx_j)].y);
                b.right = static_cast<int>(candidates[static_cast<std::size_t>(idx_j)].x + candidates[static_cast<std::size_t>(idx_j)].w);
                b.bottom = static_cast<int>(candidates[static_cast<std::size_t>(idx_j)].y + candidates[static_cast<std::size_t>(idx_j)].h);

                if (iouXYWH(a, b) > kNmsThresh) {
                    suppressed[static_cast<std::size_t>(idx_j)] = true;
                }
            }
        }
    }

    const float sx = static_cast<float>(src_w) / static_cast<float>(input_w_);
    const float sy = static_cast<float>(src_h) / static_cast<float>(input_h_);

    for (int idx : order) {
        if (idx < 0 || suppressed[static_cast<std::size_t>(idx)]) {
            continue;
        }
        const auto& c = candidates[static_cast<std::size_t>(idx)];

        YoloDetection det;
        det.class_id = c.class_id;
        det.class_name = (c.class_id >= 0 && c.class_id < static_cast<int>(labels_.size()))
                             ? labels_[static_cast<std::size_t>(c.class_id)]
                             : ("cls_" + std::to_string(c.class_id));
        det.confidence = c.score;

        const float x1 = clampf(c.x * sx, 0.0F, static_cast<float>(src_w));
        const float y1 = clampf(c.y * sy, 0.0F, static_cast<float>(src_h));
        const float x2 = clampf((c.x + c.w) * sx, 0.0F, static_cast<float>(src_w));
        const float y2 = clampf((c.y + c.h) * sy, 0.0F, static_cast<float>(src_h));
        det.left = static_cast<int>(x1);
        det.top = static_cast<int>(y1);
        det.right = static_cast<int>(x2);
        det.bottom = static_cast<int>(y2);

        detections->push_back(std::move(det));
    }

    static std::uint64_t infer_count = 0;
    ++infer_count;
    if ((infer_count % 30U) == 0U) {
        std::cout << "RKNN infer ok: outputs=" << io_num_.n_output
                  << " total_bytes=" << total_output_bytes
                  << " det_count=" << detections->size() << '\n';
    }

    ret = rknn_outputs_release(ctx_, io_num_.n_output, outputs.data());
    if (ret != RKNN_SUCC) {
        std::cerr << "rknn_outputs_release failed ret=" << ret << '\n';
        return false;
    }

    return true;
}

bool RKNNRunner::loadLabels(const std::string& labels_path) {
    std::ifstream input(labels_path);
    if (!input) {
        std::cerr << "warning: unable to open labels file: " << labels_path << '\n';
        return false;
    }

    labels_.clear();
    std::string line;
    while (std::getline(input, line)) {
        if (!line.empty()) {
            labels_.push_back(line);
        }
    }
    if (labels_.empty()) {
        std::cerr << "warning: labels file is empty: " << labels_path << '\n';
        return false;
    }
    return true;
}

void RKNNRunner::destroy() {
    if (ctx_ != 0) {
        (void)rknn_destroy(ctx_);
        ctx_ = 0;
    }
    io_num_ = {};
    initialized_ = false;
    input_w_ = 0;
    input_h_ = 0;
    labels_.clear();
}

}  // namespace rk3588::modules
