#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>

namespace rk3588::core {

// Thread-safe bounded queue: when full, drops oldest data to keep latency bounded.
template <typename T>
class BoundedQueue {
public:
    explicit BoundedQueue(std::size_t capacity) : capacity_(capacity) {}

    bool push(T item) {
        return pushWithDrop(std::move(item), nullptr);
    }

    bool pushWithDrop(T item, T* dropped_item) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (closed_) {
            return false;
        }
        if (queue_.size() >= capacity_) {
            if (dropped_item != nullptr) {
                *dropped_item = std::move(queue_.front());
            }
            queue_.pop_front();
        }
        queue_.push_back(std::move(item));
        cv_.notify_one();
        return true;
    }

    template <typename Rep, typename Period>
    bool pop_for(T& out, const std::chrono::duration<Rep, Period>& timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        const bool ready = cv_.wait_for(lock, timeout, [this] { return closed_ || !queue_.empty(); });
        if (!ready || queue_.empty()) {
            return false;
        }

        out = std::move(queue_.front());
        queue_.pop_front();
        return true;
    }

    void close() {
        std::lock_guard<std::mutex> lock(mutex_);
        closed_ = true;
        cv_.notify_all();
    }

    [[nodiscard]] std::size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

private:
    std::size_t capacity_ = 0;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<T> queue_;
    bool closed_ = false;
};

}  // namespace rk3588::core
