#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "sync/TripleBuffer.hpp"  // kCacheLine

namespace rf {

// Bounded single-producer / single-consumer ring buffer. Lock-free, wait-free,
// and allocation-free.
//
// Overflow drops the newest item and counts it rather than blocking. Input
// events arrive at tens per second against a queue drained a thousand times a
// second, so overflow means something is badly wrong -- and a dropped keypress
// is still better than stalling the render thread inside the sim's tick budget.
template <class T, std::size_t Capacity>
class SpscQueue {
    static_assert(Capacity >= 2 && (Capacity & (Capacity - 1)) == 0,
                  "Capacity must be a power of two");

public:
    bool push(const T& item) {
        const std::size_t head = head_.load(std::memory_order_relaxed);
        const std::size_t next = (head + 1) & kMask;
        if (next == tail_.load(std::memory_order_acquire)) {
            dropped_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        buffer_[head] = item;
        head_.store(next, std::memory_order_release);
        return true;
    }

    bool pop(T& out) {
        const std::size_t tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire)) return false;
        out = buffer_[tail];
        tail_.store((tail + 1) & kMask, std::memory_order_release);
        return true;
    }

    std::uint64_t dropped() const { return dropped_.load(std::memory_order_relaxed); }

private:
    static constexpr std::size_t kMask = Capacity - 1;

    std::array<T, Capacity> buffer_{};

    alignas(kCacheLine) std::atomic<std::size_t> head_{0};
    alignas(kCacheLine) std::atomic<std::size_t> tail_{0};
    alignas(kCacheLine) std::atomic<std::uint64_t> dropped_{0};
};

}  // namespace rf
