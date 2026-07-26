#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <new>

namespace rf {

// Hardcoded rather than std::hardware_destructive_interference_size: that
// constant is ABI-sensitive and GCC warns about using it in a header, and 64 is
// correct on every machine this will run on.
inline constexpr std::size_t kCacheLine = 64;

// Single-producer / single-consumer triple buffer.
//
// The producer always has a slot nobody else can see, the consumer always has a
// slot nobody else can see, and one slot sits in the middle as the handoff
// point. Neither side ever waits, and neither can observe a half-written value.
//
// The whole protocol is one atomic word: two bits of slot index plus a "there is
// something new" flag. Publishing and acquiring are each a single exchange.
//
// Why this and not a mutex: a mutex would let the renderer's frame time leak
// into the simulation's 1 ms tick budget. A 4 ms frame stall would cost four
// physics ticks, which is precisely the coupling this architecture exists to
// prevent.
template <class T>
class TripleBuffer {
public:
    TripleBuffer() = default;

    TripleBuffer(const TripleBuffer&)            = delete;
    TripleBuffer& operator=(const TripleBuffer&) = delete;

    // --- producer side -----------------------------------------------------

    // The slot the producer owns. Safe to write across multiple calls; it stays
    // the producer's until publish().
    T& writeSlot() { return slots_[write_]; }

    // Hand the current write slot to the consumer and take whatever slot the
    // consumer was last holding. Never blocks.
    void publish() {
        const std::uint32_t prev = shared_.exchange(write_ | kFresh, std::memory_order_acq_rel);
        write_ = prev & kIndexMask;
    }

    // --- consumer side -----------------------------------------------------

    // Take the most recently published slot, if there is one newer than what we
    // already hold. Returns false if the producer has published nothing since
    // the last acquire, in which case read() still returns the previous value.
    bool acquire() {
        if ((shared_.load(std::memory_order_acquire) & kFresh) == 0) return false;
        const std::uint32_t prev = shared_.exchange(read_, std::memory_order_acq_rel);
        read_ = prev & kIndexMask;
        return true;
    }

    const T& read() const { return slots_[read_]; }

private:
    static constexpr std::uint32_t kIndexMask = 0x3u;
    static constexpr std::uint32_t kFresh     = 0x4u;

    std::array<T, 3> slots_{};

    // Slots start distinct -- 0 held by the producer, 1 by the consumer, 2 in
    // the middle -- and every exchange preserves that, which is the invariant
    // the whole scheme rests on.
    alignas(kCacheLine) std::uint32_t write_{0};
    alignas(kCacheLine) std::uint32_t read_{1};
    alignas(kCacheLine) std::atomic<std::uint32_t> shared_{2};
};

}  // namespace rf
