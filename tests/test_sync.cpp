#include <doctest/doctest.h>

#include <atomic>
#include <thread>
#include <vector>

#include "sync/Channels.hpp"

using namespace rf;

namespace {

// A payload whose fields must always agree with each other. If the triple
// buffer ever let a reader see a slot mid-write, these would disagree.
struct Consistent {
    std::uint64_t a{};
    std::uint64_t b{};
    std::uint64_t c{};
};

}  // namespace

TEST_CASE("triple buffer reports freshness correctly") {
    TripleBuffer<int> tb;

    CHECK_FALSE(tb.acquire());  // nothing published yet

    tb.writeSlot() = 7;
    tb.publish();
    CHECK(tb.acquire());
    CHECK(tb.read() == 7);

    CHECK_FALSE(tb.acquire());  // no new publish since
    CHECK(tb.read() == 7);      // and the old value is still readable
}

TEST_CASE("triple buffer keeps only the newest value") {
    TripleBuffer<int> tb;
    for (int i = 0; i < 100; ++i) {
        tb.writeSlot() = i;
        tb.publish();
    }
    CHECK(tb.acquire());
    CHECK(tb.read() == 99);
}

TEST_CASE("triple buffer never tears under contention") {
    TripleBuffer<Consistent> tb;
    std::atomic<bool> stop{false};
    std::atomic<std::uint64_t> torn{0};
    std::atomic<std::uint64_t> reads{0};

    std::thread producer([&] {
        for (std::uint64_t i = 1; !stop.load(std::memory_order_relaxed); ++i) {
            Consistent& slot = tb.writeSlot();
            slot.a = i;
            slot.b = i * 3;
            slot.c = i * 7;
            tb.publish();
        }
    });

    std::thread consumer([&] {
        std::uint64_t lastSeen = 0;
        while (!stop.load(std::memory_order_relaxed)) {
            if (!tb.acquire()) continue;
            const Consistent& v = tb.read();
            if (v.a != 0) {
                if (v.b != v.a * 3 || v.c != v.a * 7) torn.fetch_add(1);
                // Published values must never go backwards.
                if (v.a < lastSeen) torn.fetch_add(1);
                lastSeen = v.a;
                reads.fetch_add(1);
            }
        }
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    stop.store(true);
    producer.join();
    consumer.join();

    CHECK(torn.load() == 0);
    CHECK(reads.load() > 100);  // the test actually exercised something
}

TEST_CASE("spsc queue preserves order and loses nothing below capacity") {
    SpscQueue<int, 8> q;
    for (int i = 0; i < 7; ++i) CHECK(q.push(i));
    CHECK_FALSE(q.push(99));  // full: capacity - 1 usable slots
    CHECK(q.dropped() == 1);

    for (int i = 0; i < 7; ++i) {
        int out = -1;
        CHECK(q.pop(out));
        CHECK(out == i);
    }
    int out = -1;
    CHECK_FALSE(q.pop(out));
}

TEST_CASE("spsc queue delivers every item in order across threads") {
    SpscQueue<std::uint64_t, 1024> q;
    constexpr std::uint64_t kCount = 200'000;

    std::atomic<bool>         ordered{true};
    std::atomic<std::uint64_t> received{0};

    std::thread consumer([&] {
        std::uint64_t expected = 0;
        while (expected < kCount) {
            std::uint64_t v = 0;
            if (!q.pop(v)) continue;
            if (v != expected) ordered.store(false);
            ++expected;
            received.fetch_add(1, std::memory_order_relaxed);
        }
    });

    for (std::uint64_t i = 0; i < kCount;) {
        if (q.push(i)) ++i;  // retry on full rather than dropping
    }
    consumer.join();

    CHECK(ordered.load());
    CHECK(received.load() == kCount);
}
