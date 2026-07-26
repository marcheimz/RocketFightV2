#pragma once

#include "core/Command.hpp"
#include "core/Snapshot.hpp"
#include "sync/SpscQueue.hpp"
#include "sync/TripleBuffer.hpp"

namespace rf {

// The two -- and only two -- objects shared between the simulation thread and
// the render thread. Both single-producer/single-consumer, both lock-free, and
// pointed in opposite directions.
using StateChannel = TripleBuffer<Snapshot>;
using CommandQueue = SpscQueue<Command, 1024>;

}  // namespace rf
