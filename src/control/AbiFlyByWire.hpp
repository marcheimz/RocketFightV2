#pragma once

#include <memory>
#include <string>

#include "control/Controller.hpp"

namespace rf {

// A FlyByWire that lives in somebody else's shared object.
//
// Layer 2, loaded at runtime across the plain-C boundary in
// include/rocketfight/flybywire_abi.h. Everything above and below it -- the
// scripted pilot, the benchmark meter, the world -- is unchanged and unaware,
// which is the whole reason the three-layer split exists: a submission is
// interchangeable with RocketFlyByWire because both are just a FlyByWire.
//
// What this class defends against is *bad libraries*, not *hostile code*. A
// missing symbol, a wrong ABI version, an unloadable file, an rf_init that
// declines the vehicle: all of those come back as a message and a null pointer.
// A segfault or an infinite loop inside rf_resolve cannot be defended against
// from inside the same process at all, and this class does not pretend to --
// that is why every evaluation runs in its own supervised process. See
// server/Sandbox.hpp.
class AbiFlyByWire final : public FlyByWire {
public:
    // Loads `path`, checks the version, binds the five entry points and calls
    // rf_init(spec). Returns nullptr and fills `error` on any failure; never
    // throws, never aborts.
    //
    // `path` is passed to dlopen as given, so it must contain a '/' -- a bare
    // filename would be resolved against the loader search path, which is not
    // something an evaluation host should ever let a submission name.
    static std::unique_ptr<AbiFlyByWire> load(const std::string& path, const RocketSpec& spec,
                                              std::string& error);

    ~AbiFlyByWire() override;

    AbiFlyByWire(const AbiFlyByWire&)            = delete;
    AbiFlyByWire& operator=(const AbiFlyByWire&) = delete;

    // Converts host types to the C structs, calls rf_resolve, then clamps and
    // scrubs the answer: level to [0, 1], gimbal to [-1, 1], anything
    // non-finite to zero. A NaN reaching World::setControl would propagate into
    // the position, the state hash and every metric, and the run would look
    // like a controller failure rather than the arithmetic accident it is.
    ControlInput resolve(const Intent& intent, const Observation& obs) override;

    void reset() override;

    const std::string& path() const { return path_; }

private:
    AbiFlyByWire() = default;

    using VersionFn  = int (*)(void);
    using InitFn     = int (*)(const void*);
    using ResetFn    = void (*)(void);
    using ResolveFn  = void (*)(const void*, const void*, void*);
    using ShutdownFn = void (*)(void);

    void*       handle_{nullptr};
    std::string path_;

    VersionFn  version_{nullptr};
    InitFn     init_{nullptr};
    ResetFn    reset_{nullptr};
    ResolveFn  resolve_{nullptr};
    ShutdownFn shutdown_{nullptr};

    std::size_t thrusterCount_{0};
};

}  // namespace rf
