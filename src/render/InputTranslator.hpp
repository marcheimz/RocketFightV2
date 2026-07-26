#pragma once

#include <array>
#include <cstddef>
#include <string>

#include "core/Command.hpp"
#include "sync/Channels.hpp"

namespace rf {

// Turns physical devices into Commands, and does nothing else.
//
// Deliberately semantics-free: it knows that the left stick is "MoveX/MoveY",
// not what moving means. Deciding that a stick direction is a desired
// acceleration is the control layer's job, on the simulation side, where it can
// be tested without a window.
//
// Devices are polled once per frame rather than driven by events. Polling gives
// the resting position of every axis immediately, survives hot-plugging, and
// cannot miss an edge because the window lost focus mid-press.
class InputTranslator {
public:
    // Emits only what changed, so an idle gamepad costs nothing.
    void poll(CommandQueue& out);

    bool        gamepadConnected() const { return gamepadConnected_; }
    std::string gamepadName() const;

    // Raw device values, for the on-screen input debug overlay. Gamepad axis
    // mappings vary by driver and by the F310's own X/D switch, so being able to
    // see the raw numbers is the difference between a five-second fix and a
    // guessing game.
    struct Debug {
        bool                    connected{};
        std::array<float, 8>    rawAxes{};
        std::array<bool, 12>    rawButtons{};
    };
    const Debug& debug() const { return debug_; }

private:
    void emitAxis(CommandQueue& out, InputAxis axis, Real value);
    void emitButton(CommandQueue& out, InputButton button, bool down);

    void pollGamepad(CommandQueue& out);
    void pollKeyboard(CommandQueue& out);

    std::array<Real, static_cast<std::size_t>(InputAxis::Count)>   axes_{};
    std::array<bool, static_cast<std::size_t>(InputButton::Count)> buttons_{};

    bool  gamepadConnected_{false};
    Debug debug_{};
};

}  // namespace rf
