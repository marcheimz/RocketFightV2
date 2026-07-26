#include "render/InputTranslator.hpp"

#include <SFML/Window/Joystick.hpp>
#include <SFML/Window/Keyboard.hpp>

#include <cmath>

namespace rf {

namespace {

constexpr unsigned kPad = 0;

// Below this an axis is considered unchanged, so a stick resting against its own
// noise floor does not flood the queue at frame rate.
constexpr Real kAxisEpsilon = Real(0.004);

// XInput layout, which is what an F310 with its rear switch in the "X" position
// reports through the Linux xpad driver. SFML normalises every axis to
// [-100, 100].
//
//   X, Y -> left stick        U, V -> right stick        Z, R -> triggers
//
// If the pad is in "D" (DirectInput) mode these move around; the input debug
// overlay exists so that is a two-minute discovery rather than a mystery.
constexpr sf::Joystick::Axis kLeftX  = sf::Joystick::Axis::X;
constexpr sf::Joystick::Axis kLeftY  = sf::Joystick::Axis::Y;
constexpr sf::Joystick::Axis kRightX = sf::Joystick::Axis::U;
constexpr sf::Joystick::Axis kRightY = sf::Joystick::Axis::V;
constexpr sf::Joystick::Axis kTrigR  = sf::Joystick::Axis::R;
constexpr sf::Joystick::Axis kTrigL  = sf::Joystick::Axis::Z;

constexpr unsigned kBtnA     = 0;
constexpr unsigned kBtnX     = 2;
constexpr unsigned kBtnBack  = 6;
constexpr unsigned kBtnStart = 7;

Real stick(sf::Joystick::Axis axis) {
    return static_cast<Real>(sf::Joystick::getAxisPosition(kPad, axis)) / Real(100);
}

// Triggers rest at -100 and bottom out at +100. Map to a sane [0, 1].
Real trigger(sf::Joystick::Axis axis) {
    return clamp((stick(axis) + Real(1)) / Real(2), Real(0), Real(1));
}

bool keyDown(sf::Keyboard::Key k) { return sf::Keyboard::isKeyPressed(k); }

Real axisFromKeys(sf::Keyboard::Key negative, sf::Keyboard::Key positive) {
    return (keyDown(positive) ? Real(1) : Real(0)) - (keyDown(negative) ? Real(1) : Real(0));
}

}  // namespace

void InputTranslator::emitAxis(CommandQueue& out, InputAxis axis, Real value) {
    Real& stored = axes_[static_cast<std::size_t>(axis)];
    if (std::abs(stored - value) < kAxisEpsilon) return;
    stored = value;
    out.push(Command::makeAxis(axis, value));
}

void InputTranslator::emitButton(CommandQueue& out, InputButton button, bool down) {
    bool& stored = buttons_[static_cast<std::size_t>(button)];
    if (stored == down) return;
    stored = down;
    out.push(Command::makeButton(button, down));
}

void InputTranslator::pollGamepad(CommandQueue& out) {
    // Left stick is the acceleration the pilot wants; its direction and
    // magnitude map onto Intent almost exactly. Y is negated because SFML
    // reports "down" as positive and the world is y-up.
    emitAxis(out, InputAxis::MoveX, stick(kLeftX));
    emitAxis(out, InputAxis::MoveY, -stick(kLeftY));

    emitAxis(out, InputAxis::AimX, stick(kRightX));
    emitAxis(out, InputAxis::AimY, -stick(kRightY));

    emitAxis(out, InputAxis::TriggerL, trigger(kTrigL));
    emitAxis(out, InputAxis::TriggerR, trigger(kTrigR));

    emitButton(out, InputButton::Fire, sf::Joystick::isButtonPressed(kPad, kBtnA));
    emitButton(out, InputButton::KillVelocity, sf::Joystick::isButtonPressed(kPad, kBtnX));
    emitButton(out, InputButton::ToggleDirect, sf::Joystick::isButtonPressed(kPad, kBtnStart));
    emitButton(out, InputButton::Reset, sf::Joystick::isButtonPressed(kPad, kBtnBack));

    for (std::size_t i = 0; i < debug_.rawAxes.size(); ++i) {
        debug_.rawAxes[i] =
            sf::Joystick::getAxisPosition(kPad, static_cast<sf::Joystick::Axis>(i));
    }
    for (std::size_t i = 0; i < debug_.rawButtons.size(); ++i) {
        debug_.rawButtons[i] = sf::Joystick::isButtonPressed(kPad, static_cast<unsigned>(i));
    }
}

void InputTranslator::pollKeyboard(CommandQueue& out) {
    using Key = sf::Keyboard::Key;

    emitAxis(out, InputAxis::MoveX, axisFromKeys(Key::A, Key::D));
    emitAxis(out, InputAxis::MoveY, axisFromKeys(Key::S, Key::W));
    emitAxis(out, InputAxis::AimX, axisFromKeys(Key::Left, Key::Right));
    emitAxis(out, InputAxis::AimY, axisFromKeys(Key::Down, Key::Up));
    emitAxis(out, InputAxis::TriggerR, keyDown(Key::W) ? Real(1) : Real(0));
    emitAxis(out, InputAxis::TriggerL, Real(0));

    emitButton(out, InputButton::Fire, keyDown(Key::Space));
    emitButton(out, InputButton::KillVelocity, keyDown(Key::X));
    emitButton(out, InputButton::ToggleDirect, keyDown(Key::Tab));
    emitButton(out, InputButton::Reset, keyDown(Key::R));
}

void InputTranslator::poll(CommandQueue& out) {
    sf::Joystick::update();
    gamepadConnected_ = sf::Joystick::isConnected(kPad);
    debug_.connected  = gamepadConnected_;

    // One device at a time. If both wrote to the same axes they would fight, and
    // whichever was polled second would win -- which looks exactly like a stuck
    // stick.
    if (gamepadConnected_) {
        pollGamepad(out);
    } else {
        pollKeyboard(out);
    }
}

std::string InputTranslator::gamepadName() const {
    if (!gamepadConnected_) return "none";
    return sf::Joystick::getIdentification(kPad).name.toAnsiString();
}

}  // namespace rf
