#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/Window/Event.hpp>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include "core/World.hpp"
#include "render/Camera.hpp"
#include "render/InputTranslator.hpp"
#include "render/Renderer.hpp"
#include "sim/SimulationLoop.hpp"
#include "sync/Channels.hpp"

namespace {

using Clock = std::chrono::steady_clock;

rf::WorldConfig chooseWorld(int argc, char** argv) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--world=orbit") return rf::orbitWorld();
        if (arg == "--world=empty") return rf::defaultWorld();
    }
    return rf::defaultWorld();
}

// Open on a view that actually contains the interesting things. Starting zoomed
// into empty space next to an unseen planet makes an orbit look like a rocket
// drifting sideways for no reason.
rf::Real framingWidth(const rf::WorldConfig& cfg) {
    constexpr rf::Real kEmptySpaceDefault = 300;  // metres across
    if (cfg.attractors.empty() || cfg.rockets.empty()) return kEmptySpaceDefault;

    rf::Real furthest = 0;
    for (const rf::Attractor& a : cfg.attractors) {
        furthest = std::max(furthest, rf::length(a.pos - cfg.rockets.front().pos) + a.radius);
    }
    return std::max(kEmptySpaceDefault, furthest * rf::Real(2.5));
}

}  // namespace

int main(int argc, char** argv) {
    // The two shared objects. Everything else lives on exactly one thread.
    rf::StateChannel state;
    rf::CommandQueue commands;

    const rf::WorldConfig worldConfig = chooseWorld(argc, argv);

    rf::SimulationLoop sim(worldConfig, state, commands);
    std::thread simThread([&sim] { sim.run(); });

    sf::RenderWindow window(sf::VideoMode({1280u, 720u}), "RocketFightV2");
    window.setVerticalSyncEnabled(true);

    rf::Renderer        renderer;
    rf::Camera          camera;
    rf::InputTranslator input;

    camera.setViewportPixels(window.getSize());
    camera.setWidthMeters(framingWidth(worldConfig));

    bool      showInputDebug = false;
    auto      lastFrame      = Clock::now();
    auto      lastAcquire    = Clock::now();
    rf::Real  frameRate      = 0;

    while (window.isOpen()) {
        while (const std::optional event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            } else if (const auto* key = event->getIf<sf::Event::KeyPressed>()) {
                if (key->code == sf::Keyboard::Key::Escape) window.close();
                if (key->code == sf::Keyboard::Key::F1) showInputDebug = !showInputDebug;
            } else if (const auto* resized = event->getIf<sf::Event::Resized>()) {
                camera.setViewportPixels(resized->size);
            } else if (const auto* wheel = event->getIf<sf::Event::MouseWheelScrolled>()) {
                camera.zoomBy(wheel->delta > 0 ? rf::Real(0.85) : rf::Real(1.0 / 0.85));
            }
        }

        input.poll(commands);

        // Take the newest published state, if there is one. If the simulation
        // has not published since the last frame we simply redraw what we have:
        // the renderer never waits on the simulation, ever.
        if (state.acquire()) lastAcquire = Clock::now();
        const rf::Snapshot& snap = state.read();

        if (!snap.rockets.empty()) camera.follow(snap.rockets.front().pos);

        const auto now      = Clock::now();
        const auto frameDt  = std::chrono::duration<rf::Real>(now - lastFrame).count();
        lastFrame           = now;
        if (frameDt > rf::Real(0)) {
            // Light smoothing, or the number is unreadable.
            frameRate = frameRate == 0 ? rf::Real(1) / frameDt
                                       : frameRate * rf::Real(0.9) +
                                             (rf::Real(1) / frameDt) * rf::Real(0.1);
        }

        rf::Renderer::Hud hud;
        hud.flyByWire      = sim.flyByWireEnabled();
        hud.frameRate      = frameRate;
        hud.snapshotAgeMs  = std::chrono::duration<rf::Real, std::milli>(now - lastAcquire).count();
        hud.inputDevice    = input.gamepadConnected() ? input.gamepadName() : "keyboard";
        hud.showInputDebug = showInputDebug;
        hud.input          = &input.debug();

        renderer.draw(window, snap, camera, hud);
        window.display();
    }

    sim.stop();
    simThread.join();

    if (commands.dropped() > 0) {
        std::cerr << "warning: dropped " << commands.dropped() << " input commands\n";
    }
    return EXIT_SUCCESS;
}
