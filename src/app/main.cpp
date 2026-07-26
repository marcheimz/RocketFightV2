#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/Window/Event.hpp>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "core/World.hpp"
#include "data/RocketCatalogue.hpp"
#include "render/Camera.hpp"
#include "render/InputTranslator.hpp"
#include "render/Renderer.hpp"
#include "sim/SimulationLoop.hpp"
#include "sync/Channels.hpp"

namespace {

using Clock = std::chrono::steady_clock;

struct Options {
    bool        orbit{false};
    std::string rocket;
};

Options parseOptions(int argc, char** argv, const rf::RocketCatalogue& catalogue) {
    Options opts;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--world=orbit") opts.orbit = true;
        if (arg == "--world=empty") opts.orbit = false;
        if (arg.rfind("--rocket=", 0) == 0) opts.rocket = arg.substr(9);
    }

    if (!opts.rocket.empty() && !catalogue.find(opts.rocket)) {
        std::cerr << "unknown rocket '" << opts.rocket << "', available:";
        for (const std::string& n : catalogue.names()) std::cerr << " " << n;
        std::cerr << "\n";
    }
    return opts;
}

// One world per vehicle, all with the same scenario. The app builds these
// because it is the layer that owns the catalogue -- the simulation is handed
// plain WorldConfig values and never learns that rockets come from files, which
// is the same reason `--rocket=` was always resolved here.
std::vector<rf::WorldConfig> buildRoster(const Options& opts, const rf::RocketCatalogue& catalogue) {
    std::vector<rf::WorldConfig> roster;
    roster.reserve(catalogue.all().size());
    for (const rf::RocketSpec& spec : catalogue.all()) {
        roster.push_back(opts.orbit ? rf::orbitWorld(spec) : rf::defaultWorld(spec));
    }
    return roster;
}

std::size_t rosterIndexOf(const rf::RocketCatalogue& catalogue, const std::string& name) {
    const std::vector<rf::RocketSpec>& all = catalogue.all();
    for (std::size_t i = 0; i < all.size(); ++i) {
        if (all[i].name.view() == name) return i;
    }
    return 0;   // findOrFirst's fallback, spelled as an index
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

    const rf::RocketCatalogue& catalogue = rf::RocketCatalogue::instance();
    for (const rf::RocketLoadError& e : catalogue.errors()) {
        std::cerr << "rocket load error: " << e.file << ": " << e.message << "\n";
    }
    if (catalogue.empty()) {
        std::cerr << "no rockets found in " << rf::RocketCatalogue::defaultRocketDir()
                  << " -- set $ROCKETFIGHT_ROCKETS to point at them\n";
        return EXIT_FAILURE;
    }

    const Options                      opts    = parseOptions(argc, argv, catalogue);
    const std::vector<rf::WorldConfig> roster  = buildRoster(opts, catalogue);
    const std::size_t                  startAt = rosterIndexOf(catalogue, opts.rocket);

    rf::SimulationLoop sim(roster[startAt], state, commands);
    sim.setWorldRoster(roster, startAt);   // before the thread exists, so no lock is needed
    std::thread simThread([&sim] { sim.run(); });

    sf::RenderWindow window(sf::VideoMode({1280u, 720u}), "RocketFightV2");
    window.setVerticalSyncEnabled(true);

    rf::Renderer        renderer;
    rf::Camera          camera;
    rf::InputTranslator input;

    camera.setViewportPixels(window.getSize());
    camera.setWidthMeters(framingWidth(roster[startAt]));

    bool      showInputDebug = false;
    auto      lastFrame      = Clock::now();
    auto      lastAcquire    = Clock::now();
    rf::Real  frameRate      = 0;

    // The vehicle is switched on the simulation thread, so the app learns about
    // it the only way it is allowed to: by reading the published snapshot.
    std::string flying;

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

        if (!snap.rockets.empty()) {
            camera.follow(snap.rockets.front().pos);

            // A new airframe means a fresh start, so re-frame rather than leave
            // the pilot wherever the last vehicle's zoom happened to be.
            const std::string_view name = snap.rockets.front().spec.name.view();
            if (name != flying) {
                flying = name;
                camera.setWidthMeters(framingWidth(roster[rosterIndexOf(catalogue, flying)]));
            }
        }

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
