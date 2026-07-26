#pragma once

#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/ConvexShape.hpp>
#include <SFML/Graphics/Font.hpp>
#include <SFML/Graphics/RectangleShape.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/Graphics/VertexArray.hpp>

#include <optional>
#include <string>

#include "core/Snapshot.hpp"
#include "render/Camera.hpp"
#include "render/InputTranslator.hpp"

namespace rf {

// Read-only view of the simulation. It receives a Snapshot and draws it; it
// cannot reach the World, cannot step it, and cannot block it.
class Renderer {
public:
    Renderer();

    // What the benchmark is doing, in the renderer's own words. Deliberately a
    // plain struct restated here rather than the control layer's type: rf_render
    // links rf_core and SFML and nothing else, and a HUD readout is not a reason
    // to change that. main() copies the fields across, and it is the layer that
    // legitimately sees both sides.
    struct Benchmark {
        bool          active{};
        bool          velocityMode{};
        Real          currentError{};
        Real          trackingErrorMean{};
        Real          settlingMean{};
        Real          impulse{};
        Real          attitudeWander{};
        Real          seconds{};
        std::uint32_t steps{};
        std::uint32_t stepsSettled{};
    };

    // Facts the renderer needs that are not part of the simulated world.
    struct Hud {
        bool        flyByWire{true};
        Real        frameRate{};
        Real        snapshotAgeMs{};
        std::string inputDevice{"none"};
        bool        showInputDebug{false};
        const InputTranslator::Debug* input{nullptr};
        Benchmark   benchmark{};
    };

    void draw(sf::RenderWindow& window, const Snapshot& snap, const Camera& camera, const Hud& hud);

private:
    static Real visualScale(const RocketView& r, const Camera& camera);

    // One pixel per unit, origin top-left, sized to the window *now*.
    //
    // Not window.getDefaultView(): SFML fixes that at window creation and its
    // resize handler only recomputes the current view's viewport, so after the
    // user drags a corner the default view still describes the old pixel size
    // and everything drawn through it comes out stretched and mispositioned.
    static sf::View screenView(const sf::RenderWindow& window);

    void drawGrid(sf::RenderWindow& window, const Camera& camera);
    void drawBounds(sf::RenderWindow& window, const Snapshot& snap);
    void drawAttractors(sf::RenderWindow& window, const Snapshot& snap);
    void drawRocket(sf::RenderWindow& window, const RocketView& r, const Camera& camera);
    void drawVectors(sf::RenderWindow& window, const RocketView& r, const Camera& camera);
    void drawActuatorPanel(sf::RenderWindow& window, const Snapshot& snap);
    void drawHud(sf::RenderWindow& window, const Snapshot& snap, const Camera& camera,
                 const Hud& hud);

    std::optional<sf::Font> font_;

    sf::VertexArray    lines_{sf::PrimitiveType::Lines};
    sf::ConvexShape    hull_{4};
    sf::ConvexShape    plume_{3};
    sf::CircleShape    disc_{1.f, 48};
    sf::RectangleShape box_{{1.f, 1.f}};
};

}  // namespace rf
