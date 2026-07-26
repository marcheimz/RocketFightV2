#pragma once

#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/ConvexShape.hpp>
#include <SFML/Graphics/Font.hpp>
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

    // Facts the renderer needs that are not part of the simulated world.
    struct Hud {
        bool        flyByWire{true};
        Real        frameRate{};
        Real        snapshotAgeMs{};
        std::string inputDevice{"none"};
        bool        showInputDebug{false};
        const InputTranslator::Debug* input{nullptr};
    };

    void draw(sf::RenderWindow& window, const Snapshot& snap, const Camera& camera, const Hud& hud);

private:
    static Real visualScale(const RocketView& r, const Camera& camera);

    void drawGrid(sf::RenderWindow& window, const Camera& camera);
    void drawBounds(sf::RenderWindow& window, const Snapshot& snap);
    void drawAttractors(sf::RenderWindow& window, const Snapshot& snap);
    void drawRocket(sf::RenderWindow& window, const RocketView& r, const Camera& camera);
    void drawVectors(sf::RenderWindow& window, const RocketView& r, const Camera& camera);
    void drawHud(sf::RenderWindow& window, const Snapshot& snap, const Camera& camera,
                 const Hud& hud);

    std::optional<sf::Font> font_;

    sf::VertexArray lines_{sf::PrimitiveType::Lines};
    sf::ConvexShape hull_{4};
    sf::ConvexShape plume_{3};
    sf::CircleShape disc_{1.f, 48};
};

}  // namespace rf
