#pragma once

#include <SFML/Graphics/Font.hpp>

#include <optional>

namespace rf {

// SFML 3 ships no default font, and there is no font file in this repo to load.
// Rather than fail, probe the usual system locations and let the HUD degrade to
// nothing if none of them exist -- a missing font should never stop the game
// from running.
std::optional<sf::Font> loadSystemFont();

}  // namespace rf
