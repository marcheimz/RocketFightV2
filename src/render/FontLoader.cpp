#include "render/FontLoader.hpp"

#include <array>
#include <filesystem>

namespace rf {

std::optional<sf::Font> loadSystemFont() {
    // Monospace first: the HUD is a column of changing numbers, and a
    // proportional font makes them jitter sideways as digits change.
    static constexpr std::array kCandidates = {
        "/usr/share/fonts/google-noto/NotoSansMono-Regular.ttf",
        "/usr/share/fonts/dejavu-sans-mono-fonts/DejaVuSansMono.ttf",
        "/usr/share/fonts/liberation-mono-fonts/LiberationMono-Regular.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
        "/usr/share/fonts/google-noto/NotoSans-Regular.ttf",
        "/usr/share/fonts/liberation-sans-fonts/LiberationSans-Regular.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
    };

    for (const char* path : kCandidates) {
        std::error_code ec;
        if (!std::filesystem::exists(path, ec)) continue;
        sf::Font font;
        if (font.openFromFile(path)) return font;
    }
    return std::nullopt;
}

}  // namespace rf
