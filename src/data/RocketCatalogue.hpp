#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "core/RocketSpec.hpp"

namespace rf {

// Loading rockets is file I/O and JSON parsing, so it lives here rather than in
// core. The rule that core touches no files is worth more than the convenience
// of putting the loader next to the struct it fills in: it is what keeps the
// simulation testable, headless and free of any notion of a filesystem.
//
// Everything below hands back plain RocketSpec values. Nothing downstream can
// tell they came from a file.

struct RocketLoadError {
    std::string file;
    std::string message;
};

// Parse one rocket definition. Returns nullopt and fills `error` on bad input --
// a malformed vehicle should be a clear message, not a silently wrong rocket.
std::optional<RocketSpec> parseRocketJson(std::string_view json, std::string& error);

std::optional<RocketSpec> loadRocketFile(const std::filesystem::path& path, std::string& error);

// Every rocket definition found, keyed by name.
class RocketCatalogue {
public:
    // Loads *.json from the given directory. If `dir` is empty, uses
    // defaultRocketDir().
    static RocketCatalogue load(const std::filesystem::path& dir = {});

    // Where rockets are looked for: $ROCKETFIGHT_ROCKETS if set, else ./rockets,
    // else the directory configured at build time. The build-time fallback is
    // what lets the binary run from anywhere without an install step.
    static std::filesystem::path defaultRocketDir();

    const RocketSpec* find(std::string_view name) const;

    // Falls back to the first entry so callers always have something to fly.
    const RocketSpec& findOrFirst(std::string_view name) const;

    const std::vector<RocketSpec>&     all() const { return specs_; }
    const std::vector<RocketLoadError>& errors() const { return errors_; }
    bool                                empty() const { return specs_.empty(); }
    std::vector<std::string>            names() const;

    // Process-wide catalogue, loaded once on first use. Callers that want a
    // specific directory construct their own instead.
    static const RocketCatalogue& instance();

private:
    std::vector<RocketSpec>     specs_;
    std::vector<RocketLoadError> errors_;
};

}  // namespace rf
