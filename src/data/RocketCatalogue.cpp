#include "data/RocketCatalogue.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <sstream>

#include "data/BuildPaths.hpp"

namespace rf {

namespace {

using Json = nlohmann::json;

// Thruster mounts and directions are far more readable in degrees and in
// fractions of the hull than in radians and metres, and a definition that is
// pleasant to hand-edit is the entire reason for having data files.
Real degreesToRadians(Real degrees) { return degrees * kPi / Real(180); }

Real require(const Json& obj, const char* key) {
    if (!obj.contains(key)) throw std::runtime_error(std::string("missing '") + key + "'");
    return obj.at(key).get<Real>();
}

Real optional(const Json& obj, const char* key, Real fallback) {
    return obj.contains(key) ? obj.at(key).get<Real>() : fallback;
}

Thruster parseThruster(const Json& j, const RocketSpec& spec) {
    Thruster t;

    // Mount is given as a fraction of half-length and half-width, so a thruster
    // "at the tail" stays at the tail when the hull is resized.
    const Json& at = j.at("at");
    t.mountLocal   = {at.at(0).get<Real>() * spec.length / Real(2),
                      at.at(1).get<Real>() * spec.width / Real(2)};

    t.directionLocal = degreesToRadians(optional(j, "direction_deg", Real(0)));

    t.maxThrust = require(j, "max_thrust");
    t.minThrust = clamp(optional(j, "min_thrust", Real(0)), Real(0), t.maxThrust);

    t.ignitionTime = optional(j, "ignition_time", Real(0));

    // Ramp rates default to "instant", so a simple vehicle can leave them out
    // and behave the way the old model did.
    t.rampUpRate   = optional(j, "ramp_up", Real(0));
    t.rampDownRate = optional(j, "ramp_down", t.rampUpRate);

    t.maxGimbal  = degreesToRadians(optional(j, "gimbal_deg", Real(0)));
    t.gimbalRate = degreesToRadians(optional(j, "gimbal_rate_deg", Real(0)));

    if (t.maxThrust <= Real(0)) throw std::runtime_error("max_thrust must be positive");
    return t;
}

}  // namespace

std::optional<RocketSpec> parseRocketJson(std::string_view json, std::string& error) {
    try {
        // ignore_comments: JSON has none, but a vehicle definition nobody can
        // annotate is a vehicle definition nobody will maintain.
        const Json j = Json::parse(json, nullptr, true, true);

        RocketSpec spec;
        spec.name   = RocketName::from(j.value("name", "unnamed"));
        spec.mass   = require(j, "mass");
        spec.length = require(j, "length");
        spec.width  = require(j, "width");

        if (spec.mass <= Real(0)) throw std::runtime_error("mass must be positive");
        if (spec.length <= Real(0) || spec.width <= Real(0)) {
            throw std::runtime_error("length and width must be positive");
        }

        if (!j.contains("thrusters") || !j.at("thrusters").is_array()) {
            throw std::runtime_error("missing 'thrusters' array");
        }

        for (const Json& t : j.at("thrusters")) {
            if (!spec.add(parseThruster(t, spec))) {
                throw std::runtime_error("more than " + std::to_string(kMaxThrusters) +
                                         " thrusters");
            }
        }

        if (spec.count() == 0) throw std::runtime_error("a rocket needs at least one thruster");

        error.clear();
        return spec;
    } catch (const std::exception& e) {
        error = e.what();
        return std::nullopt;
    }
}

std::optional<RocketSpec> loadRocketFile(const std::filesystem::path& path, std::string& error) {
    std::ifstream in(path);
    if (!in) {
        error = "cannot open file";
        return std::nullopt;
    }
    std::ostringstream buffer;
    buffer << in.rdbuf();
    return parseRocketJson(buffer.str(), error);
}

std::filesystem::path RocketCatalogue::defaultRocketDir() {
    if (const char* env = std::getenv("ROCKETFIGHT_ROCKETS")) {
        if (*env != '\0') return env;
    }

    std::error_code ec;
    const std::filesystem::path local = "rockets";
    if (std::filesystem::is_directory(local, ec)) return local;

    // Configured at build time so the binary runs from anywhere in the tree
    // without an install step.
    return kSourceRocketDir;
}

RocketCatalogue RocketCatalogue::load(const std::filesystem::path& dir) {
    RocketCatalogue out;

    const std::filesystem::path root = dir.empty() ? defaultRocketDir() : dir;

    std::error_code ec;
    if (!std::filesystem::is_directory(root, ec)) {
        out.errors_.push_back({root.string(), "rocket directory not found"});
        return out;
    }

    // Sorted, because a catalogue whose order depends on the filesystem would
    // make "the first rocket" -- and therefore the default vehicle -- vary
    // between machines.
    std::vector<std::filesystem::path> files;
    for (const auto& entry : std::filesystem::directory_iterator(root, ec)) {
        if (entry.is_regular_file() && entry.path().extension() == ".json") {
            files.push_back(entry.path());
        }
    }
    std::sort(files.begin(), files.end());

    for (const std::filesystem::path& file : files) {
        std::string error;
        if (auto spec = loadRocketFile(file, error)) {
            out.specs_.push_back(*spec);
        } else {
            out.errors_.push_back({file.string(), error});
        }
    }

    return out;
}

const RocketSpec* RocketCatalogue::find(std::string_view name) const {
    for (const RocketSpec& s : specs_) {
        if (s.name.view() == name) return &s;
    }
    return nullptr;
}

const RocketSpec& RocketCatalogue::findOrFirst(std::string_view name) const {
    if (const RocketSpec* found = find(name)) return *found;
    static const RocketSpec fallback{};
    return specs_.empty() ? fallback : specs_.front();
}

std::vector<std::string> RocketCatalogue::names() const {
    std::vector<std::string> out;
    out.reserve(specs_.size());
    for (const RocketSpec& s : specs_) out.emplace_back(s.name.view());
    return out;
}

const RocketCatalogue& RocketCatalogue::instance() {
    static const RocketCatalogue catalogue = load();
    return catalogue;
}

}  // namespace rf
