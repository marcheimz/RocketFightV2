#include <doctest/doctest.h>

#include <string>
#include <string_view>

#include "data/RocketCatalogue.hpp"

using namespace rf;

namespace {

// A minimal valid definition, so each test can vary one thing about it.
std::string minimalRocket(const std::string& extraThrusterFields = {}) {
    return R"({
        "name": "probe",
        "mass": 500.0,
        "length": 10.0,
        "width": 2.0,
        "thrusters": [
            { "at": [-1.0, 0.0], "max_thrust": 10000.0 )" +
           extraThrusterFields + R"( }
        ]
    })";
}

}  // namespace

TEST_CASE("a minimal rocket parses") {
    std::string error;
    const auto  spec = parseRocketJson(minimalRocket(), error);

    REQUIRE_MESSAGE(spec.has_value(), error);
    CHECK(spec->name.view() == "probe");
    CHECK(spec->mass == doctest::Approx(500.0));
    CHECK(spec->count() == 1);

    // "at" is a fraction of half-length and half-width, so the tail of a 10 m
    // hull is 5 m behind the centre of mass.
    CHECK(spec->thrusters[0].mountLocal.x == doctest::Approx(-5.0));
    CHECK(spec->thrusters[0].mountLocal.y == doctest::Approx(0.0));

    // Omitted fields mean "no constraint", so a simple vehicle behaves the way
    // an idealised one always did.
    CHECK(spec->thrusters[0].minThrust == doctest::Approx(0.0));
    CHECK(spec->thrusters[0].ignitionTime == doctest::Approx(0.0));
    CHECK_FALSE(spec->thrusters[0].gimballed());
}

TEST_CASE("angles are given in degrees and converted") {
    std::string error;
    const auto  spec = parseRocketJson(minimalRocket(R"(, "direction_deg": 90.0, "gimbal_deg": 15.0,
                                                        "gimbal_rate_deg": 30.0)"),
                                       error);
    REQUIRE_MESSAGE(spec.has_value(), error);

    CHECK(spec->thrusters[0].directionLocal == doctest::Approx(kPi / 2));
    CHECK(spec->thrusters[0].maxGimbal == doctest::Approx(15.0 * kPi / 180.0));
    CHECK(spec->thrusters[0].gimbalRate == doctest::Approx(30.0 * kPi / 180.0));
}

TEST_CASE("comments are allowed, because a definition nobody can annotate rots") {
    std::string error;
    const auto  spec = parseRocketJson(R"({
        // why this vehicle exists
        "name": "commented",
        "mass": 100.0, "length": 5.0, "width": 1.0,
        "thrusters": [ { "at": [-1, 0], "max_thrust": 500.0 } ]  // the only engine
    })",
                                       error);
    REQUIRE_MESSAGE(spec.has_value(), error);
    CHECK(spec->name.view() == "commented");
}

TEST_CASE("bad definitions are rejected with a reason, not silently accepted") {
    std::string error;

    CHECK_FALSE(parseRocketJson("{ not json", error).has_value());
    CHECK_FALSE(error.empty());

    // Missing a required field.
    CHECK_FALSE(parseRocketJson(R"({"name":"x","length":1,"width":1,
                                    "thrusters":[{"at":[0,0],"max_thrust":1}]})",
                                error)
                    .has_value());
    CHECK(error.find("mass") != std::string::npos);

    // Nonsense physics.
    CHECK_FALSE(parseRocketJson(R"({"name":"x","mass":-5,"length":1,"width":1,
                                    "thrusters":[{"at":[0,0],"max_thrust":1}]})",
                                error)
                    .has_value());

    // A rocket with no way to move at all.
    CHECK_FALSE(parseRocketJson(R"({"name":"x","mass":5,"length":1,"width":1,"thrusters":[]})",
                                error)
                    .has_value());
}

TEST_CASE("thrusters carry the name the file gave them") {
    std::string error;
    const auto  spec = parseRocketJson(minimalRocket(R"(, "name": "main-stbd")"), error);
    REQUIRE_MESSAGE(spec.has_value(), error);

    CHECK(spec->thrusters[0].name.view() == "main-stbd");
}

TEST_CASE("an unnamed thruster still gets a usable name") {
    std::string error;
    const auto  spec = parseRocketJson(R"({
        "name": "anon", "mass": 100.0, "length": 5.0, "width": 1.0,
        "thrusters": [
            { "at": [-1, 0], "max_thrust": 500.0 },
            { "at": [1, 0], "max_thrust": 500.0, "name": "nose" },
            { "at": [0, 1], "max_thrust": 500.0 }
        ]
    })",
                                       error);
    REQUIRE_MESSAGE(spec.has_value(), error);

    // Positional, and by index, so the fallback name and the array subscript
    // agree -- a label reading "thruster-2" that means index 3 is worse than no
    // label at all.
    CHECK(spec->thrusters[0].name.view() == "thruster-0");
    CHECK(spec->thrusters[1].name.view() == "nose");
    CHECK(spec->thrusters[2].name.view() == "thruster-2");
}

TEST_CASE("an over-long name truncates rather than overflowing its buffer") {
    std::string error;
    const auto  spec =
        parseRocketJson(minimalRocket(R"(, "name": "an-absurdly-long-thruster-name")"), error);
    REQUIRE_MESSAGE(spec.has_value(), error);

    // The name is a fixed inline buffer because the spec is copied into every
    // snapshot; truncation is the deliberate failure mode, and it must still be
    // null-terminated.
    const std::string_view got = spec->thrusters[0].name.view();
    CHECK(got.size() < spec->thrusters[0].name.chars.size());
    CHECK(got == std::string_view("an-absurdly-long-thruster-name").substr(0, got.size()));
}

TEST_CASE("minimum thrust cannot exceed maximum") {
    std::string error;
    const auto  spec =
        parseRocketJson(minimalRocket(R"(, "min_thrust": 999999.0)"), error);
    REQUIRE_MESSAGE(spec.has_value(), error);
    CHECK(spec->thrusters[0].minThrust <= spec->thrusters[0].maxThrust);
}

TEST_CASE("the shipped catalogue loads cleanly") {
    const RocketCatalogue& catalogue = RocketCatalogue::instance();

    for (const RocketLoadError& e : catalogue.errors()) {
        FAIL_CHECK("failed to load " << e.file << ": " << e.message);
    }

    REQUIRE_FALSE(catalogue.empty());
    for (const char* expected : {"classic", "norcs", "lander", "interceptor"}) {
        INFO("expected rocket: " << expected);
        CHECK(catalogue.find(expected) != nullptr);
    }
}

TEST_CASE("every shipped rocket is physically coherent") {
    for (const RocketSpec& spec : RocketCatalogue::instance().all()) {
        INFO("rocket: " << std::string(spec.name.view()));

        CHECK(spec.mass > Real(0));
        CHECK(spec.inertia() > Real(0));
        CHECK(spec.count() > 0);
        CHECK(spec.maxForwardAccel() > Real(0));

        for (std::size_t i = 0; i < spec.count(); ++i) {
            const Thruster& t = spec[i];
            CHECK(t.maxThrust > Real(0));
            CHECK(t.minThrust >= Real(0));
            CHECK(t.minThrust < t.maxThrust);
            CHECK(t.ignitionTime >= Real(0));
            // A gimbal that cannot move is a fixed nozzle wearing a costume.
            if (t.gimballed()) CHECK(t.gimbalRate > Real(0));
        }
    }
}

TEST_CASE("every shipped thruster is named, and named distinctly") {
    for (const RocketSpec& spec : RocketCatalogue::instance().all()) {
        INFO("rocket: " << std::string(spec.name.view()));

        for (std::size_t i = 0; i < spec.count(); ++i) {
            INFO("thruster: " << i);
            CHECK_FALSE(spec[i].name.empty());

            // The panel labels a column with this, so two nozzles sharing a name
            // would make the display actively misleading.
            for (std::size_t j = i + 1; j < spec.count(); ++j) {
                CHECK(spec[i].name.view() != spec[j].name.view());
            }
        }
    }
}

TEST_CASE("an unknown name falls back rather than crashing") {
    const RocketCatalogue& catalogue = RocketCatalogue::instance();
    CHECK(catalogue.find("no-such-rocket") == nullptr);
    CHECK(catalogue.findOrFirst("no-such-rocket").count() > 0);
}
