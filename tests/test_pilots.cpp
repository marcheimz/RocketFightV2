#include <doctest/doctest.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "TestRockets.hpp"
#include "control/AbiFlyByWire.hpp"
#include "data/SubmissionCatalogue.hpp"
#include "tests/TestPaths.hpp"

using namespace rf;
using rf::testing::rocket;

namespace fs = std::filesystem;

namespace {

// A scratch directory that cleans up after itself, so a failing assertion does
// not leave litter behind for the next run to trip over.
struct TempDir {
    fs::path path;

    explicit TempDir(const char* tag) {
        path = fs::temp_directory_path() / (std::string("rf-pilots-") + tag);
        fs::remove_all(path);
        fs::create_directories(path);
    }
    ~TempDir() {
        std::error_code ec;
        fs::remove_all(path, ec);
    }
};

void write(const fs::path& p, const std::string& text) {
    fs::create_directories(p.parent_path());
    std::ofstream out(p);
    out << text;
}

// copy_file does not create the destination's parent, and a filesystem_error
// out of a fixture reads as a mysterious test crash rather than a setup slip.
void installLibrary(const fs::path& dest) {
    fs::create_directories(dest.parent_path());
    fs::copy_file(std::string(kFixtureDir) + "/ok.so", dest, fs::copy_options::overwrite_existing);
}

std::string okFixture() { return std::string(kFixtureDir) + "/ok.so"; }

}  // namespace

TEST_CASE("a missing directory is no submissions, not an error") {
    // The normal case for anyone who has not run the server. The built-in
    // fly-by-wire is always available, so there is nothing to report.
    CHECK(SubmissionCatalogue::load("/definitely/not/a/directory").empty());
    CHECK(SubmissionCatalogue::load({}).empty());
}

TEST_CASE("the server's own store is read, with names from the manifests") {
    TempDir dir("store");
    installLibrary(dir.path / "submissions/0002-second/artifact.so");
    write(dir.path / "submissions/0002-second/manifest.json",
          R"({"name":"second-one","description":"the later entry"})");

    installLibrary(dir.path / "submissions/0001-first/artifact.so");
    write(dir.path / "submissions/0001-first/manifest.json",
          R"({"name":"first-one","description":"the earlier entry"})");

    const SubmissionCatalogue cat = SubmissionCatalogue::load(dir.path);
    REQUIRE(cat.size() == 2);

    // Sorted, so "next submission" means the same thing every run rather than
    // whatever order the filesystem happened to hand back.
    CHECK(cat.all()[0].id == "0001-first");
    CHECK(cat.all()[0].name == "first-one");
    CHECK(cat.all()[0].description == "the earlier entry");
    CHECK(cat.all()[1].name == "second-one");

    // dlopen is handed this as-is, so a bare filename would send the loader
    // hunting its search path instead of opening the file we meant.
    CHECK(cat.all()[0].libraryPath.find('/') != std::string::npos);
}

TEST_CASE("a submission with no artifact is skipped, not half-offered") {
    TempDir dir("noartifact");
    write(dir.path / "submissions/0001-sourceonly/manifest.json", R"({"name":"sourceonly"})");
    write(dir.path / "submissions/0001-sourceonly/source.c", "int main(void){return 0;}");

    CHECK(SubmissionCatalogue::load(dir.path).empty());
}

TEST_CASE("an unreadable manifest degrades to the directory name") {
    // Names are decoration. A broken manifest must not cost you a flyable
    // controller, because the library is the part that matters.
    TempDir dir("badmanifest");
    installLibrary(dir.path / "submissions/0007-mystery/artifact.so");
    write(dir.path / "submissions/0007-mystery/manifest.json", "{ not json at all");

    const SubmissionCatalogue cat = SubmissionCatalogue::load(dir.path);
    REQUIRE(cat.size() == 1);
    CHECK(cat.all()[0].name == "0007-mystery");
}

TEST_CASE("a plain directory of loose libraries also works") {
    // For libraries you built yourself and never submitted.
    TempDir dir("loose");
    installLibrary(dir.path / "mine.so");
    write(dir.path / "notes.txt", "ignored");

    const SubmissionCatalogue cat = SubmissionCatalogue::load(dir.path);
    REQUIRE(cat.size() == 1);
    CHECK(cat.all()[0].name == "mine");
}

TEST_CASE("a submission is re-initialised when the vehicle changes") {
    // rf_init receives the vehicle spec, so a library selected on one airframe
    // and then flown on another must be loaded again against the new one.
    // Flying norcs with classic's thruster count is the bug this guards.
    std::string error;

    auto onClassic = AbiFlyByWire::load(okFixture(), rocket("classic"), error);
    REQUIRE_MESSAGE(onClassic != nullptr, error);

    // Released before the next load: the C ABI is a singleton -- rf_init and
    // friends are global functions with no instance handle -- so two live
    // instances of one library would share its state and the first destroyed
    // would call rf_shutdown out from under the second.
    onClassic.reset();

    auto onNoRcs = AbiFlyByWire::load(okFixture(), rocket("norcs"), error);
    REQUIRE_MESSAGE(onNoRcs != nullptr, error);

    // Both airframes accepted, and the second is genuinely usable rather than
    // merely constructed.
    Observation obs;
    obs.thrusterCount = rocket("norcs").count();
    obs.maxAccel      = rocket("norcs").maxForwardAccel();

    Intent intent;
    intent.mode   = Intent::Mode::Acceleration;
    intent.vector = {Real(1), Real(0)};

    const ControlInput out = onNoRcs->resolve(intent, obs);
    for (std::size_t i = 0; i < kMaxThrusters; ++i) {
        CHECK(out.level[i] >= Real(0));
        CHECK(out.level[i] <= Real(1));
    }
}
