#include "data/SubmissionCatalogue.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <fstream>
#include <sstream>

namespace rf {

namespace {

namespace fs = std::filesystem;

// Names and descriptions are decoration: a submission with an unreadable or
// malformed manifest is still perfectly flyable, so a parse failure degrades to
// the directory name rather than dropping the entry.
void readManifest(const fs::path& path, SubmissionEntry& entry) {
    std::ifstream in(path);
    if (!in) return;

    std::ostringstream buffer;
    buffer << in.rdbuf();

    try {
        const nlohmann::json j = nlohmann::json::parse(buffer.str(), nullptr, true, true);
        entry.name             = j.value("name", entry.name);
        entry.description      = j.value("description", std::string{});
    } catch (const std::exception&) {
        // Keep the fallbacks already in `entry`.
    }
}

// dlopen is handed this string as-is, so it must contain a '/' or the loader
// goes hunting its search path instead of opening the file we meant.
std::string absolutePath(const fs::path& p) {
    std::error_code ec;
    const fs::path  abs = fs::absolute(p, ec);
    return ec ? p.string() : abs.lexically_normal().string();
}

}  // namespace

SubmissionCatalogue SubmissionCatalogue::load(const fs::path& dir) {
    SubmissionCatalogue out;

    std::error_code ec;
    if (dir.empty() || !fs::is_directory(dir, ec)) return out;

    // The server's store, if this is one.
    const fs::path store = dir / "submissions";
    if (fs::is_directory(store, ec)) {
        std::vector<fs::path> ids;
        for (const auto& entry : fs::directory_iterator(store, ec)) {
            if (entry.is_directory(ec)) ids.push_back(entry.path());
        }
        // Sorted, so the roster order -- and therefore what "next" means -- does
        // not depend on the order the filesystem happens to hand them back.
        std::sort(ids.begin(), ids.end());

        for (const fs::path& id : ids) {
            const fs::path library = id / "artifact.so";
            if (!fs::is_regular_file(library, ec)) continue;

            SubmissionEntry e;
            e.id          = id.filename().string();
            e.name        = e.id;
            e.libraryPath = absolutePath(library);
            readManifest(id / "manifest.json", e);
            out.entries_.push_back(std::move(e));
        }
        return out;
    }

    // Otherwise, a plain directory of libraries.
    std::vector<fs::path> libraries;
    for (const auto& entry : fs::directory_iterator(dir, ec)) {
        if (entry.is_regular_file(ec) && entry.path().extension() == ".so") {
            libraries.push_back(entry.path());
        }
    }
    std::sort(libraries.begin(), libraries.end());

    for (const fs::path& library : libraries) {
        SubmissionEntry e;
        e.id          = library.stem().string();
        e.name        = e.id;
        e.libraryPath = absolutePath(library);
        out.entries_.push_back(std::move(e));
    }
    return out;
}

}  // namespace rf
