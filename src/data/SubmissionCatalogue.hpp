#pragma once

#include <filesystem>
#include <string>
#include <vector>

namespace rf {

// One submitted fly-by-wire that can be flown by hand.
struct SubmissionEntry {
    std::string id;           // "0001-codex-baseline", or the file stem
    std::string name;         // from the manifest, falling back to the id
    std::string description;
    std::string libraryPath;  // absolute, and always contains a '/'
};

// Finds shared objects that can be dropped into the game as layer 2.
//
// Like RocketCatalogue, this is file I/O and therefore lives in data/ rather
// than core/ -- and, unlike the rocket catalogue, it is read once by the app
// before the simulation thread starts. The loop never learns that submissions
// come from a filesystem; it is handed a list of names and paths.
//
// Two layouts are accepted, because there are two places a library plausibly
// comes from:
//
//   <dir>/submissions/<id>/artifact.so   -- the server's own store, with a
//                                           manifest.json beside it for names
//   <dir>/*.so                           -- a directory of loose libraries you
//                                           built yourself and never submitted
class SubmissionCatalogue {
public:
    // Never throws and never reports an error: a missing directory simply means
    // no submissions, which is the normal case for someone who has not run the
    // server. The built-in fly-by-wire is always available regardless.
    static SubmissionCatalogue load(const std::filesystem::path& dir);

    const std::vector<SubmissionEntry>& all() const { return entries_; }
    bool                                empty() const { return entries_.empty(); }
    std::size_t                         size() const { return entries_.size(); }

private:
    std::vector<SubmissionEntry> entries_;
};

}  // namespace rf
