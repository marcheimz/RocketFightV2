#pragma once

#include <filesystem>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "server/Evaluator.hpp"

namespace rf {

// One submission, from the moment it is accepted to the moment it is ranked.
//
// The name and the description come from a manifest that travels *beside* the
// artifact rather than from exported string symbols inside it. That is not a
// stylistic preference: reading a symbol out of a .so means dlopening it, and
// dlopen runs the library's constructors. A server that had to execute
// untrusted code in order to find out what it was called would have lost the
// argument before it started.
struct SubmissionRecord {
    std::string id;
    std::string name;
    std::string description;
    std::string author;
    std::string language;  // "c", "cpp", or "binary" for a prebuilt .so
    std::string submittedAt;
    std::string evaluatedAt;

    // queued -> compiling -> evaluating -> one of:
    //   ok               ranked normally
    //   nondeterministic recorded, shown, deliberately not ranked
    //   crashed          every configuration failed
    //   rejected         would not compile, or would not load
    std::string status{"queued"};
    std::string message;
    std::string compileLog;

    double aggregate{0.0};
    bool   ranked{false};

    ServerJson result;  // EvaluationResult::toJson(), or null while queued

    // `full` adds the per-run detail and the compiler log. The leaderboard
    // listing omits both, because a board with twenty entries would otherwise
    // ship a megabyte of run records to draw four columns.
    ServerJson              toJson(bool full) const;
    static SubmissionRecord fromJson(const ServerJson& j);
};

// The store and the ranking, which are the same object because a leaderboard
// that forgets on restart is a leaderboard nobody submits to twice.
//
// Everything is persisted as it happens: one directory per submission holding
// the manifest, the artifact, the source if there was any, and a record of what
// happened. Restarting rebuilds the board by reading them back.
class Leaderboard {
public:
    explicit Leaderboard(std::filesystem::path dir);

    // Reads every submission directory back into memory. Malformed records are
    // skipped with a note rather than refused: one bad directory must not cost
    // the operator the other nineteen.
    std::vector<std::string> load();

    // Allocates an id and a directory. The record is written immediately, in
    // "queued" state, so a crash between accepting and evaluating leaves
    // evidence rather than a hole.
    SubmissionRecord create(const std::string& name, const std::string& description,
                            const std::string& author, const std::string& language);

    std::filesystem::path directoryFor(const std::string& id) const;

    void update(const SubmissionRecord& record);

    std::optional<SubmissionRecord> get(const std::string& id) const;

    // Ranked first, by ascending aggregate -- lower is better. Everything that
    // could not be ranked follows, in submission order, so a crashed or
    // nondeterministic entry is visible rather than dropped. Hiding failures
    // would make the board look better than the submissions are.
    std::vector<SubmissionRecord> ranking() const;

    std::size_t size() const;

private:
    void writeRecord(const SubmissionRecord& record) const;

    mutable std::mutex                      mutex_;
    std::filesystem::path                   dir_;
    std::map<std::string, SubmissionRecord> byId_;
    int                                     nextIndex_{1};
};

// UTC, ISO 8601, second resolution. Timestamps are for humans reading the
// board; nothing in the evaluation ever reads a clock.
std::string isoTimestampNow();

// Lowercase, alphanumeric and dashes, truncated. Used to build submission
// directory names out of submitted names, which is to say out of a string an
// untrusted party chose -- so "../../etc" has to come out as "etc".
std::string slugify(const std::string& text);

}  // namespace rf
