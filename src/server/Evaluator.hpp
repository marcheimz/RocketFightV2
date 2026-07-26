#pragma once

#include <nlohmann/json.hpp>

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "control/Benchmark.hpp"
#include "server/Sandbox.hpp"

namespace rf {

using ServerJson = nlohmann::ordered_json;

// ---------------------------------------------------------------------------
// Weights are data, not code.
//
// Everything here is loaded from a JSON file the operator owns and is rewritten
// with defaults if it is missing. The numbers below are a guess made before any
// real submission existed, and they are meant to be overruled -- which is only
// possible if changing them does not mean a rebuild.
//
// Because every per-run component is archived, re-weighting an existing
// leaderboard is a re-read of stored results rather than a re-run of anything.
// ---------------------------------------------------------------------------
struct EvalWeights {
    // Within one run: how the four published metrics fold into one number.
    ScoreWeights score{};

    // Across runs. A rocket or a mode absent from these maps weighs 1.0, so
    // adding a fifth airframe does not silently drop out of the aggregate.
    std::map<std::string, double> rocket;
    std::map<std::string, double> mode;

    // What a crashed, hung or refused run costs. It has to be worse than any
    // plausible real score, or "crash on the hard vehicle" becomes a strategy.
    double failurePenalty{1000.0};

    // The task matrix. Several seeds because one seed is a single realisation
    // of a random sequence, and a controller can be lucky once.
    std::vector<std::uint64_t> seeds{0x5EEDB00C00000001ull, 0x5EEDB00C00000002ull,
                                     0x5EEDB00C00000003ull};
    double                     seconds{60.0};

    // Determinism is verified by re-running the first seed of every
    // (rocket, mode) pair in a *second process* and comparing the state hash.
    // A nondeterministic entry makes every comparison on the board meaningless,
    // so it is flagged and ranked last rather than quietly averaged in.
    bool verifyDeterminism{true};

    static EvalWeights fromJson(const ServerJson& j);
    ServerJson         toJson() const;

    double weightFor(const std::string& rocketName, const std::string& modeName) const;
};

// One (rocket, mode, seed) job: what the runner said, or why it did not say
// anything. A crash is a result, so it lives in the same struct as a success.
struct RunOutcome {
    std::string   rocket;
    std::string   mode;
    std::uint64_t seed{};

    bool        ok{false};
    std::string status;  // "ok", "crashed", "timeout", or the runner's own error
    double      score{};
    std::string stateHash;
    ServerJson  metrics;
    double      wallSeconds{};
};

struct EvaluationResult {
    bool        ok{false};
    std::string status;   // "ok" | "invalid" | "crashed" | "nondeterministic"
    std::string message;

    double aggregate{};   // lower is better; failures charged at failurePenalty
    int    runsOk{};
    int    runsFailed{};

    bool        deterministic{true};
    std::string determinismNote;

    std::vector<RunOutcome> runs;

    ServerJson toJson() const;
};

// What the evaluator needs to know about the host it is running on. All of it
// has a compiled-in default and all of it is overridable, because a server that
// can only run from one directory is a server nobody can deploy.
struct EvaluatorConfig {
    std::string   runnerPath;
    std::string   rocketDir;
    std::string   scratchDir;
    SandboxLimits limits{};
    EvalWeights   weights{};
};

// Runs the full matrix, one forked-and-supervised process per job, and folds
// the answers into a ranking-ready result. Never throws: a submission that
// misbehaves must not be able to take down the request thread that submitted
// it.
EvaluationResult evaluateSubmission(const std::string& libraryPath, const EvaluatorConfig& cfg);

// Every rocket the host will evaluate against, in the order the matrix uses.
// Read from the same catalogue the runner reads, so the server cannot advertise
// a vehicle its children cannot find.
std::vector<std::string> availableRockets(const std::string& rocketDir);

}  // namespace rf
