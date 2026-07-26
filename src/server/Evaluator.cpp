#include "server/Evaluator.hpp"

#include <algorithm>
#include <map>
#include <utility>

#include "data/RocketCatalogue.hpp"

namespace rf {
namespace {

// The two experiments, spelled the way the benchmark spells them in its output.
// One spelling everywhere: the runner accepts it on the command line, emits it
// in its JSON, and the weight map is keyed by it, so there is no place for a
// translation table to fall out of step.
const char* const kModes[] = {"acceleration", "velocity"};

double lookupWeight(const std::map<std::string, double>& table, const std::string& key) {
    const auto it = table.find(key);
    // Absent means 1.0, not 0.0. A fifth rocket added as JSON must join the
    // aggregate on its own, or the first thing a new vehicle does is silently
    // not count.
    return it == table.end() ? 1.0 : it->second;
}

ServerJson parseOrDiscard(const std::string& text) {
    return ServerJson::parse(text, nullptr, /*allow_exceptions=*/false, /*ignore_comments=*/true);
}

}  // namespace

// ---------------------------------------------------------------------------
// EvalWeights
// ---------------------------------------------------------------------------

double EvalWeights::weightFor(const std::string& rocketName, const std::string& modeName) const {
    return lookupWeight(rocket, rocketName) * lookupWeight(mode, modeName);
}

EvalWeights EvalWeights::fromJson(const ServerJson& j) {
    EvalWeights w;
    if (!j.is_object()) return w;

    if (const auto s = j.find("score"); s != j.end() && s->is_object()) {
        w.score.tracking = s->value("tracking", w.score.tracking);
        w.score.settling = s->value("settling", w.score.settling);
        w.score.impulse  = s->value("impulse", w.score.impulse);
        w.score.wander   = s->value("wander", w.score.wander);
    }
    if (const auto r = j.find("rocket"); r != j.end() && r->is_object()) {
        for (const auto& [k, v] : r->items()) {
            if (v.is_number()) w.rocket[k] = v.get<double>();
        }
    }
    if (const auto m = j.find("mode"); m != j.end() && m->is_object()) {
        for (const auto& [k, v] : m->items()) {
            if (v.is_number()) w.mode[k] = v.get<double>();
        }
    }
    w.failurePenalty = j.value("failure_penalty", w.failurePenalty);
    w.seconds        = j.value("seconds", w.seconds);
    w.verifyDeterminism = j.value("verify_determinism", w.verifyDeterminism);

    if (const auto s = j.find("seeds"); s != j.end() && s->is_array() && !s->empty()) {
        w.seeds.clear();
        for (const auto& v : *s) {
            if (v.is_number_unsigned()) w.seeds.push_back(v.get<std::uint64_t>());
        }
        if (w.seeds.empty()) w.seeds.push_back(BenchmarkIntentSource::kDefaultSeed);
    }
    return w;
}

ServerJson EvalWeights::toJson() const {
    ServerJson rocketWeights = ServerJson::object();
    for (const auto& [k, v] : rocket) rocketWeights[k] = v;
    ServerJson modeWeights = ServerJson::object();
    for (const auto& [k, v] : mode) modeWeights[k] = v;

    return ServerJson{
        {"_comment",
         "Ranking policy, owned by the operator. Every per-run component is archived, so "
         "changing these re-ranks the board without re-running a single episode. Lower "
         "aggregate is better."},
        {"score",
         ServerJson{{"tracking", score.tracking},
                    {"settling", score.settling},
                    {"impulse", score.impulse},
                    {"wander", score.wander}}},
        {"rocket", rocketWeights},
        {"mode", modeWeights},
        {"failure_penalty", failurePenalty},
        {"seeds", seeds},
        {"seconds", seconds},
        {"verify_determinism", verifyDeterminism},
    };
}

// ---------------------------------------------------------------------------
// Evaluation
// ---------------------------------------------------------------------------

std::vector<std::string> availableRockets(const std::string& rocketDir) {
    const RocketCatalogue catalogue = RocketCatalogue::load(rocketDir);
    return catalogue.names();
}

namespace {

// One job, one process. Everything about the child -- the deadline, the
// rlimits, the scratch directory, the scrubbed environment -- is decided here
// and enforced by the parent, because a child cannot be trusted to limit
// itself.
RunOutcome runJob(const std::string& libraryPath, const std::string& rocketName,
                  const std::string& modeName, std::uint64_t seed, const EvaluatorConfig& cfg) {
    RunOutcome out;
    out.rocket = rocketName;
    out.mode   = modeName;
    out.seed   = seed;

    std::vector<std::string> argv{
        cfg.runnerPath,
        "--rocket=" + rocketName,
        "--mode=" + modeName,
        "--seed=" + std::to_string(seed),
        "--seconds=" + std::to_string(cfg.weights.seconds),
    };
    if (!libraryPath.empty()) argv.push_back("--library=" + libraryPath);

    // A deliberately bare environment. ROCKETFIGHT_ROCKETS is how the child
    // finds the vehicles, since its working directory is a scratch dir with no
    // ./rockets in it -- and passing it explicitly is what lets the server be
    // pointed at a different fleet without an environment variable set three
    // layers up.
    const std::vector<std::string> env{
        "PATH=/usr/bin:/bin",
        "ROCKETFIGHT_ROCKETS=" + cfg.rocketDir,
        "HOME=" + cfg.scratchDir,
        "LC_ALL=C",
    };

    const SandboxResult sr = runSandboxed(argv, cfg.scratchDir, cfg.limits, env);
    out.wallSeconds        = sr.wallSeconds;

    switch (sr.outcome) {
        case SandboxOutcome::TimedOut:
            out.status = "timeout: " + sr.describe();
            return out;
        case SandboxOutcome::Signalled:
            // The single most important line in this file. A submission that
            // segfaults is a *result* -- it gets a row on the leaderboard
            // saying so -- and not an error the server has to recover from,
            // because it happened in a process the server was already prepared
            // to lose.
            out.status = "crashed: " + sr.describe();
            return out;
        case SandboxOutcome::SpawnFailed:
            out.status = "host error: " + sr.describe();
            return out;
        case SandboxOutcome::Ok:
        case SandboxOutcome::NonZeroExit:
            break;
    }

    const ServerJson parsed = parseOrDiscard(sr.out);
    if (parsed.is_discarded() || !parsed.is_object()) {
        out.status = "runner produced no parseable JSON (" + sr.describe() + ")";
        return out;
    }
    if (const auto e = parsed.find("error"); e != parsed.end()) {
        out.status = "rejected: " + e->get<std::string>();
        return out;
    }
    if (sr.outcome != SandboxOutcome::Ok) {
        out.status = "runner " + sr.describe();
        return out;
    }

    out.ok        = true;
    out.status    = "ok";
    out.score     = parsed.value("score", 0.0);
    out.stateHash = parsed.value("state_hash", std::string{});
    if (const auto m = parsed.find("metrics"); m != parsed.end()) out.metrics = *m;
    return out;
}

}  // namespace

EvaluationResult evaluateSubmission(const std::string& libraryPath, const EvaluatorConfig& cfg) {
    EvaluationResult result;

    const std::vector<std::string> rockets = availableRockets(cfg.rocketDir);
    if (rockets.empty()) {
        result.status  = "invalid";
        result.message = "no rockets found in " + cfg.rocketDir;
        return result;
    }
    if (cfg.weights.seeds.empty()) {
        result.status  = "invalid";
        result.message = "no seeds configured";
        return result;
    }

    // Every rocket, both modes, every seed. Ranking a controller on the vehicle
    // it was tuned against is ranking the tuning.
    double weightedTotal = 0.0;
    double weightSum     = 0.0;

    for (const std::string& rocketName : rockets) {
        for (const char* modeName : kModes) {
            for (const std::uint64_t seed : cfg.weights.seeds) {
                RunOutcome run = runJob(libraryPath, rocketName, modeName, seed, cfg);

                const double w = cfg.weights.weightFor(rocketName, modeName);
                weightedTotal += w * (run.ok ? run.score : cfg.weights.failurePenalty);
                weightSum += w;

                (run.ok ? result.runsOk : result.runsFailed) += 1;
                result.runs.push_back(std::move(run));
            }
        }
    }

    result.aggregate = weightSum > 0.0 ? weightedTotal / weightSum : 0.0;

    // Determinism, checked by doing it rather than by asking. The repeat runs
    // in a brand-new process, which is what makes this test worth running: a
    // submission keyed off an address, a pid or an uninitialised byte would
    // reproduce perfectly inside one address space and fail here.
    if (cfg.weights.verifyDeterminism) {
        const std::uint64_t seed = cfg.weights.seeds.front();
        for (const RunOutcome& first : result.runs) {
            if (!first.ok || first.seed != seed) continue;

            const RunOutcome again = runJob(libraryPath, first.rocket, first.mode, seed, cfg);
            if (!again.ok || again.stateHash != first.stateHash) {
                result.deterministic = false;
                result.determinismNote = first.rocket + "/" + first.mode + ": " + first.stateHash +
                                         " then " + (again.ok ? again.stateHash : again.status);
                break;
            }
        }
    }

    if (!result.deterministic) {
        result.status = "nondeterministic";
        result.message =
            "the same configuration produced two different state hashes in two processes; "
            "a submission that does not reproduce makes every comparison on this board "
            "meaningless, so it is recorded but not ranked";
    } else if (result.runsOk == 0) {
        result.status  = "crashed";
        result.message = "no configuration completed";
    } else if (result.runsFailed > 0) {
        result.ok      = true;
        result.status  = "ok";
        result.message = std::to_string(result.runsFailed) + " of " +
                         std::to_string(result.runsOk + result.runsFailed) +
                         " runs failed and were charged the failure penalty";
    } else {
        result.ok     = true;
        result.status = "ok";
    }
    return result;
}

ServerJson EvaluationResult::toJson() const {
    ServerJson runsJson = ServerJson::array();

    // Per-rocket and per-mode means, alongside the raw runs. The aggregate is
    // one number and one number never survives contact with the question "why
    // is it bad on norcs?".
    std::map<std::string, std::pair<double, int>> byRocket;
    std::map<std::string, std::pair<double, int>> byMode;

    for (const RunOutcome& r : runs) {
        runsJson.push_back(ServerJson{
            {"rocket", r.rocket},
            {"mode", r.mode},
            {"seed", r.seed},
            {"ok", r.ok},
            {"status", r.status},
            {"score", r.score},
            {"state_hash", r.stateHash},
            {"wall_seconds", r.wallSeconds},
            {"metrics", r.metrics.is_null() ? ServerJson::object() : r.metrics},
        });
        if (!r.ok) continue;
        auto& rk = byRocket[r.rocket];
        rk.first += r.score;
        rk.second += 1;
        auto& md = byMode[r.mode];
        md.first += r.score;
        md.second += 1;
    }

    ServerJson rocketMeans = ServerJson::object();
    for (const auto& [k, v] : byRocket) rocketMeans[k] = v.second > 0 ? v.first / v.second : 0.0;
    ServerJson modeMeans = ServerJson::object();
    for (const auto& [k, v] : byMode) modeMeans[k] = v.second > 0 ? v.first / v.second : 0.0;

    return ServerJson{
        {"status", status},
        {"message", message},
        {"aggregate", aggregate},
        {"runs_ok", runsOk},
        {"runs_failed", runsFailed},
        {"deterministic", deterministic},
        {"determinism_note", determinismNote},
        {"per_rocket", rocketMeans},
        {"per_mode", modeMeans},
        {"runs", runsJson},
    };
}

}  // namespace rf
