#include <httplib.h>

#include <unistd.h>

#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "data/RocketCatalogue.hpp"
#include "server/Compile.hpp"
#include "server/Evaluator.hpp"
#include "server/Leaderboard.hpp"
#include "server/ServerPaths.hpp"

// ---------------------------------------------------------------------------
// rocketfight_server -- accept a fly-by-wire, evaluate it against every rocket,
// publish a ranking.
//
// The threading is deliberately boring: cpp-httplib's own thread pool answers
// requests, and exactly one worker thread does all the evaluating. Submissions
// are therefore evaluated one at a time, in arrival order, which is not a
// throughput decision -- a benchmark run takes 20 ms -- but a fairness one. Two
// evaluations racing for the same cores would give the second one a different
// wall-clock profile from the first, and the wall-clock deadline is the thing
// standing between this server and a submission that never returns.
// ---------------------------------------------------------------------------

namespace fs = std::filesystem;
using Json   = rf::ServerJson;

namespace {

struct Options {
    int         port{8080};
    std::string host{"0.0.0.0"};
    std::string dataDir{"server-data"};
    std::string runnerPath{rf::kDefaultRunnerPath};
    std::string includeDir{rf::kAbiIncludeDir};
    std::string rocketDir;  // empty => the catalogue's own default
    bool        allowRoot{false};
    bool        ok{true};
    std::string error;
};

bool startsWith(const std::string& s, const char* prefix) { return s.rfind(prefix, 0) == 0; }

Options parseOptions(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        try {
            if (startsWith(arg, "--port=")) {
                o.port = std::stoi(arg.substr(7));
            } else if (startsWith(arg, "--host=")) {
                o.host = arg.substr(7);
            } else if (startsWith(arg, "--data=")) {
                o.dataDir = arg.substr(7);
            } else if (startsWith(arg, "--runner=")) {
                o.runnerPath = arg.substr(9);
            } else if (startsWith(arg, "--include=")) {
                o.includeDir = arg.substr(10);
            } else if (startsWith(arg, "--rockets=")) {
                o.rocketDir = arg.substr(10);
            } else if (arg == "--allow-root") {
                o.allowRoot = true;
            } else {
                o.ok    = false;
                o.error = "unknown option '" + arg + "'";
            }
        } catch (const std::exception&) {
            o.ok    = false;
            o.error = "malformed option '" + arg + "'";
        }
    }
    return o;
}

std::string readFile(const fs::path& path) {
    std::ifstream is(path, std::ios::binary);
    if (!is) return {};
    return std::string((std::istreambuf_iterator<char>(is)), std::istreambuf_iterator<char>());
}

bool writeFile(const fs::path& path, const std::string& content) {
    std::ofstream os(path, std::ios::binary | std::ios::trunc);
    if (!os) return false;
    os.write(content.data(), static_cast<std::streamsize>(content.size()));
    return os.good();
}

// The weights file is created on first start and never overwritten afterwards.
// The operator owns it: a server that rewrote it on every boot would silently
// undo the retuning it exists to make possible.
rf::EvalWeights loadOrCreateWeights(const fs::path& path) {
    const std::string text = readFile(path);
    if (!text.empty()) {
        const Json j = Json::parse(text, nullptr, false, true);
        if (!j.is_discarded()) return rf::EvalWeights::fromJson(j);
        std::cerr << "[server] " << path << " is not valid JSON; using built-in defaults\n";
        return {};
    }
    const rf::EvalWeights defaults;
    writeFile(path, defaults.toJson().dump(2) + "\n");
    return defaults;
}

// A submission's declared language decides how the artifact is treated, so it
// is validated rather than trusted: an unknown value would otherwise fall
// through to "compile it as C" and produce a baffling error.
bool normaliseLanguage(std::string& language, const std::string& filename) {
    if (language == "c" || language == "cpp" || language == "binary") return true;
    if (language == "c++" || language == "cxx" || language == "cc") {
        language = "cpp";
        return true;
    }
    if (!language.empty()) return false;

    // Not declared: infer from the artifact's own extension, which is what a
    // submitter who forgot the field almost certainly meant.
    const fs::path p(filename);
    const std::string ext = p.extension().string();
    if (ext == ".so") language = "binary";
    else if (ext == ".c") language = "c";
    else if (ext == ".cpp" || ext == ".cc" || ext == ".cxx") language = "cpp";
    else return false;
    return true;
}

const char* const kIndexHtml = R"HTML(<!doctype html>
<html lang="en"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>RocketFight fly-by-wire leaderboard</title>
<style>
:root { color-scheme: light dark; --line:#8884; --dim:#8888; }
body { font:14px/1.5 ui-monospace,SFMono-Regular,Menlo,monospace; margin:0; padding:2rem;
       max-width:80rem; }
h1 { font-size:1.3rem; margin:0 0 .25rem; }
p.sub { color:var(--dim); margin:0 0 1.5rem; }
table { border-collapse:collapse; width:100%; }
th,td { text-align:left; padding:.4rem .6rem; border-bottom:1px solid var(--line);
        vertical-align:top; }
th { font-weight:600; color:var(--dim); font-size:.8rem; text-transform:uppercase;
     letter-spacing:.05em; }
td.n { text-align:right; font-variant-numeric:tabular-nums; }
tr.flagged td { opacity:.6; }
.desc { color:var(--dim); font-size:.85rem; max-width:32rem; }
.badge { font-size:.75rem; padding:.1rem .4rem; border:1px solid var(--line); border-radius:.2rem; }
.wrap { overflow-x:auto; }
code { background:#8881; padding:.1rem .3rem; border-radius:.2rem; }
</style></head><body>
<h1>RocketFight fly-by-wire leaderboard</h1>
<p class="sub">Lower is better. Aggregate is the weighted mean of the per-run composite over
every rocket, both modes and every seed; a failed run is charged the failure penalty.
Entries that did not reproduce are shown but not ranked.</p>
<div class="wrap"><table>
<thead><tr><th>#</th><th>name</th><th class="n">aggregate</th><th>status</th>
<th>per rocket</th><th>submitted</th></tr></thead>
<tbody id="rows"></tbody></table></div>
<p class="sub" id="foot"></p>
<script>
const fmt = x => (typeof x === "number" && isFinite(x)) ? x.toFixed(2) : "-";
async function refresh() {
  let d;
  try { d = await (await fetch("api/leaderboard")).json(); }
  catch (e) { document.getElementById("foot").textContent = "server unreachable"; return; }
  const rows = document.getElementById("rows");
  rows.textContent = "";
  d.entries.forEach((e, i) => {
    const tr = document.createElement("tr");
    if (!e.ranked) tr.className = "flagged";
    const cell = (text, cls) => {
      const td = document.createElement("td");
      if (cls) td.className = cls;
      td.textContent = text;
      return td;
    };
    tr.appendChild(cell(e.ranked ? String(i + 1) : "-", "n"));
    const nameCell = document.createElement("td");
    const strong = document.createElement("div");
    strong.textContent = e.name || e.id;
    const desc = document.createElement("div");
    desc.className = "desc";
    desc.textContent = e.description || "";
    nameCell.append(strong, desc);
    tr.appendChild(nameCell);
    tr.appendChild(cell(e.ranked ? fmt(e.aggregate) : "-", "n"));
    const st = document.createElement("td");
    const b = document.createElement("span");
    b.className = "badge";
    b.textContent = e.status;
    st.appendChild(b);
    if (e.message) { const m = document.createElement("div");
      m.className = "desc"; m.textContent = e.message; st.appendChild(m); }
    tr.appendChild(st);
    const per = Object.entries(e.per_rocket || {})
      .map(([k, v]) => k + " " + fmt(v)).join("  ");
    tr.appendChild(cell(per, "desc"));
    tr.appendChild(cell(e.submitted_at || "", "desc"));
    rows.appendChild(tr);
  });
  document.getElementById("foot").textContent =
    d.entries.length + " submissions | rockets: " + (d.rockets || []).join(", ") +
    " | seeds: " + (d.weights.seeds || []).length + " | updated " + d.generated_at;
}
refresh();
setInterval(refresh, 5000);
</script></body></html>
)HTML";

// ---------------------------------------------------------------------------
// The one worker thread. Everything untrusted happens downstream of here, in a
// forked child; this function only moves files around and reads JSON back.
// ---------------------------------------------------------------------------
class Queue {
public:
    void push(std::string id) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            ids_.push_back(std::move(id));
        }
        cv_.notify_one();
    }

    // Returns false once the queue is closed and drained, which is how the
    // worker thread learns to exit.
    bool pop(std::string& out) {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [&] { return !ids_.empty() || closed_; });
        if (ids_.empty()) return false;
        out = std::move(ids_.front());
        ids_.pop_front();
        return true;
    }

    void close() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            closed_ = true;
        }
        cv_.notify_all();
    }

private:
    std::mutex              mutex_;
    std::condition_variable cv_;
    std::deque<std::string> ids_;
    bool                    closed_{false};
};

void processSubmission(const std::string& id, rf::Leaderboard& board,
                       const rf::EvaluatorConfig& evalCfg, const std::string& includeDir) {
    auto found = board.get(id);
    if (!found) return;
    rf::SubmissionRecord record = *found;

    const fs::path dir     = board.directoryFor(id);
    const fs::path library = dir / "artifact.so";

    if (record.language != "binary") {
        record.status = "compiling";
        board.update(record);

        const std::string ext    = record.language == "cpp" ? ".cpp" : ".c";
        rf::CompileRequest req;
        req.language   = record.language;
        req.sourcePath = (dir / ("source" + ext)).string();
        req.outputPath = library.string();
        req.workDir    = evalCfg.scratchDir;
        req.includeDir = includeDir;

        const rf::CompileResult compiled = rf::compileSubmission(req, rf::defaultCompileLimits());
        record.compileLog = compiled.commandLine + "\n" + compiled.message;
        if (!compiled.ok) {
            record.status      = "rejected";
            record.message     = "compilation failed";
            record.ranked      = false;
            record.evaluatedAt = rf::isoTimestampNow();
            board.update(record);
            return;
        }
    }

    record.status = "evaluating";
    board.update(record);

    const rf::EvaluationResult result = rf::evaluateSubmission(library.string(), evalCfg);

    record.result      = result.toJson();
    record.aggregate   = result.aggregate;
    record.status      = result.status;
    record.message     = result.message;
    // Ranked only if it completed *and* reproduced. A nondeterministic entry is
    // published with its numbers so the submitter can see them, and left out of
    // the ordering so nobody else's position depends on a run that will not
    // happen the same way twice.
    record.ranked      = result.ok && result.deterministic;
    record.evaluatedAt = rf::isoTimestampNow();
    board.update(record);
}

}  // namespace

int main(int argc, char** argv) {
    const Options opts = parseOptions(argc, argv);
    if (!opts.ok) {
        std::cerr << "[server] " << opts.error << "\n";
        return EXIT_FAILURE;
    }

    // Refusing rather than warning. Every limit in Sandbox.cpp is a limit root
    // can lift, the network namespace stops being a restriction, and "never as
    // root" is the one rule in this design that is cheap to keep and expensive
    // to have broken.
    if (::geteuid() == 0 && !opts.allowRoot) {
        std::cerr << "[server] refusing to run as root: every resource limit this server "
                     "applies is one root can lift, and a submission that escapes would "
                     "escape as root. Run it as an unprivileged user, or pass --allow-root "
                     "if you genuinely know better.\n";
        return EXIT_FAILURE;
    }

    std::error_code ec;
    const fs::path  dataDir = fs::absolute(opts.dataDir, ec);
    const fs::path  scratch = dataDir / "scratch";
    fs::create_directories(scratch, ec);
    if (ec) {
        std::cerr << "[server] cannot create " << scratch << ": " << ec.message() << "\n";
        return EXIT_FAILURE;
    }

    rf::EvaluatorConfig evalCfg;
    evalCfg.runnerPath = fs::absolute(opts.runnerPath, ec).string();
    // Absolute, unconditionally, including the catalogue's own default -- which
    // is the relative path "rockets" whenever the server was started from the
    // source tree. Every child runs with the scratch directory as its cwd, so a
    // relative rocket path resolves against the wrong directory and every single
    // job comes back "unknown rocket 'classic'". Resolving it here, once,
    // against the *server's* cwd is the only place that can be got right: the
    // child has already lost the context needed to interpret it.
    evalCfg.rocketDir  = fs::absolute(opts.rocketDir.empty()
                                          ? rf::RocketCatalogue::defaultRocketDir()
                                          : fs::path(opts.rocketDir),
                                      ec)
                            .string();
    evalCfg.scratchDir = scratch.string();
    evalCfg.weights    = loadOrCreateWeights(dataDir / "weights.json");

    if (!fs::exists(evalCfg.runnerPath)) {
        std::cerr << "[server] runner not found at " << evalCfg.runnerPath
                  << " -- pass --runner=PATH\n";
        return EXIT_FAILURE;
    }

    const std::vector<std::string> rockets = rf::availableRockets(evalCfg.rocketDir);
    if (rockets.empty()) {
        std::cerr << "[server] no rockets found in " << evalCfg.rocketDir << "\n";
        return EXIT_FAILURE;
    }

    rf::Leaderboard board(dataDir);
    for (const std::string& problem : board.load()) {
        std::cerr << "[server] skipped a stored submission -- " << problem << "\n";
    }

    std::cout << "[server] data      " << dataDir << "\n"
              << "[server] runner    " << evalCfg.runnerPath << "\n"
              << "[server] rockets   " << evalCfg.rocketDir << " (" << rockets.size() << ")\n"
              << "[server] restored  " << board.size() << " submissions\n"
              << "[server] net isol. "
              << (rf::networkIsolationAvailable()
                      ? "on (unprivileged user + network namespace)"
                      : "UNAVAILABLE -- submissions will have this host's network access")
              << "\n"
              << "[server] this is hardening, not a security boundary; see SUBMISSIONS.md\n";

    Queue       queue;
    std::thread worker([&] {
        std::string id;
        while (queue.pop(id)) {
            try {
                processSubmission(id, board, evalCfg, opts.includeDir);
            } catch (const std::exception& e) {
                // The worker outliving one bad submission is the whole point of
                // catching here: a throw would end the thread and every later
                // submission would sit in "queued" forever with no explanation.
                std::cerr << "[server] evaluation of " << id << " threw: " << e.what() << "\n";
                if (auto rec = board.get(id)) {
                    rec->status  = "rejected";
                    rec->message = std::string("host error: ") + e.what();
                    rec->ranked  = false;
                    board.update(*rec);
                }
            }
        }
    });

    // Anything still "queued" from before the restart was already rewritten by
    // load() as rejected, so there is nothing to re-enqueue here -- re-running
    // it would mean re-running code the operator may have since deleted.

    httplib::Server server;
    server.set_payload_max_length(32ull * 1024 * 1024);

    server.Get("/", [](const httplib::Request&, httplib::Response& res) {
        res.set_content(kIndexHtml, "text/html; charset=utf-8");
    });

    server.Get("/api/leaderboard", [&](const httplib::Request&, httplib::Response& res) {
        Json entries = Json::array();
        for (const rf::SubmissionRecord& r : board.ranking()) entries.push_back(r.toJson(false));

        const Json out{
            {"generated_at", rf::isoTimestampNow()},
            {"benchmark", "flybywire-tracking"},
            {"abi_version", 1},
            {"rockets", rockets},
            {"modes", {"acceleration", "velocity"}},
            // Published, not just applied. Anyone can recompute this ranking
            // from the archived per-run components without re-running a thing.
            {"weights", evalCfg.weights.toJson()},
            {"entries", entries},
        };
        res.set_content(out.dump(2), "application/json");
    });

    server.Get(R"(/api/submissions/([A-Za-z0-9_.\-]+))",
               [&](const httplib::Request& req, httplib::Response& res) {
                   const auto record = board.get(req.matches[1].str());
                   if (!record) {
                       res.status = 404;
                       res.set_content(Json{{"error", "no such submission"}}.dump(2),
                                       "application/json");
                       return;
                   }
                   res.set_content(record->toJson(true).dump(2), "application/json");
               });

    server.Post("/api/submissions", [&](const httplib::Request& req, httplib::Response& res) {
        const auto reject = [&res](int status, const std::string& why) {
            res.status = status;
            res.set_content(Json{{"error", why}}.dump(2), "application/json");
        };

        if (!req.has_file("artifact")) {
            reject(400, "expected a multipart form with an 'artifact' file part");
            return;
        }
        const auto& artifact = req.get_file_value("artifact");
        if (artifact.content.empty()) {
            reject(400, "the artifact is empty");
            return;
        }

        // The manifest is read as data. Nothing about the submission is
        // discovered by loading the artifact, which is the point -- finding out
        // what a submission is called must not require running it.
        std::string name;
        std::string description;
        std::string author;
        std::string language;
        if (req.has_file("manifest")) {
            const Json j = Json::parse(req.get_file_value("manifest").content, nullptr, false, true);
            if (j.is_discarded() || !j.is_object()) {
                reject(400, "the manifest is not a JSON object");
                return;
            }
            name        = j.value("name", std::string{});
            description = j.value("description", std::string{});
            author      = j.value("author", std::string{});
            language    = j.value("language", std::string{});
            const int abi = j.value("abi_version", 1);
            if (abi != 1) {
                reject(400, "manifest declares abi_version " + std::to_string(abi) +
                                ", this host serves version 1");
                return;
            }
        }
        if (req.has_file("name")) name = req.get_file_value("name").content;
        if (req.has_file("description")) description = req.get_file_value("description").content;
        if (req.has_file("author")) author = req.get_file_value("author").content;
        if (req.has_file("language")) language = req.get_file_value("language").content;

        if (name.empty()) {
            reject(400, "a submission needs a name, in the manifest or as a form field");
            return;
        }
        if (description.empty()) {
            reject(400, "a submission needs a description; a board of anonymous numbers "
                        "teaches nobody anything");
            return;
        }
        if (!normaliseLanguage(language, artifact.filename)) {
            reject(400, "language must be one of c, cpp, binary -- or omit it and give the "
                        "artifact a .c/.cpp/.so extension");
            return;
        }

        rf::SubmissionRecord record = board.create(name, description, author, language);
        const fs::path       dir    = board.directoryFor(record.id);

        const std::string filename =
            language == "binary" ? "artifact.so" : (language == "cpp" ? "source.cpp" : "source.c");
        if (!writeFile(dir / filename, artifact.content)) {
            record.status  = "rejected";
            record.message = "could not store the artifact";
            board.update(record);
            reject(500, "could not store the artifact");
            return;
        }
        // The manifest is kept beside the artifact so the directory is
        // self-describing -- a submission archive that needs the server's
        // database to be readable is an archive nobody can inspect.
        writeFile(dir / "manifest.json", Json{{"name", name},
                                              {"description", description},
                                              {"author", author},
                                              {"language", language},
                                              {"abi_version", 1}}
                                             .dump(2) +
                                             "\n");

        queue.push(record.id);

        res.status = 202;
        res.set_content(Json{{"id", record.id},
                             {"status", "queued"},
                             {"poll", "/api/submissions/" + record.id}}
                            .dump(2),
                        "application/json");
    });

    std::cout << "[server] listening on http://" << opts.host << ":" << opts.port << "\n";
    if (!server.listen(opts.host, opts.port)) {
        std::cerr << "[server] could not bind " << opts.host << ":" << opts.port << "\n";
        queue.close();
        worker.join();
        return EXIT_FAILURE;
    }

    queue.close();
    worker.join();
    return EXIT_SUCCESS;
}
