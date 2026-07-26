#include "server/Leaderboard.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <utility>

namespace fs = std::filesystem;

namespace rf {

std::string isoTimestampNow() {
    const auto now  = std::chrono::system_clock::now();
    const auto secs = std::chrono::system_clock::to_time_t(now);
    std::tm    tm{};
    ::gmtime_r(&secs, &tm);

    std::ostringstream os;
    os << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
    return os.str();
}

std::string slugify(const std::string& text) {
    std::string out;
    out.reserve(text.size());
    for (const char c : text) {
        const auto u = static_cast<unsigned char>(c);
        if (std::isalnum(u) != 0) {
            out.push_back(static_cast<char>(std::tolower(u)));
        } else if (!out.empty() && out.back() != '-') {
            out.push_back('-');
        }
        if (out.size() >= 40) break;
    }
    while (!out.empty() && out.back() == '-') out.pop_back();
    // Everything that is not [a-z0-9-] is gone by construction, so a name of
    // "../../etc/passwd" arrives here as "etc-passwd" and a name of "..." as
    // the empty string. The caller's id prefix makes the empty case harmless.
    return out;
}

// ---------------------------------------------------------------------------
// SubmissionRecord
// ---------------------------------------------------------------------------

ServerJson SubmissionRecord::toJson(bool full) const {
    ServerJson j{
        {"id", id},
        {"name", name},
        {"description", description},
        {"author", author},
        {"language", language},
        {"submitted_at", submittedAt},
        {"evaluated_at", evaluatedAt},
        {"status", status},
        {"message", message},
        {"aggregate", aggregate},
        {"ranked", ranked},
    };

    if (!result.is_null()) {
        // The summary a listing needs, always. The rest only when asked.
        j["deterministic"] = result.value("deterministic", true);
        j["runs_ok"]       = result.value("runs_ok", 0);
        j["runs_failed"]   = result.value("runs_failed", 0);
        j["per_rocket"]    = result.value("per_rocket", ServerJson::object());
        j["per_mode"]      = result.value("per_mode", ServerJson::object());
    }
    if (full) {
        j["compile_log"] = compileLog;
        j["result"]      = result;
    }
    return j;
}

SubmissionRecord SubmissionRecord::fromJson(const ServerJson& j) {
    SubmissionRecord r;
    r.id          = j.value("id", std::string{});
    r.name        = j.value("name", std::string{});
    r.description = j.value("description", std::string{});
    r.author      = j.value("author", std::string{});
    r.language    = j.value("language", std::string{});
    r.submittedAt = j.value("submitted_at", std::string{});
    r.evaluatedAt = j.value("evaluated_at", std::string{});
    r.status      = j.value("status", std::string{"queued"});
    r.message     = j.value("message", std::string{});
    r.compileLog  = j.value("compile_log", std::string{});
    r.aggregate   = j.value("aggregate", 0.0);
    r.ranked      = j.value("ranked", false);
    if (const auto it = j.find("result"); it != j.end()) r.result = *it;
    return r;
}

// ---------------------------------------------------------------------------
// Leaderboard
// ---------------------------------------------------------------------------

Leaderboard::Leaderboard(fs::path dir) : dir_(std::move(dir)) {
    std::error_code ec;
    fs::create_directories(dir_ / "submissions", ec);
}

fs::path Leaderboard::directoryFor(const std::string& id) const {
    return dir_ / "submissions" / id;
}

void Leaderboard::writeRecord(const SubmissionRecord& record) const {
    const fs::path path = directoryFor(record.id) / "record.json";
    std::error_code ec;
    fs::create_directories(path.parent_path(), ec);

    // Written to a temporary and renamed, because a record half-written when
    // the process dies is a submission that never loads again -- and the whole
    // reason this is on disk is to survive a restart.
    const fs::path tmp = path.string() + ".tmp";
    {
        std::ofstream os(tmp, std::ios::binary | std::ios::trunc);
        if (!os) return;
        os << record.toJson(/*full=*/true).dump(2) << "\n";
    }
    fs::rename(tmp, path, ec);
}

std::vector<std::string> Leaderboard::load() {
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<std::string> problems;
    std::error_code          ec;
    const fs::path           root = dir_ / "submissions";
    if (!fs::exists(root, ec)) return problems;

    std::vector<fs::path> dirs;
    for (const auto& entry : fs::directory_iterator(root, ec)) {
        if (entry.is_directory(ec)) dirs.push_back(entry.path());
    }
    // Sorted, so restarting produces the same ordering as the run before it.
    // Directory iteration order is a filesystem detail and must not leak into
    // anything a person compares between two runs.
    std::sort(dirs.begin(), dirs.end());

    for (const fs::path& d : dirs) {
        const fs::path recordPath = d / "record.json";
        std::ifstream  is(recordPath, std::ios::binary);
        if (!is) {
            problems.push_back(d.filename().string() + ": no record.json");
            continue;
        }
        const std::string text((std::istreambuf_iterator<char>(is)),
                               std::istreambuf_iterator<char>());
        const ServerJson  j = ServerJson::parse(text, nullptr, false, true);
        if (j.is_discarded() || !j.is_object()) {
            problems.push_back(d.filename().string() + ": unparseable record.json");
            continue;
        }
        SubmissionRecord r = SubmissionRecord::fromJson(j);
        if (r.id.empty()) r.id = d.filename().string();

        // A submission that was mid-evaluation when the server died is not
        // queued any more -- nothing will pick it up -- so it is marked rather
        // than left claiming to be in progress forever.
        if (r.status == "queued" || r.status == "compiling" || r.status == "evaluating") {
            r.status  = "rejected";
            r.message = "the server restarted while this submission was being evaluated";
            r.ranked  = false;
        }

        // Ids begin with a zero-padded index; recovering the counter from the
        // highest one is what stops a restart from reusing a directory.
        try {
            nextIndex_ = std::max(nextIndex_, std::stoi(r.id.substr(0, 4)) + 1);
        } catch (const std::exception&) {
            // An id that does not start with a number is fine; it just does not
            // contribute to the counter.
        }
        byId_.emplace(r.id, std::move(r));
    }
    return problems;
}

SubmissionRecord Leaderboard::create(const std::string& name, const std::string& description,
                                     const std::string& author, const std::string& language) {
    std::lock_guard<std::mutex> lock(mutex_);

    std::ostringstream index;
    index << std::setw(4) << std::setfill('0') << nextIndex_++;

    const std::string slug = slugify(name);
    SubmissionRecord  r;
    r.id          = slug.empty() ? index.str() : index.str() + "-" + slug;
    r.name        = name;
    r.description = description;
    r.author      = author;
    r.language    = language;
    r.submittedAt = isoTimestampNow();
    r.status      = "queued";

    std::error_code ec;
    fs::create_directories(directoryFor(r.id), ec);
    writeRecord(r);

    byId_[r.id] = r;
    return r;
}

void Leaderboard::update(const SubmissionRecord& record) {
    std::lock_guard<std::mutex> lock(mutex_);
    byId_[record.id] = record;
    writeRecord(record);
}

std::optional<SubmissionRecord> Leaderboard::get(const std::string& id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto                  it = byId_.find(id);
    if (it == byId_.end()) return std::nullopt;
    return it->second;
}

std::vector<SubmissionRecord> Leaderboard::ranking() const {
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<SubmissionRecord> out;
    out.reserve(byId_.size());
    for (const auto& [id, r] : byId_) out.push_back(r);

    std::stable_sort(out.begin(), out.end(),
                     [](const SubmissionRecord& a, const SubmissionRecord& b) {
                         if (a.ranked != b.ranked) return a.ranked;
                         if (!a.ranked) return a.id < b.id;
                         if (a.aggregate != b.aggregate) return a.aggregate < b.aggregate;
                         return a.id < b.id;
                     });
    return out;
}

std::size_t Leaderboard::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return byId_.size();
}

}  // namespace rf
