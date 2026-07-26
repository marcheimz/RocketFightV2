#pragma once

#include <string>

#include "server/Sandbox.hpp"

namespace rf {

// Turning submitted source into a shared object.
//
// Accepting source is *less* dangerous than accepting a prebuilt .so -- the
// operator can at least read it, and diffing two submissions is a text diff --
// but it is not safe, and nothing here pretends otherwise. A compiler is a
// program that reads attacker-controlled input and writes files, and a
// `#include "/etc/passwd"` compiles as readily as anything else. So the
// compiler runs under exactly the same supervision a submission does: a
// wall-clock SIGKILL, rlimits, a scratch working directory, a scrubbed
// environment and best-effort network isolation.
//
// It is given a longer deadline and a bigger address space than an evaluation,
// because a C++ template expansion is legitimately slower and hungrier than
// sixty seconds of flying.

struct CompileRequest {
    std::string language;    // "c" or "cpp"
    std::string sourcePath;
    std::string outputPath;  // the .so to produce
    std::string workDir;     // scratch; the compiler's cwd
    std::string includeDir;  // where rocketfight/flybywire_abi.h lives
};

struct CompileResult {
    bool        ok{false};
    std::string message;     // compiler diagnostics, or why it never ran
    std::string commandLine; // published, so a submitter can reproduce the build
    double      wallSeconds{};
};

SandboxLimits defaultCompileLimits();

// Never throws. A compiler that crashes, hangs or refuses is a failed
// submission, not a failed server.
CompileResult compileSubmission(const CompileRequest& request, const SandboxLimits& limits);

}  // namespace rf
