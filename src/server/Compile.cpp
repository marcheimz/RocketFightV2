#include "server/Compile.hpp"

#include <vector>

#include "server/ServerPaths.hpp"

namespace rf {

SandboxLimits defaultCompileLimits() {
    SandboxLimits limits;
    limits.wallSeconds       = 60.0;
    limits.cpuSeconds        = 45;
    limits.addressSpaceBytes = 4096ull << 20;  // template expansion is hungry
    limits.fileSizeBytes     = 64ull << 20;
    limits.openFiles         = 256;
    limits.processes         = 64;  // the driver forks cc1 and the linker
    limits.maxOutputBytes    = 1ull << 20;
    return limits;
}

CompileResult compileSubmission(const CompileRequest& request, const SandboxLimits& limits) {
    CompileResult result;

    const bool isCpp = request.language == "cpp" || request.language == "c++" ||
                       request.language == "cxx";
    const char* compiler = isCpp ? kCxxCompiler : kCCompiler;

    std::vector<std::string> argv{
        compiler,
        isCpp ? "-std=c++20" : "-std=c99",
        "-O2",
        // The same flag the whole project is built with, for the same reason:
        // fusing a*b+c into an FMA changes results between optimisation levels,
        // and a submission whose score depends on how the server felt like
        // compiling it is not a submission that can be ranked.
        "-ffp-contract=off",
        "-fPIC",
        "-shared",
        // Only the five RF_EXPORT entry points reach the dynamic symbol table.
        // Everything else stays private, which is what keeps a submission that
        // statically links a helper library from colliding with the runner.
        "-fvisibility=hidden",
        "-Wall",
        "-Wextra",
        "-I" + request.includeDir,
        request.sourcePath,
        "-o",
        request.outputPath,
        "-lm",
    };

    for (std::size_t i = 0; i < argv.size(); ++i) {
        result.commandLine += (i > 0 ? " " : "") + argv[i];
    }

    // A minimal environment. The compiler needs a PATH to find its own
    // subprograms on some installations, and nothing else the server happens to
    // have been started with.
    const std::vector<std::string> env{
        "PATH=/usr/bin:/bin",
        "HOME=" + request.workDir,
        "TMPDIR=" + request.workDir,
        "LC_ALL=C",
    };

    const SandboxResult sr = runSandboxed(argv, request.workDir, limits, env);
    result.wallSeconds     = sr.wallSeconds;

    // Warnings on a successful build are worth keeping: a submitter who never
    // sees them will never fix them, and -Wall on somebody else's code is the
    // cheapest review there is.
    result.message = sr.err.empty() ? sr.out : sr.err;
    if (!sr.ok()) {
        result.ok = false;
        if (result.message.empty()) result.message = sr.describe();
        else result.message = sr.describe() + "\n" + result.message;
        return result;
    }

    result.ok = true;
    return result;
}

}  // namespace rf
