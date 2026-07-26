#pragma once

#include <nlohmann/json.hpp>

#include <cstdint>
#include <string>

#include "eval/BenchmarkRunner.hpp"

namespace rf {

// ordered_json, not json: the default keeps keys in a std::map, which sorts
// them and puts "runs" in the middle of the header. Parsers do not care, but a
// human checking a submission by eye does, and it costs nothing.
using BenchJson = nlohmann::ordered_json;

// A uint64 as a hex string, not a JSON number. JSON's number type is a double
// almost everywhere it is parsed, and a 64-bit hash does not survive that --
// which would quietly turn the determinism check into a comparison of two
// rounded values that always agree.
std::string hex64(std::uint64_t v);

// One run, in the shape rocketfight_bench has always emitted. Shared rather
// than duplicated because rocketfight_runner's output is parsed by the
// submission server: if the two spellings drifted, the server would silently
// start reading zeros out of a run that succeeded.
BenchJson toJson(const BenchmarkRun& run, const ScoreWeights& weights);

// The task definition, so an archived result can be re-read without the binary
// that produced it. Seed and duration travel with it because they are the two
// parts of "what was run" that are not in BenchmarkConfig.
BenchJson toJson(const BenchmarkConfig& config, std::uint64_t seed, Real seconds);
BenchJson toJson(const ScoreWeights& weights);

}  // namespace rf
