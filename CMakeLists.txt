cmake_minimum_required(VERSION 3.24)
# C is enabled for exactly one reason: examples/submissions/simple_c is a
# submission written in plain C, and it has to be *built* as C or it proves
# nothing about the ABI being language-agnostic.
project(RocketFightV2 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# ---------------------------------------------------------------------------
# Determinism flags.
#
# These are not optional niceties: -ffast-math would let the compiler reassociate
# float arithmetic, and FP contraction (fusing a*b+c into an FMA) can differ
# between -O0 and -O2. Either would make a RelWithDebInfo run disagree with a
# Debug run, which is exactly the guarantee this project sells.
# ---------------------------------------------------------------------------
add_library(rf_determinism INTERFACE)
target_compile_options(rf_determinism INTERFACE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-ffp-contract=off>
)

add_library(rf_warnings INTERFACE)
target_compile_options(rf_warnings INTERFACE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wshadow;-Wconversion;-Wno-sign-conversion>
)

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
include(FetchContent)

set(SFML_BUILD_AUDIO   OFF CACHE BOOL "" FORCE)
set(SFML_BUILD_NETWORK OFF CACHE BOOL "" FORCE)
set(SFML_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(SFML_BUILD_DOC     OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS  OFF CACHE BOOL "" FORCE)

FetchContent_Declare(SFML
    GIT_REPOSITORY https://github.com/SFML/SFML.git
    GIT_TAG        3.0.2
    GIT_SHALLOW    ON
    SYSTEM
)

FetchContent_Declare(nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.11.3
    GIT_SHALLOW    ON
    SYSTEM
)

FetchContent_Declare(doctest
    GIT_REPOSITORY https://github.com/doctest/doctest.git
    GIT_TAG        v2.4.11
    GIT_SHALLOW    ON
    SYSTEM
)

# Header-only, no OpenSSL, no zlib: the submission server serves a JSON
# endpoint and one static page on a trusted network, and every optional feature
# is another dependency to build on a machine that only wanted to rank rockets.
set(HTTPLIB_REQUIRE_OPENSSL OFF CACHE BOOL "" FORCE)
set(HTTPLIB_REQUIRE_ZLIB    OFF CACHE BOOL "" FORCE)
set(HTTPLIB_REQUIRE_BROTLI  OFF CACHE BOOL "" FORCE)
set(HTTPLIB_COMPILE         OFF CACHE BOOL "" FORCE)

FetchContent_Declare(httplib
    GIT_REPOSITORY https://github.com/yhirose/cpp-httplib.git
    GIT_TAG        v0.18.3
    GIT_SHALLOW    ON
    SYSTEM
)

FetchContent_MakeAvailable(SFML nlohmann_json doctest httplib)

find_package(Threads REQUIRED)

# ---------------------------------------------------------------------------
# rf_core -- the simulation. Pure C++. Links nothing.
#
# The whole architecture rests on this target having no dependency on SFML,
# on threads, or on any clock. If that ever changes, the separation is gone.
# ---------------------------------------------------------------------------
add_library(rf_core STATIC
    src/core/World.cpp
    src/core/Rocket.cpp
)
target_include_directories(rf_core PUBLIC src)
target_link_libraries(rf_core PUBLIC rf_determinism PRIVATE rf_warnings)

# ---------------------------------------------------------------------------
# rf_data -- rocket definitions loaded from JSON.
#
# Separate from rf_core on purpose. Reading files and parsing JSON is I/O, and
# core's whole claim is that it does none. This target does the I/O and hands
# back plain RocketSpec values; nothing downstream can tell they came from disk.
# ---------------------------------------------------------------------------
set(ROCKETFIGHT_ROCKET_DIR "${CMAKE_CURRENT_SOURCE_DIR}/rockets")
configure_file(src/data/BuildPaths.hpp.in ${CMAKE_CURRENT_BINARY_DIR}/generated/data/BuildPaths.hpp @ONLY)

add_library(rf_data STATIC
    src/data/RocketCatalogue.cpp
)
target_include_directories(rf_data PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/generated)
target_link_libraries(rf_data PUBLIC rf_core PRIVATE nlohmann_json::nlohmann_json rf_warnings)

# ---------------------------------------------------------------------------
# rf_sync -- lock-free SPSC primitives. Header-only.
# ---------------------------------------------------------------------------
add_library(rf_sync INTERFACE)
target_include_directories(rf_sync INTERFACE src)
target_link_libraries(rf_sync INTERFACE rf_core Threads::Threads)

# ---------------------------------------------------------------------------
# rf_control -- the three control layers. Still no SFML.
# ---------------------------------------------------------------------------
#
# Benchmark.cpp is here rather than in rf_eval because rf_sim (the live game)
# and rf_eval (the headless harness) both have to drive the identical scripted
# sequence, and rf_sim links rf_control but not rf_eval. Putting it here reaches
# both without a new edge -- and without rf_sim picking up, through
# rf_eval -> rf_data, a dependency on file I/O.
add_library(rf_control STATIC
    src/control/ThrustAllocator.cpp
    src/control/RocketFlyByWire.cpp
    src/control/HumanInput.cpp
    src/control/ScriptedControllers.cpp
    src/control/Benchmark.cpp
    src/control/Registry.cpp
)
target_link_libraries(rf_control PUBLIC rf_core PRIVATE rf_warnings)

# The reference submission is a shared object that statically links this and
# rf_core, so both have to be position-independent. Nothing else changes.
set_target_properties(rf_core rf_control PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---------------------------------------------------------------------------
# rf_abi -- the submission boundary.
#
# include/rocketfight/flybywire_abi.h is the *public* header: it is the whole
# contract, it is plain C, and it is the only file a submitter needs. It is
# installed-shaped (its own include root, namespaced directory) rather than
# living in src/, because src/ is this project's internals and a submission must
# not be able to reach them by accident.
#
# Separate from rf_control so that dlopen does not become a dependency of the
# game: rocketfight links rf_control and has no business needing libdl.
# ---------------------------------------------------------------------------
add_library(rf_abi_headers INTERFACE)
target_include_directories(rf_abi_headers INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(rf_abi STATIC
    src/control/AbiFlyByWire.cpp
)
target_link_libraries(rf_abi
    PUBLIC rf_control rf_abi_headers
    PRIVATE ${CMAKE_DL_LIBS} rf_warnings)

# ---------------------------------------------------------------------------
# rf_eval -- headless episode + parallel batch runner. Still no SFML.
# ---------------------------------------------------------------------------
add_library(rf_eval STATIC
    src/eval/EpisodeRunner.cpp
    src/eval/BatchRunner.cpp
    src/eval/BenchmarkRunner.cpp
    src/eval/BenchmarkJson.cpp
)
target_link_libraries(rf_eval
    PUBLIC rf_core rf_control rf_data Threads::Threads nlohmann_json::nlohmann_json
    PRIVATE rf_warnings)

# ---------------------------------------------------------------------------
# rf_server -- the submission service: sandbox, compiler, evaluator, store.
#
# Linux-only, and unapologetically so: fork, execve, setrlimit, unshare and
# process groups have no portable spelling, and a supervisor written against
# the portable subset would be a supervisor that cannot actually kill anything.
# Nothing else in the project depends on this target.
# ---------------------------------------------------------------------------
set(ROCKETFIGHT_RUNNER_PATH "${CMAKE_BINARY_DIR}/rocketfight_runner")
set(ROCKETFIGHT_ABI_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")
configure_file(src/server/ServerPaths.hpp.in
               ${CMAKE_CURRENT_BINARY_DIR}/generated/server/ServerPaths.hpp @ONLY)

add_library(rf_server STATIC
    src/server/Sandbox.cpp
    src/server/Compile.cpp
    src/server/Evaluator.cpp
    src/server/Leaderboard.cpp
)
target_include_directories(rf_server PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/generated)
target_link_libraries(rf_server
    PUBLIC rf_core rf_control rf_data nlohmann_json::nlohmann_json
    PRIVATE rf_warnings)

# ---------------------------------------------------------------------------
# rf_sim -- the real-time paced 1 kHz loop.
# ---------------------------------------------------------------------------
add_library(rf_sim STATIC
    src/sim/SimulationLoop.cpp
)
target_link_libraries(rf_sim PUBLIC rf_core rf_control rf_sync Threads::Threads PRIVATE rf_warnings)

# ---------------------------------------------------------------------------
# rf_render -- the only library that knows SFML exists.
# ---------------------------------------------------------------------------
add_library(rf_render STATIC
    src/render/Renderer.cpp
    src/render/Camera.cpp
    src/render/InputTranslator.cpp
    src/render/FontLoader.cpp
)
target_link_libraries(rf_render PUBLIC rf_core rf_sync SFML::Graphics PRIVATE rf_warnings)

# ---------------------------------------------------------------------------
# Executables
# ---------------------------------------------------------------------------
add_executable(rocketfight src/app/main.cpp)
target_link_libraries(rocketfight PRIVATE rf_core rf_sim rf_render rf_control rf_data rf_sync rf_warnings)

add_executable(rocketfight_eval src/eval/main.cpp)
target_link_libraries(rocketfight_eval PRIVATE rf_core rf_eval rf_control rf_data rf_warnings)

# The headless benchmark. JSON on stdout, no window, no clock -- shaped for a
# submission server to drive rather than for a person to read.
add_executable(rocketfight_bench src/eval/bench_main.cpp)
target_link_libraries(rocketfight_bench PRIVATE rf_core rf_eval rf_control rf_data rf_warnings)

# One benchmark job, one process, JSON on stdout. This is what the submission
# server forks: untrusted code runs here and nowhere else, under a wall-clock
# kill it cannot influence. See src/eval/runner_main.cpp for why it is a
# separate binary rather than a thread.
add_executable(rocketfight_runner src/eval/runner_main.cpp)
target_link_libraries(rocketfight_runner PRIVATE rf_core rf_eval rf_abi rf_data rf_warnings)

# Accepts a fly-by-wire, evaluates it against every rocket in its own supervised
# process, publishes a ranking. Depends on rf_server for everything interesting;
# this file is routes, flags and one worker thread.
add_executable(rocketfight_server src/server/server_main.cpp)
target_link_libraries(rocketfight_server PRIVATE
    rf_server rf_data httplib::httplib Threads::Threads rf_warnings)
add_dependencies(rocketfight_server rocketfight_runner)

# ---------------------------------------------------------------------------
# Example submissions, built as they would be if somebody submitted them.
#
# MODULE rather than SHARED because these are dlopen targets and nothing links
# against them, and visibility is hidden so that only the five RF_EXPORT entry
# points are in the dynamic symbol table -- which is what stops the reference
# submission's statically-linked copy of rf_control from colliding with the
# runner's.
# ---------------------------------------------------------------------------
add_library(rf_example_reference MODULE examples/submissions/reference_cpp/flybywire.cpp)
target_link_libraries(rf_example_reference PRIVATE rf_control rf_abi_headers rf_warnings)

# Plain C99, linked against nothing but libm. If this ever needs more, the ABI
# has stopped being language-agnostic and has become a C++ plugin interface.
add_library(rf_example_simple_c MODULE examples/submissions/simple_c/flybywire.c)
target_link_libraries(rf_example_simple_c PRIVATE rf_abi_headers m)
target_compile_options(rf_example_simple_c PRIVATE
    $<$<C_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wshadow;-Wconversion>)

set_target_properties(rf_example_reference rf_example_simple_c PROPERTIES
    PREFIX ""
    OUTPUT_NAME flybywire
    C_VISIBILITY_PRESET hidden
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON)
set_target_properties(rf_example_reference PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/examples/reference_cpp)
set_target_properties(rf_example_simple_c PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/examples/simple_c)

# A submission that never returns from rf_resolve. Built, and built as C, for
# the same reason the working examples are: a hostile fixture that is not
# compiled by the same rules as a real submission is not testing the same path.
add_library(rf_example_hostile_spin MODULE examples/submissions/hostile_spin/flybywire.c)
target_link_libraries(rf_example_hostile_spin PRIVATE rf_abi_headers)
set_target_properties(rf_example_hostile_spin PROPERTIES
    PREFIX "" OUTPUT_NAME flybywire C_VISIBILITY_PRESET hidden
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/examples/hostile_spin)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
enable_testing()

# ABI failure-path fixtures: one source, six variants, each wrong in exactly one
# way. tests/test_submission.cpp loads them by path and asserts what the host
# does about each. They live under the test tree rather than examples/ because
# nothing here is an example of anything -- they exist to be rejected.
set(RF_FIXTURE_DIR ${CMAKE_BINARY_DIR}/tests/fixtures)
foreach(variant OK NO_RESOLVE WRONG_VERSION INIT_DECLINES SEGFAULT SLEEP NAN NONDETERMINISTIC)
    string(TOLOWER ${variant} lower)
    add_library(rf_fixture_${lower} MODULE tests/fixtures/badlib.c)
    target_link_libraries(rf_fixture_${lower} PRIVATE rf_abi_headers)
    target_compile_definitions(rf_fixture_${lower} PRIVATE RF_FIXTURE_${variant})
    set_target_properties(rf_fixture_${lower} PROPERTIES
        PREFIX "" OUTPUT_NAME ${lower} C_VISIBILITY_PRESET hidden
        LIBRARY_OUTPUT_DIRECTORY ${RF_FIXTURE_DIR})
endforeach()

configure_file(tests/TestPaths.hpp.in
               ${CMAKE_CURRENT_BINARY_DIR}/generated/tests/TestPaths.hpp @ONLY)

add_executable(rf_tests
    tests/main.cpp
    tests/test_math.cpp
    tests/test_physics.cpp
    tests/test_determinism.cpp
    tests/test_sync.cpp
    tests/test_control.cpp
    tests/test_rockets.cpp
    tests/test_actuators.cpp
    tests/test_data.cpp
    tests/test_sim.cpp
    tests/test_benchmark.cpp
    tests/test_submission.cpp
)
target_include_directories(rf_tests PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/generated)
# The fixtures are dlopened by path and the runner is forked by path, so neither
# is a link dependency -- but both have to exist before the suite runs, and a
# test that fails because CMake happened to schedule it first is a test nobody
# trusts afterwards.
add_dependencies(rf_tests rocketfight_runner
    rf_fixture_ok rf_fixture_no_resolve rf_fixture_wrong_version
    rf_fixture_init_declines rf_fixture_segfault rf_fixture_sleep rf_fixture_nan
    rf_fixture_nondeterministic)
target_link_libraries(rf_tests PRIVATE
    rf_core rf_control rf_data rf_sim rf_sync rf_eval rf_abi rf_server
    doctest::doctest Threads::Threads rf_warnings)

add_test(NAME rf_tests COMMAND rf_tests)
