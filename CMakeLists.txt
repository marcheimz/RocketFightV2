cmake_minimum_required(VERSION 3.24)
project(RocketFightV2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
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

FetchContent_MakeAvailable(SFML nlohmann_json doctest)

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
add_library(rf_control STATIC
    src/control/ThrustAllocator.cpp
    src/control/RocketFlyByWire.cpp
    src/control/HumanInput.cpp
    src/control/ScriptedControllers.cpp
    src/control/Registry.cpp
)
target_link_libraries(rf_control PUBLIC rf_core PRIVATE rf_warnings)

# ---------------------------------------------------------------------------
# rf_eval -- headless episode + parallel batch runner. Still no SFML.
# ---------------------------------------------------------------------------
add_library(rf_eval STATIC
    src/eval/EpisodeRunner.cpp
    src/eval/BatchRunner.cpp
)
target_link_libraries(rf_eval PUBLIC rf_core rf_control rf_data Threads::Threads PRIVATE rf_warnings)

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

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
enable_testing()

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
)
target_link_libraries(rf_tests PRIVATE
    rf_core rf_control rf_data rf_sim rf_sync rf_eval doctest::doctest Threads::Threads rf_warnings)

add_test(NAME rf_tests COMMAND rf_tests)
