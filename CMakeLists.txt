cmake_minimum_required(VERSION 3.25)


set(CMAKE_POLICY_VERSION_MINIMUM "3.5")

# Project name and version
project(physics-engine VERSION 1.0 LANGUAGES CXX)

# Set C++ standard (C++17 is ideal for modern game development)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Automatically Download and Setup SFML (FetchContent) ---
include(FetchContent)

FetchContent_Declare(
    sfml
    GIT_REPOSITORY https://github.com/SFML/SFML.git
    GIT_TAG        3.0.0 # Using stable 3.0.0 tag
)

# Download and include SFML in the project
# (This takes some time during the first build as it compiles SFML)
FetchContent_MakeAvailable(sfml)

# --- Source Files ---
# Find all .cpp files inside 'src' and its subdirectories recursively
file(GLOB_RECURSE SOURCES "src/*.cpp")

# --- Create Executable ---
# CRITICAL: Only use ${SOURCES}. Do NOT list files manually here if GLOB_RECURSE is used.
add_executable(${PROJECT_NAME} ${SOURCES})

# --- Include Directories ---
target_include_directories(${PROJECT_NAME} PRIVATE src)

# --- Link Libraries ---
target_link_libraries(${PROJECT_NAME} PRIVATE sfml-graphics sfml-window sfml-system)

# --- Copy Assets (Fonts/Images) ---
# Automatically copy the 'assets' folder to the build directory next to the .exe after build
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_SOURCE_DIR}/assets
    $<TARGET_FILE_DIR:${PROJECT_NAME}>/assets
)