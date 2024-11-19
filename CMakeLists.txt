cmake_minimum_required(VERSION 3.20)

# Project name and C++ standard
project(foliage_be)

set(CMAKE_CXX_STANDARD 20)

# Include directories (adjust paths as necessary)
include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/third-party
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)



# Create the library target with shared code
add_library(foliage_lib STATIC
        # Source files
        third-party/tinyxml2.cpp
        src/OSM.cpp
        src/Geometry.cpp
        src/QuadTree.cpp
        src/LayeredAStarPathfinder.cpp
        src/object.cpp
        # Add other shared source files if any
)

# Ensure the library has access to the necessary include directories
target_include_directories(foliage_lib PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/third-party
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Link any required libraries for foliage_lib (if needed)
# target_link_libraries(foliage_lib PUBLIC ...)

# Create the main executable
add_executable(foliage_be
        main.cpp
        # You can add any source files specific to the main executable here
)

# Link the main executable against the library
target_link_libraries(foliage_be PRIVATE foliage_lib)

# Ensure the main executable has access to necessary includes
target_include_directories(foliage_be PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/third-party
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Now we create the test executable
add_executable(foliage_be_tests
        src/test/LayeredAStarPathfinderTest.cpp
        src/test/QuadTreeTest.cpp
        # Add other test source files if necessary
)

# Link the test executable against the library and GoogleTest libraries
target_link_libraries(foliage_be_tests
        PRIVATE
        foliage_lib
        gtest_main
        gtest
        pthread  # Add pthread if using on Linux/macOS
)

# Ensure the test executable has access to necessary includes
target_include_directories(foliage_be_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/third-party
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_compile_options(foliage_be PRIVATE -fsanitize=undefined -g3)
target_link_options(foliage_be PRIVATE -fsanitize=undefined -g3)

target_compile_options(foliage_be_tests PRIVATE -fsanitize=undefined -g3)
target_link_options(foliage_be_tests PRIVATE -fsanitize=undefined -g3)

target_compile_options(foliage_lib PRIVATE -fsanitize=undefined -g3)
target_link_options(foliage_lib PRIVATE -fsanitize=undefined -g3)

# Enable testing
enable_testing()

# Add test discovery (automatically find all the tests in the test executable)
include(GoogleTest)
gtest_discover_tests(foliage_be_tests)
