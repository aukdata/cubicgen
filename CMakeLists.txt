# CMakeLists.txt

set(CMAKE_CXX_STANDARD 20)
# Enable all warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wall -Wextra -Wpedantic -fsanitize=address -fsanitize=leak)
elseif(MSVC)
    add_compile_options(/W4)
endif()

# Set the minimum required version of CMake
cmake_minimum_required(VERSION 3.12)

# Set the project name
project(cubicgen)

# Add the main.cpp file to the project
add_library(cubicgen STATIC cubicgen.cpp)

# Add the executable for test_anim
add_executable(test_anim main.cpp)

# Link the cubicgen library to the test_anim executable
target_link_libraries(test_anim cubicgen)
