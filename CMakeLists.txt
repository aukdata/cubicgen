# CMakeLists.txt

# Set the minimum required version of CMake
cmake_minimum_required(VERSION 3.12)

# Set the project name
project(cubicgen)

# Set the C++ standard to C++20
set(CMAKE_CXX_STANDARD 20)

# Enable all warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wall -Wextra -Wpedantic)
    add_link_options(-fsanitize=address -fsanitize=leak)
elseif(MSVC)
    add_compile_options(/W4)
endif()

# Add the executable for sample
add_executable(sample sample.cpp)
