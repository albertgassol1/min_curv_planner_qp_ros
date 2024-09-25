# Set the example name
project(examples)

find_package(Boost 1.65 REQUIRED COMPONENTS program_options)
if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
else()
    message(FATAL_ERROR "Boost not found. Please install Boost and try again.")
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../bin)

# Add an executable for the example
add_executable(example example.cpp)

# Link the example with the library

target_link_libraries(example PRIVATE ${Boost_LIBRARIES} min_curv_lib)

# Set the include directories for the example (where to find A.hpp and B.hpp)
target_include_directories(example PRIVATE ${CMAKE_SOURCE_DIR}/library)
