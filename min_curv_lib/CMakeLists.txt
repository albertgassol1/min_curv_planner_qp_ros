# Set the library name
project(min_curv_lib)

find_package(Eigen3 3.3 REQUIRED)
find_package(OsqpEigen REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${OsqpEigen_INCLUDE_DIRS})

# Include the headers (public headers)
set(LIBRARY_PUBLIC_HEADERS base_cubic_spline.hpp 
                           cubic_b_spline.hpp
                           cubic_spline.hpp
                           curv_min.hpp
                           )

# Create a library target with the source files A.cpp and B.cpp
add_library(min_curv_lib STATIC base_cubic_spline.cpp
                                cubic_b_spline.cpp
                                cubic_spline.cpp
                                curv_min.cpp
                                )

# Link the library with Eigen and OsqpEigen
target_link_libraries(min_curv_lib Eigen3::Eigen OsqpEigen::OsqpEigen)

# Specify the include directories that this library exposes to the user
target_include_directories(min_curv_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Install the library and the headers
install(TARGETS min_curv_lib DESTINATION lib)
install(FILES ${LIBRARY_PUBLIC_HEADERS} DESTINATION include)
