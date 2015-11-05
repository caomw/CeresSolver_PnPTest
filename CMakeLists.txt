cmake_minimum_required(VERSION 2.8)

project(pnp)

find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# pnp
add_executable(pnp pnp.cpp)
target_link_libraries(pnp ${CERES_LIBRARIES})
target_link_libraries(pnp ${OpenCV_LIBS} )
