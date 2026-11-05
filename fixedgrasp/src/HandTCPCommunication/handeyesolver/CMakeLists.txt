project(handeyesolver)

cmake_minimum_required (VERSION 3.8)
# add eigen dependencies
list(APPEND CMAKE_PREFIX_PATH "/usr/include/eigen3")
find_package(Eigen3 REQUIRED)
# add thread dependencies
find_package(Threads REQUIRED)

file(GLOB sources "src/*.cpp")
add_library(handeyesolver  ${sources})
target_include_directories(handeyesolver PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(handeyesolver PUBLIC Threads::Threads Eigen3::Eigen ${catkin_LIBRARIES} ${JSONCPP_LIBRARIES})