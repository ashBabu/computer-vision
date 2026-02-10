cmake_minimum_required(VERSION 3.5)
project(ba_uni_washington)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Ceres REQUIRED)
find_package(Eigen3)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
)

add_executable(bundle_adjustment 
    bundle_adjustment.cpp 
)
target_link_libraries(bundle_adjustment 
    PRIVATE ceres
    ${EIGEN3_LIBRARIES}
)
install(TARGETS bundle_adjustment 
    DESTINATION lib/${PROJECT_NAME}
)
#########