cmake_minimum_required(VERSION 3.5)
project(bundle_adjustment)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Ceres REQUIRED)
find_package(Eigen3) 

include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
)
#########
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

add_executable(bundle_adjustment_triangulate_world_points 
    bundle_adjustment_triangulate_world_points.cpp 
)
target_link_libraries(bundle_adjustment_triangulate_world_points 
    PRIVATE ceres
    ${EIGEN3_LIBRARIES}
)
install(TARGETS bundle_adjustment_triangulate_world_points 
    DESTINATION lib/${PROJECT_NAME}
)
#########

add_executable(bundle_adjustment_tri_wpoints_noisy_2d 
    bundle_adjustment_tri_wpoints_noisy_2d.cpp 
)
target_link_libraries(bundle_adjustment_tri_wpoints_noisy_2d 
    PRIVATE ceres
    ${EIGEN3_LIBRARIES}
)
install(TARGETS bundle_adjustment_tri_wpoints_noisy_2d 
    DESTINATION lib/${PROJECT_NAME}
)
#########
add_executable(bundle_adjustment_only_obs 
    bundle_adjustment_only_obs.cpp 
)
target_link_libraries(bundle_adjustment_only_obs 
    PRIVATE ceres
    ${EIGEN3_LIBRARIES}
)
install(TARGETS bundle_adjustment_only_obs 
    DESTINATION lib/${PROJECT_NAME}
)
#########