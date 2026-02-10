cmake_minimum_required(VERSION 3.5)
project(bundle_adjustment)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Ceres REQUIRED)
find_package(Eigen3 QUIET) # optional; Ceres may have been built with Eigen

include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
)

###############
add_executable(read_and_check_json_eigen 
    test/read_n_check_json_eigen.cpp 
)
target_link_libraries(read_and_check_json_eigen 
    PRIVATE ceres
    ${EIGEN3_LIBRARIES}
)
install(TARGETS read_and_check_json_eigen 
    DESTINATION lib/${PROJECT_NAME}
)
#########

add_executable(bundle_adjustment 
    test/bundle_adjustment.cpp 
)
target_link_libraries(bundle_adjustment 
    PRIVATE ceres
    ${EIGEN3_LIBRARIES}
)
install(TARGETS bundle_adjustment 
    DESTINATION lib/${PROJECT_NAME}
)
#########

add_executable(compare_error 
    test/compare_error.cpp 
)
target_link_libraries(compare_error 
    PRIVATE ceres
    ${EIGEN3_LIBRARIES}
)
install(TARGETS compare_error 
    DESTINATION lib/${PROJECT_NAME}
)
#########

add_executable(triangulation 
    test/triangulation.cpp 
)
target_link_libraries(triangulation 
    PRIVATE ceres
    ${EIGEN3_LIBRARIES}
)
install(TARGETS triangulation 
    DESTINATION lib/${PROJECT_NAME}
)
#########

add_executable(bundle_adjustment_triangulate_world_points 
    test/bundle_adjustment_triangulate_world_points.cpp 
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
    test/bundle_adjustment_tri_wpoints_noisy_2d.cpp 
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
    test/bundle_adjustment_only_obs.cpp 
)
target_link_libraries(bundle_adjustment_only_obs 
    PRIVATE ceres
    ${EIGEN3_LIBRARIES}
)
install(TARGETS bundle_adjustment_only_obs 
    DESTINATION lib/${PROJECT_NAME}
)
#########