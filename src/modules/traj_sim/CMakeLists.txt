add_library(ecl_traj_sim gvar.cpp gen_utils.cpp generator.cpp)

target_include_directories(ecl_traj_sim PUBLIC ${ECL_SOURCE_DIR})

target_link_libraries(ecl_traj_sim PRIVATE ecl)
