add_library(rk_core INTERFACE)
target_include_directories(rk_core INTERFACE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(rk_core INTERFACE project_warnings)
