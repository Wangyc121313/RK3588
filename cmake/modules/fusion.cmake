add_library(rk_fusion INTERFACE)
target_include_directories(rk_fusion INTERFACE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(rk_fusion INTERFACE rk_core)
