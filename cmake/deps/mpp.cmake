find_path(MPP_INCLUDE_DIR
    NAMES rk_mpi.h
    PATHS /usr/include/rockchip /usr/local/include/rockchip /usr/include /usr/local/include
)

find_library(MPP_LIB
    NAMES rockchip_mpp rk_mpi
    PATHS /usr/lib /usr/local/lib /usr/lib/aarch64-linux-gnu
)

set(RK_DEPS_MPP_FOUND OFF)
if(MPP_INCLUDE_DIR AND MPP_LIB)
    set(RK_DEPS_MPP_FOUND ON)
endif()
