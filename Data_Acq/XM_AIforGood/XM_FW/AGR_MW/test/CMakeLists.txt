# ============================================================
# AGR_MW DOP Core — host-side unit tests (standalone)
#
# AGR_MW 는 module_type:library (자체 펌웨어 빌드 없음). 이 프로젝트는
# 그와 독립적인 호스트 테스트로, Core(순수 C, HAL/RTOS 무의존)만 컴파일한다.
#
#   cmake -S test -B test/build
#   cmake --build test/build
#   ctest --test-dir test/build --output-on-failure
# ============================================================
cmake_minimum_required(VERSION 3.15)
project(agr_mw_dop_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(DOP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../DOP)

add_executable(test_sdo
    test_sdo_explicit_length.c
    ${DOP_DIR}/Core/agr_sdo_protocol.c
    ${DOP_DIR}/Core/agr_od.c
)
target_include_directories(test_sdo PRIVATE ${DOP_DIR} ${DOP_DIR}/Core)

if(NOT MSVC)
    target_compile_options(test_sdo PRIVATE -Wall -Wextra)
endif()

set(PNP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../PnP)

add_executable(test_pnp_identify
    test_pnp_identify.c
    ${PNP_DIR}/agr_pnp_identify.c
    ${PNP_DIR}/agr_pnp_master.c
    ${DOP_DIR}/agr_nmt.c
)
target_include_directories(test_pnp_identify PRIVATE ${DOP_DIR} ${DOP_DIR}/Core ${PNP_DIR})

if(NOT MSVC)
    target_compile_options(test_pnp_identify PRIVATE -Wall -Wextra)
endif()

add_executable(test_od_index
    test_od_index.c
    ${DOP_DIR}/Core/agr_od.c
)
target_include_directories(test_od_index PRIVATE ${DOP_DIR} ${DOP_DIR}/Core)

if(NOT MSVC)
    target_compile_options(test_od_index PRIVATE -Wall -Wextra)
endif()

enable_testing()
add_test(NAME sdo_explicit_length COMMAND test_sdo)
add_test(NAME pnp_identify COMMAND test_pnp_identify)
add_test(NAME od_sorted_index COMMAND test_od_index)
