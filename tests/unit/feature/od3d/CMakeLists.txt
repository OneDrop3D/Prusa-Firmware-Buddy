add_executable(pairing_od3d_tests ${CMAKE_CURRENT_SOURCE_DIR}/pairing_od3d_test.cpp ${CMAKE_SOURCE_DIR}/src/feature/od3d/pairing_od3d.cpp)
target_include_directories(
  pairing_od3d_tests PUBLIC ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/src/common
                            ${CMAKE_SOURCE_DIR}/src/module/random/random/include
  )
add_catch_test(pairing_od3d_tests)

add_executable(
  spool_draw_od3d_tests ${CMAKE_CURRENT_SOURCE_DIR}/spool_draw_test.cpp
                        ${CMAKE_SOURCE_DIR}/src/gui/od3d/spool_draw.cpp
  )
target_include_directories(spool_draw_od3d_tests PUBLIC ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/src/common)
target_link_libraries(spool_draw_od3d_tests buddy_utils)
add_catch_test(spool_draw_od3d_tests)
