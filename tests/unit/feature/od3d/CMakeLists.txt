add_executable(pairing_od3d_tests ${CMAKE_CURRENT_SOURCE_DIR}/pairing_od3d_test.cpp ${CMAKE_SOURCE_DIR}/src/feature/od3d/pairing_od3d.cpp)
target_include_directories(
  pairing_od3d_tests PUBLIC ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/src/common
                            ${CMAKE_SOURCE_DIR}/src/module/random/random/include
  )
add_catch_test(pairing_od3d_tests)
