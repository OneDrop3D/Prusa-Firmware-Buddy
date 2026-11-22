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

add_executable(
  print_info_od3d_tests ${CMAKE_CURRENT_SOURCE_DIR}/print_info_test.cpp
                        ${CMAKE_SOURCE_DIR}/src/gui/od3d/print_info_od3d.cpp
  )
target_include_directories(print_info_od3d_tests PUBLIC ${CMAKE_SOURCE_DIR}/src)
add_catch_test(print_info_od3d_tests)

add_executable(text_od3d_tests ${CMAKE_CURRENT_SOURCE_DIR}/text_od3d_test.cpp)
target_include_directories(text_od3d_tests PUBLIC ${CMAKE_SOURCE_DIR}/src)
add_catch_test(text_od3d_tests)

add_executable(
  draw_od3d_tests
  ${CMAKE_CURRENT_SOURCE_DIR}/draw_od3d_test.cpp ${CMAKE_SOURCE_DIR}/src/gui/od3d/thumbnail_od3d.cpp
  ${CMAKE_SOURCE_DIR}/src/gui/qoi_decoder.cpp ${CMAKE_SOURCE_DIR}/src/common/utils/color.cpp
  ${CMAKE_SOURCE_DIR}/src/common/str_utils.cpp
  )
target_include_directories(
  draw_od3d_tests
  PUBLIC ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/src/common ${CMAKE_SOURCE_DIR}/src/guiapi/include
         ${CMAKE_SOURCE_DIR}/src/lang ${CMAKE_SOURCE_DIR}/include
  )
target_link_libraries(draw_od3d_tests buddy_utils byte_utils)
add_catch_test(draw_od3d_tests)

add_executable(
  skew_od3d_tests ${CMAKE_CURRENT_SOURCE_DIR}/skew_od3d_test.cpp ${CMAKE_SOURCE_DIR}/src/feature/od3d/skew_od3d.cpp
  )
target_include_directories(skew_od3d_tests PUBLIC ${CMAKE_SOURCE_DIR}/src)
add_catch_test(skew_od3d_tests)
add_executable(
  music_od3d_tests ${CMAKE_CURRENT_SOURCE_DIR}/music_od3d_test.cpp
                   ${CMAKE_SOURCE_DIR}/src/feature/od3d/music_od3d.cpp
  )
target_include_directories(music_od3d_tests PUBLIC ${CMAKE_SOURCE_DIR}/src)
add_catch_test(music_od3d_tests)
add_executable(
  spool_usage_od3d_tests ${CMAKE_CURRENT_SOURCE_DIR}/spool_usage_test.cpp
                         ${CMAKE_SOURCE_DIR}/src/feature/od3d/spool_usage_math_od3d.cpp
  )
target_include_directories(spool_usage_od3d_tests PUBLIC ${CMAKE_SOURCE_DIR}/src)
add_catch_test(spool_usage_od3d_tests)
add_executable(
  belt_tension_od3d_tests ${CMAKE_CURRENT_SOURCE_DIR}/belt_tension_test.cpp
                          ${CMAKE_SOURCE_DIR}/src/feature/od3d/belt_tension_math_od3d.cpp
  )
target_include_directories(belt_tension_od3d_tests PUBLIC ${CMAKE_SOURCE_DIR}/src)
add_catch_test(belt_tension_od3d_tests)
add_executable(
  speed_map_od3d_tests ${CMAKE_CURRENT_SOURCE_DIR}/speed_map_test.cpp
                       ${CMAKE_SOURCE_DIR}/src/feature/od3d/speed_map_math_od3d.cpp
  )
target_include_directories(speed_map_od3d_tests PUBLIC ${CMAKE_SOURCE_DIR}/src)
add_catch_test(speed_map_od3d_tests)
add_executable(
  damping_od3d_tests
  ${CMAKE_CURRENT_SOURCE_DIR}/damping_test.cpp ${CMAKE_SOURCE_DIR}/src/feature/od3d/damping_math_od3d.cpp
  ${CMAKE_SOURCE_DIR}/src/feature/od3d/belt_tension_math_od3d.cpp
  )
target_include_directories(damping_od3d_tests PUBLIC ${CMAKE_SOURCE_DIR}/src)
add_catch_test(damping_od3d_tests)
add_executable(
  pa_math_od3d_tests
  ${CMAKE_CURRENT_SOURCE_DIR}/pa_math_test.cpp ${CMAKE_SOURCE_DIR}/src/feature/od3d/pa_math_od3d.cpp
  ${CMAKE_SOURCE_DIR}/src/feature/od3d/force_trace_od3d.cpp ${CMAKE_SOURCE_DIR}/src/feature/od3d/pattern_math_od3d.cpp
  )
target_include_directories(pa_math_od3d_tests PUBLIC ${CMAKE_SOURCE_DIR}/src)
add_catch_test(pa_math_od3d_tests)
add_executable(
  mpc_od3d_tests ${CMAKE_CURRENT_SOURCE_DIR}/mpc_od3d_test.cpp ${CMAKE_SOURCE_DIR}/src/feature/od3d/mpc_od3d.cpp
  )
target_include_directories(mpc_od3d_tests PUBLIC ${CMAKE_SOURCE_DIR}/src)
add_catch_test(mpc_od3d_tests)
add_executable(
  chamber_climate_od3d_tests ${CMAKE_CURRENT_SOURCE_DIR}/chamber_climate_test.cpp
                             ${CMAKE_SOURCE_DIR}/src/feature/od3d/chamber_climate_math_od3d.cpp
  )
target_include_directories(chamber_climate_od3d_tests PUBLIC ${CMAKE_SOURCE_DIR}/src)
add_catch_test(chamber_climate_od3d_tests)
add_executable(
  clean_resume_od3d_tests ${CMAKE_CURRENT_SOURCE_DIR}/clean_resume_test.cpp
                          ${CMAKE_SOURCE_DIR}/src/feature/od3d/clean_resume_math_od3d.cpp
  )
target_include_directories(clean_resume_od3d_tests PUBLIC ${CMAKE_SOURCE_DIR}/src)
add_catch_test(clean_resume_od3d_tests)
add_executable(
  spool_calib_od3d_tests
  ${CMAKE_CURRENT_SOURCE_DIR}/spool_calib_math_test.cpp ${CMAKE_SOURCE_DIR}/src/feature/od3d/spool_calib_math_od3d.cpp
  ${CMAKE_SOURCE_DIR}/src/feature/od3d/pa_math_od3d.cpp ${CMAKE_SOURCE_DIR}/src/feature/od3d/force_trace_od3d.cpp
  ${CMAKE_SOURCE_DIR}/src/feature/od3d/pattern_math_od3d.cpp
  )
target_include_directories(spool_calib_od3d_tests PUBLIC ${CMAKE_SOURCE_DIR}/src)
add_catch_test(spool_calib_od3d_tests)
add_executable(
  lookahead_od3d_tests ${CMAKE_CURRENT_SOURCE_DIR}/lookahead_od3d_test.cpp
                       ${CMAKE_SOURCE_DIR}/src/feature/od3d/lookahead_math_od3d.cpp
  )
target_include_directories(lookahead_od3d_tests PUBLIC ${CMAKE_SOURCE_DIR}/src)
add_catch_test(lookahead_od3d_tests)

add_executable(island_face_od3d_tests ${CMAKE_CURRENT_SOURCE_DIR}/island_face_test.cpp)
target_include_directories(island_face_od3d_tests PUBLIC ${CMAKE_SOURCE_DIR}/src)
add_catch_test(island_face_od3d_tests)

add_executable(gesture_od3d_tests ${CMAKE_CURRENT_SOURCE_DIR}/gesture_od3d_test.cpp)
target_include_directories(
  gesture_od3d_tests PUBLIC ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/src/common ${CMAKE_SOURCE_DIR}/src/guiapi/include
  )
add_catch_test(gesture_od3d_tests)

add_executable(
  spinner_od3d_tests ${CMAKE_CURRENT_SOURCE_DIR}/spinner_od3d_test.cpp ${CMAKE_SOURCE_DIR}/src/gui/od3d/spinner_od3d.cpp
                     ${CMAKE_SOURCE_DIR}/src/common/utils/color.cpp ${CMAKE_SOURCE_DIR}/src/common/str_utils.cpp
  )
target_include_directories(
  spinner_od3d_tests PUBLIC ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/src/common ${CMAKE_SOURCE_DIR}/src/guiapi/include
                            ${CMAKE_SOURCE_DIR}/src/lang ${CMAKE_SOURCE_DIR}/include
  )
target_link_libraries(spinner_od3d_tests buddy_utils byte_utils)
add_catch_test(spinner_od3d_tests)
