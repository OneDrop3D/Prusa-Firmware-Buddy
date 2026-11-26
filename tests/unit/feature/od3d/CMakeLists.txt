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

add_executable(object_map_od3d_tests ${CMAKE_CURRENT_SOURCE_DIR}/object_map_test.cpp)
target_include_directories(object_map_od3d_tests PUBLIC ${CMAKE_SOURCE_DIR}/src)
add_catch_test(object_map_od3d_tests)

add_executable(text_od3d_tests ${CMAKE_CURRENT_SOURCE_DIR}/text_od3d_test.cpp)
target_include_directories(text_od3d_tests PUBLIC ${CMAKE_SOURCE_DIR}/src)
add_catch_test(text_od3d_tests)

add_executable(
  draw_od3d_tests
  ${CMAKE_CURRENT_SOURCE_DIR}/draw_od3d_test.cpp ${CMAKE_SOURCE_DIR}/src/gui/od3d/thumbnail_od3d.cpp
  ${CMAKE_SOURCE_DIR}/src/gui/qoi_decoder.cpp ${CMAKE_SOURCE_DIR}/src/common/utils/color.cpp
  ${CMAKE_SOURCE_DIR}/src/common/str_utils.cpp ${CMAKE_SOURCE_DIR}/src/guiapi/src/Rect16.cpp
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
  mesh_memory_od3d_tests ${CMAKE_CURRENT_SOURCE_DIR}/mesh_memory_test.cpp
                         ${CMAKE_SOURCE_DIR}/src/feature/od3d/mesh_memory_math_od3d.cpp
  )
target_include_directories(mesh_memory_od3d_tests PUBLIC ${CMAKE_SOURCE_DIR}/src)
add_catch_test(mesh_memory_od3d_tests)
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

add_executable(
  collision_od3d_tests ${CMAKE_CURRENT_SOURCE_DIR}/collision_test.cpp
                       ${CMAKE_SOURCE_DIR}/src/feature/od3d/collision_math_od3d.cpp
  )
target_include_directories(collision_od3d_tests PUBLIC ${CMAKE_SOURCE_DIR}/src)
add_catch_test(collision_od3d_tests)

add_executable(
  frost_od3d_tests ${CMAKE_CURRENT_SOURCE_DIR}/frost_od3d_test.cpp ${CMAKE_SOURCE_DIR}/src/gui/od3d/frost_job_od3d.cpp
  )
target_include_directories(
  frost_od3d_tests PUBLIC ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/src/common ${CMAKE_SOURCE_DIR}/src/guiapi/include
                          ${CMAKE_SOURCE_DIR}/src/lang ${CMAKE_SOURCE_DIR}/include
  )
target_link_libraries(frost_od3d_tests buddy_utils byte_utils)
add_catch_test(frost_od3d_tests)

add_executable(
  sleep_od3d_tests ${CMAKE_CURRENT_SOURCE_DIR}/sleep_od3d_test.cpp
                   ${CMAKE_SOURCE_DIR}/src/feature/od3d/sleep_math_od3d.cpp
  )
target_include_directories(sleep_od3d_tests PUBLIC ${CMAKE_SOURCE_DIR}/src)
add_catch_test(sleep_od3d_tests)

add_executable(layout_od3d_tests ${CMAKE_CURRENT_SOURCE_DIR}/layout_od3d_test.cpp)
target_include_directories(
  layout_od3d_tests PUBLIC ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/src/common ${CMAKE_SOURCE_DIR}/src/guiapi/include
                           ${CMAKE_SOURCE_DIR}/src/lang ${CMAKE_SOURCE_DIR}/include
  )
target_link_libraries(layout_od3d_tests buddy_utils byte_utils)
add_catch_test(layout_od3d_tests)

add_executable(ptfe_od3d_tests ${CMAKE_CURRENT_SOURCE_DIR}/ptfe_math_test.cpp)
target_include_directories(ptfe_od3d_tests PUBLIC ${CMAKE_SOURCE_DIR}/src)
add_catch_test(ptfe_od3d_tests)

add_executable(mmu_speed_od3d_tests ${CMAKE_CURRENT_SOURCE_DIR}/mmu_speed_test.cpp)
target_include_directories(mmu_speed_od3d_tests PUBLIC ${CMAKE_SOURCE_DIR}/src)
add_catch_test(mmu_speed_od3d_tests)

add_executable(tip_od3d_tests ${CMAKE_CURRENT_SOURCE_DIR}/tip_test.cpp)
target_include_directories(tip_od3d_tests PUBLIC ${CMAKE_SOURCE_DIR}/src)
add_catch_test(tip_od3d_tests)

add_executable(filament_memory_od3d_tests ${CMAKE_CURRENT_SOURCE_DIR}/filament_memory_test.cpp)
target_include_directories(filament_memory_od3d_tests PUBLIC ${CMAKE_SOURCE_DIR}/src)
add_catch_test(filament_memory_od3d_tests)

add_executable(
  calib_registry_od3d_tests ${CMAKE_CURRENT_SOURCE_DIR}/calib_registry_test.cpp
                            ${CMAKE_SOURCE_DIR}/src/feature/od3d/calib_registry_od3d.cpp
  )
target_include_directories(calib_registry_od3d_tests PUBLIC ${CMAKE_SOURCE_DIR}/src)
add_catch_test(calib_registry_od3d_tests)

add_executable(filament_presets_od3d_tests ${CMAKE_CURRENT_SOURCE_DIR}/filament_presets_test.cpp)
target_include_directories(filament_presets_od3d_tests PUBLIC ${CMAKE_SOURCE_DIR}/src)
add_catch_test(filament_presets_od3d_tests)
add_executable(
  avr109_od3d_tests ${CMAKE_CURRENT_SOURCE_DIR}/avr109_test.cpp ${CMAKE_SOURCE_DIR}/src/feature/od3d/avr109_od3d.cpp
  )
target_include_directories(avr109_od3d_tests PUBLIC ${CMAKE_SOURCE_DIR}/src)
add_catch_test(avr109_od3d_tests)

add_executable(
  shaper_fit_od3d_tests ${CMAKE_CURRENT_SOURCE_DIR}/shaper_fit_test.cpp
                        ${CMAKE_SOURCE_DIR}/src/feature/od3d/shaper_fit_math_od3d.cpp
  )
target_include_directories(
  shaper_fit_od3d_tests PUBLIC ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/lib/Marlin ${CMAKE_SOURCE_DIR}/include
                               ${CMAKE_SOURCE_DIR}/src/common/utils
  )
target_link_libraries(shaper_fit_od3d_tests buddy_utils)
add_catch_test(shaper_fit_od3d_tests)

add_executable(
  nozzle_expansion_od3d_tests ${CMAKE_CURRENT_SOURCE_DIR}/nozzle_expansion_test.cpp
                              ${CMAKE_SOURCE_DIR}/src/feature/od3d/nozzle_expansion_math_od3d.cpp
  )
target_include_directories(
  nozzle_expansion_od3d_tests PUBLIC ${CMAKE_SOURCE_DIR}/src
                                     ${CMAKE_SOURCE_DIR}/tests/unit/feature/nozzle_thermal_compensation/stub_include
  )
add_catch_test(nozzle_expansion_od3d_tests)

add_executable(
  e_jump_od3d_tests ${CMAKE_CURRENT_SOURCE_DIR}/e_jump_math_test.cpp
                    ${CMAKE_SOURCE_DIR}/src/feature/od3d/e_jump_math_od3d.cpp
  )
target_include_directories(e_jump_od3d_tests PUBLIC ${CMAKE_SOURCE_DIR}/src)
add_catch_test(e_jump_od3d_tests)

add_executable(perf_od3d_tests ${CMAKE_CURRENT_SOURCE_DIR}/perf_od3d_test.cpp)
target_include_directories(perf_od3d_tests PUBLIC ${CMAKE_SOURCE_DIR}/src)
add_catch_test(perf_od3d_tests)

add_executable(
  vibration_math_od3d_tests
  ${CMAKE_CURRENT_SOURCE_DIR}/vibration_math_test.cpp ${CMAKE_SOURCE_DIR}/src/feature/od3d/vibration_math_od3d.cpp
  ${CMAKE_SOURCE_DIR}/src/feature/od3d/speed_map_math_od3d.cpp
  )
target_include_directories(vibration_math_od3d_tests PUBLIC ${CMAKE_SOURCE_DIR}/src)
add_catch_test(vibration_math_od3d_tests)
