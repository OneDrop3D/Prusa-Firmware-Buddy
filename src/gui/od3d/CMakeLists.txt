target_sources(
  firmware
  PRIVATE accelerometer_guide_od3d.cpp
          atelier_od3d.cpp
          dialog_wait_od3d.cpp
          island_od3d.cpp
          menu_od3d.cpp
          perf_od3d.cpp
          print_info_od3d.cpp
          screen_belts_od3d.cpp
          screen_climate_od3d.cpp
          screen_damping_od3d.cpp
          screen_fan_response_od3d.cpp
          screen_files_od3d.cpp
          screen_filament_presets_od3d.cpp
          screen_home_od3d.cpp
          screen_menu_od3d.cpp
          screen_move_od3d.cpp
          screen_music_od3d.cpp
          screen_printing_od3d.cpp
          screen_resume_od3d.cpp
          screen_settings_od3d.cpp
          screen_progress_od3d.cpp
          screen_sleep_od3d.cpp
          screen_speed_map_od3d.cpp
          screen_temperatures_od3d.cpp
          sleep_gui_od3d.cpp
          spinner_od3d.cpp
          frost_job_od3d.cpp
          frost_od3d.cpp
          spool_draw.cpp
          spool_sheet_od3d.cpp
          thumbnail_od3d.cpp
          value_editor_od3d.cpp
          widgets_od3d.cpp
  )
if(NOT HAS_INDX)
  # Régulation MPC (F7, OD3D_MPCTEMP) et équerrage XY (F15, OD3D_SKEW_CORRECTION_XY) : pas avec l'INDX
  target_sources(firmware PRIVATE screen_mpc_od3d.cpp screen_skew_od3d.cpp)
endif()
if(HAS_MMU2)
  # Écrans du MMU : emplacements, changement de bobine (M704/M705), tube PTFE, vitesse de la poulie, mise à jour USB
  target_sources(
    firmware PRIVATE screen_mmu_od3d.cpp screen_mmu_speed_od3d.cpp screen_mmu_usb_od3d.cpp screen_ptfe_od3d.cpp
                     screen_spool_od3d.cpp
    )
endif()
if(HAS_LOADCELL AND HAS_MMU2)
  # Mesures à la cellule de charge par emplacement du MMU (feature/od3d/CMakeLists.txt) et assistant de calibration
  target_sources(
    firmware
    PRIVATE screen_calib_assistant_od3d.cpp
            screen_collision_od3d.cpp
            screen_e_jump_od3d.cpp
            screen_nozzle_expansion_od3d.cpp
            screen_pa_od3d.cpp
            screen_spool_calib_od3d.cpp
    )
endif()
