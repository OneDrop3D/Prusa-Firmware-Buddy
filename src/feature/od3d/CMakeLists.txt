target_sources(firmware PRIVATE music_od3d.cpp music_player_od3d.cpp pairing_od3d.cpp skew_od3d.cpp)
if(HAS_SPOOL_JOIN)
  target_sources(firmware PRIVATE spool_join_od3d.cpp)
endif()
if(HAS_MMU2 AND HAS_FILAMENT_TRACKER)
  # F13 : suivi du poids des bobines du MMU
  target_sources(firmware PRIVATE spool_usage_math_od3d.cpp spool_usage_od3d.cpp)
endif()
if(HAS_LOADCELL AND HAS_MMU2)
  # F4 : outil de mesure par la cellule de charge et pressure advance par emplacement
  target_sources(
    firmware PRIVATE force_probe_od3d.cpp force_trace_od3d.cpp pa_math_od3d.cpp pa_od3d.cpp
    )
endif()
