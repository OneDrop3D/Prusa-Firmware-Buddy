target_sources(firmware PRIVATE music_od3d.cpp music_player_od3d.cpp pairing_od3d.cpp skew_od3d.cpp)
if(HAS_SPOOL_JOIN)
  target_sources(firmware PRIVATE spool_join_od3d.cpp)
endif()
if(HAS_MMU2 AND HAS_FILAMENT_TRACKER)
  # F13 : suivi du poids des bobines du MMU
  target_sources(firmware PRIVATE spool_usage_math_od3d.cpp spool_usage_od3d.cpp)
endif()
if(HAS_MANUAL_BELT_TUNING AND HAS_INPUT_SHAPER_CALIBRATION)
  # F14 : tension des courroies mesurée par l'accéléromètre (M1992) ; F22 : vitesses critiques (M1993)
  target_sources(
    firmware PRIVATE belt_tension_math_od3d.cpp belt_tension_od3d.cpp speed_map_math_od3d.cpp speed_map_od3d.cpp
    )
endif()
