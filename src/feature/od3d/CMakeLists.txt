target_sources(firmware PRIVATE pairing_od3d.cpp)
if(HAS_SPOOL_JOIN)
  target_sources(firmware PRIVATE spool_join_od3d.cpp)
endif()
if(HAS_MMU2 AND HAS_FILAMENT_TRACKER)
  # F13 : suivi du poids des bobines du MMU
  target_sources(firmware PRIVATE spool_usage_math_od3d.cpp spool_usage_od3d.cpp)
endif()
