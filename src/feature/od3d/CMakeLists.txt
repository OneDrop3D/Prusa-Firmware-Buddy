target_sources(firmware PRIVATE music_od3d.cpp music_player_od3d.cpp pairing_od3d.cpp skew_od3d.cpp)
if(HAS_SPOOL_JOIN)
  target_sources(firmware PRIVATE spool_join_od3d.cpp)
endif()
