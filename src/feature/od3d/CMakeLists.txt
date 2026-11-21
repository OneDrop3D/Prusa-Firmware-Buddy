target_sources(firmware PRIVATE pairing_od3d.cpp)
if(HAS_SPOOL_JOIN)
  target_sources(firmware PRIVATE spool_join_od3d.cpp)
endif()
