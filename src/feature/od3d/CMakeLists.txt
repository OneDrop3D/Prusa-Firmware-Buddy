target_sources(
  firmware PRIVATE lookahead_math_od3d.cpp lookahead_od3d.cpp mpc_control_od3d.cpp mpc_od3d.cpp music_od3d.cpp
                   music_player_od3d.cpp pairing_od3d.cpp skew_od3d.cpp sleep_math_od3d.cpp sleep_od3d.cpp
  )
if(HAS_CHAMBER_API)
  # F29 : climat de la chambre (recettes par matière) et descente douce du plateau
  target_sources(firmware PRIVATE chamber_climate_math_od3d.cpp chamber_climate_od3d.cpp)
endif()
if(HAS_PAUSE)
  # F30 : reprise sans cicatrice (purge hors pièce, essuyage, désrétraction calibrée, descente lente)
  target_sources(firmware PRIVATE clean_resume_math_od3d.cpp clean_resume_od3d.cpp)
endif()
if(HAS_SPOOL_JOIN)
  target_sources(firmware PRIVATE spool_join_od3d.cpp)
endif()
if(HAS_MMU2)
  # Mise à jour du firmware du MMU par son USB : M709 S2 coupe la communication
  target_sources(firmware PRIVATE mmu_usb_od3d.cpp)
  # F41 : tube PTFE avant du MMU mesuré (M1999)
  target_sources(firmware PRIVATE ptfe_od3d.cpp)
endif()
if(HAS_MMU2 AND HAS_FILAMENT_TRACKER)
  # F13 : suivi du poids des bobines du MMU
  target_sources(firmware PRIVATE spool_usage_math_od3d.cpp spool_usage_od3d.cpp)
endif()
if(HAS_MANUAL_BELT_TUNING AND HAS_INPUT_SHAPER_CALIBRATION)
  # F14 : tension des courroies mesurée par l'accéléromètre (M1992) ; F22 : vitesses critiques (M1993) ; F33 :
  # amortissement des axes (M1997) ; ACC-1 : vérification commune de l'accéléromètre
  target_sources(
    firmware
    PRIVATE accelerometer_od3d.cpp
            belt_tension_math_od3d.cpp
            belt_tension_od3d.cpp
            damping_math_od3d.cpp
            damping_od3d.cpp
            speed_map_math_od3d.cpp
            speed_map_od3d.cpp
    )
endif()
if(HAS_LOADCELL AND HAS_MMU2)
  # F4 : outil de mesure par la cellule de charge et pressure advance par emplacement ; F5, F6 : débit maximal et
  # température minimale par emplacement ; F38 : sécurité collision / warping
  target_sources(
    firmware
    PRIVATE collision_math_od3d.cpp
            collision_od3d.cpp
            force_probe_od3d.cpp
            force_trace_od3d.cpp
            measure_session_od3d.cpp
            pa_math_od3d.cpp
            pa_od3d.cpp
            pattern_math_od3d.cpp
            spool_calib_math_od3d.cpp
            spool_calib_od3d.cpp
    )
endif()
