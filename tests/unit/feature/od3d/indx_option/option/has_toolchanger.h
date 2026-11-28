#pragma once

// OneDrop3D : les tests de print_info_od3d sont faits avec l'INDX (8 têtes, print_info::max_tools)
#define HAS_TOOLCHANGER() 1

#ifdef __cplusplus

namespace option {
inline constexpr bool has_toolchanger = 1;
}

#endif
