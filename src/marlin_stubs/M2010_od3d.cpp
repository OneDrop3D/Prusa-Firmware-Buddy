/// @file
/// OneDrop3D (lot UI-0) : M2010, mesure de l'écran écrite sur la clé (gui/od3d/perf_od3d.hpp).
#include "PrusaGcodeSuite.hpp"

#include <common/marlin_server.hpp>
#include <core/serial.h>
#include <gui/od3d/perf_od3d.hpp>

/** \addtogroup G-Codes
 * @{
 */

/**
 *### M2010: OneDrop3D — mesure de l'écran
 *
 * Internal GCode (Réglages › Fonctions OneDrop3D › « Mesure de l'écran sur la clé »)
 *
 *#### Usage
 *
 *    M2010
 *
 * Ajoute à /usb/OD3D_UI.CSV les dessins de l'écran gardés en RAM depuis le dernier M2010 (64 lignes au plus), puis
 * les oublie. Colonnes : t_ms, screen (vtable, à nommer avec utils/od3d/decode_ui_csv.py et l'ELF de la CI), cause
 * (k molette, t toucher, - aucune), input_ms (entrée → fin du dessin), draws, span_ms, draw_us, bytes, windows,
 * dma_wait_us. Refusé pendant une impression.
 */
void PrusaGcodeSuite::M2010() {
    if (marlin_server::is_printing()) {
        SERIAL_ERROR_MSG("M2010: not while printing");
        return;
    }
    if (!od3d::perf::dump("/usb/OD3D_UI.CSV")) {
        SERIAL_ERROR_MSG("M2010: cannot write /usb/OD3D_UI.CSV");
    }
}

/** @}*/
