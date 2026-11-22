/// @file
/// OneDrop3D (F41) : M1999, mesure automatique du tube PTFE avant du MMU (feature/od3d/ptfe_od3d.hpp).
#include "PrusaGcodeSuite.hpp"

#include <option/has_mmu2.h>
#if HAS_MMU2()

    #include <feature/od3d/ptfe_od3d.hpp>
    #include <gcode/parser.h>
    #include <core/serial.h>

/** \addtogroup G-Codes
 * @{
 */

/**
 *### M1999: OneDrop3D — tube PTFE avant du MMU mesuré
 *
 * Internal GCode (Réglages › Matériel › MMU › Mesurer automatiquement, assistant de calibration).
 *
 *#### Usage
 *
 *    M1999 [B1 [P<0-4>]] [L<341-1000>]
 *
 *#### Parameters
 *
 * - `B1` - mesure la distance FINDA → capteur de l'extrudeur avec la bobine préchargée de l'emplacement P (0 par
 *          défaut), par le test de chargement de Prusa (buse froide permise), puis écrit la longueur avec sa marge
 *          dans le MMU (registre 0x22). Refusé pendant une impression, MMU pas prêt, capteur de l'extrudeur absent ou
 *          en panne, filament dans l'extrudeur ou dans le sélecteur.
 * - `L` - écrit cette longueur (mm) dans le MMU puis la relit (menu « Tube PTFE avant ») ; affiche les deux valeurs.
 *         Hors de 341 à 1000 mm, rien n'est écrit.
 * - sans paramètre : affiche le résultat de la dernière mesure
 */
void PrusaGcodeSuite::M1999() {
    namespace pt = od3d::ptfe;
    if (parser.seenval('L')) {
        const uint16_t asked = parser.value_ushort();
        const uint16_t mmu = pt::write_length(asked);
        SERIAL_ECHOLNPAIR("M1999 asked mm:", asked, " MMU mm:", mmu);
        return;
    }
    if (parser.seenval('B') && parser.value_bool()) {
        pt::measure(parser.byteval('P', 0));
    }
    const pt::Status s = pt::status();
    SERIAL_ECHOLNPAIR("M1999 result:", uint8_t(s.result), " measured mm:", s.measured_mm, " saved mm:", s.saved_mm, " MMU mm:", s.mmu_mm);
}

/** @}*/

#endif // HAS_MMU2()
