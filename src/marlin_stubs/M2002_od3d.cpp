/// @file
/// OneDrop3D (N4) : M2002, vitesse de la poulie du MMU mesurée (feature/od3d/mmu_speed_od3d.hpp).
#include "PrusaGcodeSuite.hpp"

#include <option/has_mmu2.h>
#if HAS_MMU2()

    #include <feature/od3d/mmu_speed_od3d.hpp>
    #include <gcode/parser.h>
    #include <core/serial.h>

/** \addtogroup G-Codes
 * @{
 */

/**
 *### M2002: OneDrop3D — vitesse de la poulie du MMU mesurée
 *
 * Internal GCode (Atelier › Réglages du MMU › Vitesse de la poulie, assistant de calibration).
 *
 *#### Usage
 *
 *    M2002 [P<0-4> | R]
 *
 *#### Parameters
 *
 * - `P` - avec la bobine préchargée de cet emplacement : test de chargement de Prusa (M1704, buse froide permise) à
 *         95, 115, 135, 155, 175 puis 190 mm/s, arrêt au premier palier avec une erreur, un réessai, un patinage ou un
 *         cycle plus lent ; 80 % du dernier palier passé est vérifié puis gardé et renvoyé au MMU à chaque démarrage.
 *         Refusé pendant une impression, MMU pas prêt, capteur de l'extrudeur absent ou en panne, filament dans
 *         l'extrudeur ou dans le sélecteur. S'arrête depuis l'écran entre deux cycles.
 * - `R` - vitesse de Prusa (95 mm/s)
 * - sans paramètre : vitesse envoyée au MMU et dernière mesure
 */
void PrusaGcodeSuite::M2002() {
    namespace ms = od3d::mmu_speed;
    if (parser.seen('R')) {
        ms::reset();
    } else if (parser.seenval('P')) {
        ms::measure(parser.byteval('P'));
    }
    const ms::Saved s = ms::saved();
    const ms::Measurement &m = ms::last();
    SERIAL_ECHOLNPAIR("M2002 speed mm/s:", ms::speed_of(s), " cycle ds Prusa:", s.prusa_ds, " now:", s.cycle_ds, " result:", uint8_t(m.result));
}

/** @}*/

#endif // HAS_MMU2()
