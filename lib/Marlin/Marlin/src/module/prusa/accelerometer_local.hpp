#pragma once

#include <accelerometer/common_structs.hpp>

// One-time boot init
void prusa_accelerometer_local_init();

void prusa_accelerometer_handle_polling();
void prusa_accelerometer_handle_spi_finish();
void prusa_accelerometer_handle_spi_error();

/// OneDrop3D (F38) : un échantillon du poller toujours actif, depuis une interruption, seulement si l'accéléromètre
/// répond et que personne ne l'a emprunté (PrusaAccelerometer) ; false sinon
bool prusa_accelerometer_local_try_get_sample(accelerometer::RawAcceleration &sample);
