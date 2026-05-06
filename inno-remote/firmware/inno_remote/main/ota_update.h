#pragma once

#include "esp_err.h"

/**
 * Perform OTA firmware update from an HTTP URL.
 *
 * Downloads the firmware image, writes it to the inactive OTA partition,
 * validates, and reboots into the new image.
 *
 * @param url  Full HTTP URL to the firmware .bin file
 *             (e.g. "http://192.168.6.13:8556/inno_remote.bin")
 * @return ESP_OK on success (device will reboot before returning),
 *         or an error code if the update failed.
 */
esp_err_t ota_update_from_url(const char *url);
