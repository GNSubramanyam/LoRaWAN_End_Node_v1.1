/**
 * @file    lora_keys.h
 * @brief   Per-device LoRaWAN key derivation from MCU UID + a project-wide secret.
 *
 * The hardcoded keys in se-identity.h are the published NIST AES test vector.
 * Anyone with a $20 SDR can decrypt or spoof that traffic. This module derives
 * per-device AES-128 keys at boot from the STM32 96-bit unique ID and a
 * project secret (PROJECT_SECRET) so every node ends up with its own AppKey
 * and NwkKey while still being deterministic enough to provision in ChirpStack
 * from the DevEUI list.
 *
 * Workflow:
 *   1. Replace PROJECT_SECRET in lora_keys.c with a real 16-byte random value.
 *      Treat this value as a secret: it is the master key for the whole fleet.
 *   2. Flash all devices with the firmware (same binary).
 *   3. Collect each device's DevEUI (printed at boot — see GetUniqueId() output).
 *   4. Run the offline derivation tool with PROJECT_SECRET + DevEUI list to
 *      produce a CSV of (DevEUI, AppKey, NwkKey).
 *   5. Bulk-import that CSV into ChirpStack.
 */

#ifndef LORA_KEYS_H
#define LORA_KEYS_H

#include <stdint.h>

/**
 * @brief Derive and install per-device AppKey and NwkKey into the LoRaWAN
 *        secure element, replacing the values set by se-identity.h.
 *
 * Must be called AFTER LmHandlerInit() / LmHandlerConfigure() but BEFORE
 * LmHandlerJoin(), so the join handshake uses the per-device keys.
 *
 * @return 0 on success, non-zero on failure.
 */
int LoRa_InstallPerDeviceKeys(void);

#endif /* LORA_KEYS_H */
