/**
 * @file    lora_keys.c
 * @brief   Per-device LoRaWAN key derivation. See lora_keys.h for workflow.
 */

#include "lora_keys.h"
#include "stm32wlxx_hal.h"
#include "LmHandler.h"
#include "LoRaMacTypes.h"
#include "lorawan_aes.h"
#include "sys_app.h"
#include <string.h>

/* ---------------------------------------------------------------------------
 * PROJECT_SECRET
 *
 * REPLACE THE BYTES BELOW BEFORE FLASHING ANY PRODUCTION DEVICE.
 *
 * Generate with e.g.   openssl rand -hex 16
 * The same 16 bytes must be used in:
 *   - this firmware (every device)
 *   - your offline provisioning tool that produces the ChirpStack CSV
 *
 * If this stays at the all-0xAA placeholder, the firmware refuses to install
 * keys and falls back to the (insecure) hardcoded keys in se-identity.h, with
 * a warning log. That prevents accidentally shipping the placeholder.
 * --------------------------------------------------------------------------- */
static const uint8_t PROJECT_SECRET[16] = {
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA
};

/* Domain-separation tags so AppKey and NwkKey come from independent derivations. */
static const uint8_t TAG_APP_KEY[4] = { 'A', 'P', 'P', 'K' };
static const uint8_t TAG_NWK_KEY[4] = { 'N', 'W', 'K', 'K' };

/* Build the 16-byte AES input block: [MCU_UID 12B][TAG 4B]. */
static void build_kdf_input(uint8_t out_block[16], const uint8_t tag[4])
{
    uint32_t w0 = HAL_GetUIDw0();
    uint32_t w1 = HAL_GetUIDw1();
    uint32_t w2 = HAL_GetUIDw2();

    out_block[0]  = (uint8_t)(w0 >> 0);
    out_block[1]  = (uint8_t)(w0 >> 8);
    out_block[2]  = (uint8_t)(w0 >> 16);
    out_block[3]  = (uint8_t)(w0 >> 24);
    out_block[4]  = (uint8_t)(w1 >> 0);
    out_block[5]  = (uint8_t)(w1 >> 8);
    out_block[6]  = (uint8_t)(w1 >> 16);
    out_block[7]  = (uint8_t)(w1 >> 24);
    out_block[8]  = (uint8_t)(w2 >> 0);
    out_block[9]  = (uint8_t)(w2 >> 8);
    out_block[10] = (uint8_t)(w2 >> 16);
    out_block[11] = (uint8_t)(w2 >> 24);
    out_block[12] = tag[0];
    out_block[13] = tag[1];
    out_block[14] = tag[2];
    out_block[15] = tag[3];
}

static int secret_is_placeholder(void)
{
    for (int i = 0; i < 16; i++) {
        if (PROJECT_SECRET[i] != 0xAA) return 0;
    }
    return 1;
}

static void derive_key(const uint8_t tag[4], uint8_t out_key[16])
{
    uint8_t input[16];
    lorawan_aes_context ctx[1];

    build_kdf_input(input, tag);
    lorawan_aes_set_key(PROJECT_SECRET, sizeof(PROJECT_SECRET), ctx);
    lorawan_aes_encrypt(input, out_key, ctx);
}

int LoRa_InstallPerDeviceKeys(void)
{
    if (secret_is_placeholder())
    {
        APP_LOG(TS_ON, VLEVEL_L,
            "WARNING: PROJECT_SECRET still at placeholder \xE2\x80\x94 using hardcoded test keys. "
            "Edit lora_keys.c and rebuild before production.\r\n");
        return -1;
    }

    uint8_t app_key[16];
    uint8_t nwk_key[16];

    derive_key(TAG_APP_KEY, app_key);
    derive_key(TAG_NWK_KEY, nwk_key);

    if (LmHandlerSetKey(APP_KEY, app_key) != LORAMAC_HANDLER_SUCCESS)
    {
        APP_LOG(TS_ON, VLEVEL_L, "LoRa_InstallPerDeviceKeys: APP_KEY install failed\r\n");
        return -2;
    }
    if (LmHandlerSetKey(NWK_KEY, nwk_key) != LORAMAC_HANDLER_SUCCESS)
    {
        APP_LOG(TS_ON, VLEVEL_L, "LoRa_InstallPerDeviceKeys: NWK_KEY install failed\r\n");
        return -3;
    }

    APP_LOG(TS_ON, VLEVEL_L, "Per-device AppKey/NwkKey installed.\r\n");

    /* Best-effort: scrub local copies. */
    memset(app_key, 0, sizeof(app_key));
    memset(nwk_key, 0, sizeof(nwk_key));

    return 0;
}
