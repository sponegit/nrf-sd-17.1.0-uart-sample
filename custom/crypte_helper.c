#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "nordic_common.h"
#include "nrf.h"

#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_drv_power.h"
#include "nrf_crypto.h"
#include "nrf_crypto_error.h"
#include "mem_manager.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define TEST_AES_MAC_SIZE                              (10)
#define TEST_NRF_CRYPTO_EXAMPLE_AES_MAX_TEXT_SIZE      (100)
#define AES_ERROR_CHECK(error)  \
    do {            \
        if (error)  \
        {           \
            NRF_LOG_RAW_INFO("\r\nError = 0x%x\r\n%s\r\n",           \
                             (error),                                \
                             nrf_crypto_error_string_get(error));    \
            return; \
        }           \
    } while (0);

static uint8_t m_test_key[16] =    { 't', 'e', 's', 't', '1', '2',
                                        't', 'e', 's', 't', '1', '2',
                                        '1', '2', '3', '4' };
static char m_test_plain_text[] =  { 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, // 16 bytes
                                        0x30, 0x30, 0x30, 0x72, 0xFB, 0xBA, 
                                        0x10, 0x19, 0xEF, 0xD9 };
static char m_test_encrypted_text[TEST_NRF_CRYPTO_EXAMPLE_AES_MAX_TEXT_SIZE];
static char m_test_decrypted_text[TEST_NRF_CRYPTO_EXAMPLE_AES_MAX_TEXT_SIZE];

static void test_crypt_ccm(void)
{
    uint32_t    len;
    ret_code_t  ret_val;

    static uint8_t     mac[TEST_AES_MAC_SIZE];   // 10bytes
    static uint8_t     nonce[]  = {                 // 12 bytes
      0x6B, 0x65, 0x79, 0x70, 0x6C, 0x65,
      0x09, 0x67, 0xC5, 0x69, 0x44, 0xDB
    };

    static nrf_crypto_aead_context_t ccm_ctx;

    memset(mac,   0, sizeof(mac));
    // memset(nonce, 0, sizeof(nonce));


    len = sizeof(m_test_plain_text);

    NRF_LOG_INFO("PRINT :: plain text >>");
    NRF_LOG_HEXDUMP_INFO(m_test_plain_text, len);

    /* Init encrypt and decrypt context */
    NRF_LOG_INFO("init crypto >>");
    ret_val = nrf_crypto_aead_init(&ccm_ctx,
                                   &g_nrf_crypto_aes_ccm_128_info,
                                   m_test_key);
    AES_ERROR_CHECK(ret_val);

    /* encrypt and tag text */
    NRF_LOG_INFO("encrypt >>");
    ret_val = nrf_crypto_aead_crypt(&ccm_ctx,
                                    NRF_CRYPTO_ENCRYPT,
                                    nonce,
                                    sizeof(nonce),
                                    NULL,
                                    0,
                                    (uint8_t *)m_test_plain_text,
                                    len,
                                    (uint8_t *)m_test_encrypted_text,
                                    mac,
                                    sizeof(mac));
    AES_ERROR_CHECK(ret_val);

    NRF_LOG_INFO("PRINT :: encrypted text >>");
    NRF_LOG_HEXDUMP_INFO(m_test_encrypted_text, len);
    NRF_LOG_INFO("PRINT :: mac >>");
    NRF_LOG_HEXDUMP_INFO(mac, sizeof(mac));

    /* decrypt text */
    NRF_LOG_INFO("decrypt >>");
    ret_val = nrf_crypto_aead_crypt(&ccm_ctx,
                                    NRF_CRYPTO_DECRYPT,
                                    nonce,
                                    sizeof(nonce),
                                    NULL,
                                    0,
                                    (uint8_t *)m_test_encrypted_text,
                                    len,
                                    (uint8_t *)m_test_decrypted_text,
                                    mac,
                                    sizeof(mac));
    AES_ERROR_CHECK(ret_val);

    ret_val = nrf_crypto_aead_uninit(&ccm_ctx);
    AES_ERROR_CHECK(ret_val);

    NRF_LOG_INFO("PRINT :: decrypted text >>");
    NRF_LOG_HEXDUMP_INFO(m_test_decrypted_text, len);

    if (memcmp(m_test_plain_text, m_test_decrypted_text, strlen(m_test_plain_text)) == 0)
    {
        NRF_LOG_RAW_INFO("AES CCM example executed successfully.\r\n");
    }
    else
    {
        NRF_LOG_RAW_INFO("AES CCM example failed!!!.\r\n");
    }
}

static void crypto_init(void) {
    uint32_t                     err_code;
    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    if (!nrf_drv_power_init_check()){
      err_code = nrf_drv_power_init(NULL);
      NRF_LOG_ERROR("Crypte -  nrf_drv_power_init %d", err_code);
      APP_ERROR_CHECK(err_code);
    }
    

    if (!nrf_crypto_is_initialized()){
      err_code = nrf_crypto_init();
      NRF_LOG_ERROR("Crypte -  nrf_crypto_init %d", err_code);
      APP_ERROR_CHECK(err_code);
    }

    err_code = nrf_mem_init();
    NRF_LOG_ERROR("Crypte -  nrf_mem_init %d", err_code);
    APP_ERROR_CHECK(err_code);
}

void Crypte_Init() {
    NRF_LOG_INFO("Crypte_Init");
    crypto_init();
}