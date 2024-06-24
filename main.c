/**
 * Copyright (c) 2014 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */


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

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "peer_manager.h"
#include "peer_manager_handler.h"

#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME                     "Nordic_UART_MT(AES)"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION                18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                      1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                      0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                      0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS                  0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES           BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                       0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE              7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE              16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */
#define TEST_AES_CCM_MAC_SIZE                              (10)
#define TEST_AES_GCM_MAC_SIZE                              (16)
#define TEST_AES_NONCE_SIZE                            (12)
#define TEST_AES_NONCE1_SIZE                           (6)
#define TEST_AES_NONCE2_SIZE                           (6)
#define TEST_BLE_MSG_BODY_MIN_SIZE                     (16)
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
static uint8_t m_nonce_key[6] =    { 't', 'e', 's', 't', '1', '2' };

                                        
static bool m_use_crypte = true;
static bool m_crypte_ccm = false; // true : ccm, false : gcm

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
// NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
NRF_BLE_QWRS_DEF(m_qwr, NRF_SDH_BLE_TOTAL_LINK_COUNT);
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

// static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};

static uint8_t m_rssi_index[NRF_SDH_BLE_TOTAL_LINK_COUNT] = { 0, };
static int16_t m_rssi_sum[NRF_SDH_BLE_TOTAL_LINK_COUNT] = { 0, };
static int8_t m_tx_power = 4;

static void test_crypt_ccm(void);
static void test_crypt_gcm(void);
static void ble_data_send(uint16_t conn_handle, uint8_t *p_data, uint16_t length);
static uint32_t ble_data_decrypt(uint8_t* p_data, uint16_t length, uint8_t* p_mac, uint8_t mac_length, uint8_t* p_nonce2, uint8_t* p_decrypt);
static uint32_t ble_data_encrypt(uint8_t* p_data, uint16_t length, uint8_t* p_mac, uint8_t mac_length, uint8_t* p_nonce2, uint8_t* p_encrypt);
static void gap_conn_params_update(uint16_t conn_handle);

void print_connection_parameters(ble_gap_conn_params_t const * p_conn_params)
{
    uint16_t min_conn_interval = p_conn_params->min_conn_interval * 1.25;
    uint16_t max_conn_interval = p_conn_params->max_conn_interval * 1.25;
    uint16_t conn_sup_timeout = p_conn_params->conn_sup_timeout * 10;

    NRF_LOG_INFO("Connection Interval Min: %d ms", min_conn_interval);
    NRF_LOG_INFO("Connection Interval Max: %d ms", max_conn_interval);
    NRF_LOG_INFO("Connection Timeout: %d ms", conn_sup_timeout);
    NRF_LOG_INFO("Slave Latency: %d", p_conn_params->slave_latency);
}

static void generate_random_nonce(uint8_t *nonce, size_t start, size_t size) 
{
    for (size_t i = start; i < size; ++i) {
        nonce[i] = rand() % 256;
    }
}

static void set_tx_power_conn(uint16_t handle, int8_t tx_power) 
{
    if (handle != BLE_CONN_HANDLE_INVALID) {
        NRF_LOG_INFO("set_tx_power_conn:handle=%d,value=%d", handle, tx_power);
        uint32_t err_code;
        err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, 0, tx_power);
        if (err_code != NRF_SUCCESS) {
            NRF_LOG_ERROR("set_tx_power_conn:result=%d", err_code);
        }
    }
}

static void app_ble_disconnect(uint16_t handle, uint8_t hci_status_code)
{
    if (handle != BLE_CONN_HANDLE_INVALID) {
        NRF_LOG_INFO("app_ble_disconnect:handle=%d,hci_status_code=%d", handle, hci_status_code);
        uint32_t err_code;
        err_code = sd_ble_gap_disconnect(handle, hci_status_code);
        if (err_code  != NRF_SUCCESS){
          NRF_LOG_ERROR("app_ble_disconnect:result=%d", err_code);
        }
        APP_ERROR_CHECK(err_code);
    }
}

static void set_tx_power_conn_all(int8_t tx_power) 
{
    for(int i =0; i < NRF_SDH_BLE_TOTAL_LINK_COUNT;i++){
        set_tx_power_conn(m_qwr[i].conn_handle, tx_power);
    }
}

static void set_tx_power_adv(int8_t tx_power) 
{
    NRF_LOG_INFO("set_tx_power_adv: value=%d", tx_power);
    uint32_t err_code;
    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, 0, tx_power);
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_ERROR("set_tx_power_adv, result=%d", err_code);
    }
}

static void app_ble_disconnect_all(uint8_t hci_status_code)
{
    NRF_LOG_INFO("app_ble_disconnect_all:hci_status_code=%d", hci_status_code);

    for(int i =0; i < NRF_SDH_BLE_TOTAL_LINK_COUNT;i++){
        app_ble_disconnect(m_qwr[i].conn_handle, hci_status_code);
    }
}

static void ble_data_send_error_event(uint16_t handle, uint8_t code, uint32_t err_code) 
{
    uint8_t data[6];
    data[0] = 0xff;
    data[1] = code;    
    data[2] = (err_code >> 24) & 0xFF;
    data[3] = (err_code >> 16) & 0xFF;
    data[4] = (err_code >> 8) & 0xFF;
    data[5] = err_code & 0xFF;
    ble_data_send(handle, data, 6);    
}

static void ble_data_send_echo(uint16_t handle, uint8_t *p_data, uint16_t length) 
{
    uint8_t id;
    id = p_data[0];
    uint8_t echo_length = length + 1;
    uint8_t echo_data[echo_length];

    echo_data[0] = id;
    echo_data[1] = 0x00;

    if (length > 1) {
        memcpy(&echo_data[2], &p_data[1], length - 1);
    }

    if (m_use_crypte) {
        uint8_t body_length;
        if (echo_length < TEST_BLE_MSG_BODY_MIN_SIZE)
            body_length = TEST_BLE_MSG_BODY_MIN_SIZE;
        else 
            body_length = echo_length;

        uint8_t mac_length;
        if (m_crypte_ccm)
          mac_length = TEST_AES_CCM_MAC_SIZE;
        else
          mac_length = TEST_AES_GCM_MAC_SIZE;


        uint8_t data_mac[mac_length];
        uint8_t data_nonce2[TEST_AES_NONCE2_SIZE];
        uint8_t data_out[body_length];


        uint32_t err_code;
        if (echo_length < TEST_BLE_MSG_BODY_MIN_SIZE) {
            uint8_t data_body[body_length];

            memcpy(data_body, echo_data, echo_length);
            generate_random_nonce(data_body, echo_length, TEST_BLE_MSG_BODY_MIN_SIZE - echo_length);

            err_code = ble_data_encrypt(
                data_body,
                body_length,
                data_mac,
                mac_length,
                data_nonce2,
                data_out
            );
        }
        else {
            err_code = ble_data_encrypt(
                echo_data,
                body_length,
                data_mac,
                mac_length,
                data_nonce2,
                data_out
            );
        }

        if (err_code == 0) {
          uint8_t data_all[mac_length + TEST_AES_NONCE2_SIZE + body_length];
          uint8_t start = 0;
          memcpy(&data_all[start], data_mac, mac_length); start += mac_length;
          memcpy(&data_all[start], data_nonce2, TEST_AES_NONCE2_SIZE); start += TEST_AES_NONCE2_SIZE;
          memcpy(&data_all[start], data_out, body_length);

          ble_data_send(handle, data_all, mac_length + TEST_AES_NONCE2_SIZE + body_length);
        }else {
          ble_data_send_error_event(handle, 0x01, err_code);
        }
    }else {
        ble_data_send(handle, echo_data, echo_length);
    }
}

static void on_uart_receive(uint16_t handle, uint8_t *p_data, uint16_t length) 
{
    if (length < 1)
        return;

    if(!m_use_crypte){
        ble_data_send_echo(handle, p_data, length);
        return;
    }


    // define...
    uint8_t mac_length;
    if (m_crypte_ccm)
      mac_length = TEST_AES_CCM_MAC_SIZE;
    else
      mac_length = TEST_AES_GCM_MAC_SIZE;

    uint8_t min_msg_size;

    if (length < mac_length + TEST_AES_NONCE2_SIZE + TEST_BLE_MSG_BODY_MIN_SIZE){
        // invalid length...
        ble_data_send_error_event(handle, 0x00, 0);
        return;
    }

    uint8_t body_length = length - (mac_length + TEST_AES_NONCE2_SIZE);
    uint8_t data_mac[mac_length];
    uint8_t data_nonce2[TEST_AES_NONCE2_SIZE];
    uint8_t data_body[body_length];
    uint8_t data_out[body_length];

    // copy
    uint8_t start = 0;
    memcpy(data_mac, &p_data[start], mac_length); start += mac_length;
    memcpy(data_nonce2, &p_data[start], TEST_AES_NONCE2_SIZE); start += TEST_AES_NONCE2_SIZE;
    memcpy(data_body, &p_data[start], body_length);

    // decrypt
    uint32_t result;
    result = ble_data_decrypt(
        data_body,
        body_length,
        data_mac,
        mac_length,
        data_nonce2,
        data_out
    );

    if (result != 0) {
        // fail crypt 
        ble_data_send_error_event(handle, 0x02, result);
        return;
    }

    // success
    uint8_t id = data_out[0];

    switch(id) {
        case 0x05:
        case 0x06:{
                ble_data_send_echo(handle, data_out, 1);
            }
            break;
        case 0x30: {
                ble_data_send_echo(handle, data_out, 7);
            }
            break;
        default: {
                ble_data_send_error_event(handle, 0x03, id);
            }
            break;
    }
}

static void on_rssi_changed(uint16_t handle, int16_t rssi) {
    CRITICAL_REGION_ENTER();

    for(int i =0; i < NRF_SDH_BLE_TOTAL_LINK_COUNT;i++){
        if (m_qwr[i].conn_handle == handle){
            m_rssi_index[i]++;
            m_rssi_sum[i] += rssi;          
            break;
        }
    }

    CRITICAL_REGION_EXIT();
}

static void calculate_average_rssi(uint8_t index) {
    CRITICAL_REGION_ENTER();
    if (m_rssi_index[index] != 0) {
        int16_t average_rssi = m_rssi_sum[index] / m_rssi_index[index];
        NRF_LOG_INFO("Average RSSI: %d dBm, len: %d", average_rssi, m_rssi_index[index]);

        m_rssi_sum[index] = 0;
        m_rssi_index[index] = 0;
    }
    CRITICAL_REGION_EXIT();
}

static void on_connection(uint8_t index, uint16_t handle)
{
    uint32_t err_code;

    CRITICAL_REGION_ENTER();

    m_rssi_index[index] = 0;
    m_rssi_sum[index] = 0;

    set_tx_power_conn(handle, m_tx_power);
    err_code = sd_ble_gap_rssi_start(handle, 0, 0);
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_ERROR("on_connection:handle=%d,sd_ble_gap_rssi_start.err=%d", handle, err_code);
    }else {
        NRF_LOG_INFO("on_connection:sd_ble_gap_rssi_start:handle=%d", handle);
    }

    CRITICAL_REGION_EXIT();
}

static void on_disconnection(uint8_t index, uint16_t handle)
{
    uint32_t err_code;

    CRITICAL_REGION_ENTER();

    err_code = sd_ble_gap_rssi_stop(handle);
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_ERROR("on_disconnection:handle=%d,sd_ble_gap_rssi_stop.err=%d", handle, err_code);
    }else {
        NRF_LOG_INFO("on_disconnection:handle=%d,sd_ble_gap_rssi_stop", handle);
    }

    m_rssi_index[index] = 0;
    m_rssi_sum[index] = 0;

    CRITICAL_REGION_EXIT();
}

/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}

static void ble_data_send(uint16_t conn_handle, uint8_t *p_data, uint16_t length)
{
    uint32_t err_code;
    do
    {
        err_code = ble_nus_data_send(&m_nus, p_data, &length, conn_handle);
        if ((err_code != NRF_ERROR_INVALID_STATE) &&
            (err_code != NRF_ERROR_RESOURCES) &&
            (err_code != NRF_ERROR_NOT_FOUND))
        {
            APP_ERROR_CHECK(err_code);
        }
    } while (err_code == NRF_ERROR_RESOURCES);
}


/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_UNKNOWN);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

static void gap_conn_params_update(uint16_t conn_handle)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   conn_params;

    conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    conn_params.slave_latency     = SLAVE_LATENCY;
    conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_conn_param_update(conn_handle, &conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

// it's normal
static uint32_t ble_data_encrypt(uint8_t* p_data, uint16_t length, uint8_t* p_mac, uint8_t mac_length, uint8_t* p_nonce2, uint8_t* p_encrypt)
{
    uint32_t    err_code;

    uint8_t     nonce[TEST_AES_NONCE_SIZE];
    
    nrf_crypto_aead_context_t ccm_ctx;
    uint8_t encrypted_data[length];
    nrf_crypto_aead_info_t const * p_aead_info;
    
    
    memcpy(nonce, m_nonce_key, TEST_AES_NONCE1_SIZE);
    generate_random_nonce(nonce, TEST_AES_NONCE1_SIZE, TEST_AES_NONCE2_SIZE);
    memcpy(p_nonce2, &nonce[TEST_AES_NONCE1_SIZE], TEST_AES_NONCE2_SIZE);

    if (m_crypte_ccm)
      p_aead_info = &g_nrf_crypto_aes_ccm_128_info;
    else
      p_aead_info = &g_nrf_crypto_aes_gcm_128_info;

    

    /* Init encrypt and decrypt context */
    err_code = nrf_crypto_aead_init(&ccm_ctx,
                                   p_aead_info,
                                   m_test_key);

                             
    if (err_code != NRF_SUCCESS) {
        AES_ERROR_CHECK(err_code);
        return err_code;
    }

    /* encrypt and tag text */
    err_code = nrf_crypto_aead_crypt(&ccm_ctx,
                                    NRF_CRYPTO_ENCRYPT,
                                    nonce,
                                    TEST_AES_NONCE_SIZE,
                                    NULL,
                                    0,
                                    p_data,
                                    length,
                                    p_encrypt,
                                    p_mac,
                                    mac_length);

    if (err_code != NRF_SUCCESS) {
        AES_ERROR_CHECK(err_code);
        return err_code;
    }

    /* Cleanup */
    err_code = nrf_crypto_aead_uninit(&ccm_ctx);
    AES_ERROR_CHECK(err_code);

    return NRF_SUCCESS;
}

static uint32_t ble_data_decrypt(uint8_t* p_data, uint16_t length, uint8_t* p_mac, uint8_t mac_length, uint8_t* p_nonce2, uint8_t* p_decrypt)
{
    uint32_t    err_code;

    uint8_t     nonce[TEST_AES_NONCE_SIZE];
    
    nrf_crypto_aead_context_t ccm_ctx;
    uint8_t encrypted_data[length];
    nrf_crypto_aead_info_t const * p_aead_info;
    
    
    memcpy(nonce, m_nonce_key, TEST_AES_NONCE1_SIZE);
    memcpy(&nonce[TEST_AES_NONCE1_SIZE], p_nonce2, TEST_AES_NONCE2_SIZE);

    if (m_crypte_ccm)
      p_aead_info = &g_nrf_crypto_aes_ccm_128_info;
    else
      p_aead_info = &g_nrf_crypto_aes_gcm_128_info;

    /* Init encrypt and decrypt context */
    err_code = nrf_crypto_aead_init(&ccm_ctx,
                                   p_aead_info,
                                   m_test_key);

                             
    if (err_code != NRF_SUCCESS) {
        AES_ERROR_CHECK(err_code);
        return err_code;
    }

    /* decrypt and tag text */
    err_code = nrf_crypto_aead_crypt(&ccm_ctx,
                                    NRF_CRYPTO_DECRYPT,
                                    nonce,
                                    TEST_AES_NONCE_SIZE,
                                    NULL,
                                    0,
                                    p_data,
                                    length,
                                    p_decrypt,
                                    p_mac,
                                    mac_length);

    if (err_code != NRF_SUCCESS) {
        AES_ERROR_CHECK(err_code);
        return err_code;
    }

    /* Cleanup */
    err_code = nrf_crypto_aead_uninit(&ccm_ctx);
    AES_ERROR_CHECK(err_code);

    return NRF_SUCCESS;
}

static void test_crypt_ccm(void)
{
    uint32_t    len;
    ret_code_t  ret_val;

    uint8_t     mac[TEST_AES_CCM_MAC_SIZE];   // 10bytes
    uint8_t     nonce[]  = {                 // 12 bytes
      0x6B, 0x65, 0x79, 0x70, 0x6C, 0x65,
      0x09, 0x67, 0xC5, 0x69, 0x44, 0xDB
    };
    
    nrf_crypto_aead_context_t ccm_ctx;

    // memset(mac,   0, sizeof(mac));
    // memset(nonce, 0, sizeof(nonce));

    uint8_t plain_text[] =  { 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, // 16 bytes
                                        0x30, 0x30, 0x30, 0x72, 0xFB, 0xBA, 
                                        0x10, 0x19, 0xEF, 0xD9 };

    uint8_t encrypted_text[TEST_NRF_CRYPTO_EXAMPLE_AES_MAX_TEXT_SIZE];
    uint8_t decrypted_text[TEST_NRF_CRYPTO_EXAMPLE_AES_MAX_TEXT_SIZE];

    len = sizeof(plain_text);


    //NRF_LOG_INFO("plain_text");
    //NRF_LOG_HEXDUMP_INFO(plain_text, len);
    
    /* Init encrypt and decrypt context */
    ret_val = nrf_crypto_aead_init(&ccm_ctx,
                                   &g_nrf_crypto_aes_ccm_128_info,
                                   m_test_key);
    AES_ERROR_CHECK(ret_val);

    /* encrypt and tag text */
    ret_val = nrf_crypto_aead_crypt(&ccm_ctx,
                                    NRF_CRYPTO_ENCRYPT,
                                    nonce,
                                    sizeof(nonce),
                                    NULL,
                                    0,
                                    (uint8_t *)plain_text,
                                    len,
                                    (uint8_t *)encrypted_text,
                                    mac,
                                    sizeof(mac));
    AES_ERROR_CHECK(ret_val);

    NRF_LOG_INFO("encrypted text");
    NRF_LOG_HEXDUMP_INFO(encrypted_text, len);
    
    NRF_LOG_INFO("mac");
    NRF_LOG_HEXDUMP_INFO(mac, sizeof(mac));

    return;

    /* decrypt text */
    ret_val = nrf_crypto_aead_crypt(&ccm_ctx,
                                    NRF_CRYPTO_DECRYPT,
                                    nonce,
                                    sizeof(nonce),
                                    NULL,
                                    0,
                                    (uint8_t *)encrypted_text,
                                    len,
                                    (uint8_t *)decrypted_text,
                                    mac,
                                    sizeof(mac));
    AES_ERROR_CHECK(ret_val);

    ret_val = nrf_crypto_aead_uninit(&ccm_ctx);
    AES_ERROR_CHECK(ret_val);

    //NRF_LOG_INFO("decrypted_text");
    //NRF_LOG_HEXDUMP_INFO(decrypted_text, len);

    if (memcmp(plain_text, decrypted_text, len) == 0)
    {
        NRF_LOG_RAW_INFO("AES CCM example executed successfully.\r\n");
    }
    else
    {
        NRF_LOG_RAW_INFO("AES CCM example failed!!!.\r\n");
    }
}

static void test_crypt_gcm(void)
{
    uint32_t    len;
    ret_code_t  ret_val;

    uint8_t     mac[TEST_AES_GCM_MAC_SIZE];   // 10bytes
    uint8_t     nonce[]  = {                 // 12 bytes
      0x6B, 0x65, 0x79, 0x70, 0x6C, 0x65,
      0x09, 0x67, 0xC5, 0x69, 0x44, 0xDB
    };
    
    nrf_crypto_aead_context_t gcm_ctx;

    // memset(mac,   0, sizeof(mac));
    // memset(nonce, 0, sizeof(nonce));

    uint8_t plain_text[] =  { 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, // 16 bytes
                                        0x30, 0x30, 0x30, 0x72, 0xFB, 0xBA, 
                                        0x10, 0x19, 0xEF, 0xD9 };

    uint8_t encrypted_text[TEST_NRF_CRYPTO_EXAMPLE_AES_MAX_TEXT_SIZE];
    uint8_t decrypted_text[TEST_NRF_CRYPTO_EXAMPLE_AES_MAX_TEXT_SIZE];

    len = sizeof(plain_text);


    //NRF_LOG_INFO("plain_text");
    //NRF_LOG_HEXDUMP_INFO(plain_text, len);
    
    /* Init encrypt and decrypt context */
    ret_val = nrf_crypto_aead_init(&gcm_ctx,
                                   &g_nrf_crypto_aes_gcm_128_info,
                                   m_test_key);
    AES_ERROR_CHECK(ret_val);

    /* encrypt and tag text */
    ret_val = nrf_crypto_aead_crypt(&gcm_ctx,
                                    NRF_CRYPTO_ENCRYPT,
                                    nonce,
                                    sizeof(nonce),
                                    NULL,
                                    0,
                                    (uint8_t *)plain_text,
                                    len,
                                    (uint8_t *)encrypted_text,
                                    mac,
                                    sizeof(mac));
    AES_ERROR_CHECK(ret_val);

    NRF_LOG_INFO("encrypted text");
    NRF_LOG_HEXDUMP_INFO(encrypted_text, len);
    
    NRF_LOG_INFO("mac");
    NRF_LOG_HEXDUMP_INFO(mac, sizeof(mac));

    /* decrypt text */
    ret_val = nrf_crypto_aead_crypt(&gcm_ctx,
                                    NRF_CRYPTO_DECRYPT,
                                    nonce,
                                    sizeof(nonce),
                                    NULL,
                                    0,
                                    (uint8_t *)encrypted_text,
                                    len,
                                    (uint8_t *)decrypted_text,
                                    mac,
                                    sizeof(mac));
    AES_ERROR_CHECK(ret_val);

    ret_val = nrf_crypto_aead_uninit(&gcm_ctx);
    AES_ERROR_CHECK(ret_val);

    //NRF_LOG_INFO("decrypted_text");
    //NRF_LOG_HEXDUMP_INFO(decrypted_text, len);

    if (memcmp(plain_text, decrypted_text, len) == 0)
    {
        NRF_LOG_RAW_INFO("AES GCM example executed successfully.\r\n");
    }
    else
    {
        NRF_LOG_RAW_INFO("AES GCM example failed!!!.\r\n");
    }
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{

    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint32_t err_code;

        NRF_LOG_INFO("Received data from BLE NUS. Writing data on UART.");
        NRF_LOG_HEXDUMP_INFO(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

        on_uart_receive(p_evt->conn_handle, p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

        for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
        {
            do
            {
                err_code = app_uart_put(p_evt->params.rx_data.p_data[i]);
                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                {
                    NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
                    APP_ERROR_CHECK(err_code);
                }
            } while (err_code == NRF_ERROR_BUSY);
        }
        if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
        {
            while (app_uart_put('\n') == NRF_ERROR_BUSY);
        }
    }

}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    for(int i =0; i < NRF_SDH_BLE_TOTAL_LINK_COUNT; i++){
        err_code = nrf_ble_qwr_init(&m_qwr[i], &qwr_init);
        APP_ERROR_CHECK(err_code);
    }

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(p_evt->conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;
    ble_gap_evt_t * const p_gap_evt = (ble_gap_evt_t * const)&p_ble_evt->evt.gap_evt;
    uint32_t    periph_link_cnt = ble_conn_state_peripheral_conn_count(); // Number of peripheral links.

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            // todo : check code
            // err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            // APP_ERROR_CHECK(err_code);

            for(int i =0; i < NRF_SDH_BLE_TOTAL_LINK_COUNT;i++){
              if(m_qwr[i].conn_handle == BLE_CONN_HANDLE_INVALID){
                err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr[i], p_gap_evt->conn_handle);
                APP_ERROR_CHECK(err_code);

                on_connection(i, p_gap_evt->conn_handle);

                break;
              }
            }

           print_connection_parameters(&p_gap_evt->params.connected.conn_params);
           // setting conn_param..
           // gap_conn_params_update(p_gap_evt->conn_handle);

           // set again

            if(periph_link_cnt != NRF_SDH_BLE_TOTAL_LINK_COUNT)
              advertising_start();
            break;

        case BLE_GAP_EVT_DISCONNECTED:
             NRF_LOG_INFO("Connection 0x%x has been disconnected. Reason: 0x%X",
                 p_gap_evt->conn_handle,
                 p_gap_evt->params.disconnected.reason);

            // handleStopSession(p_gap_evt->conn_handle);

             for(int i =0; i < NRF_SDH_BLE_TOTAL_LINK_COUNT; i++){
                if(m_qwr[i].conn_handle == p_gap_evt->conn_handle){
                    on_disconnection(i, p_gap_evt->conn_handle);
                    m_qwr[i].conn_handle = BLE_CONN_HANDLE_INVALID;
                    break;
                }
            }

            if (periph_link_cnt == (NRF_SDH_BLE_TOTAL_LINK_COUNT - 1)){
                // Advertising is not running when all connections are taken, and must therefore be started.
                advertising_start();
            }
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_gap_evt->conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(p_gap_evt->conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(p_gap_evt->conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_gap_evt->conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_gap_evt->conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_RSSI_CHANGED:
            on_rssi_changed(p_gap_evt->conn_handle, p_ble_evt->evt.gap_evt.params.rssi_changed.rssi);
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE:
            NRF_LOG_INFO("Connection parameters updated");
            print_connection_parameters(&p_gap_evt->params.conn_param_update.conn_params);
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            NRF_LOG_INFO("Connection parameters update requested");
            print_connection_parameters(&p_gap_evt->params.conn_param_update_request.conn_params);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    for(int i =0; i < NRF_SDH_BLE_TOTAL_LINK_COUNT;i++){
        if(m_qwr[i].conn_handle == p_evt->conn_handle){
            m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
            NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);    
        }
    }

    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    //
    // check this function....
    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    uint32_t periph_link_cnt;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            app_ble_disconnect_all(BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            break;

        case BSP_EVENT_WHITELIST_OFF:
            periph_link_cnt = ble_conn_state_peripheral_conn_count(); // Number of peripheral links.

            if (periph_link_cnt == (NRF_SDH_BLE_TOTAL_LINK_COUNT - 1)){
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            //index++;

            //if ((data_array[index - 1] == '\n') ||
            //    (data_array[index - 1] == '\r') ||
            //    (index >= m_ble_nus_max_data_len))
            //{
            //    if (index > 1)
            //    {
            //        NRF_LOG_DEBUG("Ready to send data over BLE NUS");
            //        NRF_LOG_HEXDUMP_DEBUG(data_array, index);

            //        do
            //        {
            //            uint16_t length = (uint16_t)index;
            //            err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
            //            if ((err_code != NRF_ERROR_INVALID_STATE) &&
            //                (err_code != NRF_ERROR_RESOURCES) &&
            //                (err_code != NRF_ERROR_NOT_FOUND))
            //            {
            //                APP_ERROR_CHECK(err_code);
            //            }
            //        } while (err_code == NRF_ERROR_RESOURCES);
            //    }

            //    index = 0;
            //}
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */

static void crypto_init(void) {
    uint32_t                     err_code;

    //err_code = nrf_drv_clock_init();
    //APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_power_init(NULL);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_crypto_init();
    APP_ERROR_CHECK(err_code);

    err_code = nrf_mem_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    ble_advdata_conn_int_t conn_int;
    conn_int.min_conn_interval = MIN_CONN_INTERVAL;
    conn_int.max_conn_interval = MAX_CONN_INTERVAL;
    init.advdata.p_slave_conn_int = &conn_int;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    bool delete_bonds = false;

    pm_handler_on_pm_evt(p_evt);
    pm_handler_disconnect_on_sec_failure(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            // advertising_start(&delete_bonds);
            advertising_start();
            break;

       case PM_EVT_CONN_SEC_CONFIG_REQ: 
       {
            pm_conn_sec_config_t config = {.allow_repairing = true};
            pm_conn_sec_config_reply(p_evt->conn_handle, &config);
       } break;

        default:
            break;
    }
}

/**@brief Function for the Peer Manager initialization. */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}



/**@brief Application main function.
 */
int main(void)
{
    bool erase_bonds;


    // Initialize.
    uart_init();
    log_init();
    crypto_init();
    timers_init();
    buttons_leds_init(&erase_bonds);
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

    // Start execution.
    printf("\r\nUART started.\r\n");
    NRF_LOG_INFO("Debug logging for UART over RTT started.(new version)");
    advertising_start();
    //test_crypt_gcm();
    
    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}


/**
 * @}
 */
