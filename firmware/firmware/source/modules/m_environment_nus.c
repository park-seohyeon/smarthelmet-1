/*
  Copyright (c) 2010 - 2017, Nordic Semiconductor ASA
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.

  2. Redistributions in binary form, except as embedded into a Nordic
     Semiconductor ASA integrated circuit in a product or a software update for
     such product, must reproduce the above copyright notice, this list of
     conditions and the following disclaimer in the documentation and/or other
     materials provided with the distribution.

  3. Neither the name of Nordic Semiconductor ASA nor the names of its
     contributors may be used to endorse or promote products derived from this
     software without specific prior written permission.

  4. This software, with or without modification, must only be used with a
     Nordic Semiconductor ASA integrated circuit.

  5. Any software provided in binary form under this license must not be reverse
     engineered, decompiled, modified and/or disassembled.

  THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
  OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
  OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
// motion include header
#include "m_motion.h"
#include "m_motion_flash.h"
#include "drv_motion.h"
#include "sdk_errors.h"
#include "ble_tms.h"
// environment include header
#include "m_environment.h"
#include "drv_speaker.h"
#include <string.h>
#include "app_util_platform.h"
#include "m_environment_flash.h"
#include "drv_humidity.h"
#include "drv_pressure.h"
#include "drv_gas_sensor.h"
#include "drv_color.h"
#include "app_timer.h"
#include "pca20020.h"
#include "nrf_delay.h"
#include "fstorage.h"
#include "m_ui.h"
#include "ble_nus.h"
#define  NRF_LOG_MODULE_NAME "m_env         "
#include "nrf_log.h"
#include "macros_common.h"
#include <math.h>

//sound play define
#define LIMIT_GAS 1200
#define LIMIT_ACC 1850
#define LIMIT_GRAVITY 0
#define LIMIT_TEMP 30


// motion define
#define RAW_PARAM_NUM                 9     // Number of raw parameters (3 * acc + 3 * gyro + 3 * compass).
#define RAW_Q_FORMAT_ACC_INTEGER_BITS 6     // Number of bits used for integer part of raw data.
#define RAW_Q_FORMAT_GYR_INTEGER_BITS 11    // Number of bits used for integer part of raw data.
#define RAW_Q_FORMAT_CMP_INTEGER_BITS 12    // Number of bits used for integer part of raw data.

/**@brief Different GAS sensor states.
 */
typedef enum
{
    GAS_STATE_IDLE,
    GAS_STATE_WARMUP,
    GAS_STATE_ACTIVE
}gas_state_t;

#define M_GAS_CALIB_INTERVAL_MS (1000 * 60 * 60) ///< Humidity and temperature calibration interval for the gas sensor [ms].
#define M_GAS_BASELINE_WRITE_MS (1000 * 60 * 30) ///< Stored baseline calibration delay for the gas sensor [ms].

static void temperature_timeout_handler(void * p_context); ///< Temperature handler, forward declaration.
static void humidity_timeout_handler(void * p_context);    ///< Humidity handler, forward declaration.

static ble_nus_t              m_nus;
static ble_tes_t              m_tes;            ///< Structure to identify the Thingy Environment Service.
static ble_tes_config_t     * m_p_config;       ///< Configuraion pointer.
static const ble_tes_config_t m_default_config = ENVIRONMENT_CONFIG_DEFAULT; ///< Default configuraion.
static m_gas_baseline_t     * m_p_baseline;     ///< Baseline pointer.
static const m_gas_baseline_t m_default_baseline = ENVIRONMENT_BASELINE_DEFAULT; ///< Default baseline.

static ble_tms_config_t     * m_config;
static const ble_tms_config_t m_default_config_motion = MOTION_DEFAULT_CONFIG;



static bool        m_get_humidity                   = false;
static bool        m_get_temperature                = false;
static bool        m_calib_gas_sensor               = false;
static gas_state_t m_gas_state                      = GAS_STATE_IDLE;
static bool        m_temp_humid_for_gas_calibration = false;    ///< Set when the gas sensor requires temperature and humidity for calibration.
static bool        m_temp_humid_for_ble_transfer    = false;    ///< Set when humidity or temperature is requested over BLE.

static uint32_t calibrate_gas_sensor(uint16_t humid, float temp);
static uint32_t gas_load_baseline_flash(uint16_t * p_gas_baseline);

APP_TIMER_DEF(temperature_timer_id);
APP_TIMER_DEF(pressure_timer_id);
APP_TIMER_DEF(humidity_timer_id);
APP_TIMER_DEF(color_timer_id);
APP_TIMER_DEF(gas_calib_timer_id);

// motion function define
static uint32_t m_motion_configuration_apply(ble_tms_config_t * p_config)
{
    uint32_t err_code;
    drv_motion_cfg_t motion_cfg;

    NULL_PARAM_CHECK(p_config);

    NRF_LOG_INFO("motion_cfg.pedo_interval_ms: %d\r\n", p_config->pedo_interval_ms);
    NRF_LOG_INFO("motion_cfg.temp_interval_ms: %d\r\n", p_config->temp_interval_ms);
    NRF_LOG_INFO("motion_cfg.compass_interval_ms: %d\r\n", p_config->compass_interval_ms);

    motion_cfg.pedo_interval_ms    = p_config->pedo_interval_ms;
    motion_cfg.temp_interval_ms    = p_config->temp_interval_ms;
    motion_cfg.compass_interval_ms = p_config->compass_interval_ms;
    motion_cfg.motion_freq_hz      = p_config->motion_freq_hz;
    motion_cfg.wake_on_motion      = p_config->wake_on_motion;

    err_code = drv_motion_config(&motion_cfg);
    APP_ERROR_CHECK(err_code);

    return NRF_SUCCESS;
}

uint32_t m_motion_sleep_prepare(bool wakeup)
{
    return drv_motion_sleep_prepare(wakeup);
}

static void drv_motion_evt_handler(drv_motion_evt_t const * p_evt, void * p_data, uint32_t size)
{ 

    switch (*p_evt)
    {
        case DRV_MOTION_EVT_RAW:
        {
            APP_ERROR_CHECK_BOOL(size == sizeof(int32_t) * RAW_PARAM_NUM);

            ble_tms_raw_t data;
            //uint8_t raw_acc_data[7]; // uint8_t  int8_t
            //uint8_t raw_gyro_data[7];
            //uint8_t raw_compass_data[7];
            uint8_t accident[3];
            uint16_t judge;
            uint32_t err_code;
            int32_t     * p_raw = (int32_t *)p_data;

            /* p_raw is in 16Q16 format. This is compressed for BLE transfer. */

            // Set upper and lower overflow limits.

            static const int16_t overflow_limit_upper[RAW_PARAM_NUM] = {
                                                    (1 << (RAW_Q_FORMAT_ACC_INTEGER_BITS - 1)) - 1,
                                                    (1 << (RAW_Q_FORMAT_ACC_INTEGER_BITS - 1)) - 1,
                                                    (1 << (RAW_Q_FORMAT_ACC_INTEGER_BITS - 1)) - 1,
                                                    (1 << (RAW_Q_FORMAT_GYR_INTEGER_BITS - 1)) - 1,
                                                    (1 << (RAW_Q_FORMAT_GYR_INTEGER_BITS - 1)) - 1,
                                                    (1 << (RAW_Q_FORMAT_GYR_INTEGER_BITS - 1)) - 1,
                                                    (1 << (RAW_Q_FORMAT_CMP_INTEGER_BITS - 1)) - 1,
                                                    (1 << (RAW_Q_FORMAT_CMP_INTEGER_BITS - 1)) - 1,
                                                    (1 << (RAW_Q_FORMAT_CMP_INTEGER_BITS - 1)) - 1};

            static const int16_t overflow_limit_lower[RAW_PARAM_NUM] = {
                                                    -(1 << (RAW_Q_FORMAT_ACC_INTEGER_BITS - 1)),
                                                    -(1 << (RAW_Q_FORMAT_ACC_INTEGER_BITS - 1)),
                                                    -(1 << (RAW_Q_FORMAT_ACC_INTEGER_BITS - 1)),
                                                    -(1 << (RAW_Q_FORMAT_GYR_INTEGER_BITS - 1)),
                                                    -(1 << (RAW_Q_FORMAT_GYR_INTEGER_BITS - 1)),
                                                    -(1 << (RAW_Q_FORMAT_GYR_INTEGER_BITS - 1)),
                                                    -(1 << (RAW_Q_FORMAT_CMP_INTEGER_BITS - 1)),
                                                    -(1 << (RAW_Q_FORMAT_CMP_INTEGER_BITS - 1)),
                                                    -(1 << (RAW_Q_FORMAT_CMP_INTEGER_BITS - 1))};

            int16_t overflow_check;

            for (uint8_t i = 0; i < RAW_PARAM_NUM; i++)
            {
                overflow_check = p_raw[i] >> 16;    // Right shift 16 to remove decimal part.

                if (overflow_check >= overflow_limit_upper[i])
                {
                    NRF_LOG_WARNING("p_raw[%d] over limit. Val: %d limit: %d \r\n", i, overflow_check, overflow_limit_upper[i]);
                    p_raw[i] = overflow_limit_upper[i] << 16;
                }
                else if (overflow_check < overflow_limit_lower[i])
                {
                    NRF_LOG_WARNING("p_raw[%d] below limit. Val: %d limit: %d \r\n", i, overflow_check, overflow_limit_lower[i]);
                    p_raw[i] = overflow_limit_lower[i] << 16;
                }
                else
                {
                    // No overflow has occured.
                }
            }

            data.accel.x =      (uint16_t)(p_raw[0] >> RAW_Q_FORMAT_ACC_INTEGER_BITS);
            data.accel.y =      (uint16_t)(p_raw[1] >> RAW_Q_FORMAT_ACC_INTEGER_BITS);
            data.accel.z =      (uint16_t)(p_raw[2] >> RAW_Q_FORMAT_ACC_INTEGER_BITS);

            data.gyro.x =       (uint16_t)(p_raw[3] >> RAW_Q_FORMAT_GYR_INTEGER_BITS);
            data.gyro.y =       (uint16_t)(p_raw[4] >> RAW_Q_FORMAT_GYR_INTEGER_BITS);
            data.gyro.z =       (uint16_t)(p_raw[5] >> RAW_Q_FORMAT_GYR_INTEGER_BITS);

            data.compass.y =   -(uint16_t)(p_raw[6] >> RAW_Q_FORMAT_CMP_INTEGER_BITS); // Changed axes and inverted. Corrected for rotation of axes.
            data.compass.x =    (uint16_t)(p_raw[7] >> RAW_Q_FORMAT_CMP_INTEGER_BITS); // Changed axes. Corrected for rotation of axes.
            data.compass.z =    (uint16_t)(p_raw[8] >> RAW_Q_FORMAT_CMP_INTEGER_BITS);

            #if NRF_LOG_ENABLED
                NRF_LOG_DEBUG("DRV_MOTION_EVT_RAW:\r\n");

                double f_buf;
                char buffer[8];

                f_buf = (double)p_raw[0];
                f_buf = f_buf/(1<<16);
                sprintf(buffer, "%.2f", f_buf);
                NRF_LOG_DEBUG(" accel.x [G's] = %s:\r\n", nrf_log_push(buffer));

                f_buf = (double)p_raw[1];
                f_buf = f_buf/(1<<16);
                sprintf(buffer, "%.2f", f_buf);
                NRF_LOG_DEBUG(" accel.y [G's] = %s:\r\n", nrf_log_push(buffer));

                f_buf = (double)p_raw[2];
                f_buf = f_buf/(1<<16);
                sprintf(buffer, "%.2f", f_buf);
                NRF_LOG_DEBUG(" accel.z [G's] = %s:\r\n", nrf_log_push(buffer));

                
                f_buf = (double)p_raw[3];
                f_buf = f_buf/(1<<16);
                sprintf(buffer, "%.2f", f_buf);
                NRF_LOG_DEBUG(" gyro.x [deg/s] = %s:\r\n", nrf_log_push(buffer));

                f_buf = (double)p_raw[4];
                f_buf = f_buf/(1<<16);
                sprintf(buffer, "%.2f", f_buf);
                NRF_LOG_DEBUG(" gyro.y [deg/s] = %s:\r\n", nrf_log_push(buffer));

                f_buf = (double)p_raw[5];
                f_buf = f_buf/(1<<16);
                sprintf(buffer, "%.2f", f_buf);
                NRF_LOG_DEBUG(" gyro.z [deg/s] = %s:\r\n", nrf_log_push(buffer));


                f_buf = (double)p_raw[7];  // Changed axes. Corrected for rotation of axes.
                f_buf = f_buf/(1<<16);
                sprintf(buffer, "%.2f", f_buf);
                NRF_LOG_DEBUG(" mag.x [uT] = %s:\r\n", nrf_log_push(buffer));

                f_buf = -(double)p_raw[6]; // Changed axes and inverted. Corrected for rotation of axes.
                f_buf = f_buf/(1<<16);
                sprintf(buffer, "%.2f", f_buf);
                NRF_LOG_DEBUG(" mag.y [uT] = %s:\r\n", nrf_log_push(buffer));

                f_buf = (double)p_raw[8];
                f_buf = f_buf/(1<<16);
                sprintf(buffer, "%.2f", f_buf);
                NRF_LOG_DEBUG(" mag.z [uT] = %s:\r\n", nrf_log_push(buffer));

            #endif

            //NRF_LOG_INFO("drv_motion_evt_handler raw x:, %d, - y:, %d, - z:, %d, \r\n", (uint16_t)(p_raw[0] >> RAW_Q_FORMAT_ACC_INTEGER_BITS),
            //                                                                            (uint16_t)(p_raw[1] >> RAW_Q_FORMAT_ACC_INTEGER_BITS),
            //                                                                            (uint16_t)(p_raw[2] >> RAW_Q_FORMAT_ACC_INTEGER_BITS));
#if 0
            raw_acc_data[0] = 0xF4;
            raw_acc_data[1] = 0xFF & (data.accel.x >> 8);
            raw_acc_data[2] = 0xFF & data.accel.x;
            raw_acc_data[3] = 0xFF & (data.accel.y >> 8);
            raw_acc_data[4] = 0xFF & data.accel.y;
            raw_acc_data[5] = 0XFF & (data.accel.z >> 8);
            raw_acc_data[6] = 0XFF & data.accel.z;
            if((m_nus.conn_handle != BLE_CONN_HANDLE_INVALID) && (m_nus.is_notification_enabled))
            {
                //if add when raw_acc_data over limit value
                NRF_LOG_INFO("motion data send,\r\n");
                err_code = ble_nus_string_send(&m_nus, raw_acc_data, 7);
                APP_ERROR_CHECK(err_code);
            }
            
            raw_gyro_data[0] = 0xF5;
            raw_gyro_data[1] = 0xFF & (data.gyro.x >> 8);
            raw_gyro_data[2] = 0xFF & data.gyro.x;
            raw_gyro_data[3] = 0xFF & (data.gyro.y >> 8);
            raw_gyro_data[4] = 0xFF & data.gyro.y;
            raw_gyro_data[5] = 0xFF & (data.gyro.z >> 8);
            raw_gyro_data[6] = 0xFF & data.gyro.z;
            if((m_nus.conn_handle != BLE_CONN_HANDLE_INVALID) && (m_nus.is_notification_enabled))
            {
                //if add when raw_acc_data over limit value
                err_code = ble_nus_string_send(&m_nus, raw_gyro_data, 7);
                APP_ERROR_CHECK(err_code);
            }

            raw_compass_data[0] = 0xF6;
            raw_compass_data[1] = 0xFF & (data.compass.x >> 8);
            raw_compass_data[2] = 0xFF & data.compass.x;
            raw_compass_data[3] = 0xFF & (data.compass.y >> 8);
            raw_compass_data[4] = 0xFF & data.compass.y;
            raw_compass_data[5] = 0xFF & (data.compass.z >> 8);
            raw_compass_data[6] = 0xFF & data.compass.z;
            if((m_nus.conn_handle != BLE_CONN_HANDLE_INVALID) && (m_nus.is_notification_enabled))
            {
                //if add when raw_acc_data over limit value
                err_code = ble_nus_string_send(&m_nus, raw_compass_data, 7);
                APP_ERROR_CHECK(err_code);
            }
#endif
            judge = (uint16_t)sqrt(data.accel.x*data.accel.x + data.accel.y*data.accel.y + data.accel.z*data.accel.z);
            //judge = (uint16_t)(data.accel.x + data.accel.y + data.accel.z);
            
            accident[0] = 0xF4;
            accident[1] = 0xFF &(judge >> 8);
            accident[2] = 0xFF & judge;
            NRF_LOG_INFO("accident data : %d \r\n", judge);\
            if(judge > LIMIT_ACC)
            {
                drv_speaker_sample_play(6);

                if((m_nus.conn_handle != BLE_CONN_HANDLE_INVALID) && (m_nus.is_notification_enabled))
                {
                    //if add when raw_acc_data over limit value
                    err_code = ble_nus_string_send(&m_nus, accident, 3);
                    APP_ERROR_CHECK(err_code);
                }
            }
            //(void)ble_tms_raw_set(&m_tms, &data);
        }
        break;

        case DRV_MOTION_EVT_QUAT:
        {
            APP_ERROR_CHECK_BOOL(size == sizeof(int32_t) * 4);

            ble_tms_quat_t data;
            int32_t      * p_quat = (int32_t *)p_data;

            data.w = p_quat[0];
            data.x = p_quat[1];
            data.y = p_quat[2];
            data.z = p_quat[3];

            #if NRF_LOG_ENABLED
                static const uint8_t QUAT_ELEMENTS = 4;
                double f_buf;
                char buffer[QUAT_ELEMENTS][7];

                for (uint8_t i = 0; i < QUAT_ELEMENTS; i++)
                {
                    f_buf = (double)p_quat[i];
                    f_buf = f_buf/(1<<30);
                    sprintf(buffer[i], "% 1.3f", f_buf);
                }

                NRF_LOG_DEBUG("DRV_MOTION_EVT_QUAT: w:%s x:%s y:%s z:%s\r\n", nrf_log_push(buffer[0]),
                                                                              nrf_log_push(buffer[1]),
                                                                              nrf_log_push(buffer[2]),
                                                                              nrf_log_push(buffer[3]));
            #endif

            //(void)ble_tms_quat_set(&m_tms, &data);
        }
        break;

        case DRV_MOTION_EVT_EULER:
        {
            APP_ERROR_CHECK_BOOL(size == sizeof(long) * 3);

            ble_tms_euler_t data;
            int32_t      * p_euler = (int32_t *)p_data;

            data.roll   = p_euler[0];
            data.pitch  = p_euler[1];
            data.yaw    = p_euler[2];

            NRF_LOG_DEBUG("DRV_MOTION_EVT_EULER, [deg]:  roll(x):%3d   pitch(y):%3d   yaw(z):%3d  \r\n", data.roll/(1<<16), data.pitch/(1<<16), data.yaw/(1<<16));

            //(void)ble_tms_euler_set(&m_tms, &data);
        }
        break;

        case DRV_MOTION_EVT_ROT_MAT:
        {
            APP_ERROR_CHECK_BOOL(size == sizeof(int32_t) * 9);

            ble_tms_rot_mat_t data;
            int32_t         * p_matrix = (int32_t *)p_data;

            data.matrix[0] = (int16_t)(p_matrix[0] >> 16);
            data.matrix[1] = (int16_t)(p_matrix[1] >> 16);
            data.matrix[2] = (int16_t)(p_matrix[2] >> 16);
            data.matrix[3] = (int16_t)(p_matrix[3] >> 16);
            data.matrix[4] = (int16_t)(p_matrix[4] >> 16);
            data.matrix[5] = (int16_t)(p_matrix[5] >> 16);
            data.matrix[6] = (int16_t)(p_matrix[6] >> 16);
            data.matrix[7] = (int16_t)(p_matrix[7] >> 16);
            data.matrix[8] = (int16_t)(p_matrix[8] >> 16);

            #if NRF_LOG_ENABLED
                static const uint8_t ROT_MAT_ELEMENTS = 9;
                char buffer[ROT_MAT_ELEMENTS][6];
                double tmp;
                for(uint8_t i = 0; i<ROT_MAT_ELEMENTS; i++)
                {
                    tmp = p_matrix[i]/(double)(1<<30);
                    sprintf(buffer[i], "% 1.2f", tmp);
                }
                
                NRF_LOG_DEBUG("DRV_MOTION_EVT_ROT_MAT:\r\n");
                NRF_LOG_DEBUG("[%s %s %s]\r\n", nrf_log_push(buffer[0]), nrf_log_push(buffer[1]), nrf_log_push(buffer[2]));
                NRF_LOG_DEBUG("[%s %s %s]\r\n", nrf_log_push(buffer[3]), nrf_log_push(buffer[4]), nrf_log_push(buffer[5]));
                NRF_LOG_DEBUG("[%s %s %s]\r\n", nrf_log_push(buffer[6]), nrf_log_push(buffer[7]), nrf_log_push(buffer[8]));
            #endif

            //(void)ble_tms_rot_mat_set(&m_tms, &data);
        }
        break;

        case DRV_MOTION_EVT_HEADING:
        {
            APP_ERROR_CHECK_BOOL(size == sizeof(long));
            ble_tms_heading_t heading = *(ble_tms_heading_t *)p_data;

            NRF_LOG_DEBUG("DRV_MOTION_EVT_HEADING [deg]:  h: %d\r\n", heading/(1<<16));

            //(void)ble_tms_heading_set(&m_tms, &heading);
        }
        break;

        case DRV_MOTION_EVT_GRAVITY:
        {
            uint8_t gravity_data[7];
            uint32_t err_code;

            APP_ERROR_CHECK_BOOL(size == sizeof(float) * 3);

            ble_tms_gravity_t data;
            float           * p_gravity = (float *)p_data;

            data.x = p_gravity[0];
            data.y = p_gravity[1];
            data.z = p_gravity[2];

            #if NRF_LOG_ENABLED
                static const uint8_t GRAVITY_ELEMENTS = 3;
                char buffer[GRAVITY_ELEMENTS][8];

                for (uint8_t i = 0; i<GRAVITY_ELEMENTS; i++)
                {
                    sprintf(buffer[i], "% 2.3f", p_gravity[i]);
                }

                NRF_LOG_DEBUG("DRV_MOTION_EVT_GRAVITY [m/s^2]:  [%s, %s, %s]\r\n", nrf_log_push(buffer[0]),
                                                                                   nrf_log_push(buffer[1]),
                                                                                   nrf_log_push(buffer[2]));
            NRF_LOG_INFO("gravity x: %d y: %d z: %d \r\n", data.x, data.y, data.z);
            #endif
            gravity_data[0] = 0xF5;
            gravity_data[1] = 0xFF & ((int16_t)data.x  >> 8);
            gravity_data[2] = 0xFF & (int16_t)data.x;
            gravity_data[3] = 0xFF & ((int16_t)data.y >> 8);
            gravity_data[4] = 0xFF & (int16_t)data.y;
            gravity_data[5] = 0XFF & ((int16_t)data.z >> 8);
            gravity_data[6] = 0XFF & (int16_t)data.z;
            if(((int)data.x == LIMIT_GRAVITY) && ((int)data.y == LIMIT_GRAVITY))
            {
                drv_speaker_sample_play(6);
                if((m_nus.conn_handle != BLE_CONN_HANDLE_INVALID) && (m_nus.is_notification_enabled))
                {
                    //if add when raw_acc_data over limit value
                    NRF_LOG_INFO("motion data send,\r\n");
                    err_code = ble_nus_string_send(&m_nus, gravity_data, 7);
                    APP_ERROR_CHECK(err_code);
                }
            }
            //ble_nus_gravity_send(&m_nus, &data);
        }
        break;

        case DRV_MOTION_EVT_TAP:
        {
            APP_ERROR_CHECK_BOOL(size == 2);

            ble_tms_tap_t data;
            uint8_t * p_tap = (uint8_t *)p_data;

            data.dir = p_tap[0];
            data.cnt = p_tap[1];

            NRF_LOG_DEBUG("DRV_MOTION_EVT_TAP: [%d %d]\r\n", data.cnt,
                                                             data.dir);

            //(void)ble_tms_tap_set(&m_tms, &data);
        }
        break;

        case DRV_MOTION_EVT_ORIENTATION:
        {
            APP_ERROR_CHECK_BOOL(size == sizeof(uint8_t));

            NRF_LOG_DEBUG("DRV_MOTION_EVT_ORIENTATION: %d\r\n", *(ble_tms_orientation_t *)p_data);

            //(void)ble_tms_orientation_set(&m_tms, (ble_tms_orientation_t *)p_data);
        }
        break;

        case DRV_MOTION_EVT_PEDOMETER:
        {
            APP_ERROR_CHECK_BOOL(size == sizeof(unsigned long) * 2);

            ble_tms_pedo_t  data;
            unsigned long * p_pedo = (unsigned long *)p_data;

            data.steps   = p_pedo[0];
            data.time_ms = p_pedo[1];

            NRF_LOG_DEBUG("DRV_MOTION_EVT_PEDOMETER: %d steps %d ms\r\n", p_pedo[0],
                                                                          p_pedo[1]);

            //(void)ble_tms_pedo_set(&m_tms, &data);
        }
        break;

        default:
            NRF_LOG_WARNING("drv_motion_evt_handler: Unknown data!\r\n");
            break;
    }

/*
    if(*p_evt == DRV_MOTION_EVT_RAW | *p_evt == DRV_MOTION_EVT_GRAVITY){
        APP_ERROR_CHECK_BOOL(size == sizeof(int32_t) * RAW_PARAM_NUM);

        ble_tms_raw_t data;
        //uint8_t raw_acc_data[7]; // uint8_t  int8_t
        //uint8_t raw_gyro_data[7];
        //uint8_t raw_compass_data[7];
        uint8_t accident[3];
        uint16_t judge;
        uint32_t err_code;
        int32_t     * p_raw = (int32_t *)p_data;

        /* p_raw is in 16Q16 format. This is compressed for BLE transfer. */
        // Set upper and lower overflow limits.
/*
        static const int16_t overflow_limit_upper[RAW_PARAM_NUM] = {
                                                (1 << (RAW_Q_FORMAT_ACC_INTEGER_BITS - 1)) - 1,
                                                (1 << (RAW_Q_FORMAT_ACC_INTEGER_BITS - 1)) - 1,
                                                (1 << (RAW_Q_FORMAT_ACC_INTEGER_BITS - 1)) - 1,
                                                (1 << (RAW_Q_FORMAT_GYR_INTEGER_BITS - 1)) - 1,
                                                (1 << (RAW_Q_FORMAT_GYR_INTEGER_BITS - 1)) - 1,
                                                (1 << (RAW_Q_FORMAT_GYR_INTEGER_BITS - 1)) - 1,
                                                (1 << (RAW_Q_FORMAT_CMP_INTEGER_BITS - 1)) - 1,
                                                (1 << (RAW_Q_FORMAT_CMP_INTEGER_BITS - 1)) - 1,
                                                (1 << (RAW_Q_FORMAT_CMP_INTEGER_BITS - 1)) - 1};

        static const int16_t overflow_limit_lower[RAW_PARAM_NUM] = {
                                                -(1 << (RAW_Q_FORMAT_ACC_INTEGER_BITS - 1)),
                                                -(1 << (RAW_Q_FORMAT_ACC_INTEGER_BITS - 1)),
                                                -(1 << (RAW_Q_FORMAT_ACC_INTEGER_BITS - 1)),
                                                -(1 << (RAW_Q_FORMAT_GYR_INTEGER_BITS - 1)),
                                                -(1 << (RAW_Q_FORMAT_GYR_INTEGER_BITS - 1)),
                                                -(1 << (RAW_Q_FORMAT_GYR_INTEGER_BITS - 1)),
                                                -(1 << (RAW_Q_FORMAT_CMP_INTEGER_BITS - 1)),
                                                -(1 << (RAW_Q_FORMAT_CMP_INTEGER_BITS - 1)),
                                                -(1 << (RAW_Q_FORMAT_CMP_INTEGER_BITS - 1))};

        int16_t overflow_check;

        for (uint8_t i = 0; i < RAW_PARAM_NUM; i++)
        {
            overflow_check = p_raw[i] >> 16;    // Right shift 16 to remove decimal part.
            if (overflow_check >= overflow_limit_upper[i])
            {
                NRF_LOG_WARNING("p_raw[%d] over limit. Val: %d limit: %d \r\n", i, overflow_check, overflow_limit_upper[i]);
                p_raw[i] = overflow_limit_upper[i] << 16;
            }
            else if (overflow_check < overflow_limit_lower[i])
            {
                NRF_LOG_WARNING("p_raw[%d] below limit. Val: %d limit: %d \r\n", i, overflow_check, overflow_limit_lower[i]);
                p_raw[i] = overflow_limit_lower[i] << 16;
            }
            else
            {
                // No overflow has occured.
            }
        }

        data.accel.x =      (uint16_t)(p_raw[0] >> RAW_Q_FORMAT_ACC_INTEGER_BITS);
        data.accel.y =      (uint16_t)(p_raw[1] >> RAW_Q_FORMAT_ACC_INTEGER_BITS);
        data.accel.z =      (uint16_t)(p_raw[2] >> RAW_Q_FORMAT_ACC_INTEGER_BITS);

        data.gyro.x =       (uint16_t)(p_raw[3] >> RAW_Q_FORMAT_GYR_INTEGER_BITS);
        data.gyro.y =       (uint16_t)(p_raw[4] >> RAW_Q_FORMAT_GYR_INTEGER_BITS);
        data.gyro.z =       (uint16_t)(p_raw[5] >> RAW_Q_FORMAT_GYR_INTEGER_BITS);

        data.compass.y =   -(uint16_t)(p_raw[6] >> RAW_Q_FORMAT_CMP_INTEGER_BITS); // Changed axes and inverted. Corrected for rotation of axes.
        data.compass.x =    (uint16_t)(p_raw[7] >> RAW_Q_FORMAT_CMP_INTEGER_BITS); // Changed axes. Corrected for rotation of axes.
        data.compass.z =    (uint16_t)(p_raw[8] >> RAW_Q_FORMAT_CMP_INTEGER_BITS);

        #if NRF_LOG_ENABLED
            NRF_LOG_DEBUG("DRV_MOTION_EVT_RAW:\r\n");

            double f_buf;
            char buffer1[8];

            f_buf = (double)p_raw[0];
            f_buf = f_buf/(1<<16);
            sprintf(buffer1, "%.2f", f_buf);
            NRF_LOG_DEBUG(" accel.x [G's] = %s:\r\n", nrf_log_push(buffer1));

            f_buf = (double)p_raw[1];
            f_buf = f_buf/(1<<16);
            sprintf(buffer1, "%.2f", f_buf);
            NRF_LOG_DEBUG(" accel.y [G's] = %s:\r\n", nrf_log_push(buffer1));

            f_buf = (double)p_raw[2];
            f_buf = f_buf/(1<<16);
            sprintf(buffer1, "%.2f", f_buf);
            NRF_LOG_DEBUG(" accel.z [G's] = %s:\r\n", nrf_log_push(buffer1));

                
            f_buf = (double)p_raw[3];
            f_buf = f_buf/(1<<16);
            sprintf(buffer1, "%.2f", f_buf);
            NRF_LOG_DEBUG(" gyro.x [deg/s] = %s:\r\n", nrf_log_push(buffer1));

            f_buf = (double)p_raw[4];
            f_buf = f_buf/(1<<16);
            sprintf(buffer1, "%.2f", f_buf);
            NRF_LOG_DEBUG(" gyro.y [deg/s] = %s:\r\n", nrf_log_push(buffer1));

            f_buf = (double)p_raw[5];
            f_buf = f_buf/(1<<16);
            sprintf(buffer1, "%.2f", f_buf);
            NRF_LOG_DEBUG(" gyro.z [deg/s] = %s:\r\n", nrf_log_push(buffer1));


            f_buf = (double)p_raw[7];  // Changed axes. Corrected for rotation of axes.
            f_buf = f_buf/(1<<16);
            sprintf(buffer1, "%.2f", f_buf);
            NRF_LOG_DEBUG(" mag.x [uT] = %s:\r\n", nrf_log_push(buffer1));

            f_buf = -(double)p_raw[6]; // Changed axes and inverted. Corrected for rotation of axes.
            f_buf = f_buf/(1<<16);
            sprintf(buffer1, "%.2f", f_buf);
            NRF_LOG_DEBUG(" mag.y [uT] = %s:\r\n", nrf_log_push(buffer1));
            f_buf = (double)p_raw[8];
            f_buf = f_buf/(1<<16);
            sprintf(buffer1, "%.2f", f_buf);
            NRF_LOG_DEBUG(" mag.z [uT] = %s:\r\n", nrf_log_push(buffer1));

        #endif

        //NRF_LOG_INFO("drv_motion_evt_handler raw x:, %d, - y:, %d, - z:, %d, \r\n", (uint16_t)(p_raw[0] >> RAW_Q_FORMAT_ACC_INTEGER_BITS),
        //                                                                            (uint16_t)(p_raw[1] >> RAW_Q_FORMAT_ACC_INTEGER_BITS),
        //                                                                            (uint16_t)(p_raw[2] >> RAW_Q_FORMAT_ACC_INTEGER_BITS));
#if 0
            raw_acc_data[0] = 0xF4;
            raw_acc_data[1] = 0xFF & (data.accel.x >> 8);
            raw_acc_data[2] = 0xFF & data.accel.x;
            raw_acc_data[3] = 0xFF & (data.accel.y >> 8);
            raw_acc_data[4] = 0xFF & data.accel.y;
            raw_acc_data[5] = 0XFF & (data.accel.z >> 8);
            raw_acc_data[6] = 0XFF & data.accel.z;
            if((m_nus.conn_handle != BLE_CONN_HANDLE_INVALID) && (m_nus.is_notification_enabled))
            {
                //if add when raw_acc_data over limit value
                NRF_LOG_INFO("motion data send,\r\n");
                err_code = ble_nus_string_send(&m_nus, raw_acc_data, 7);
                APP_ERROR_CHECK(err_code);
            }
            
            raw_gyro_data[0] = 0xF5;
            raw_gyro_data[1] = 0xFF & (data.gyro.x >> 8);
            raw_gyro_data[2] = 0xFF & data.gyro.x;
            raw_gyro_data[3] = 0xFF & (data.gyro.y >> 8);
            raw_gyro_data[4] = 0xFF & data.gyro.y;
            raw_gyro_data[5] = 0xFF & (data.gyro.z >> 8);
            raw_gyro_data[6] = 0xFF & data.gyro.z;
            if((m_nus.conn_handle != BLE_CONN_HANDLE_INVALID) && (m_nus.is_notification_enabled))
            {
                //if add when raw_acc_data over limit value
                err_code = ble_nus_string_send(&m_nus, raw_gyro_data, 7);
                APP_ERROR_CHECK(err_code);
            }

            raw_compass_data[0] = 0xF6;
            raw_compass_data[1] = 0xFF & (data.compass.x >> 8);
            raw_compass_data[2] = 0xFF & data.compass.x;
            raw_compass_data[3] = 0xFF & (data.compass.y >> 8);
            raw_compass_data[4] = 0xFF & data.compass.y;
            raw_compass_data[5] = 0xFF & (data.compass.z >> 8);
            raw_compass_data[6] = 0xFF & data.compass.z;
            if((m_nus.conn_handle != BLE_CONN_HANDLE_INVALID) && (m_nus.is_notification_enabled))
            {
                //if add when raw_acc_data over limit value
                err_code = ble_nus_string_send(&m_nus, raw_compass_data, 7);
                APP_ERROR_CHECK(err_code);
            }
#endif
        judge = (uint16_t)sqrt(data.accel.x*data.accel.x + data.accel.y*data.accel.y + data.accel.z*data.accel.z);
        //judge = (uint16_t)(data.accel.x + data.accel.y + data.accel.z);
            
        accident[0] = 0xF4;
        accident[1] = 0xFF &(judge >> 8);
        accident[2] = 0xFF & judge;
        NRF_LOG_INFO("accident data : %d \r\n", judge);\
        if(judge > LIMIT_ACC)
        {
            drv_speaker_sample_play(6);

            if((m_nus.conn_handle != BLE_CONN_HANDLE_INVALID) && (m_nus.is_notification_enabled))
            {
                //if add when raw_acc_data over limit value
                err_code = ble_nus_string_send(&m_nus, accident, 3);
                APP_ERROR_CHECK(err_code);
            }
        }
        //(void)ble_tms_raw_set(&m_tms, &data);
        
// gravity info
        uint8_t gravity_data[7];

        APP_ERROR_CHECK_BOOL(size == sizeof(float) * 3);
    
        ble_tms_gravity_t data1;
        float           * p_gravity = (float *)p_data;

        data1.x = p_gravity[0];
        data1.y = p_gravity[1];
        data1.z = p_gravity[2];

        #if NRF_LOG_ENABLED
        static const uint8_t GRAVITY_ELEMENTS = 3;
        char buffer2[GRAVITY_ELEMENTS][8];
        for (uint8_t i = 0; i<GRAVITY_ELEMENTS; i++)
        {
            sprintf(buffer2[i], "% 2.3f", p_gravity[i]);
        }

        NRF_LOG_DEBUG("DRV_MOTION_EVT_GRAVITY [m/s^2]:  [%s, %s, %s]\r\n", nrf_log_push(buffer2[0]),
                                                                                   nrf_log_push(buffer2[1]),
                                                                                   nrf_log_push(buffer2[2]));
        NRF_LOG_INFO("gravity x: %d y: %d z: %d \r\n", data1.x, data1.y, data1.z);
        #endif
        gravity_data[0] = 0xF5;
        gravity_data[1] = 0xFF & ((int16_t)data1.x  >> 8);
        gravity_data[2] = 0xFF & (int16_t)data1.x;
        gravity_data[3] = 0xFF & ((int16_t)data1.y >> 8);
        gravity_data[4] = 0xFF & (int16_t)data1.y;
        gravity_data[5] = 0XFF & ((int16_t)data1.z >> 8);
        gravity_data[6] = 0XFF & (int16_t)data1.z;
        if(data1.x == LIMIT_GRAVITY && data1.y == LIMIT_GRAVITY)
        {
            drv_speaker_sample_play(6);
            if((m_nus.conn_handle != BLE_CONN_HANDLE_INVALID) && (m_nus.is_notification_enabled))
            {
                //if add when raw_acc_data over limit value
                NRF_LOG_INFO("motion data1 send,\r\n");
                err_code = ble_nus_string_send(&m_nus, gravity_data, 7);
                APP_ERROR_CHECK(err_code);
            }
        }
        //ble_nus_gravity_send(&m_nus, &data);
    }else{
        NRF_LOG_WARNING("drv_motion_evt_handler: Unknown data!\r\n");
    }
*/
}


/**@brief Function for converting the temperature sample.
 */
static void temperature_conv_data(float in_temp, ble_tes_temperature_t * p_out_temp)
{
    float f_decimal;

    p_out_temp->integer = (int8_t)in_temp;
    f_decimal = in_temp - p_out_temp->integer;
    p_out_temp->decimal = (uint8_t)(f_decimal * 100.0f);
    NRF_LOG_DEBUG("temperature_conv_data: Temperature: ,%d.%d,C\r\n", p_out_temp->integer, p_out_temp->decimal);
}


/**@brief Function for converting the humidity sample.
 */
static void humidity_conv_data(uint8_t humid, ble_tes_humidity_t * p_out_humid)
{
   *p_out_humid = (uint8_t)humid;
   NRF_LOG_DEBUG("humidity_conv_data: Relative Humidty: ,%d,%%\r\n", humid);
}


/**@brief Function for converting the pressure sample.
 */
static void pressure_conv_data(float in_press, ble_tes_pressure_t * p_out_press)
{
    float f_decimal;

    p_out_press->integer = (int32_t)in_press;
    f_decimal = in_press - p_out_press->integer;
    p_out_press->decimal = (uint8_t)(f_decimal * 100.0f);
    NRF_LOG_DEBUG("pressure_conv_data: Pressure/Altitude: %d.%d Pa/m\r\n", p_out_press->integer, p_out_press->decimal);
}


/**@brief Pressure sensor event handler.
 */
static void drv_pressure_evt_handler(drv_pressure_evt_t const * p_event)
{
    switch (p_event->type)
    {
        case DRV_PRESSURE_EVT_DATA:
        {
            if (p_event->mode == DRV_PRESSURE_MODE_BAROMETER)
            {
                ble_tes_pressure_t pressure;
                pressure_conv_data(drv_pressure_get(),&pressure);
//                (void)ble_tes_pressure_set(&m_tes, &pressure);

            }
        }
        break;

        case DRV_PRESSURE_EVT_ERROR:
            APP_ERROR_CHECK_BOOL(false);
            break;

        default:
            break;
    }
}


/**@brief Humidity sensor event handler.
 */
static void drv_humidity_evt_handler(drv_humidity_evt_t event)
{
    uint32_t err_code;
    NRF_LOG_INFO("drv_humidity_evt_handler [0x%x]\n",event);
    if (event == DRV_HUMIDITY_EVT_DATA)
    {
        ble_tes_temperature_t temp;
        ble_tes_humidity_t humid;

        float temperature = drv_humidity_temp_get();
        uint16_t humidity = drv_humidity_get();
        uint8_t data[3];

        temperature_conv_data(temperature, &temp);
        humidity_conv_data(humidity, &humid);

        if (m_calib_gas_sensor == true)
        {
            err_code = calibrate_gas_sensor(humidity, temperature);
            APP_ERROR_CHECK(err_code);
            m_calib_gas_sensor = false;
        }
        //if (m_get_temperature == true)
        {
                data[0] = 0xF1;             /* 0xF1 : Temperature Indicator */
                data[1] = temp.integer;
                data[2] = temp.decimal;
          if(temp.integer>LIMIT_TEMP || temp.integer<0){
                if((m_nus.conn_handle != BLE_CONN_HANDLE_INVALID) && (m_nus.is_notification_enabled))
                {
                    err_code = ble_nus_string_send(&m_nus, data, 3);
                    APP_ERROR_CHECK(err_code);
                    NRF_LOG_INFO("++++ temperature data: mode: %d \r\n", temp.integer);
                }
                m_get_temperature = false;
          }
        }

        //if (m_get_humidity == true)
        {
            data[0] = 0xF2;             /* 0xF2 : Humidity Indicator */
            data[1] = humid;
            if((m_nus.conn_handle != BLE_CONN_HANDLE_INVALID) && (m_nus.is_notification_enabled))
            {
                err_code = ble_nus_string_send(&m_nus, data, 2);
                APP_ERROR_CHECK(err_code);
            }
            m_get_humidity = false;
        }
    }
    else
    {
        APP_ERROR_CHECK_BOOL(false);
    }
}


/**@brief Gas sensor data handler.
 */
static void drv_gas_data_handler(drv_gas_sensor_data_t const * p_data)
{   
    NRF_LOG_INFO(">>>>>> drv_gas_data_handler\r\n");
    if (p_data != NULL)
    {
        ble_tes_gas_t data;
        uint8_t gas_data[5];
        uint32_t err_code;
        data.eco2_ppm = p_data->ec02_ppm;
        data.tvoc_ppb = p_data->tvoc_ppb;

        NRF_LOG_INFO("gas_data_handler eCO2:, %d, - TVOC:, %d,\r\n", p_data->ec02_ppm,
                                                                      p_data->tvoc_ppb);

        gas_data[0] = 0xF3;             /* 0xF3 : Gas Indicator */
        gas_data[1] = 0xFF & (data.eco2_ppm >> 8);
        gas_data[2] = 0xFF & data.eco2_ppm;
        gas_data[3] = 0xFF & (data.tvoc_ppb >> 8);
        gas_data[4] = 0xFF & data.tvoc_ppb;
        if(p_data->ec02_ppm > LIMIT_GAS)
          {
              drv_speaker_sample_play(6);

              if((m_nus.conn_handle != BLE_CONN_HANDLE_INVALID) && (m_nus.is_notification_enabled))
              {
                NRF_LOG_INFO("gas data send,\r\n");
                err_code = ble_nus_string_send(&m_nus, gas_data, 5);
                APP_ERROR_CHECK(err_code);
              }
          }
        m_get_temperature = false;

        #if defined (ENV_DEBUG)
            uint32_t err_code;
            uint16_t dummy_baseline;
            uint16_t dummy_raw_adc;
            uint8_t dummy_current;
            err_code = drv_gas_sensor_baseline_get(&dummy_baseline);
            APP_ERROR_CHECK(err_code);
            err_code = drv_gas_sensor_raw_data_get(&dummy_current, &dummy_raw_adc);
            APP_ERROR_CHECK(err_code);
        #endif
    }
}


/**@brief Color sensor data handler.
 */
static void drv_color_data_handler(drv_color_data_t const * p_data)
{
    (void)drv_ext_light_off(DRV_EXT_RGB_LED_SENSE);

    if (p_data != NULL)
    {
        ble_tes_color_t data;
        NRF_LOG_DEBUG("color_data_handler r: %d - g: %d - b: %d - c: %d\r\n", p_data->red,
                                                                              p_data->green,
                                                                              p_data->blue,
                                                                              p_data->clear);
        data.red   = p_data->red;
        data.green = p_data->green;
        data.blue  = p_data->blue;
        data.clear = p_data->clear;
        //(void)ble_tes_color_set(&m_tes, &data);
    }
}


/**@brief Function for handling temperature timer timeout event.
 *
 * @details This function will read the temperature at the configured rate.
 */
static void temperature_timeout_handler(void * p_context)
{
    uint32_t err_code;
    m_get_temperature = true;
    NRF_LOG_INFO(">>> temperature_timeout_handler\n");
    // Read temperature from humidity sensor.
    err_code = drv_humidity_sample();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting temperature sampling.
 */
static uint32_t temperature_start(void)
{
    NRF_LOG_INFO("++++ Temp start: mode: 0x%x \r\n", m_p_config->temperature_interval_ms);
    uint32_t err_code;

    m_get_temperature = true;
    m_temp_humid_for_ble_transfer = true;

    err_code = drv_humidity_enable();
    RETURN_IF_ERROR(err_code);

    err_code = drv_humidity_sample();
    RETURN_IF_ERROR(err_code);

    err_code = app_timer_start(temperature_timer_id,
                               APP_TIMER_TICKS(m_p_config->temperature_interval_ms),
                               NULL);
    RETURN_IF_ERROR(err_code);

    return NRF_SUCCESS;
}


/**@brief Function for stopping temperature sampling.
 */
static uint32_t temperature_stop(bool disable_drv)
{
    uint32_t err_code;
    m_get_temperature = false;

    err_code = app_timer_stop(temperature_timer_id);
    RETURN_IF_ERROR(err_code);

    if (disable_drv)
    {
        m_temp_humid_for_ble_transfer = false;

        if (!m_temp_humid_for_gas_calibration) // If used by the gas sensor, do not turn off.
        {
            return drv_humidity_disable();
        }
        else
        {
            return NRF_SUCCESS;
        }
    }
    else
    {
        return NRF_SUCCESS;
    }
}


/**@brief Function for handling pressure timer timout event.
 *
 * @details This function will read the pressure at the configured rate.
 */
static void pressure_timeout_handler(void * p_context)
{
    uint32_t err_code;

    err_code = drv_pressure_sample();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting pressure sampling.
 */
static uint32_t pressure_start(void)
{
    NRF_LOG_INFO("++++ Pressure start: mode: 0x%x \r\n", m_p_config->pressure_interval_ms);
    uint32_t err_code;

    err_code = drv_pressure_enable();
    APP_ERROR_CHECK(err_code);

    err_code = drv_pressure_sample();
    APP_ERROR_CHECK(err_code);


    return app_timer_start(pressure_timer_id,
                           APP_TIMER_TICKS(m_p_config->pressure_interval_ms),
                           NULL);
}


/**@brief Function for stopping pressure sampling.
 */
static uint32_t pressure_stop(void)
{
    uint32_t err_code;

    err_code = app_timer_stop(pressure_timer_id);
    RETURN_IF_ERROR(err_code);

    return drv_pressure_disable();
}

/**@brief Function for handling gas sensor calibration.
 *
 * @details This function will read the humidity and temperature at the configured rate.
 */
static void gas_calib_timeout_handler(void * p_context)
{
    uint32_t err_code;
    uint16_t gas_baseline = 0;

    if (m_gas_state == GAS_STATE_WARMUP)
    {
        err_code = gas_load_baseline_flash(&gas_baseline);
        APP_ERROR_CHECK(err_code);

        if (gas_baseline == 0)
        {
            NRF_LOG_WARNING("No valid baseline stored in flash. Baseline not written to gas sensor.\r\n");
        }
        else
        {
            err_code = drv_gas_sensor_baseline_set(gas_baseline);
            APP_ERROR_CHECK(err_code);
        }

        m_gas_state = GAS_STATE_ACTIVE;

        (void)app_timer_stop(gas_calib_timer_id);

    }
    else if (m_gas_state == GAS_STATE_ACTIVE)
    {
        // For later implementation of gas sensor humidity and temperature calibration.
    }
    else
    {
        // Should never happen.
    }
}


/**@brief Sends the sampled humidity and temperature to the gas sensor for calibration.
 *
 * @note Not currently used.
 */
static uint32_t calibrate_gas_sensor(uint16_t humid, float temp)
{
    uint32_t err_code;

    if (m_temp_humid_for_gas_calibration) // Check that the gas sensor is still enabled.
    {
        uint16_t rh_ppt    = humid * 10;
        int32_t temp_mdeg = (int32_t)(temp * 1000.0f);

        NRF_LOG_DEBUG("Calibrating gas sensor: humid out %d [ppt], temp out: %d [mdeg C]\r\n", rh_ppt, temp_mdeg);

        err_code = drv_gas_sensor_calibrate_humid_temp(rh_ppt, temp_mdeg);
        RETURN_IF_ERROR(err_code);

        return NRF_SUCCESS;
    }
    else
    {
        return NRF_SUCCESS; // Do nothing.
    }
}


/**@brief Stops the humidity and temperature sensor, used to calibrate the gas sensor.
 *
 * @note Not currently used.
 */
static uint32_t humidity_temp_stop_for_gas_calibration(void)
{
    uint32_t err_code;

    m_temp_humid_for_gas_calibration = false;

    if (m_temp_humid_for_ble_transfer)
    {
        // The temprature or humidity is being transferred over BLE. Do nothing.
    }
    else
    {
        err_code = drv_humidity_disable();
        RETURN_IF_ERROR(err_code);
    }

    return app_timer_stop(gas_calib_timer_id);
}


/**@brief Function for handling humidity timer timout event.
 *
 * @details This function will read the humidity at the configured rate.
 */
static void humidity_timeout_handler(void * p_context)
{
    uint32_t err_code;
    m_get_humidity = true;

    // Sample humidity sensor.
    err_code = drv_humidity_sample();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting humidity sampling.
 */
static uint32_t humidity_start(void)
{
    NRF_LOG_INFO("++++ Humidity start: mode: 0x%x \r\n", m_get_humidity);
    uint32_t err_code;

    m_get_humidity = true;
    m_temp_humid_for_ble_transfer = true;

    err_code = drv_humidity_enable();
    RETURN_IF_ERROR(err_code);

    err_code = drv_humidity_sample();
    RETURN_IF_ERROR(err_code);

    return app_timer_start(humidity_timer_id,
                           APP_TIMER_TICKS(5000),
                           NULL);
}


/**@brief Function for stopping humidity sampling.
 */
static uint32_t humidity_stop(bool disable_drv)
{
    uint32_t err_code;

    m_get_humidity = false;

    err_code = app_timer_stop(humidity_timer_id);
    RETURN_IF_ERROR(err_code);

    if (disable_drv)
    {
        m_temp_humid_for_ble_transfer = false;

        if (!m_temp_humid_for_gas_calibration) // If used by the gas sensor, do not turn off.
        {
            return drv_humidity_disable();
        }
        else
        {
            return NRF_SUCCESS;
        }
    }
    else
    {
        return NRF_SUCCESS;
    }
}

/**@brief Loads the gas sensor baseline values from flash storage.
 */
static uint32_t gas_load_baseline_flash(uint16_t * p_gas_baseline)
{
    // No explicit flash load performed here, since this is done by the m_environment module at init and stored in m_p_config.
    switch(m_p_config->gas_interval_mode)
    {
        case GAS_MODE_250MS:
            *p_gas_baseline = m_p_baseline->mode_250ms;
            NRF_LOG_DEBUG("Gas sensor baseline loaded from flash, value 0x%04x, mode: GAS_MODE_250MS \r\n", *p_gas_baseline);
        break;

        case GAS_MODE_1S:
            *p_gas_baseline = m_p_baseline->mode_1s;
            NRF_LOG_DEBUG("Gas sensor baseline loaded from flash, value 0x%04x, mode: GAS_MODE_1S \r\n", *p_gas_baseline);
        break;

        case GAS_MODE_10S:
            *p_gas_baseline = m_p_baseline->mode_10s;
            NRF_LOG_DEBUG("Gas sensor baseline loaded from flash, value 0x%04x, mode: GAS_MODE_10S \r\n", *p_gas_baseline);
        break;

        case GAS_MODE_60S:
            *p_gas_baseline = m_p_baseline->mode_60s;
            NRF_LOG_DEBUG("Gas sensor baseline loaded from flash, value 0x%04x, mode: GAS_MODE_60S \r\n", *p_gas_baseline);
        break;

        default:
            return NRF_ERROR_INVALID_STATE;
    }

    return NRF_SUCCESS;
}


/**@brief Stores the gas sensor baseline values to flash storage.
 */
static uint32_t gas_store_baseline_flash(uint16_t baseline)
{
    uint32_t err_code;

    switch(m_p_config->gas_interval_mode)
    {
        case GAS_MODE_250MS:
            m_p_baseline->mode_250ms = baseline;
            NRF_LOG_DEBUG("Gas sensor baseline stored to flash, value 0x%04x, mode: GAS_MODE_250MS\r\n", baseline);
        break;

        case GAS_MODE_1S:
            m_p_baseline->mode_1s = baseline;
            NRF_LOG_DEBUG("Gas sensor baseline stored to flash, value 0x%04x, mode: GAS_MODE_1S\r\n", baseline);
        break;

        case GAS_MODE_10S:
            m_p_baseline->mode_10s = baseline;
            NRF_LOG_DEBUG("Gas sensor baseline stored to flash, value 0x%04x, mode: GAS_MODE_10S\r\n", baseline);
        break;

        case GAS_MODE_60S:
            m_p_baseline->mode_60s = baseline;
            NRF_LOG_DEBUG("Gas sensor baseline stored to flash, value 0x%04x, mode: GAS_MODE_60S\r\n", baseline);
        break;

        default:
            return NRF_ERROR_INVALID_STATE;
    }

    err_code = m_env_flash_baseline_store(m_p_baseline); // Store new baseline values to flash.
    RETURN_IF_ERROR(err_code);

    return NRF_SUCCESS;
}


static uint32_t gas_start(void)
{
    NRF_LOG_INFO("++++ Gas start: mode: 0x%x \r\n", m_p_config->gas_interval_mode);

    uint32_t err_code;
    drv_gas_sensor_mode_t mode;

    switch (m_p_config->gas_interval_mode)
    {
        case GAS_MODE_250MS:
            mode = DRV_GAS_SENSOR_MODE_250MS;
            break;
        case GAS_MODE_1S:
            mode = DRV_GAS_SENSOR_MODE_1S;
            break;
        case GAS_MODE_10S:
            mode = DRV_GAS_SENSOR_MODE_10S;
            break;
        case GAS_MODE_60S:
            mode = DRV_GAS_SENSOR_MODE_60S;
            break;
        default:
            mode = DRV_GAS_SENSOR_MODE_10S;
            break;
    }

    err_code = drv_gas_sensor_start(DRV_GAS_SENSOR_MODE_1S);
    RETURN_IF_ERROR(err_code);

    m_gas_state = GAS_STATE_WARMUP;

    return app_timer_start(gas_calib_timer_id,
                           APP_TIMER_TICKS(M_GAS_BASELINE_WRITE_MS),
                           NULL);
}


static uint32_t gas_stop(void)
{
    uint32_t err_code;
    uint16_t baseline;

    if (m_gas_state == GAS_STATE_ACTIVE)
    {
        err_code = humidity_temp_stop_for_gas_calibration();
        RETURN_IF_ERROR(err_code);

        err_code = drv_gas_sensor_baseline_get(&baseline);
        RETURN_IF_ERROR(err_code);

        err_code = gas_store_baseline_flash(baseline);
        RETURN_IF_ERROR(err_code);
    }

    m_gas_state = GAS_STATE_IDLE;

    return drv_gas_sensor_stop();
}


/**@brief Function for handling color timer timeout event.
 *
 * @details This function will read the color at the configured rate.
 */
static void color_timeout_handler(void * p_context)
{
    uint32_t                    err_code;
    drv_ext_light_rgb_intensity_t color;

    color.r = m_p_config->color_config.led_red;
    color.g = m_p_config->color_config.led_green;
    color.b = m_p_config->color_config.led_blue;
    (void)drv_ext_light_rgb_intensity_set(DRV_EXT_RGB_LED_SENSE, &color);

    err_code = drv_color_sample();
    APP_ERROR_CHECK(err_code);
}


static uint32_t color_start(void)
{
    NRF_LOG_INFO("++++ Color start: mode: 0x%x \r\n", m_p_config->color_interval_ms);
    uint32_t                    err_code;
    drv_ext_light_rgb_intensity_t color;

    color.r = m_p_config->color_config.led_red;
    color.g = m_p_config->color_config.led_green;
    color.b = m_p_config->color_config.led_blue;

    (void)drv_ext_light_rgb_intensity_set(DRV_EXT_RGB_LED_SENSE, &color);

    err_code = drv_color_start();
    APP_ERROR_CHECK(err_code);

    err_code = drv_color_sample();
    APP_ERROR_CHECK(err_code);

    return app_timer_start(color_timer_id,
                           APP_TIMER_TICKS(m_p_config->color_interval_ms),
                           NULL);
}


static uint32_t color_stop(void)
{
    uint32_t err_code;

    err_code = app_timer_stop(color_timer_id);
    APP_ERROR_CHECK(err_code);

    (void)drv_ext_light_off(DRV_EXT_RGB_LED_SENSE);

    err_code = drv_color_stop();
    APP_ERROR_CHECK(err_code);

    return NRF_SUCCESS;
}


static uint32_t config_verify(ble_tes_config_t * p_config)
{
    uint32_t err_code;

    if ( (p_config->temperature_interval_ms < BLE_TES_CONFIG_TEMPERATURE_INT_MIN)    ||
         (p_config->temperature_interval_ms > BLE_TES_CONFIG_TEMPERATURE_INT_MAX)    ||
         (p_config->pressure_interval_ms < BLE_TES_CONFIG_PRESSURE_INT_MIN)          ||
         (p_config->pressure_interval_ms > BLE_TES_CONFIG_PRESSURE_INT_MAX)          ||
         (p_config->humidity_interval_ms < BLE_TES_CONFIG_HUMIDITY_INT_MIN)          ||
         (p_config->humidity_interval_ms > BLE_TES_CONFIG_HUMIDITY_INT_MAX)          ||
         (p_config->color_interval_ms < BLE_TES_CONFIG_COLOR_INT_MIN)                ||
         (p_config->color_interval_ms > BLE_TES_CONFIG_COLOR_INT_MAX)                ||
         (p_config->gas_interval_mode < BLE_TES_CONFIG_GAS_MODE_MIN)                 ||
         ((int)p_config->gas_interval_mode > (int)BLE_TES_CONFIG_GAS_MODE_MAX))
    {
        err_code = m_env_flash_config_store((ble_tes_config_t *)&m_default_config);
        APP_ERROR_CHECK(err_code);
    }

    return NRF_SUCCESS;
}


/**@brief Function for applying the configuration.
 *
 */
static uint32_t config_apply(ble_tes_config_t * p_config)
{
#if 0
    uint32_t err_code;

    NULL_PARAM_CHECK(p_config);

    (void)temperature_stop(false);
    (void)pressure_stop();
    (void)humidity_stop(true);
    (void)color_stop();

    if ((p_config->temperature_interval_ms > 0) &&
        (m_tes.is_temperature_notif_enabled) )
    {
        err_code = temperature_start();
        APP_ERROR_CHECK(err_code);
    }

    if ((p_config->pressure_interval_ms > 0) &&
        (m_tes.is_pressure_notif_enabled) )
    {
        err_code = pressure_start();
        APP_ERROR_CHECK(err_code);
    }

    if ((p_config->humidity_interval_ms > 0) &&
        (m_tes.is_humidity_notif_enabled) )
    {
        err_code = humidity_start();
        APP_ERROR_CHECK(err_code);
    }

    if ((p_config->color_interval_ms > 0) &&
        (m_tes.is_color_notif_enabled) )
    {
        err_code = color_start();
        APP_ERROR_CHECK(err_code);
    }
#endif
    return NRF_SUCCESS;
}


/**@brief Function for handling event from the Thingy Environment Service.
 *
 * @details This function will process the data received from the Thingy Environment BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_tes    Thingy Environment Service structure.
 * @param[in] evt_type Thingy Environment Service event type.
 * @param[in] p_data   Event data.
 * @param[in] length   Length of the data.
 */
static void ble_tes_evt_handler( ble_tes_t        * p_tes,
                                 ble_tes_evt_type_t evt_type,
                                 uint8_t          * p_data,
                                 uint16_t           length)
{
#if 0
    uint32_t err_code;

    switch (evt_type)
    {
        case BLE_TES_EVT_NOTIF_TEMPERATURE:
            NRF_LOG_DEBUG("tes_evt_handler: BLE_TES_EVT_NOTIF_TEMPERATURE: %d\r\n", p_tes->is_temperature_notif_enabled);
            if (p_tes->is_temperature_notif_enabled)
            {
                err_code = temperature_start();
                APP_ERROR_CHECK(err_code);
            }
            else
            {
                err_code = temperature_stop(1);
                APP_ERROR_CHECK(err_code);
            }
            break;
        case BLE_TES_EVT_NOTIF_PRESSURE:
            NRF_LOG_DEBUG("tes_evt_handler: BLE_TES_EVT_NOTIF_PRESSURE: %d\r\n", p_tes->is_pressure_notif_enabled);
            if (p_tes->is_pressure_notif_enabled)
            {
                err_code = pressure_start();
                APP_ERROR_CHECK(err_code);
            }
            else
            {
                err_code = pressure_stop();
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BLE_TES_EVT_NOTIF_HUMIDITY:
            NRF_LOG_DEBUG("tes_evt_handler: BLE_TES_EVT_NOTIF_HUMIDITY: %d\r\n", p_tes->is_humidity_notif_enabled);
            if (p_tes->is_humidity_notif_enabled)
            {
                err_code = humidity_start();
                APP_ERROR_CHECK(err_code);
            }
            else
            {
                err_code = humidity_stop(p_tes->is_temperature_notif_enabled == false);
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BLE_TES_EVT_NOTIF_GAS:
            NRF_LOG_DEBUG("tes_evt_handler: BLE_TES_EVT_NOTIF_GAS: %d\r\n", p_tes->is_gas_notif_enabled);
            if (p_tes->is_gas_notif_enabled)
            {
                err_code = gas_start();
                APP_ERROR_CHECK(err_code);
            }
            else
            {
                err_code = gas_stop();
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BLE_TES_EVT_NOTIF_COLOR:
            NRF_LOG_DEBUG("tes_evt_handler: BLE_TES_EVT_NOTIF_COLOR: %d\r\n", p_tes->is_color_notif_enabled);
            if (p_tes->is_color_notif_enabled)
            {
                err_code = color_start();
                APP_ERROR_CHECK(err_code);
            }
            else
            {
                err_code = color_stop();
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BLE_TES_EVT_CONFIG_RECEIVED:
        {
            NRF_LOG_DEBUG("tes_evt_handler: BLE_TES_EVT_CONFIG_RECEIVED: %d\r\n", length);
            APP_ERROR_CHECK_BOOL(length == sizeof(ble_tes_config_t));

            err_code = m_env_flash_config_store((ble_tes_config_t *)p_data);
            APP_ERROR_CHECK(err_code);

            err_code = config_apply((ble_tes_config_t *)p_data);
            APP_ERROR_CHECK(err_code);
        }
        break;
        default:
            break;

    }
#endif
}

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
    uint32_t err_code;

    NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.\r\n");
    NRF_LOG_HEXDUMP_DEBUG(p_data, length);
}



/**@brief Function for initializing the Thingy Environment Service.
 *
 * @details This callback function will be called from the ble handling module to initialize the Thingy Environment service.
 *
 * @retval NRF_SUCCESS If initialization was successful.
 */
static uint32_t environment_service_init(bool major_minor_fw_ver_changed)
{
    uint32_t              err_code;
#if 0
    ble_tes_temperature_t temperature = {.integer = 0, .decimal = 0};
    ble_tes_pressure_t    pressure    = {.integer = 0, .decimal = 0};
    ble_tes_humidity_t    humidity    = 0;
    ble_tes_color_t       color       = {.red = 0, .green = 0, .blue = 0, .clear = 0};
    ble_tes_gas_t         gas         = {.eco2_ppm = 0, .tvoc_ppb = 0};
    ble_tes_init_t        tes_init;
#endif
    /**@brief Load configuration from flash. */
    err_code = m_env_flash_init(&m_default_config, &m_p_config, &m_default_baseline, &m_p_baseline);
    RETURN_IF_ERROR(err_code);

    err_code = m_motion_flash_init(&m_default_config_motion, &m_config);
    RETURN_IF_ERROR(err_code);

    if (major_minor_fw_ver_changed)
    {
        err_code = m_env_flash_config_store(&m_default_config);
        APP_ERROR_CHECK(err_code);

        err_code = m_env_flash_baseline_store(&m_default_baseline);
        APP_ERROR_CHECK(err_code);

        err_code = m_motion_flash_config_store(&m_default_config_motion);
        APP_ERROR_CHECK(err_code);
    }

    err_code = config_verify(m_p_config);
    RETURN_IF_ERROR(err_code);

    err_code = m_motion_configuration_apply(m_config);
    RETURN_IF_ERROR(err_code);
#if 0
    memset(&tes_init, 0, sizeof(tes_init));

    tes_init.p_init_temperature = &temperature;
    tes_init.p_init_pressure = &pressure;
    tes_init.p_init_humidity = &humidity;
    tes_init.p_init_color = &color;
    tes_init.p_init_gas = &gas;
    tes_init.p_init_config = m_p_config;
    tes_init.evt_handler = ble_tes_evt_handler;

    NRF_LOG_INFO("Init: ble_tes_init \r\n");
    err_code = ble_tes_init(&m_tes, &tes_init);
    RETURN_IF_ERROR(err_code);

    (void)config_apply(m_p_config);
#else
    ble_nus_init_t nus_init;

    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;
    NRF_LOG_INFO("environment, motion service init : ble_nus_init \r\n");

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
#endif
    return NRF_SUCCESS;
}


/**@brief Function for passing the BLE event to the Thingy Environment service.
 *
 * @details This callback function will be called from the BLE handling module.
 *
 * @param[in] p_ble_evt    Pointer to the BLE event.
 */
static void environment_on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    //ble_tes_on_ble_evt(&m_tes, p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);

    switch(p_ble_evt->header.evt_id )
    {
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("motion_on_ble_evt : BLE_GAP_EVT_DISCONNECTED \r\n");

            err_code = m_environment_stop();
            APP_ERROR_CHECK(err_code);

            err_code = drv_motion_disable(DRV_MOTION_FEATURE_MASK);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_GAP_EVT_CONNECTED:            
            NRF_LOG_INFO("environment_on_ble_evt, motion_on_ble_evt : BLE_GAP_EVT_CONNECTED \r\n");

            err_code = temperature_start();
            err_code = gas_start();
            APP_ERROR_CHECK(err_code);

            err_code = drv_motion_enable(DRV_MOTION_FEATURE_MASK);
            APP_ERROR_CHECK(err_code);
            break;
        default:
            break;
    }
}


/**@brief Function for initializing the humidity/temperature sensor
 */
static uint32_t humidity_sensor_init(const nrf_drv_twi_t * p_twi_instance)
{
    ret_code_t               err_code = NRF_SUCCESS;

    static const nrf_drv_twi_config_t twi_config =
    {
        .scl                = TWI_SCL,
        .sda                = TWI_SDA,
        .frequency          = NRF_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    drv_humidity_init_t    init_params =
    {
        .twi_addr            = HTS221_ADDR,
        .pin_int             = HTS_INT,
        .p_twi_instance      = p_twi_instance,
        .p_twi_cfg           = &twi_config,
        .evt_handler         = drv_humidity_evt_handler
    };

    err_code = drv_humidity_init(&init_params);

    return err_code;
}


static uint32_t pressure_sensor_init(const nrf_drv_twi_t * p_twi_instance)
{
    drv_pressure_init_t init_params;

    static const nrf_drv_twi_config_t twi_config =
    {
        .scl                = TWI_SCL,
        .sda                = TWI_SDA,
        .frequency          = NRF_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    init_params.twi_addr                = LPS22HB_ADDR;
    init_params.pin_int                 = LPS_INT;
    init_params.p_twi_instance          = p_twi_instance;
    init_params.p_twi_cfg               = &twi_config;
    init_params.evt_handler             = drv_pressure_evt_handler;
    init_params.mode                    = DRV_PRESSURE_MODE_BAROMETER;

    return drv_pressure_init(&init_params);
}


static uint32_t gas_sensor_init(const nrf_drv_twi_t * p_twi_instance)
{
    uint32_t       err_code;
    drv_gas_init_t init_params;

    static const nrf_drv_twi_config_t twi_config =
    {
        .scl                = TWI_SCL,
        .sda                = TWI_SDA,
        .frequency          = NRF_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    init_params.p_twi_instance = p_twi_instance;
    init_params.p_twi_cfg      = &twi_config;
    init_params.twi_addr       = CCS811_ADDR;
    init_params.data_handler   = drv_gas_data_handler;

    err_code = drv_gas_sensor_init(&init_params);
    RETURN_IF_ERROR(err_code);

    return NRF_SUCCESS;
}


static uint32_t color_sensor_init(const nrf_drv_twi_t * p_twi_instance)
{
    uint32_t err_code;
    drv_color_init_t init_params;

    static const nrf_drv_twi_config_t twi_config =
    {
        .scl                = TWI_SCL,
        .sda                = TWI_SDA,
        .frequency          = NRF_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    init_params.p_twi_instance = p_twi_instance;
    init_params.p_twi_cfg      = &twi_config;
    init_params.twi_addr       = BH1745_ADDR;
    init_params.data_handler   = drv_color_data_handler;

    err_code = drv_color_init(&init_params);
    RETURN_IF_ERROR(err_code);

    return NRF_SUCCESS;
}


uint32_t m_environment_start(void)
{
    return NRF_SUCCESS;
}


uint32_t m_environment_stop(void)
{
    uint32_t err_code;

    err_code = temperature_stop(false);
    APP_ERROR_CHECK(err_code);

    err_code = pressure_stop();
    APP_ERROR_CHECK(err_code);

    err_code = humidity_stop(true);
    APP_ERROR_CHECK(err_code);

    err_code = color_stop();
    APP_ERROR_CHECK(err_code);

    err_code = gas_stop();
    APP_ERROR_CHECK(err_code);

    return NRF_SUCCESS;
}


uint32_t m_environment_init(m_ble_service_handle_t * p_handle, m_environment_init_t * p_params, m_motion_init_t * m_p_params)
{
    uint32_t err_code;
    drv_motion_twi_init_t motion_params_mpu9250;
    drv_motion_twi_init_t motion_params_lis2dh12;

    static const nrf_drv_twi_config_t twi_config_mpu9250 =
    {
        .scl                = TWI_SCL,
        .sda                = TWI_SDA,
        .frequency          = NRF_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    static const nrf_drv_twi_config_t twi_config_lis2dh12 =
    {
        #if defined(THINGY_HW_v0_7_0)
            .scl = TWI_SCL,
            .sda = TWI_SDA,
        #elif  defined(THINGY_HW_v0_8_0)
            .scl = TWI_SCL,
            .sda = TWI_SDA,
        #elif  defined(THINGY_HW_v0_9_0)
            .scl = TWI_SCL,
            .sda = TWI_SDA,
        #else
            .scl = TWI_SCL_EXT,
            .sda = TWI_SDA_EXT,
        #endif
        .frequency          = NRF_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    NULL_PARAM_CHECK(p_handle);
    NULL_PARAM_CHECK(p_params);
    NULL_PARAM_CHECK(m_p_params);


    NRF_LOG_INFO("Init: \r\n");

    p_handle->ble_evt_cb = environment_on_ble_evt;
    p_handle->init_cb    = environment_service_init;

    motion_params_mpu9250.p_twi_instance = m_p_params->p_twi_instance;
    motion_params_mpu9250.p_twi_cfg      = &twi_config_mpu9250;

    motion_params_lis2dh12.p_twi_instance = m_p_params->p_twi_instance;
    motion_params_lis2dh12.p_twi_cfg      = &twi_config_lis2dh12;

    /**@brief Init drivers */
    err_code = pressure_sensor_init(p_params->p_twi_instance);
    APP_ERROR_CHECK(err_code);

    err_code = humidity_sensor_init(p_params->p_twi_instance);
    APP_ERROR_CHECK(err_code);

    err_code = gas_sensor_init(p_params->p_twi_instance);
    APP_ERROR_CHECK(err_code);

    err_code = color_sensor_init(p_params->p_twi_instance);
    APP_ERROR_CHECK(err_code);

    //motion Init drivers
    err_code = drv_motion_init(drv_motion_evt_handler, &motion_params_mpu9250, &motion_params_lis2dh12);
    RETURN_IF_ERROR(err_code);

    /**@brief Init application timers */
    err_code = app_timer_create(&temperature_timer_id, APP_TIMER_MODE_REPEATED, temperature_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&pressure_timer_id, APP_TIMER_MODE_REPEATED, pressure_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&humidity_timer_id, APP_TIMER_MODE_REPEATED, humidity_timeout_handler);
    RETURN_IF_ERROR(err_code);

    err_code = app_timer_create(&color_timer_id, APP_TIMER_MODE_REPEATED, color_timeout_handler);
    RETURN_IF_ERROR(err_code);

    err_code = app_timer_create(&gas_calib_timer_id, APP_TIMER_MODE_REPEATED, gas_calib_timeout_handler);
    RETURN_IF_ERROR(err_code);

    return NRF_SUCCESS;
}