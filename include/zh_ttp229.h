/**
 * @file zh_ttp229.h
 *
 * @brief Driver for TTP229 capacitive touch pad controller via SPI-like
 *        interface (SCL/SDO) using ESP-IDF RMT peripheral.
 *
 * This module provides a FreeRTOS-based driver for the TTP229 touch
 * controller. It uses the RMT transmitter to generate SCL clock pulses
 * and the RMT receiver to capture SDO data, enabling reading of touch
 * status from 8 or 16 pad configurations. Each touch event triggers an
 * ISR that initiates RMT transfer and posts an event with the touched
 * pad number.
 *
 * Key features:
 * - Supports 8-pad and 16-pad TTP229 configurations
 * - RMT-based SPI-like communication (no GPIO bit-banging)
 * - FreeRTOS task for debouncing and event dispatching
 * - Per-device unique numbering for multi-device support
 * - Error statistics tracking (RMT, queue, stack)
 */

#pragma once

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "driver/gpio.h"
#include "esp_event.h"
#include "zh_vector.h"

#define ZH_TTP229_INIT_CONFIG_DEFAULT()         \
    {                                           \
        .task_priority = 1,                     \
        .stack_size = configMINIMAL_STACK_SIZE, \
        .queue_size = 1,                        \
        .work_mode = ZH_TTP229_8_PAD,           \
        .rmt_tx_start_delay = 0,                \
        .debounce_time = 100,                   \
        .device_number = 0,                     \
        .scl_gpio = GPIO_NUM_MAX,               \
        .sdo_gpio = GPIO_NUM_MAX}

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief External task handle for the TTP229 ISR processing task.
     */
    extern TaskHandle_t zh_ttp229;

    /**
     * @brief Opaque handle returned by zh_ttp229_init().
     *
     * Internal fields are defined in zh_ttp229.c. Users must not access
     * the handle's members directly.
     */
    typedef struct _zh_ttp229_handle_t zh_ttp229_handle_t;

    /**
     * @brief TTP229 touch pad work mode (number of pads).
     */
    typedef enum
    {
        ZH_TTP229_8_PAD = 8,   /*!< 8-pad TTP229 configuration */
        ZH_TTP229_16_PAD = 16, /*!< 16-pad TTP229 configuration */
    } zh_ttp229_work_mode_t;

    /**
     * @brief Configuration structure for TTP229 initialization.
     *
     * Use ZH_TTP229_INIT_CONFIG_DEFAULT() to obtain sensible defaults
     * and override only the fields that differ from defaults.
     */
    typedef struct
    {
        uint16_t stack_size;             /*!< Task stack size in bytes. @note Minimum: configMINIMAL_STACK_SIZE */
        uint16_t debounce_time;          /*!< Button debounce time in milliseconds. @note Minimum: 10 ms */
        uint8_t task_priority;           /*!< FreeRTOS task priority. @note Minimum: 1 */
        uint8_t queue_size;              /*!< Event queue depth. @note Minimum: 1 */
        uint8_t device_number;           /*!< Unique device identifier. @note Must be > 0 */
        uint8_t rmt_tx_start_delay;      /*!< RMT TX delay in microseconds after SDO interrupt. @note Minimum: 15 us */
        gpio_num_t sdo_gpio;             /*!< SDO (Serial Data Out) GPIO pin */
        gpio_num_t scl_gpio;             /*!< SCL (Serial Clock) GPIO pin */
        zh_ttp229_work_mode_t work_mode; /*!< Touch pad configuration (8 or 16 pads) */
    } zh_ttp229_init_config_t;

    /**
     * @brief Error and runtime statistics for the TTP229 driver.
     *
     * Retrieved via zh_ttp229_get_stats(). Reset with zh_ttp229_reset_stats().
     */
    typedef struct
    {
        uint32_t rmt_driver_error;     /*!< Count of RMT driver errors */
        uint32_t event_post_error;     /*!< Count of esp_event_post failures */
        uint32_t queue_overflow_error; /*!< Count of event queue overflows */
        uint32_t min_stack_size;       /*!< Minimum free stack (bytes) observed */
    } zh_ttp229_stats_t;

    ESP_EVENT_DECLARE_BASE(ZH_TTP229);

    /**
     * @brief Event payload delivered on touch detection.
     *
     * Should be used with the ZH_TTP229 event base.
     */
    typedef struct
    {
        uint8_t pad_number;    /*!< Number of the pad that triggered the interrupt (1-based) */
        uint8_t device_number; /*!< Unique device identifier matching the init config */
    } zh_ttp229_event_on_isr_t;

    /**
     * @brief Initialize the TTP229 touch pad driver.
     *
     * Allocates a handle, validates the configuration, initializes RMT
     * TX/RMT channels, installs the GPIO ISR handler, and creates the
     * processing task. Supports multiple devices via unique device
     * numbers.
     *
     * @param[in] config Pointer to initialization configuration (must not be NULL)
     * @param[out] handle Pointer to unique touch pad handle (must not be NULL)
     *
     * @return ESP_OK on success
     * @return ESP_ERR_INVALID_ARG if config or handle is NULL, or if any configuration value is out of range
     * @return ESP_ERR_INVALID_STATE if the device is already initialized
     * @return ESP_FAIL on resource allocation or peripheral failure
     */
    esp_err_t zh_ttp229_init(const zh_ttp229_init_config_t *config, zh_ttp229_handle_t **handle);

    /**
     * @brief Deinitialize a TTP229 touch pad instance.
     *
     * Disables RMT channels, removes the GPIO ISR handler, frees the
     * handle, and removes the device from the internal device list.
     * The processing task and shared resources are deleted only when
     * the last device is deinitialized.
     *
     * @param[in,out] handle Pointer to unique touch pad handle (must not be NULL)
     *
     * @return ESP_OK on success
     * @return ESP_ERR_INVALID_ARG if handle is NULL or points to a device not found in the internal list
     */
    esp_err_t zh_ttp229_deinit(zh_ttp229_handle_t **handle);

    /**
     * @brief Retrieve a pointer to the current error statistics.
     *
     * The returned pointer remains valid until zh_ttp229_reset_stats()
     * is called or the module is deinitialized.
     *
     * @return Pointer to the static zh_ttp229_stats_t structure
     */
    const zh_ttp229_stats_t *zh_ttp229_get_stats(void);

    /**
     * @brief Reset all error and runtime statistics to zero.
     *
     * Call this function to clear accumulated counters. The
     * min_stack_size field is also cleared and will be updated on the
     * next processing cycle.
     */
    void zh_ttp229_reset_stats(void);

#ifdef __cplusplus
}
#endif