/**
 * @file zh_ttp229.h
 */

#pragma once

#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "driver/gpio.h"
#include "esp_event.h"

/**
 * @brief TTP229 touch pad initial default values.
 */
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

    extern TaskHandle_t zh_ttp229; /*!< Unique task handle. */

    /**
     * @brief Enumeration of TTP229 touch pad work mode.
     */
    typedef enum
    {
        ZH_TTP229_8_PAD = 8,
        ZH_TTP229_16_PAD = 16,
    } zh_ttp229_work_mode_t;

    /**
     * @brief Structure for initial initialization of TTP229 touch pad.
     */
    typedef struct
    {
        uint16_t stack_size;             /*!< Stack size for task for the TTP229 touch pad isr processing processing. @note The minimum size is configMINIMAL_STACK_SIZE. */
        uint16_t debounce_time;          /*!< Touch pad button debounce time. @note Minimum value is 10 (in milliseconds). */
        uint8_t task_priority;           /*!< Task priority for the TTP229 touch pad isr processing. @note Minimum value is 1. */
        uint8_t queue_size;              /*!< Queue size for task for the touch pad processing. @note Minimum value is 1. */
        uint8_t device_number;           /*!< Touch pad unique number. @note Must be greater than 0. */
        uint8_t rmt_tx_start_delay;      /*!< The time between the low-level pulse in the interrupt and the actual start time of the RMT signal generator (in microseconds). @note Minimum value is 15. */
        gpio_num_t sdo_gpio;             /*!< SDO GPIO. */
        gpio_num_t scl_gpio;             /*!< SCL GPIO. */
        zh_ttp229_work_mode_t work_mode; /*!< Touch pad work mode. */
    } zh_ttp229_init_config_t;

    /**
     * @brief TTP229 touch pad handle.
     */
    typedef struct
    {
        bool is_initialized;               /*!< Touch pad initialization flag. */
        gpio_num_t sdo_gpio;               /*!< SDO GPIO. */
        uint8_t device_number;             /*!< Touch pad unique number. */
        uint8_t rmt_tx_start_delay;        /*!< The time between the low-level pulse in the interrupt and the actual start time of the RMT signal generator (in microseconds). */
        uint16_t debounce_time;            /*!< Touch pad button debounce_time. */
        zh_ttp229_work_mode_t work_mode;   /*!< Touch pad work mode. */
        rmt_symbol_word_t rx_symbols;      /*!< Received symbols. */
        rmt_channel_handle_t tx_channel;   /*!< Unique RMT TX device handle. */
        rmt_channel_handle_t rx_channel;   /*!< Unique RMT RX device handle. */
        rmt_encoder_handle_t copy_encoder; /*!< Unique copy encoder handle. */
    } zh_ttp229_handle_t;

    /**
     * @brief Structure for error statistics storage.
     */
    typedef struct
    {
        uint32_t rmt_driver_error;     /*!< Number of RMT driver error. */
        uint32_t event_post_error;     /*!< Number of event post error. */
        uint32_t queue_overflow_error; /*!< Number of queue overflow error. */
        uint32_t min_stack_size;       /*!< Minimum free stack size. */
    } zh_ttp229_stats_t;

    ESP_EVENT_DECLARE_BASE(ZH_TTP229);

    /**
     * @brief Structure for sending data to the event handler when cause an interrupt.
     *
     * @note Should be used with ZH_TTP229 event base.
     */
    typedef struct
    {
        uint8_t pad_number;    /*!< The pad that caused the interrupt. */
        uint8_t device_number; /*!< Touch pad unique number. */
    } zh_ttp229_event_on_isr_t;

    /**
     * @brief Initialize TTP229 touch pad.
     *
     * @param[in] config Pointer to TTP229 initialized configuration structure. Can point to a temporary variable.
     * @param[out] handle Pointer to unique TTP229 handle.
     *
     * @attention I2C driver must be initialized first.
     *
     * @note Before initialize the expander recommend initialize zh_ttp229_init_config_t structure with default values.
     *
     * @code zh_ttp229_init_config_t config = ZH_TTP229_INIT_CONFIG_DEFAULT() @endcode
     *
     * @return ESP_OK if success or an error code otherwise.
     */
    esp_err_t zh_ttp229_init(const zh_ttp229_init_config_t *config, zh_ttp229_handle_t *handle);

    /**
     * @brief Deinitialize TTP229 touch pad.
     *
     * @param[in] handle Pointer to unique TTP229 handle.
     *
     * @return ESP_OK if success or an error code otherwise.
     */
    esp_err_t zh_ttp229_deinit(zh_ttp229_handle_t *handle);

    /**
     * @brief Get error statistics.
     *
     * @return Pointer to the statistics structure.
     */
    const zh_ttp229_stats_t *zh_ttp229_get_stats(void);

    /**
     * @brief Reset error statistics.
     */
    void zh_ttp229_reset_stats(void);

#ifdef __cplusplus
}
#endif