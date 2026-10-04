#include "zh_ttp229.h"

static const char *TAG = "zh_ttp229";

#define ZH_LOGI(msg, ...) ESP_LOGI(TAG, msg, ##__VA_ARGS__)
#define ZH_LOGE(msg, err, ...) ESP_LOGE(TAG, "[%s:%d:%s] " msg, __FILE__, __LINE__, esp_err_to_name(err), ##__VA_ARGS__)

#define ZH_ERROR_CHECK(cond, err, cleanup, msg, ...) \
    if (!(cond))                                     \
    {                                                \
        ZH_LOGE(msg, err, ##__VA_ARGS__);            \
        cleanup;                                     \
        return err;                                  \
    }

#define ZH_ERROR_CHECK_VOID(cond, cleanup, msg, ...) \
    if (!(cond))                                     \
    {                                                \
        ZH_LOGE(msg, ESP_FAIL, ##__VA_ARGS__);       \
        cleanup;                                     \
        return;                                      \
    }

#define ZH_ERROR_CHECK_CONT(cond, cleanup, msg, ...) \
    if (!(cond))                                     \
    {                                                \
        ZH_LOGE(msg, ESP_FAIL, ##__VA_ARGS__);       \
        cleanup;                                     \
    }

/**
 * @brief Internal handle for a single TTP229 device instance.
 *
 * Fields are initialized by zh_ttp229_init() and used by the
 * ISR handler, processing task, and deinitialization.
 */
struct _zh_ttp229_handle_t
{
    gpio_num_t sdo_gpio;               /*!< SDO (Serial Data Out) GPIO pin for touch data */
    gpio_num_t scl_gpio;               /*!< SCL (Serial Clock) GPIO pin for clock generation */
    uint8_t device_number;             /*!< Unique device number assigned at initialization */
    uint8_t rmt_tx_start_delay;        /*!< Time between SDO low-level pulse and RMT TX start (in microseconds) */
    uint8_t scl_symbols_count;         /*!< Number of SCL clock symbols to transmit (8 or 16) */
    uint16_t debounce_time;            /*!< Button debounce time in milliseconds */
    TickType_t last_event_tick;        /*!< Tick count of the last posted event, -1 before first event */
    volatile bool rx_needs_reset;      /*!< Flag requesting RMT RX channel reset, set from ISR context */
    rmt_symbol_word_t scl_symbols[16]; /*!< Precomputed SCL clock symbols for RMT TX (up to 16 pads) */
    rmt_symbol_word_t rx_symbols;      /*!< Single RMT RX symbol buffer captured from the SDO line */
    rmt_channel_handle_t tx_channel;   /*!< RMT TX channel handle (SCL clock generation) */
    rmt_channel_handle_t rx_channel;   /*!< RMT RX channel handle (SDO data capture) */
    rmt_encoder_handle_t copy_encoder; /*!< Copy encoder handle for RMT TX */
};

/**
 * @brief Queue entry passed from RMT RX callback to the ISR processing task.
 *
 * Contains a pointer to the TTP229 device handle for the device
 * that triggered the interrupt and the first RMT symbol captured
 * from the SDO line.
 */
typedef struct
{
    zh_ttp229_handle_t *handle;  /*!< Pointer to the TTP229 device handle */
    rmt_symbol_word_t rx_symbol; /*!< RMT symbol word captured from the SDO line */
} zh_ttp229_queue_t;

TaskHandle_t zh_ttp229 = NULL;             /*!< FreeRTOS task handle for the ISR processing task */
static QueueHandle_t _queue_handle = NULL; /*!< Internal event queue for RMT RX notifications */
static zh_ttp229_stats_t _stats = {0};     /*!< Accumulated error and runtime statistics */
static zh_vector_t *_vector = NULL;        /*!< Vector of device number values for all initialized TTP229 devices */

/**
 * @brief Validate the user-provided initialization configuration.
 *
 * Checks all parameter ranges, ensures work_mode is valid, and
 * verifies device number uniqueness across initialized devices.
 *
 * @param config Pointer to the user configuration
 * @param handle Pointer to the allocated handle (partially filled)
 *
 * @return ESP_OK if all checks pass
 * @return ESP_ERR_INVALID_ARG on invalid parameter values
 */
static esp_err_t _zh_ttp229_validate_config(const zh_ttp229_init_config_t *config, zh_ttp229_handle_t *handle);

/**
 * @brief Configure GPIO for SDO input with positive-edge interrupt.
 *
 * Checks that both GPIO pins are valid and different, installs the ISR
 * service if not yet installed, configures SDO as input with
 * GPIO_INTR_POSEDGE, and registers the ISR handler.
 *
 * @param config Pointer to the user configuration
 * @param handle Pointer to the handle (stores sdo_gpio and scl_gpio)
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if GPIO pins are invalid or identical
 * @return ESP_FAIL on ISR service or handler installation failure
 */
static esp_err_t _zh_ttp229_gpio_init(const zh_ttp229_init_config_t *config, zh_ttp229_handle_t *handle);

/**
 * @brief Initialize RMT TX and RX channels with callbacks.
 *
 * Creates TX channel on SCL GPIO, RX channel on SDO GPIO, allocates
 * the copy encoder, enables both channels, and registers the RX done
 * callback.
 *
 * @param config Pointer to the user configuration
 * @param handle Pointer to the handle (stores RMT handles)
 *
 * @return ESP_OK on success
 * @return ESP_FAIL on RMT channel, encoder, or callback registration failure
 */
static esp_err_t _zh_ttp229_rmt_init(const zh_ttp229_init_config_t *config, zh_ttp229_handle_t *handle);

/**
 * @brief Create the shared event queue on first device initialization.
 *
 * The queue is created only when vector_size equals 1 (first device).
 *
 * @param config Pointer to the user configuration
 *
 * @return ESP_OK on success
 * @return ESP_FAIL if queue allocation fails
 */
static esp_err_t _zh_ttp229_resources_init(const zh_ttp229_init_config_t *config);

/**
 * @brief Create the ISR processing task on first device initialization.
 *
 * The task is created only when vector_size equals 1 (first device).
 *
 * @param config Pointer to the user configuration
 *
 * @return ESP_OK on success
 * @return ESP_FAIL if task creation fails
 */
static esp_err_t _zh_ttp229_task_init(const zh_ttp229_init_config_t *config);

/**
 * @brief Roll back shared resources (queue, task) if the failed init was
 *        the first (and only) device.
 *
 * Must be called BEFORE zh_vector_delete_back(&_vector) — otherwise the
 * vector size will no longer reflect "first device".
 */
static void _zh_ttp229_rollback_task_and_resources(void);

/**
 * @brief GPIO ISR handler triggered on SDO positive edge.
 *
 * Disables further GPIO interrupts, arms RMT receive on SDO, emits the
 * SDO start pulse, and issues the SCL clock pulses via RMT transmit on
 * SCL. On a transmit failure the RX reset flag is set and a notify entry
 * is enqueued so the processing task can recover the RX channel; GPIO
 * interrupts are re-enabled on any error path.
 *
 * @param arg Pointer to the zh_ttp229_handle_t for this device
 */
static void _zh_ttp229_isr_handler(void *arg);

/**
 * @brief FreeRTOS task that processes RMT RX data and posts events.
 *
 * Receives queue entries from the RMT RX callback, resets the RX channel
 * when flagged, decodes the pad number from the entry's rx_symbol
 * duration1, and posts a ZH_TTP229 event for a detected touch. Touches
 * within the debounce window are dropped by comparing tick counts rather
 * than delaying. Re-enables GPIO interrupts after each entry and runs
 * until the queue is deleted.
 *
 * @param pvParameter Unused
 */
static void _zh_ttp229_isr_processing_task(void *pvParameter);

/**
 * @brief RMT RX done callback that enqueues data to the processing task.
 *
 * Returns early when the queue is not yet created. Otherwise copies the
 * received symbol into a zh_ttp229_queue_t entry and sends it to
 * _queue_handle. On a full queue the overflow error counter is incremented
 * and GPIO interrupts are re-enabled so the device is not left stalled.
 *
 * @param channel RMT channel handle (unused)
 * @param edata RX done event data (unused)
 * @param user_data Pointer to the zh_ttp229_handle_t
 *
 * @return true if a context switch was requested
 * @return false otherwise
 */
static bool _zh_ttp229_rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data);

ESP_EVENT_DEFINE_BASE(ZH_TTP229);

esp_err_t zh_ttp229_init(const zh_ttp229_init_config_t *config, zh_ttp229_handle_t **handle)
{
    ZH_LOGI("Touch pad initialization started.");
    ZH_ERROR_CHECK(config != NULL && handle != NULL, ESP_ERR_INVALID_ARG, NULL, "Touch pad initialization failed. Invalid argument.");
    ZH_ERROR_CHECK(*handle == NULL, ESP_ERR_INVALID_STATE, NULL, "Touch pad initialization failed. Touch pad is already initialized.");
    *handle = heap_caps_calloc(1, sizeof(zh_ttp229_handle_t), MALLOC_CAP_8BIT);
    ZH_ERROR_CHECK(*handle != NULL, ESP_ERR_NO_MEM, NULL, "Touch pad initialization failed. Failed to allocate touch pad handle.");
    ZH_ERROR_CHECK(_zh_ttp229_validate_config(config, *handle) == ESP_OK, ESP_ERR_INVALID_ARG, heap_caps_free(*handle); *handle = NULL, "Touch pad initialization failed. Initial configuration check failed.");
    if (_vector == NULL)
    {
        ZH_ERROR_CHECK(zh_vector_init(&_vector, (uint16_t)sizeof(uint8_t)) == ESP_OK, ESP_FAIL, heap_caps_free(*handle); *handle = NULL, "Touch pad initialization failed. Failed to create vector.");
    }
    ZH_ERROR_CHECK(zh_vector_push_back(&_vector, &config->device_number) == ESP_OK, ESP_FAIL, heap_caps_free(*handle); *handle = NULL, "Touch pad initialization failed. Failed to add vector data.");
    ZH_ERROR_CHECK(_zh_ttp229_resources_init(config) == ESP_OK, ESP_FAIL, _zh_ttp229_rollback_task_and_resources();
                   {ZH_ERROR_CHECK_CONT(zh_vector_delete_back(&_vector) == ESP_OK, NULL, "Failed delete vector data.")};
                   heap_caps_free(*handle); *handle = NULL, "Touch pad initialization failed. Resources initialization failed.");
    ZH_ERROR_CHECK(_zh_ttp229_task_init(config) == ESP_OK, ESP_FAIL, _zh_ttp229_rollback_task_and_resources();
                   {ZH_ERROR_CHECK_CONT(zh_vector_delete_back(&_vector) == ESP_OK, NULL, "Failed delete vector data.")};
                   heap_caps_free(*handle); *handle = NULL, "Touch pad initialization failed. Processing task initialization failed.");
    (*handle)->debounce_time = config->debounce_time;
    (*handle)->last_event_tick = -1;
    ZH_ERROR_CHECK(_zh_ttp229_gpio_init(config, *handle) == ESP_OK, ESP_FAIL, _zh_ttp229_rollback_task_and_resources();
                   {ZH_ERROR_CHECK_CONT(zh_vector_delete_back(&_vector) == ESP_OK, NULL, "Failed delete vector data.")};
                   heap_caps_free(*handle); *handle = NULL, "Touch pad initialization failed. GPIO initialization failed.");
    ZH_ERROR_CHECK(_zh_ttp229_rmt_init(config, *handle) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK_CONT(gpio_isr_handler_remove((*handle)->sdo_gpio) == ESP_OK, NULL, "Remove GPIO isr handler failed.")};
                   {ZH_ERROR_CHECK_CONT(gpio_reset_pin((*handle)->sdo_gpio) == ESP_OK, NULL, "Reset GPIO failed.")};
                   {ZH_ERROR_CHECK_CONT(gpio_reset_pin((*handle)->scl_gpio) == ESP_OK, NULL, "Reset GPIO failed.")};
                   _zh_ttp229_rollback_task_and_resources();
                   {ZH_ERROR_CHECK_CONT(zh_vector_delete_back(&_vector) == ESP_OK, NULL, "Failed delete vector data.")};
                   heap_caps_free(*handle); *handle = NULL, "Touch pad initialization failed. RMT initialization failed.");
    if (_stats.min_stack_size == 0)
    {
        _stats.min_stack_size = config->stack_size;
    }
    (*handle)->rx_needs_reset = false;
    ZH_LOGI("Touch pad initialization completed successfully.");
    return ESP_OK;
}

esp_err_t zh_ttp229_deinit(zh_ttp229_handle_t **handle)
{
    ZH_LOGI("Touch pad deinitialization started.");
    ZH_ERROR_CHECK(handle != NULL && *handle != NULL, ESP_ERR_INVALID_ARG, NULL, "Touch pad deinitialization failed. Invalid argument.");
    int32_t index = 0;
    ZH_ERROR_CHECK(zh_vector_find_item(&_vector, &(*handle)->device_number, &index) == ESP_OK, ESP_FAIL, NULL, "Touch pad deinitialization failed. Failed to find vector item.");
    ZH_ERROR_CHECK_CONT(zh_vector_delete_item(&_vector, (uint16_t)index) == ESP_OK, NULL, "Touch pad deinitialization failed. Vector delete item failed.");
    ZH_ERROR_CHECK_CONT(gpio_isr_handler_remove((*handle)->sdo_gpio) == ESP_OK, NULL, "Touch pad deinitialization failed. Remove GPIO isr handler failed.");
    ZH_ERROR_CHECK_CONT(rmt_disable((*handle)->tx_channel) == ESP_OK, NULL, "Touch pad deinitialization failed. TX channel disable failed.");
    ZH_ERROR_CHECK_CONT(rmt_disable((*handle)->rx_channel) == ESP_OK, NULL, "Touch pad deinitialization failed. RX channel disable failed.");
    ZH_ERROR_CHECK_CONT(rmt_del_encoder((*handle)->copy_encoder) == ESP_OK, NULL, "Touch pad deinitialization failed. Delete encoder failed.");
    ZH_ERROR_CHECK_CONT(rmt_del_channel((*handle)->tx_channel) == ESP_OK, NULL, "Touch pad deinitialization failed. Delete TX channel failed.");
    ZH_ERROR_CHECK_CONT(rmt_del_channel((*handle)->rx_channel) == ESP_OK, NULL, "Touch pad deinitialization failed. Delete RX channel failed.");
    ZH_ERROR_CHECK_CONT(gpio_reset_pin((*handle)->sdo_gpio) == ESP_OK, NULL, "Touch pad deinitialization failed. Reset GPIO failed.");
    ZH_ERROR_CHECK_CONT(gpio_reset_pin((*handle)->scl_gpio) == ESP_OK, NULL, "Touch pad deinitialization failed. Reset GPIO failed.");
    (*handle)->rx_channel = NULL;
    (*handle)->tx_channel = NULL;
    uint16_t vector_size = 0;
    ZH_ERROR_CHECK(zh_vector_get_size(&_vector, &vector_size) == ESP_OK, ESP_FAIL, heap_caps_free(*handle); *handle = NULL, "Touch pad deinitialization failed. Failed to get vector size.");
    if (vector_size == 0)
    {
        if (zh_ttp229 != NULL)
        {
            vTaskDelete(zh_ttp229);
            zh_ttp229 = NULL;
        }
        if (_queue_handle != NULL)
        {
            vQueueDelete(_queue_handle);
            _queue_handle = NULL;
        }
        ZH_ERROR_CHECK(zh_vector_free(&_vector) == ESP_OK, ESP_FAIL, heap_caps_free(*handle); *handle = NULL, "Touch pad deinitialization failed. Free vector failed.");
    }
    heap_caps_free(*handle);
    *handle = NULL;
    ZH_LOGI("Touch pad deinitialization completed successfully.");
    return ESP_OK;
}

const zh_ttp229_stats_t *zh_ttp229_get_stats(void)
{
    return &_stats;
}

void zh_ttp229_reset_stats(void)
{
    ZH_LOGI("Error statistic reset started.");
    _stats.rmt_driver_error = 0;
    _stats.event_post_error = 0;
    _stats.queue_overflow_error = 0;
    _stats.min_stack_size = 0;
    ZH_LOGI("Error statistic reset successfully.");
}

static esp_err_t _zh_ttp229_validate_config(const zh_ttp229_init_config_t *config, zh_ttp229_handle_t *handle)
{
    ZH_ERROR_CHECK(config->task_priority >= 1 && config->stack_size >= configMINIMAL_STACK_SIZE, ESP_ERR_INVALID_ARG, NULL, "Invalid task settings.");
    ZH_ERROR_CHECK(config->queue_size >= 1, ESP_ERR_INVALID_ARG, NULL, "Invalid queue size.");
    ZH_ERROR_CHECK(config->debounce_time >= 10, ESP_ERR_INVALID_ARG, NULL, "Invalid debounce time.");
    ZH_ERROR_CHECK(config->rmt_tx_start_delay >= 15, ESP_ERR_INVALID_ARG, NULL, "Invalid RMT TX start delay.");
    ZH_ERROR_CHECK(config->work_mode == ZH_TTP229_8_PAD || config->work_mode == ZH_TTP229_16_PAD, ESP_ERR_INVALID_ARG, NULL, "Invalid touch pad work mode.");
    ZH_ERROR_CHECK(config->device_number > 0, ESP_ERR_INVALID_ARG, NULL, "Invalid touch pad number.");
    if (_vector != NULL)
    {
        int32_t index = 0;
        ZH_ERROR_CHECK(zh_vector_find_item(&_vector, &config->device_number, &index) == ESP_ERR_NOT_FOUND, ESP_ERR_INVALID_ARG, NULL, "Touch pad number already present.");
    }
    handle->rmt_tx_start_delay = config->rmt_tx_start_delay;
    handle->device_number = config->device_number;
    return ESP_OK;
}

static esp_err_t _zh_ttp229_gpio_init(const zh_ttp229_init_config_t *config, zh_ttp229_handle_t *handle)
{
    ZH_ERROR_CHECK(config->scl_gpio < GPIO_NUM_MAX && config->sdo_gpio < GPIO_NUM_MAX, ESP_ERR_INVALID_ARG, NULL, "Invalid GPIO number.");
    ZH_ERROR_CHECK(config->scl_gpio != config->sdo_gpio, ESP_ERR_INVALID_ARG, NULL, "SCL GPIO and SDO GPIO is same.");
    gpio_config_t sdo_config = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << config->sdo_gpio),
        .intr_type = GPIO_INTR_POSEDGE};
    ZH_ERROR_CHECK(gpio_config(&sdo_config) == ESP_OK, ESP_FAIL, NULL, "GPIO initialization failed.");
    esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_LOWMED);
    ZH_ERROR_CHECK(err == ESP_OK || err == ESP_ERR_INVALID_STATE, ESP_FAIL,
                   {ZH_ERROR_CHECK_CONT(gpio_reset_pin(config->sdo_gpio) == ESP_OK, NULL, "Reset GPIO failed.")}, "Failed install isr service.");
    ZH_ERROR_CHECK(gpio_isr_handler_add(config->sdo_gpio, _zh_ttp229_isr_handler, handle) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK_CONT(gpio_reset_pin(config->sdo_gpio) == ESP_OK, NULL, "Reset GPIO failed.")}, "Interrupt initialization failed.");
    handle->sdo_gpio = config->sdo_gpio;
    handle->scl_gpio = config->scl_gpio;
    return ESP_OK;
}

static esp_err_t _zh_ttp229_rmt_init(const zh_ttp229_init_config_t *config, zh_ttp229_handle_t *handle)
{
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = config->scl_gpio,
#if defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S2)
        .mem_block_symbols = 64,
#else
        .mem_block_symbols = 48,
#endif
        .resolution_hz = 1000000,
        .trans_queue_depth = 1,
        .flags.init_level = 1,
    };
    ZH_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &handle->tx_channel) == ESP_OK, ESP_FAIL, NULL, "TX channel creation failed.");
    rmt_copy_encoder_config_t copy_encoder_config = {};
    ZH_ERROR_CHECK(rmt_new_copy_encoder(&copy_encoder_config, &handle->copy_encoder) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK_CONT(rmt_del_channel(handle->tx_channel) == ESP_OK, NULL, "Delete channel failed.")}, "Copy encoder creation failed.");
    ZH_ERROR_CHECK(rmt_enable(handle->tx_channel) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK_CONT(rmt_del_encoder(handle->copy_encoder) == ESP_OK, NULL, "Delete encoder failed.")};
                   {ZH_ERROR_CHECK_CONT(rmt_del_channel(handle->tx_channel) == ESP_OK, NULL, "Delete channel failed.")}, "Enable TX channel failed.");
    rmt_rx_channel_config_t rx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = config->sdo_gpio,
#if defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S2)
        .mem_block_symbols = 64,
#else
        .mem_block_symbols = 48,
#endif
        .resolution_hz = 1000000,
    };
    handle->scl_symbols_count = (uint8_t)config->work_mode;
    for (uint8_t i = 0; i < handle->scl_symbols_count; ++i)
    {
        handle->scl_symbols[i] = (rmt_symbol_word_t){
            .duration0 = 2,
            .level0 = 0,
            .duration1 = 2,
            .level1 = 1,
        };
    }
    ZH_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config, &handle->rx_channel) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK_CONT(rmt_disable(handle->tx_channel) == ESP_OK, NULL, "TX channel disable failed.")};
                   {ZH_ERROR_CHECK_CONT(rmt_del_encoder(handle->copy_encoder) == ESP_OK, NULL, "Delete encoder failed.")};
                   {ZH_ERROR_CHECK_CONT(rmt_del_channel(handle->tx_channel) == ESP_OK, NULL, "Delete TX channel failed.")}, "RX channel creation failed.");
    ZH_ERROR_CHECK(rmt_enable(handle->rx_channel) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK_CONT(rmt_disable(handle->tx_channel) == ESP_OK, NULL, "TX channel disable failed.")};
                   {ZH_ERROR_CHECK_CONT(rmt_del_encoder(handle->copy_encoder) == ESP_OK, NULL, "Delete encoder failed.")};
                   {ZH_ERROR_CHECK_CONT(rmt_del_channel(handle->tx_channel) == ESP_OK, NULL, "Delete TX channel failed.")};
                   {ZH_ERROR_CHECK_CONT(rmt_del_channel(handle->rx_channel) == ESP_OK, NULL, "Delete RX channel failed.")}, "RX channel creation failed.");
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = _zh_ttp229_rmt_rx_done_callback,
    };
    ZH_ERROR_CHECK(rmt_rx_register_event_callbacks(handle->rx_channel, &cbs, handle) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK_CONT(rmt_disable(handle->tx_channel) == ESP_OK, NULL, "TX channel disable failed.")};
                   {ZH_ERROR_CHECK_CONT(rmt_disable(handle->rx_channel) == ESP_OK, NULL, "RX channel disable failed.")};
                   {ZH_ERROR_CHECK_CONT(rmt_del_encoder(handle->copy_encoder) == ESP_OK, NULL, "Delete encoder failed.")};
                   {ZH_ERROR_CHECK_CONT(rmt_del_channel(handle->tx_channel) == ESP_OK, NULL, "Delete TX channel failed.")};
                   {ZH_ERROR_CHECK_CONT(rmt_del_channel(handle->rx_channel) == ESP_OK, NULL, "Delete RX channel failed.")}, "TX callback register failed.");
    return ESP_OK;
}

static esp_err_t _zh_ttp229_resources_init(const zh_ttp229_init_config_t *config)
{
    uint16_t vector_size = 0;
    ZH_ERROR_CHECK(zh_vector_get_size(&_vector, &vector_size) == ESP_OK, ESP_FAIL, NULL, "Failed to get vector size.");
    if (vector_size == 1)
    {
        _queue_handle = xQueueCreate(config->queue_size, sizeof(zh_ttp229_queue_t));
        ZH_ERROR_CHECK(_queue_handle != NULL, ESP_FAIL, NULL, "Failed to create queue.");
    }
    return ESP_OK;
}

static esp_err_t _zh_ttp229_task_init(const zh_ttp229_init_config_t *config)
{
    uint16_t vector_size = 0;
    ZH_ERROR_CHECK(zh_vector_get_size(&_vector, &vector_size) == ESP_OK, ESP_FAIL, NULL, "Failed to get vector size.");
    if (vector_size == 1)
    {
        ZH_ERROR_CHECK(xTaskCreatePinnedToCore(&_zh_ttp229_isr_processing_task, "zh_ttp229_isr_processing", config->stack_size, NULL, config->task_priority, &zh_ttp229, tskNO_AFFINITY) == pdPASS,
                       ESP_FAIL, NULL, "Failed to create isr processing task.");
    }
    return ESP_OK;
}

static void _zh_ttp229_rollback_task_and_resources(void)
{
    if (_vector != NULL)
    {
        uint16_t vector_size = 0;
        ZH_ERROR_CHECK_VOID(zh_vector_get_size(&_vector, &vector_size) == ESP_OK, NULL, "Failed to get vector size.");
        if (vector_size == 1)
        {
            if (zh_ttp229 != NULL)
            {
                vTaskDelete(zh_ttp229);
                zh_ttp229 = NULL;
            }
            if (_queue_handle != NULL)
            {
                vQueueDelete(_queue_handle);
                _queue_handle = NULL;
            }
        }
    }
}

static void _zh_ttp229_isr_handler(void *arg)
{
    zh_ttp229_handle_t *ttp229_handle = (zh_ttp229_handle_t *)arg;
    if (ttp229_handle->rx_channel == NULL || ttp229_handle->tx_channel == NULL)
    {
        return;
    }
    gpio_intr_disable(ttp229_handle->sdo_gpio);
    rmt_receive_config_t rx_receive_config = {.signal_range_min_ns = 100, .signal_range_max_ns = 2000000};
    if (rmt_receive(ttp229_handle->rx_channel, &ttp229_handle->rx_symbols, sizeof(ttp229_handle->rx_symbols), &rx_receive_config) != ESP_OK)
    {
        ++_stats.rmt_driver_error;
        ttp229_handle->rx_needs_reset = true;
        if (_queue_handle != NULL)
        {
            zh_ttp229_queue_t notify = {0};
            notify.handle = ttp229_handle;
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            if (xQueueSendFromISR(_queue_handle, &notify, &xHigherPriorityTaskWoken) != pdTRUE)
            {
                ++_stats.queue_overflow_error;
                gpio_intr_enable(ttp229_handle->sdo_gpio);
            }
            if (xHigherPriorityTaskWoken == pdTRUE)
            {
                portYIELD_FROM_ISR(pdTRUE);
            }
        }
        return;
    }
    gpio_set_direction(ttp229_handle->sdo_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(ttp229_handle->sdo_gpio, 0);
    gpio_set_level(ttp229_handle->sdo_gpio, 1);
    gpio_set_direction(ttp229_handle->sdo_gpio, GPIO_MODE_INPUT);
    rmt_transmit_config_t tx_transmit_config = {.loop_count = 0, .flags.eot_level = 1, .flags.queue_nonblocking = 1};
    if (rmt_transmit(ttp229_handle->tx_channel, ttp229_handle->copy_encoder, ttp229_handle->scl_symbols, ttp229_handle->scl_symbols_count * sizeof(rmt_symbol_word_t), &tx_transmit_config) != ESP_OK)
    {
        ++_stats.rmt_driver_error;
        ttp229_handle->rx_needs_reset = true;
        if (_queue_handle != NULL)
        {
            zh_ttp229_queue_t notify = {0};
            notify.handle = ttp229_handle;
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            if (xQueueSendFromISR(_queue_handle, &notify, &xHigherPriorityTaskWoken) != pdTRUE)
            {
                ++_stats.queue_overflow_error;
                gpio_intr_enable(ttp229_handle->sdo_gpio);
            }
            if (xHigherPriorityTaskWoken == pdTRUE)
            {
                portYIELD_FROM_ISR(pdTRUE);
            }
        }
        return;
    }
}

static void _zh_ttp229_isr_processing_task(void *pvParameter)
{
    (void)pvParameter;
    zh_ttp229_queue_t ttp229_queue = {0};
    while (xQueueReceive(_queue_handle, &ttp229_queue, portMAX_DELAY) == pdTRUE)
    {
        zh_ttp229_handle_t *ttp229_handle = ttp229_queue.handle;
        if (ttp229_handle->rx_needs_reset == true)
        {
            ttp229_handle->rx_needs_reset = false;
            ZH_ERROR_CHECK_CONT(rmt_disable(ttp229_handle->rx_channel) == ESP_OK, ++_stats.rmt_driver_error, "Failed to disable RX channel.");
            ZH_ERROR_CHECK_CONT(rmt_enable(ttp229_handle->rx_channel) == ESP_OK, ++_stats.rmt_driver_error, "Failed to enable RX channel.");
        }
        const uint16_t duration1 = ttp229_queue.rx_symbol.duration1;
        const uint16_t base = ttp229_handle->rmt_tx_start_delay;
        uint8_t pad_number = 0;
        if (duration1 == (base - 1) || duration1 == base || duration1 == (base + 1))
        {
            pad_number = 1;
        }
        else if (duration1 == (base + 3) || duration1 == (base + 4) || duration1 == (base + 5))
        {
            pad_number = 2;
        }
        else if (duration1 == (base + 7) || duration1 == (base + 8) || duration1 == (base + 9))
        {
            pad_number = 3;
        }
        else if (duration1 == (base + 11) || duration1 == (base + 12) || duration1 == (base + 13))
        {
            pad_number = 4;
        }
        else if (duration1 == (base + 15) || duration1 == (base + 16) || duration1 == (base + 17))
        {
            pad_number = 5;
        }
        else if (duration1 == (base + 19) || duration1 == (base + 20) || duration1 == (base + 21))
        {
            pad_number = 6;
        }
        else if (duration1 == (base + 23) || duration1 == (base + 24) || duration1 == (base + 25))
        {
            pad_number = 7;
        }
        else if (duration1 == (base + 27) || duration1 == (base + 28) || duration1 == (base + 29))
        {
            pad_number = 8;
        }
        else if (duration1 == (base + 31) || duration1 == (base + 32) || duration1 == (base + 33))
        {
            pad_number = 9;
        }
        else if (duration1 == (base + 35) || duration1 == (base + 36) || duration1 == (base + 37))
        {
            pad_number = 10;
        }
        else if (duration1 == (base + 39) || duration1 == (base + 40) || duration1 == (base + 41))
        {
            pad_number = 11;
        }
        else if (duration1 == (base + 43) || duration1 == (base + 44) || duration1 == (base + 45))
        {
            pad_number = 12;
        }
        else if (duration1 == (base + 47) || duration1 == (base + 48) || duration1 == (base + 49))
        {
            pad_number = 13;
        }
        else if (duration1 == (base + 51) || duration1 == (base + 52) || duration1 == (base + 53))
        {
            pad_number = 14;
        }
        else if (duration1 == (base + 55) || duration1 == (base + 56) || duration1 == (base + 57))
        {
            pad_number = 15;
        }
        else if (duration1 == (base + 59) || duration1 == (base + 60) || duration1 == (base + 61))
        {
            pad_number = 16;
        }
        if (pad_number > ttp229_handle->scl_symbols_count)
        {
            pad_number = 0;
        }
        TickType_t now = xTaskGetTickCount();
        TickType_t debounce_ticks = pdMS_TO_TICKS(ttp229_handle->debounce_time);
        bool first_event = (ttp229_handle->last_event_tick == (TickType_t)-1) ? true : false;
        bool debounce_elapsed = first_event || ((now - ttp229_handle->last_event_tick) >= debounce_ticks) ? true : false;
        if (pad_number != 0 && debounce_elapsed == true)
        {
            ttp229_handle->last_event_tick = now;
            zh_ttp229_event_on_isr_t ttp229_data = {0};
            ttp229_data.pad_number = pad_number;
            ttp229_data.device_number = ttp229_handle->device_number;
            ZH_ERROR_CHECK_CONT(esp_event_post(ZH_TTP229, 0, &ttp229_data, sizeof(zh_ttp229_event_on_isr_t), pdMS_TO_TICKS(10)) == ESP_OK, ++_stats.event_post_error, "Touch pad isr processing failed. Failed to post interrupt event.");
        }
        gpio_intr_enable(ttp229_handle->sdo_gpio);
        _stats.min_stack_size = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
    }
    vTaskDelete(NULL);
}

static bool _zh_ttp229_rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    (void)channel;
    (void)edata;
    zh_ttp229_handle_t *ttp229_handle = (zh_ttp229_handle_t *)user_data;
    if (_queue_handle == NULL)
    {
        return false;
    }
    zh_ttp229_queue_t ttp229_queue = {0};
    ttp229_queue.handle = ttp229_handle;
    ttp229_queue.rx_symbol = ttp229_handle->rx_symbols;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xQueueSendFromISR(_queue_handle, &ttp229_queue, &xHigherPriorityTaskWoken) != pdTRUE)
    {
        ++_stats.queue_overflow_error;
        gpio_intr_enable(ttp229_handle->sdo_gpio);
    }
    if (xHigherPriorityTaskWoken == pdTRUE)
    {
        return true;
    }
    return false;
}