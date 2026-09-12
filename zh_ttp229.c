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

#define ZH_ERROR_CHECK_CONT(cond, cleanup, msg, ...) \
    if (!(cond))                                     \
    {                                                \
        ZH_LOGE(msg, ESP_FAIL, ##__VA_ARGS__);       \
        cleanup;                                     \
        continue;                                    \
    }

/** @brief Internal handle for a single TTP229 device instance.
 *
 * Fields are initialized by zh_ttp229_init() and used by the
 * ISR handler, processing task, and deinitialization.
 */
struct _zh_ttp229_handle_t
{
    gpio_num_t sdo_gpio;               /*!< SDO (Serial Data Out) GPIO pin for touch data */
    uint8_t device_number;             /*!< Unique device number assigned at initialization */
    uint8_t rmt_tx_start_delay;        /*!< Time between SDO low-level pulse and RMT TX start (in microseconds) */
    uint16_t debounce_time;            /*!< Button debounce time in milliseconds */
    zh_ttp229_work_mode_t work_mode;   /*!< Touch pad configuration (8 or 16 pads) */
    rmt_symbol_word_t rx_symbols;      /*!< Received RMT symbol containing duration1 (pad data) */
    rmt_channel_handle_t tx_channel;   /*!< RMT TX channel handle (SCL clock generation) */
    rmt_channel_handle_t rx_channel;   /*!< RMT RX channel handle (SDO data capture) */
    rmt_encoder_handle_t copy_encoder; /*!< Copy encoder handle for RMT TX */
};

/** @brief Queue entry passed from RMT RX callback to the ISR processing task.
 *
 * Contains a pointer to the TTP229 device handle for the device
 * that triggered the interrupt.
 */
typedef struct
{
    zh_ttp229_handle_t *handle; /*!< Pointer to the TTP229 device handle */
} zh_ttp229_queue_t;

TaskHandle_t zh_ttp229 = NULL;             /*!< FreeRTOS task handle for the ISR processing task */
static QueueHandle_t _queue_handle = NULL; /*!< Internal event queue */
static zh_ttp229_stats_t _stats = {0};     /*!< Accumulated error and runtime statistics */
static zh_vector_t *_vector = NULL;        /*!< Vector of device_number values for all initialized TTP229 devices */

/** @brief Validate the user-provided initialization configuration.
 *
 * Checks all parameter ranges, ensures work_mode is valid, and
 * verifies device_number uniqueness across initialized devices.
 *
 * @param config Pointer to the user configuration
 * @param handle Pointer to the allocated handle (partially filled)
 *
 * @return ESP_OK if all checks pass
 * @return ESP_ERR_INVALID_ARG on invalid parameter values
 */
static esp_err_t _zh_ttp229_validate_config(const zh_ttp229_init_config_t *config, zh_ttp229_handle_t *handle);

/** @brief Configure GPIO for SDO input with positive-edge interrupt.
 *
 * Installs the ISR service if not yet installed, configures SDO as
 * input with GPIO_INTR_POSEDGE, and registers the ISR handler.
 *
 * @param config Pointer to the user configuration
 * @param handle Pointer to the handle (stores sdo_gpio)
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if GPIO pins are invalid or identical
 * @return ESP_FAIL on ISR service or handler installation failure
 */
static esp_err_t _zh_ttp229_gpio_init(const zh_ttp229_init_config_t *config, zh_ttp229_handle_t *handle);

/** @brief Initialize RMT TX and RX channels with callbacks.
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

/** @brief Create the shared event queue on first device initialization.
 *
 * The queue is created only when vector_size equals 1 (first device).
 *
 * @param config Pointer to the user configuration
 *
 * @return ESP_OK on success
 * @return ESP_FAIL if queue allocation fails
 */
static esp_err_t _zh_ttp229_resources_init(const zh_ttp229_init_config_t *config);

/** @brief Create the ISR processing task on first device initialization.
 *
 * The task is created only when vector_size equals 1 (first device).
 *
 * @param config Pointer to the user configuration
 *
 * @return ESP_OK on success
 * @return ESP_FAIL if task creation fails
 */
static esp_err_t _zh_ttp229_task_init(const zh_ttp229_init_config_t *config);

/** @brief GPIO ISR handler triggered on SDO positive edge.
 *
 * Disables further interrupts, starts RMT receive on SDO, generates
 * SCL clock pulses via RMT transmit on SCL, and yields to higher
 * priority tasks.
 *
 * @param arg Pointer to the zh_ttp229_handle_t for this device
 */
static void _zh_ttp229_isr_handler(void *arg);

/** @brief FreeRTOS task that processes RMT RX data and posts events.
 *
 * Receives queue entries from the RMT RX callback, decodes the pad
 * number from rx_symbols.duration1, posts a ZH_TTP229 event for
 * detected touches, debounces by delaying, then re-enables GPIO
 * interrupts. Runs until the queue is deleted.
 *
 * @param pvParameter Unused
 */
static void _zh_ttp229_isr_processing_task(void *pvParameter);

/** @brief RMT RX done callback that enqueues data to the processing task.
 *
 * Sends a zh_ttp229_queue_t entry to _queue_handle. Increments
 * queue overflow error counter if the send fails.
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
    ZH_ERROR_CHECK(_zh_ttp229_validate_config(config, *handle) == ESP_OK, ESP_FAIL, heap_caps_free(*handle); *handle = NULL, "Touch pad initialization failed. Initial configuration check failed.");
    if (_vector == NULL)
    {
        ZH_ERROR_CHECK(zh_vector_init(&_vector, sizeof(uint8_t)) == ESP_OK, ESP_FAIL, heap_caps_free(*handle); *handle = NULL, "Touch pad initialization failed. Failed to create vector.");
    }
    ZH_ERROR_CHECK(zh_vector_push_back(&_vector, &config->device_number) == ESP_OK, ESP_FAIL, heap_caps_free(*handle); *handle = NULL, "Touch pad initialization failed. Failed to add vector data.");
    ZH_ERROR_CHECK(_zh_ttp229_resources_init(config) == ESP_OK, ESP_FAIL, zh_vector_delete_back(&_vector); heap_caps_free(*handle); *handle = NULL, "Touch pad initialization failed. Resources initialization failed.");
    // clang-format off
    ZH_ERROR_CHECK(_zh_ttp229_task_init(config) == ESP_OK, ESP_FAIL,
                   zh_vector_delete_back(&_vector); heap_caps_free(*handle); *handle = NULL, "Touch pad initialization failed. Processing task initialization failed.");
    ZH_ERROR_CHECK(_zh_ttp229_rmt_init(config, *handle) == ESP_OK, ESP_FAIL,
                   zh_vector_delete_back(&_vector); heap_caps_free(*handle); *handle = NULL, "Touch pad initialization failed. RMT initialization failed.");
    // clang-format on
    ZH_ERROR_CHECK(_zh_ttp229_gpio_init(config, *handle) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(rmt_disable((*handle)->tx_channel) == ESP_OK, ESP_FAIL, NULL, "TX channel disable failed.")};
                   {ZH_ERROR_CHECK(rmt_disable((*handle)->rx_channel) == ESP_OK, ESP_FAIL, NULL, "RX channel disable failed.")};
                   {ZH_ERROR_CHECK(rmt_del_encoder((*handle)->copy_encoder) == ESP_OK, ESP_FAIL, NULL, "Delete encoder failed.")};
                   {ZH_ERROR_CHECK(rmt_del_channel((*handle)->tx_channel) == ESP_OK, ESP_FAIL, NULL, "Delete TX channel failed.")};
                   {ZH_ERROR_CHECK(rmt_del_channel((*handle)->rx_channel) == ESP_OK, ESP_FAIL, NULL, "Delete RX channel failed.")};
                   zh_vector_delete_back(&_vector); heap_caps_free(*handle); *handle = NULL, "Touch pad initialization failed. GPIO initialization failed.");
    if (_stats.min_stack_size == 0)
    {
        _stats.min_stack_size = config->stack_size;
    }
    (*handle)->debounce_time = config->debounce_time;
    ZH_LOGI("Touch pad initialization completed successfully.");
    return ESP_OK;
}

esp_err_t zh_ttp229_deinit(zh_ttp229_handle_t **handle)
{
    ZH_LOGI("Touch pad deinitialization started.");
    ZH_ERROR_CHECK(handle != NULL && *handle != NULL, ESP_ERR_INVALID_ARG, NULL, "Touch pad deinitialization failed. Invalid argument.");
    ZH_ERROR_CHECK(rmt_disable((*handle)->tx_channel) == ESP_OK, ESP_FAIL, NULL, "Touch pad deinitialization failed. TX channel disable failed.");
    ZH_ERROR_CHECK(rmt_disable((*handle)->rx_channel) == ESP_OK, ESP_FAIL, NULL, "Touch pad deinitialization failed. RX channel disable failed.");
    ZH_ERROR_CHECK(rmt_del_encoder((*handle)->copy_encoder) == ESP_OK, ESP_FAIL, NULL, "Touch pad deinitialization failed. Delete encoder failed.");
    ZH_ERROR_CHECK(rmt_del_channel((*handle)->tx_channel) == ESP_OK, ESP_FAIL, NULL, "Touch pad deinitialization failed. Delete TX channel failed.");
    ZH_ERROR_CHECK(rmt_del_channel((*handle)->rx_channel) == ESP_OK, ESP_FAIL, NULL, "Touch pad deinitialization failed. Delete RX channel failed.");
    ZH_ERROR_CHECK(gpio_isr_handler_remove((*handle)->sdo_gpio) == ESP_OK, ESP_FAIL, NULL, "Touch pad deinitialization failed. Remove GPIO isr handler failed.");
    ZH_ERROR_CHECK(gpio_reset_pin((*handle)->sdo_gpio) == ESP_OK, ESP_FAIL, NULL, "Touch pad deinitialization failed. Reset GPIO failed.");
    int32_t index = 0;
    ZH_ERROR_CHECK(zh_vector_find_item(&_vector, &(*handle)->device_number, &index) == ESP_ERR_NOT_FOUND, ESP_ERR_INVALID_ARG, NULL, "Touch pad deinitialization failed. Failed to find vector item.");
    ZH_ERROR_CHECK(zh_vector_delete_item(&_vector, (uint16_t)index) == ESP_OK, ESP_FAIL, NULL, "Touch pad deinitialization failed. Vector delete item failed.");
    uint16_t vector_size = 0;
    ZH_ERROR_CHECK(zh_vector_get_size(&_vector, &vector_size) == ESP_OK, ESP_FAIL, NULL, "Touch pad deinitialization failed. Failed to get vector size.");
    if (vector_size == 0)
    {
        vQueueDelete(_queue_handle);
        _queue_handle = NULL;
        vTaskDelete(zh_ttp229);
        zh_ttp229 = NULL;
        ZH_ERROR_CHECK(zh_vector_free(&_vector) == ESP_OK, ESP_FAIL, NULL, "Touch pad deinitialization failed. Free vector failed.");
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
    handle->work_mode = config->work_mode;
    return ESP_OK;
}

static esp_err_t _zh_ttp229_gpio_init(const zh_ttp229_init_config_t *config, zh_ttp229_handle_t *handle)
{
    ZH_ERROR_CHECK(config->scl_gpio < GPIO_NUM_MAX && config->sdo_gpio < GPIO_NUM_MAX, ESP_ERR_INVALID_ARG, NULL, "Invalid GPIO number.")
    ZH_ERROR_CHECK(config->scl_gpio != config->sdo_gpio, ESP_ERR_INVALID_ARG, NULL, "SCL GPIO and SDO GPIO is same.")
    gpio_config_t sdo_config = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << config->sdo_gpio),
        .intr_type = GPIO_INTR_POSEDGE};
    ZH_ERROR_CHECK(gpio_config(&sdo_config) == ESP_OK, ESP_FAIL, NULL, "GPIO initialization failed.");
    esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_LOWMED);
    ZH_ERROR_CHECK(err == ESP_OK || err == ESP_ERR_INVALID_STATE, ESP_FAIL,
                   {ZH_ERROR_CHECK(gpio_reset_pin(config->sdo_gpio) == ESP_OK, ESP_FAIL, NULL, "Reset GPIO failed.")}, "Failed install isr service.");
    ZH_ERROR_CHECK(gpio_isr_handler_add(config->sdo_gpio, _zh_ttp229_isr_handler, handle) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(gpio_reset_pin(config->sdo_gpio) == ESP_OK, ESP_FAIL, NULL, "Reset GPIO failed.")}, "Interrupt initialization failed.");
    handle->sdo_gpio = config->sdo_gpio;
    return ESP_OK;
}

static esp_err_t _zh_ttp229_rmt_init(const zh_ttp229_init_config_t *config, zh_ttp229_handle_t *handle)
{
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = config->scl_gpio,
#if defined CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
        .mem_block_symbols = 64,
#else
        .mem_block_symbols = 48,
#endif
        .resolution_hz = 1000000,
        .trans_queue_depth = 4,
        .flags.init_level = 1,
    };
    ZH_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &handle->tx_channel) == ESP_OK, ESP_FAIL, NULL, "TX channel creation failed.");
    rmt_copy_encoder_config_t copy_encoder_config = {};
    ZH_ERROR_CHECK(rmt_new_copy_encoder(&copy_encoder_config, &handle->copy_encoder) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(rmt_del_channel(handle->tx_channel) == ESP_OK, ESP_FAIL, NULL, "Delete channel failed.")}, "Copy encoder creation failed.");
    ZH_ERROR_CHECK(rmt_enable(handle->tx_channel) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(rmt_del_encoder(handle->copy_encoder) == ESP_OK, ESP_FAIL, NULL, "Delete encoder failed.")};
                   {ZH_ERROR_CHECK(rmt_del_channel(handle->tx_channel) == ESP_OK, ESP_FAIL, NULL, "Delete channel failed.")}, "Enable TX channel failed.");
    rmt_rx_channel_config_t rx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = config->sdo_gpio,
#if defined CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
        .mem_block_symbols = 64,
#else
        .mem_block_symbols = 48,
#endif
        .resolution_hz = 1000000,
    };
    ZH_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config, &handle->rx_channel) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(rmt_disable(handle->tx_channel) == ESP_OK, ESP_FAIL, NULL, "TX channel disable failed.")};
                   {ZH_ERROR_CHECK(rmt_del_encoder(handle->copy_encoder) == ESP_OK, ESP_FAIL, NULL, "Delete encoder failed.")};
                   {ZH_ERROR_CHECK(rmt_del_channel(handle->tx_channel) == ESP_OK, ESP_FAIL, NULL, "Delete TX channel failed.")}, "TX channel creation failed.");
    ZH_ERROR_CHECK(rmt_enable(handle->rx_channel) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(rmt_disable(handle->tx_channel) == ESP_OK, ESP_FAIL, NULL, "TX channel disable failed.")};
                   {ZH_ERROR_CHECK(rmt_del_encoder(handle->copy_encoder) == ESP_OK, ESP_FAIL, NULL, "Delete encoder failed.")};
                   {ZH_ERROR_CHECK(rmt_del_channel(handle->tx_channel) == ESP_OK, ESP_FAIL, NULL, "Delete TX channel failed.")};
                   {ZH_ERROR_CHECK(rmt_del_channel(handle->rx_channel) == ESP_OK, ESP_FAIL, NULL, "Delete RX channel failed.")}, "RX channel creation failed.");
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = _zh_ttp229_rmt_rx_done_callback,
    };
    ZH_ERROR_CHECK(rmt_rx_register_event_callbacks(handle->rx_channel, &cbs, handle) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(rmt_disable(handle->tx_channel) == ESP_OK, ESP_FAIL, NULL, "TX channel disable failed.")};
                   {ZH_ERROR_CHECK(rmt_disable(handle->rx_channel) == ESP_OK, ESP_FAIL, NULL, "RX channel disable failed.")};
                   {ZH_ERROR_CHECK(rmt_del_encoder(handle->copy_encoder) == ESP_OK, ESP_FAIL, NULL, "Delete encoder failed.")};
                   {ZH_ERROR_CHECK(rmt_del_channel(handle->tx_channel) == ESP_OK, ESP_FAIL, NULL, "Delete TX channel failed.")};
                   {ZH_ERROR_CHECK(rmt_del_channel(handle->rx_channel) == ESP_OK, ESP_FAIL, NULL, "Delete RX channel failed.")}, "TX callback registar failed.");
    return ESP_OK;
}

static esp_err_t _zh_ttp229_resources_init(const zh_ttp229_init_config_t *config)
{
    uint16_t vector_size = 0;
    zh_vector_get_size(&_vector, &vector_size);
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
    zh_vector_get_size(&_vector, &vector_size);
    if (vector_size == 1)
    {
        ZH_ERROR_CHECK(xTaskCreatePinnedToCore(&_zh_ttp229_isr_processing_task, "zh_ttp229_isr_processing", config->stack_size, NULL, config->task_priority, &zh_ttp229, tskNO_AFFINITY) == pdPASS,
                       ESP_FAIL, NULL, "Failed to create isr processing task.");
    }
    return ESP_OK;
}

static void IRAM_ATTR _zh_ttp229_isr_handler(void *arg)
{
    zh_ttp229_handle_t *ttp229_handle = (zh_ttp229_handle_t *)arg;
    gpio_intr_disable(ttp229_handle->sdo_gpio);
    rmt_receive_config_t rx_receive_config = {.signal_range_min_ns = 100, .signal_range_max_ns = 2000000};
    // Start RMT reception on SDO to capture touch data
    if (rmt_receive(ttp229_handle->rx_channel, &ttp229_handle->rx_symbols, sizeof(ttp229_handle->rx_symbols), &rx_receive_config) != ESP_OK)
    {
        ++_stats.rmt_driver_error;
    }
    // Toggle SDO: set output low then high to trigger the RMT RX start
    gpio_set_direction(ttp229_handle->sdo_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(ttp229_handle->sdo_gpio, 0);
    gpio_set_level(ttp229_handle->sdo_gpio, 1);
    gpio_set_direction(ttp229_handle->sdo_gpio, GPIO_MODE_INPUT);
    // Generate SCL clock pulses via RMT transmit for the configured work mode
    rmt_symbol_word_t scl_symbols[ttp229_handle->work_mode];
    for (uint8_t i = 0; i < (uint8_t)ttp229_handle->work_mode; ++i)
    {
        scl_symbols[i] = (rmt_symbol_word_t){.duration0 = 2, .level0 = 0, .duration1 = 2, .level1 = 1};
    }
    rmt_transmit_config_t tx_transmit_config = {.loop_count = 0, .flags.eot_level = 1, .flags.queue_nonblocking = 1};
    if (rmt_transmit(ttp229_handle->tx_channel, ttp229_handle->copy_encoder, scl_symbols, sizeof(scl_symbols), &tx_transmit_config) != ESP_OK)
    {
        ++_stats.rmt_driver_error;
    }
    portYIELD_FROM_ISR();
}

static void IRAM_ATTR _zh_ttp229_isr_processing_task(void *pvParameter)
{
    (void)pvParameter;
    zh_ttp229_queue_t ttp229_queue = {0};
    while (xQueueReceive(_queue_handle, &ttp229_queue, portMAX_DELAY) == pdTRUE)
    {
        zh_ttp229_handle_t *ttp229_handle = ttp229_queue.handle;
        uint8_t pad_number = 0;
        // Decode pad number from duration1: each pad has a unique range
        // of (rmt_tx_start_delay + offset) values, with offset increasing
        // by 2 per pad (pad N uses offset 2*(N-1)+1, ±1 tolerance)
        if ((ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay - 1)) ||
            (ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay)) ||
            (ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 1)))
        {
            pad_number = 1;
        }
        else if ((ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 3)) ||
                 (ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 4)) ||
                 (ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 5)))
        {
            pad_number = 2;
        }
        else if ((ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 7)) ||
                 (ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 8)) ||
                 (ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 9)))
        {
            pad_number = 3;
        }
        else if ((ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 11)) ||
                 (ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 12)) ||
                 (ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 13)))
        {
            pad_number = 4;
        }
        else if ((ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 15)) ||
                 (ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 16)) ||
                 (ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 17)))
        {
            pad_number = 5;
        }
        else if ((ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 19)) ||
                 (ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 20)) ||
                 (ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 21)))
        {
            pad_number = 6;
        }
        else if ((ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 23)) ||
                 (ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 24)) ||
                 (ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 25)))
        {
            pad_number = 7;
        }
        else if ((ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 27)) ||
                 (ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 28)) ||
                 (ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 29)))
        {
            pad_number = 8;
        }
        else if ((ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 31)) ||
                 (ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 32)) ||
                 (ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 33)))
        {
            pad_number = 9;
        }
        else if ((ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 35)) ||
                 (ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 36)) ||
                 (ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 37)))
        {
            pad_number = 10;
        }
        else if ((ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 39)) ||
                 (ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 40)) ||
                 (ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 41)))
        {
            pad_number = 11;
        }
        else if ((ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 43)) ||
                 (ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 44)) ||
                 (ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 45)))
        {
            pad_number = 12;
        }
        else if ((ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 47)) ||
                 (ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 48)) ||
                 (ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 49)))
        {
            pad_number = 13;
        }
        else if ((ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 51)) ||
                 (ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 52)) ||
                 (ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 53)))
        {
            pad_number = 14;
        }
        else if ((ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 55)) ||
                 (ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 56)) ||
                 (ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 57)))
        {
            pad_number = 15;
        }
        else if ((ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 59)) ||
                 (ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 60)) ||
                 (ttp229_handle->rx_symbols.duration1 == (ttp229_handle->rmt_tx_start_delay + 61)))
        {
            pad_number = 16;
        }
        if (pad_number != 0)
        {
            zh_ttp229_event_on_isr_t ttp229_data = {0};
            ttp229_data.pad_number = pad_number;
            ttp229_data.device_number = ttp229_handle->device_number;
            ZH_ERROR_CHECK_CONT(esp_event_post(ZH_TTP229, 0, &ttp229_data, sizeof(zh_ttp229_event_on_isr_t), 1000 / portTICK_PERIOD_MS) == ESP_OK, ++_stats.event_post_error, "Touch pad isr processing failed. Failed to post interrupt event.");
        }
        vTaskDelay(ttp229_handle->debounce_time / portTICK_PERIOD_MS);
        gpio_intr_enable(ttp229_handle->sdo_gpio);
        _stats.min_stack_size = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
    }
    vTaskDelete(NULL);
}

static bool IRAM_ATTR _zh_ttp229_rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    (void)edata;
    zh_ttp229_handle_t *ttp229_handle = (zh_ttp229_handle_t *)user_data;
    zh_ttp229_queue_t ttp229_queue = {0};
    ttp229_queue.handle = ttp229_handle;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xQueueSendFromISR(_queue_handle, &ttp229_queue, &xHigherPriorityTaskWoken) != pdTRUE)
    {
        ++_stats.queue_overflow_error;
    }
    if (xHigherPriorityTaskWoken == pdTRUE)
    {
        return true;
    }
    return false;
}