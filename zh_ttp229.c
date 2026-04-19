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

typedef struct
{
    zh_ttp229_handle_t *handle;
} zh_ttp229_queue_t;

TaskHandle_t zh_ttp229 = NULL;
static QueueHandle_t _queue_handle = NULL;

volatile static uint8_t _pad_counter = {0};
volatile static uint8_t _pad_number_matrix[4] = {0};
static zh_ttp229_stats_t _stats = {0};

static esp_err_t _zh_ttp229_validate_config(const zh_ttp229_init_config_t *config, zh_ttp229_handle_t *handle);
static esp_err_t _zh_ttp229_gpio_init(const zh_ttp229_init_config_t *config, zh_ttp229_handle_t *handle);
static esp_err_t _zh_ttp229_rmt_init(const zh_ttp229_init_config_t *config, zh_ttp229_handle_t *handle);
static esp_err_t _zh_ttp229_resources_init(const zh_ttp229_init_config_t *config);
static esp_err_t _zh_ttp229_task_init(const zh_ttp229_init_config_t *config);
static void _zh_ttp229_isr_handler(void *arg);
static void _zh_ttp229_isr_processing_task(void *pvParameter);
static bool _zh_ttp229_rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data);

ESP_EVENT_DEFINE_BASE(ZH_TTP229);

esp_err_t zh_ttp229_init(const zh_ttp229_init_config_t *config, zh_ttp229_handle_t *handle) // -V2008
{
    ZH_LOGI("Touch pad initialization started.");
    ZH_ERROR_CHECK(config != NULL && handle != NULL, ESP_ERR_INVALID_ARG, NULL, "Touch pad initialization failed. Invalid argument.");
    ZH_ERROR_CHECK(handle->is_initialized == false, ESP_ERR_INVALID_STATE, NULL, "Touch pad initialization failed. Touch pad is already initialized.");
    ZH_ERROR_CHECK(_pad_counter < sizeof(_pad_number_matrix), ESP_ERR_INVALID_ARG, NULL, "Touch pad initialization failed. Maximum quantity reached.");
    ZH_ERROR_CHECK(_zh_ttp229_validate_config(config, handle) == ESP_OK, ESP_FAIL, NULL, "Touch pad initialization failed. Initial configuration check failed.");
    ZH_ERROR_CHECK(_zh_ttp229_resources_init(config) == ESP_OK, ESP_FAIL, NULL, "Touch pad initialization failed. Resources initialization failed.");
    ZH_ERROR_CHECK(_zh_ttp229_task_init(config) == ESP_OK, ESP_FAIL,
                   vQueueDelete(_queue_handle), "Touch pad initialization failed. Processing task initialization failed.");
    ZH_ERROR_CHECK(_zh_ttp229_rmt_init(config, handle) == ESP_OK, ESP_FAIL,
                   vQueueDelete(_queue_handle);
                   vTaskDelete(zh_ttp229), "Touch pad initialization failed. RMT initialization failed.");
    ZH_ERROR_CHECK(_zh_ttp229_gpio_init(config, handle) == ESP_OK, ESP_FAIL,
                   ZH_ERROR_CHECK(rmt_disable(handle->tx_channel) == ESP_OK, ESP_FAIL, NULL, "TX channel disable failed.");
                   ZH_ERROR_CHECK(rmt_disable(handle->rx_channel) == ESP_OK, ESP_FAIL, NULL, "RX channel disable failed.");
                   ZH_ERROR_CHECK(rmt_del_encoder(handle->copy_encoder) == ESP_OK, ESP_FAIL, NULL, "Delete encoder failed.");
                   ZH_ERROR_CHECK(rmt_del_channel(handle->tx_channel) == ESP_OK, ESP_FAIL, NULL, "Delete TX channel failed.");
                   ZH_ERROR_CHECK(rmt_del_channel(handle->rx_channel) == ESP_OK, ESP_FAIL, NULL, "Delete RX channel failed.");
                   vQueueDelete(_queue_handle);
                   vTaskDelete(zh_ttp229), "Encoder initialization failed. GPIO initialization failed.");
    if (_stats.min_stack_size == 0)
    {
        _stats.min_stack_size = config->stack_size;
    }
    handle->debounce_time = config->debounce_time;
    handle->is_initialized = true;
    ++_pad_counter;
    for (uint8_t i = 0; i < sizeof(_pad_number_matrix); ++i)
    {
        if (_pad_number_matrix[i] == 0)
        {
            _pad_number_matrix[i] = handle->device_number;
            break;
        }
    }
    ZH_LOGI("Touch pad initialization completed successfully.");
    return ESP_OK;
}

esp_err_t zh_ttp229_deinit(zh_ttp229_handle_t *handle) // -V2008
{
    ZH_LOGI("Touch pad deinitialization started.");
    ZH_ERROR_CHECK(handle != NULL, ESP_ERR_INVALID_ARG, NULL, "Touch pad deinitialization failed. Invalid argument.");
    ZH_ERROR_CHECK(handle->is_initialized == true, ESP_FAIL, NULL, "Touch pad deinitialization failed. Touch pad not initialized.");
    ZH_ERROR_CHECK(rmt_disable(handle->tx_channel) == ESP_OK, ESP_FAIL, NULL, "Touch pad deinitialization failed. TX channel disable failed.");
    ZH_ERROR_CHECK(rmt_disable(handle->rx_channel) == ESP_OK, ESP_FAIL, NULL, "Touch pad deinitialization failed. RX channel disable failed.");
    ZH_ERROR_CHECK(rmt_del_encoder(handle->copy_encoder) == ESP_OK, ESP_FAIL, NULL, "Touch pad deinitialization failed. Delete encoder failed.");
    ZH_ERROR_CHECK(rmt_del_channel(handle->tx_channel) == ESP_OK, ESP_FAIL, NULL, "Touch pad deinitialization failed. Delete TX channel failed.");
    ZH_ERROR_CHECK(rmt_del_channel(handle->rx_channel) == ESP_OK, ESP_FAIL, NULL, "Touch pad deinitialization failed. Delete RX channel failed.");
    ZH_ERROR_CHECK(gpio_isr_handler_remove((gpio_num_t)handle->sdo_gpio) == ESP_OK, ESP_FAIL, NULL, "Touch pad deinitialization failed. Remove GPIO isr handler failed.");
    ZH_ERROR_CHECK(gpio_reset_pin((gpio_num_t)handle->sdo_gpio) == ESP_OK, ESP_FAIL, NULL, "Touch pad deinitialization failed. Reset GPIO failed.");
    if (_pad_counter == 1)
    {
        vQueueDelete(_queue_handle);
        vTaskDelete(zh_ttp229);
    }
    handle->is_initialized = false;
    --_pad_counter;
    for (uint8_t i = 0; i < sizeof(_pad_number_matrix); ++i)
    {
        if (_pad_number_matrix[i] == handle->device_number)
        {
            _pad_number_matrix[i] = 0;
            break;
        }
    }
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

static esp_err_t _zh_ttp229_validate_config(const zh_ttp229_init_config_t *config, zh_ttp229_handle_t *handle) // -V2008
{
    ZH_ERROR_CHECK(config->task_priority >= 1 && config->stack_size >= configMINIMAL_STACK_SIZE, ESP_ERR_INVALID_ARG, NULL, "Invalid task settings.");
    ZH_ERROR_CHECK(config->queue_size >= 1, ESP_ERR_INVALID_ARG, NULL, "Invalid queue size.");
    ZH_ERROR_CHECK(config->debounce_time >= 10, ESP_ERR_INVALID_ARG, NULL, "Invalid debounce time.");
    ZH_ERROR_CHECK(config->rmt_tx_start_delay >= 15, ESP_ERR_INVALID_ARG, NULL, "Invalid RMT TX start delay.");
    ZH_ERROR_CHECK(config->work_mode == ZH_TTP229_8_PAD || config->work_mode == ZH_TTP229_16_PAD, ESP_ERR_INVALID_ARG, NULL, "Invalid touch pad work mode.");
    ZH_ERROR_CHECK(config->device_number > 0, ESP_ERR_INVALID_ARG, NULL, "Invalid touch pad number.");
    for (uint8_t i = 0; i < sizeof(_pad_number_matrix); ++i)
    {
        ZH_ERROR_CHECK(config->device_number != _pad_number_matrix[i], ESP_ERR_INVALID_ARG, NULL, "Touch pad number already present.");
    }
    handle->rmt_tx_start_delay = config->rmt_tx_start_delay;
    handle->device_number = config->device_number;
    handle->work_mode = config->work_mode;
    return ESP_OK;
}

static esp_err_t _zh_ttp229_gpio_init(const zh_ttp229_init_config_t *config, zh_ttp229_handle_t *handle) // -V2008
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
                   ZH_ERROR_CHECK(gpio_reset_pin((gpio_num_t)config->sdo_gpio) == ESP_OK, ESP_FAIL, NULL, "Reset GPIO failed."), "Failed install isr service.");
    ZH_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)config->sdo_gpio, _zh_ttp229_isr_handler, handle) == ESP_OK, ESP_FAIL,
                   ZH_ERROR_CHECK(gpio_reset_pin((gpio_num_t)config->sdo_gpio) == ESP_OK, ESP_FAIL, NULL, "Reset GPIO failed."), "Interrupt initialization failed.");
    handle->sdo_gpio = config->sdo_gpio;
    return ESP_OK;
}

static esp_err_t _zh_ttp229_rmt_init(const zh_ttp229_init_config_t *config, zh_ttp229_handle_t *handle) // -V2008
{
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = config->scl_gpio,
        .mem_block_symbols = 64,
        .resolution_hz = 1000000,
        .trans_queue_depth = 4,
        .flags.init_level = 1,
    };
    ZH_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &handle->tx_channel) == ESP_OK, ESP_FAIL, NULL, "TX channel creation failed.");
    rmt_copy_encoder_config_t copy_encoder_config = {};
    ZH_ERROR_CHECK(rmt_new_copy_encoder(&copy_encoder_config, &handle->copy_encoder) == ESP_OK, ESP_FAIL,
                   ZH_ERROR_CHECK(rmt_del_channel(handle->tx_channel) == ESP_OK, ESP_FAIL, NULL, "Delete channel failed."), "Copy encoder creation failed.");
    ZH_ERROR_CHECK(rmt_enable(handle->tx_channel) == ESP_OK, ESP_FAIL,
                   ZH_ERROR_CHECK(rmt_del_encoder(handle->copy_encoder) == ESP_OK, ESP_FAIL, NULL, "Delete encoder failed.");
                   ZH_ERROR_CHECK(rmt_del_channel(handle->tx_channel) == ESP_OK, ESP_FAIL, NULL, "Delete channel failed."), "Enable TX channel failed.");
    rmt_rx_channel_config_t rx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = config->sdo_gpio,
        .mem_block_symbols = 64,
        .resolution_hz = 1000000,
    };
    ZH_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config, &handle->rx_channel) == ESP_OK, ESP_FAIL,
                   ZH_ERROR_CHECK(rmt_disable(handle->tx_channel) == ESP_OK, ESP_FAIL, NULL, "TX channel disable failed.");
                   ZH_ERROR_CHECK(rmt_del_encoder(handle->copy_encoder) == ESP_OK, ESP_FAIL, NULL, "Delete encoder failed.");
                   ZH_ERROR_CHECK(rmt_del_channel(handle->tx_channel) == ESP_OK, ESP_FAIL, NULL, "Delete TX channel failed."), "TX channel creation failed.");
    ZH_ERROR_CHECK(rmt_enable(handle->rx_channel) == ESP_OK, ESP_FAIL,
                   ZH_ERROR_CHECK(rmt_disable(handle->tx_channel) == ESP_OK, ESP_FAIL, NULL, "TX channel disable failed.");
                   ZH_ERROR_CHECK(rmt_del_encoder(handle->copy_encoder) == ESP_OK, ESP_FAIL, NULL, "Delete encoder failed.");
                   ZH_ERROR_CHECK(rmt_del_channel(handle->tx_channel) == ESP_OK, ESP_FAIL, NULL, "Delete TX channel failed.");
                   ZH_ERROR_CHECK(rmt_del_channel(handle->rx_channel) == ESP_OK, ESP_FAIL, NULL, "Delete RX channel failed."), "RX channel creation failed.");
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = _zh_ttp229_rmt_rx_done_callback,
    };
    ZH_ERROR_CHECK(rmt_rx_register_event_callbacks(handle->rx_channel, &cbs, handle) == ESP_OK, ESP_FAIL,
                   ZH_ERROR_CHECK(rmt_disable(handle->tx_channel) == ESP_OK, ESP_FAIL, NULL, "TX channel disable failed.");
                   ZH_ERROR_CHECK(rmt_disable(handle->rx_channel) == ESP_OK, ESP_FAIL, NULL, "RX channel disable failed.");
                   ZH_ERROR_CHECK(rmt_del_encoder(handle->copy_encoder) == ESP_OK, ESP_FAIL, NULL, "Delete encoder failed.");
                   ZH_ERROR_CHECK(rmt_del_channel(handle->tx_channel) == ESP_OK, ESP_FAIL, NULL, "Delete TX channel failed.");
                   ZH_ERROR_CHECK(rmt_del_channel(handle->rx_channel) == ESP_OK, ESP_FAIL, NULL, "Delete RX channel failed."), "TX callback registar failed.");
    return ESP_OK;
}

static esp_err_t _zh_ttp229_resources_init(const zh_ttp229_init_config_t *config)
{
    if (_pad_counter == 0)
    {
        _queue_handle = xQueueCreate(config->queue_size, sizeof(zh_ttp229_queue_t));
        ZH_ERROR_CHECK(_queue_handle != NULL, ESP_FAIL, NULL, "Failed to create queue.");
    }
    return ESP_OK;
}

static esp_err_t _zh_ttp229_task_init(const zh_ttp229_init_config_t *config)
{
    if (_pad_counter == 0)
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
    if (rmt_receive(ttp229_handle->rx_channel, &ttp229_handle->rx_symbols, sizeof(ttp229_handle->rx_symbols), &rx_receive_config) != ESP_OK)
    {
        ++_stats.rmt_driver_error;
    }
    gpio_set_direction(ttp229_handle->sdo_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(ttp229_handle->sdo_gpio, 0);
    gpio_set_level(ttp229_handle->sdo_gpio, 1);
    gpio_set_direction(ttp229_handle->sdo_gpio, GPIO_MODE_INPUT);
    rmt_symbol_word_t scl_symbols[ttp229_handle->work_mode];
    for (uint8_t i = 0; i < ttp229_handle->work_mode; ++i)
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

static void IRAM_ATTR _zh_ttp229_isr_processing_task(void *pvParameter) // -V2008
{
    zh_ttp229_queue_t ttp229_queue = {0};
    while (xQueueReceive(_queue_handle, &ttp229_queue, portMAX_DELAY) == pdTRUE)
    {
        zh_ttp229_handle_t *ttp229_handle = ttp229_queue.handle;
        uint8_t pad_number = 0;
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
            esp_err_t err = esp_event_post(ZH_TTP229, 0, &ttp229_data, sizeof(zh_ttp229_event_on_isr_t), 1000 / portTICK_PERIOD_MS);
            if (err != ESP_OK)
            {
                ++_stats.event_post_error;
                ZH_LOGE("Touch pad isr processing failed. Failed to post interrupt event.", err);
            }
        }
        vTaskDelay(ttp229_handle->debounce_time / portTICK_PERIOD_MS);
        gpio_intr_enable(ttp229_handle->sdo_gpio);
        _stats.min_stack_size = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
    }
    vTaskDelete(NULL);
}

static bool IRAM_ATTR _zh_ttp229_rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
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