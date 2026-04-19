# ESP32 ESP-IDF component for TTP229 touch pad (via RMT driver)

## Tested on

1. [ESP32 ESP-IDF v6.0.0](https://docs.espressif.com/projects/esp-idf/en/v6.0/esp32/index.html)

## SAST Tools

[PVS-Studio](https://pvs-studio.com/pvs-studio/?utm_source=website&utm_medium=github&utm_campaign=open_source) - static analyzer for C, C++, C#, and Java code.

## Features

1. Support up to 4 touch pads on one device.
2. Support 8 or 16 pad work mode.

## Note

1. After start/reset first touch not work.
2. Multi touch not supported.

## Attention

1. For correct operation, please enable the following settings in the menuconfig:

```text
CONFIG_GPIO_CTRL_FUNC_IN_IRAM
CONFIG_RMT_ENCODER_FUNC_IN_IRAM
CONFIG_RMT_TX_ISR_HANDLER_IN_IRAM
CONFIG_RMT_RX_ISR_HANDLER_IN_IRAM
CONFIG_RMT_RECV_FUNC_IN_IRAM
CONFIG_RMT_TX_ISR_CACHE_SAFE
CONFIG_RMT_RX_ISR_CACHE_SAFE
```

## Using

In an existing project, run the following command to install the components:

```text
cd ../your_project/components
git clone https://github.com/aZholtikov/zh_ttp229
```

In the application, add the component:

```c
#include "zh_ttp229.h"
```

## Examples

One touch pad on device:

```c
#include "zh_ttp229.h"

#define TTP229_NUMBER 0x01

zh_ttp229_handle_t ttp229_handle = {0};

void zh_ttp229_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

void app_main(void)
{
    esp_log_level_set("zh_ttp229", ESP_LOG_ERROR);
    esp_event_loop_create_default();
    esp_event_handler_instance_register(ZH_TTP229, ESP_EVENT_ANY_ID, &zh_ttp229_event_handler, NULL, NULL);
    zh_ttp229_init_config_t config = ZH_TTP229_INIT_CONFIG_DEFAULT();
    config.scl_gpio = GPIO_NUM_25;
    config.sdo_gpio = GPIO_NUM_27;
    config.work_mode = ZH_TTP229_16_PAD;
    config.device_number = TTP229_NUMBER;
    config.debounce_time = 300;
    // The time between the low-level pulse in the interrupt and the actual start time of the RMT signal generator (in microseconds).
    // It is determined experimentally or using a logic analyzer.
    config.rmt_tx_start_delay = 19;
    zh_ttp229_init(&config, &ttp229_handle);
    for (;;)
    {
        const zh_ttp229_stats_t *stats = zh_ttp229_get_stats();
        printf("Number of RMT driver error: %ld.\n", stats->rmt_driver_error);
        printf("Number of event post error: %ld.\n", stats->event_post_error);
        printf("Number of queue overflow error: %ld.\n", stats->queue_overflow_error);
        printf("Minimum free stack size: %ld.\n", stats->min_stack_size);
        vTaskDelay(60000 / portTICK_PERIOD_MS);
    }
}

void zh_ttp229_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    zh_ttp229_event_on_isr_t *ttp229_event = event_data;
    printf("Touch pad number %d pad number %d pressed.\n", ttp229_event->device_number, ttp229_event->pad_number);
}
```
