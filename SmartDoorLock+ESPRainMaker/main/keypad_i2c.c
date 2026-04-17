#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "keypad_i2c.h"

static const char *TAG = "KEYPAD";

// I2C configuration
#ifndef I2C_MASTER_SDA_GPIO
#define I2C_MASTER_SDA_GPIO 4
#endif
#ifndef I2C_MASTER_SCL_GPIO
#define I2C_MASTER_SCL_GPIO 5
#endif
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define PCF8574_ADDR 0x20

static const uint8_t row_pins[4] = {0,1,2,3};
static const uint8_t col_pins[4] = {4,5,6,7};

static const char keymap[4][4] = {
    {'1','2','3','A'},
    {'4','5','6','B'},
    {'7','8','9','C'},
    {'*','0','#','D'}
};

// Queue để gửi phím
static QueueHandle_t key_queue = NULL;

// --- I2C read/write ---
static esp_err_t pcf_read_byte(uint8_t *val)
{
    if(!val) return ESP_ERR_INVALID_ARG;
    return i2c_master_read_from_device(I2C_MASTER_NUM, PCF8574_ADDR, val, 1, pdMS_TO_TICKS(1000));
}

static esp_err_t pcf_write_byte(uint8_t val)
{
    return i2c_master_write_to_device(I2C_MASTER_NUM, PCF8574_ADDR, &val, 1, pdMS_TO_TICKS(1000));
}

// --- Khởi tạo I2C ---
static void i2c_master_init(void)
{
    static bool initialized = false;
    if(initialized) return;
    initialized = true;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_GPIO,
        .scl_io_num = I2C_MASTER_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {.clk_speed = I2C_MASTER_FREQ_HZ},
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

// --- Task scan keypad ---
static void keypad_task(void *arg)
{
    uint8_t all_high = 0xFF;
    pcf_write_byte(all_high);

    uint8_t stable_count[4][4] = {0};
    char last_sent[4][4] = {0};

    for(;;)
    {
        uint8_t in = 0xFF;

        for(int c=0;c<4;c++)
        {
            uint8_t out = all_high & ~(1<<col_pins[c]);
            pcf_write_byte(out);
            vTaskDelay(pdMS_TO_TICKS(5));
            pcf_read_byte(&in);

            for(int r=0;r<4;r++)
            {
                if(!(in & (1<<row_pins[r])))
                {
                    if(stable_count[r][c] < 3) stable_count[r][c]++;
                }
                else
                {
                    stable_count[r][c] = 0;
                    last_sent[r][c] = 0;
                }

                if(stable_count[r][c]>=3 && !last_sent[r][c])
                {
                    char k = keymap[r][c];
                    if(key_queue) xQueueSend(key_queue, &k, 0);
                    last_sent[r][c] = 1;
                }
            }
            pcf_write_byte(all_high);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// --- API lấy key ---
char keypad_get_key_timeout(uint32_t timeout_ms)
{
    char k;
    if(key_queue && xQueueReceive(key_queue, &k, pdMS_TO_TICKS(timeout_ms))==pdTRUE)
        return k;
    return 0;
}

// --- API đọc password ---
int keypad_read_password(char* out_pass, int max_len, uint32_t timeout_ms)
{
    int idx = 0;
    TickType_t start = xTaskGetTickCount();
    TickType_t total = pdMS_TO_TICKS(timeout_ms);

    while(idx < max_len-1)
    {
        if((xTaskGetTickCount()-start)>=total) return 0;

        char key = keypad_get_key_timeout(50);
        if(key==0) continue;

        if(key=='#'){ out_pass[idx]='\0'; return idx; }
        if(key=='*'){ if(idx>0) idx--; continue; }

        out_pass[idx++] = key;
    }
    out_pass[idx]='\0';
    return idx;
}

// --- Bắt đầu task ---
void keypad_start_task(void)
{
    if(key_queue==NULL)
        key_queue = xQueueCreate(16,sizeof(char));

    i2c_master_init();
    vTaskDelay(pdMS_TO_TICKS(50));
    xTaskCreate(keypad_task,"keypad_task",3072,NULL,5,NULL);
}
