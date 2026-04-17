/* Switch Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_event.h>
#include <nvs_flash.h>

#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_types.h>
#include <esp_rmaker_standard_params.h>
#include <esp_rmaker_standard_devices.h>
#include <esp_rmaker_schedule.h>
#include <esp_rmaker_scenes.h>
#include <esp_rmaker_console.h>
#include <esp_rmaker_ota.h>

#include <esp_rmaker_common_events.h>

#include <app_network.h>
#include <app_insights.h>

#include "app_priv.h"

#include <stdbool.h> 
#include <stdlib.h>  
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_err.h"
#include "MFRC522.h"
#include "keypad_i2c.h"
#include "card_storage.h"
#include "oled_i2c.h" 

#define PIN_NUM_MISO 2
#define PIN_NUM_MOSI 6
#define PIN_NUM_CLK  7
#define PIN_NUM_CS   10
#define PIN_NUM_RST  3

#define I2C_MASTER_SDA_GPIO 4
#define I2C_MASTER_SCL_GPIO 5

// OLED Configuration
#define OLED_I2C_PORT I2C_NUM_0
#define OLED_SDA_PIN I2C_MASTER_SDA_GPIO
#define OLED_SCL_PIN I2C_MASTER_SCL_GPIO
#define OLED_CLK_SPEED 400000

// ========== THÊM CHÂN RELAY ==========
// RELAY_PIN được điều khiển bởi app_driver.c qua CONFIG_EXAMPLE_OUTPUT_GPIO (GPIO 8)
#define RELAY_PIN CONFIG_EXAMPLE_OUTPUT_GPIO
#define RELAY_ON_LEVEL 1   // 1 = HIGH để bật relay, 0 = LOW để bật relay
#define DOOR_OPEN_TIME 3000 // Thời gian mở cửa (ms) - 3 giây
// ====================================

// ========== THÊM CHÂN BUZZER ==========
#define BUZZER_PIN 1        // GPIO 1 cho buzzer (an toàn với ESP32-C3)
#define BUZZER_ON_LEVEL 1   // 1 = HIGH để bật buzzer
#define BUZZER_BEEP_TIME 500 // Thời gian kêu mỗi lần (ms)
// ======================================

// ========== NÚT NHẤN VẬT LÝ (THOÁT KHẨN CẤP) ==========
#define EXIT_BUTTON_PIN 18  // GPIO 18 - Nút thoát riêng
#define BUTTON_ACTIVE_LEVEL 0  // Active LOW (nhấn = 0)
// ======================================================


static const char *TAG = "RC522+Keypad+rainmaker";
esp_rmaker_device_t *switch_device;

// Forward declarations
static spi_device_handle_t handle;
static void reset_rc522_reader(void);

// Cờ để báo cần reset RC522 (tránh gọi SPI từ callback)
volatile bool need_rc522_reset = false;

// ========== THỐNG KÊ HỆ THỐNG ==========
typedef struct {
    uint32_t total_door_opens;      // Tổng số lần mở cửa
    uint32_t rfid_success;          // RFID + mật khẩu đúng
    uint32_t rfid_failed;           // RFID hoặc mật khẩu sai
    uint32_t intruder_alerts;       // Số lần cảnh báo
    uint32_t app_opens;             // Số lần mở từ app
    char most_frequent_user[32];   // Người dùng thường xuyên nhất
} SystemStats;

static SystemStats stats = {0};

static esp_rmaker_device_t *stats_device_global = NULL;

void update_stats_to_rainmaker(void) {
    // Tạo chuỗi thống kê để log
    char stats_str[256];
    snprintf(stats_str, sizeof(stats_str),
             "Thong ke:\n"
             "Mo cua: %lu\n"
             "RFID thanh cong: %lu\n"
             "RFID that bai: %lu\n"
             " Canh bao: %lu\n"
             "Mo tu app: %lu\n"
             " Thuong xuyen: %s",
             stats.total_door_opens,
             stats.rfid_success,
             stats.rfid_failed,
             stats.intruder_alerts,
             stats.app_opens,
             stats.most_frequent_user[0] ? stats.most_frequent_user : "Chua co");
    
    ESP_LOGI(TAG, "%s", stats_str);
    
    // Cập nhật lên RainMaker
    if (stats_device_global) {
        esp_rmaker_param_update_and_report(
            esp_rmaker_device_get_param_by_name(stats_device_global, "Tổng Số Lần Mở"),
            esp_rmaker_int(stats.total_door_opens));
        esp_rmaker_param_update_and_report(
            esp_rmaker_device_get_param_by_name(stats_device_global, "RFID Thành Công"),
            esp_rmaker_int(stats.rfid_success));
        esp_rmaker_param_update_and_report(
            esp_rmaker_device_get_param_by_name(stats_device_global, "RFID Thất Bại"),
            esp_rmaker_int(stats.rfid_failed));
        esp_rmaker_param_update_and_report(
            esp_rmaker_device_get_param_by_name(stats_device_global, "Cảnh Báo Xâm Nhập"),
            esp_rmaker_int(stats.intruder_alerts));
        esp_rmaker_param_update_and_report(
            esp_rmaker_device_get_param_by_name(stats_device_global, "Mở Từ App"),
            esp_rmaker_int(stats.app_opens));
        esp_rmaker_param_update_and_report(
            esp_rmaker_device_get_param_by_name(stats_device_global, "Người Dùng Nhiều Nhất"),
            esp_rmaker_str(stats.most_frequent_user[0] ? stats.most_frequent_user : "Chưa có"));
        ESP_LOGI(TAG, "✓ Đã cập nhật thống kê lên RainMaker");
    }
}
// =======================================

// Task riêng để kết nối WiFi (không block main)
static void wifi_connect_task(void *arg) {
    esp_err_t err = app_network_set_custom_mfg_data(MGF_DATA_DEVICE_TYPE_SWITCH, MFG_DATA_DEVICE_SUBTYPE_SWITCH);
    err = app_network_start(POP_TYPE_RANDOM);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "✓ WiFi connected - RainMaker available");
    } else {
        ESP_LOGW(TAG, "WiFi connection failed - Running offline mode");
    }
    vTaskDelete(NULL);
}

/* Callback to handle commands received from the RainMaker cloud */
// Task để xử lý relay tự động tắt (không block MQTT callback)
static void relay_auto_close_task(void *arg) {
    ESP_LOGI(TAG, "Mở cửa từ RainMaker trong %d giây...", DOOR_OPEN_TIME/1000);
    
    // Cập nhật thống kê
    stats.total_door_opens++;
    stats.app_opens++;
    update_stats_to_rainmaker();
    
    // Bật relay
    app_driver_set_state(true);
    
    // Đợi DOOR_OPEN_TIME
    vTaskDelay(pdMS_TO_TICKS(DOOR_OPEN_TIME));
    
    // Tắt relay
    app_driver_set_state(false);
    
    // Cập nhật trạng thái về app
    if (switch_device) {
        esp_rmaker_param_update_and_report(
            esp_rmaker_device_get_param_by_name(switch_device, ESP_RMAKER_DEF_POWER_NAME),
            esp_rmaker_bool(false));
    }
    
    ESP_LOGI(TAG, "Đã đóng cửa tự động");
    
    // Đặt cờ để main loop reset RC522
    need_rc522_reset = true;
    
    vTaskDelete(NULL);  // Xóa task khi hoàn thành
}

static esp_err_t write_cb(const esp_rmaker_device_t *device, const esp_rmaker_param_t *param,
            const esp_rmaker_param_val_t val, void *priv_data, esp_rmaker_write_ctx_t *ctx)
{
    if (ctx) {
        ESP_LOGI(TAG, "Received write request via : %s", esp_rmaker_device_cb_src_to_str(ctx->src));
    }
    if (strcmp(esp_rmaker_param_get_name(param), ESP_RMAKER_DEF_POWER_NAME) == 0) {
        ESP_LOGI(TAG, "Received value = %s for %s - %s",
                val.val.b? "true" : "false", esp_rmaker_device_get_name(device),
                esp_rmaker_param_get_name(param));
        
        // Chỉ xử lý khi BẬT (true) - Mở cửa tạm thời
        if (val.val.b) {
            // Cập nhật trạng thái ngay lập tức
            esp_rmaker_param_update(param, esp_rmaker_bool(true));
            
            // Tạo task riêng để xử lý relay (không block callback)
            xTaskCreate(relay_auto_close_task, "relay_close", 4096, NULL, 5, NULL);
        } else {
            // Lệnh TẮT - Bỏ qua (vì relay tự động tắt sau 3 giây)
            ESP_LOGI(TAG, "==> OFF command ignored (relay auto-closes)");
        }
    }
    return ESP_OK;
}
/* Event handler for catching RainMaker events */
static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data)
{
    if (event_base == RMAKER_EVENT) {
        switch (event_id) {
            case RMAKER_EVENT_INIT_DONE:
                ESP_LOGI(TAG, "RainMaker Initialised.");
                break;
            case RMAKER_EVENT_CLAIM_STARTED:
                ESP_LOGI(TAG, "RainMaker Claim Started.");
                break;
            case RMAKER_EVENT_CLAIM_SUCCESSFUL:
                ESP_LOGI(TAG, "RainMaker Claim Successful.");
                break;
            case RMAKER_EVENT_CLAIM_FAILED:
                ESP_LOGI(TAG, "RainMaker Claim Failed.");
                break;
            case RMAKER_EVENT_LOCAL_CTRL_STARTED:
                ESP_LOGI(TAG, "Local Control Started.");
                break;
            case RMAKER_EVENT_LOCAL_CTRL_STOPPED:
                ESP_LOGI(TAG, "Local Control Stopped.");
                break;
            default:
                ESP_LOGW(TAG, "Unhandled RainMaker Event: %"PRIi32, event_id);
        }
    } else if (event_base == RMAKER_COMMON_EVENT) {
        switch (event_id) {
            case RMAKER_EVENT_REBOOT:
                ESP_LOGI(TAG, "Rebooting in %d seconds.", *((uint8_t *)event_data));
                break;
            case RMAKER_EVENT_WIFI_RESET:
                ESP_LOGI(TAG, "Wi-Fi credentials reset.");
                break;
            case RMAKER_EVENT_FACTORY_RESET:
                ESP_LOGI(TAG, "Node reset to factory defaults.");
                break;
            case RMAKER_MQTT_EVENT_CONNECTED:
                ESP_LOGI(TAG, "MQTT Connected.");
                break;
            case RMAKER_MQTT_EVENT_DISCONNECTED:
                ESP_LOGI(TAG, "MQTT Disconnected.");
                break;
            case RMAKER_MQTT_EVENT_PUBLISHED:
                ESP_LOGI(TAG, "MQTT Published. Msg id: %d.", *((int *)event_data));
                break;
            default:
                ESP_LOGW(TAG, "Unhandled RainMaker Common Event: %"PRIi32, event_id);
        }
    } else if (event_base == APP_NETWORK_EVENT) {
        switch (event_id) {
            case APP_NETWORK_EVENT_QR_DISPLAY:
                ESP_LOGI(TAG, "Provisioning QR : %s", (char *)event_data);
                break;
            case APP_NETWORK_EVENT_PROV_TIMEOUT:
                ESP_LOGI(TAG, "Provisioning Timed Out. Please reboot.");
                break;
            case APP_NETWORK_EVENT_PROV_RESTART:
                ESP_LOGI(TAG, "Provisioning has restarted due to failures.");
                break;
            default:
                ESP_LOGW(TAG, "Unhandled App Wi-Fi Event: %"PRIi32, event_id);
                break;
        }
    } else if (event_base == RMAKER_OTA_EVENT) {
        switch(event_id) {
            case RMAKER_OTA_EVENT_STARTING:
                ESP_LOGI(TAG, "Starting OTA.");
                break;
            case RMAKER_OTA_EVENT_IN_PROGRESS:
                ESP_LOGI(TAG, "OTA is in progress.");
                break;
            case RMAKER_OTA_EVENT_SUCCESSFUL:
                ESP_LOGI(TAG, "OTA successful.");
                break;
            case RMAKER_OTA_EVENT_FAILED:
                ESP_LOGI(TAG, "OTA Failed.");
                break;
            case RMAKER_OTA_EVENT_REJECTED:
                ESP_LOGI(TAG, "OTA Rejected.");
                break;
            case RMAKER_OTA_EVENT_DELAYED:
                ESP_LOGI(TAG, "OTA Delayed.");
                break;
            case RMAKER_OTA_EVENT_REQ_FOR_REBOOT:
                ESP_LOGI(TAG, "Firmware image downloaded. Please reboot your device to apply the upgrade.");
                break;
            default:
                ESP_LOGW(TAG, "Unhandled OTA Event: %"PRIi32, event_id);
                break;
        }
    } else {
        ESP_LOGW(TAG, "Invalid event received!");
    }
}

// handle SPI đã được khai báo ở đầu file (static spi_device_handle_t handle;)

// THẺ ADMIN
#define ADMIN_UID_SIZE 4
uint8_t ADMIN_CARD_UID[ADMIN_UID_SIZE] = {0x21, 0x5F, 0x1E, 0x4C}; // Thay đổi sau khi biết UID

// Danh sách thẻ (sẽ được load từ Flash)
AllowedCard allowed_cards[MAX_ALLOWED_CARDS];
int num_allowed_cards = 0;

// Forward declarations
void admin_menu(void);
void admin_view_all_passwords(void);
void admin_change_password_mode(void);
void admin_reset_all_passwords(void);
void reset_rc522_reader(void);

// Hàm hiển thị OLED
void oled_show_message(const char *line1, const char *line2, const char *line3, const char *line4);
void oled_show_card_detected(const char *name);
void oled_show_password_prompt(void);
void oled_show_access_granted(void);
void oled_show_access_denied(void);
void oled_show_admin_menu(void);
void oled_show_ready(void);

// ========== HÀM ĐIỀU KHIỂN BUZZER ==========
void init_buzzer(void)
{
    // Cấu hình LEDC timer với 10-bit resolution để tăng cường độ
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,  // 0-1023 (max duty)
        .timer_num = LEDC_TIMER_1,
        .freq_hz = 2500,  // 2.5kHz - tần số cao hơn = to hơn cho buzzer active
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);
    
    // Cấu hình LEDC channel
    ledc_channel_config_t channel_conf = {
        .gpio_num = BUZZER_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,  // Ban đầu TẮT
        .hpoint = 0
    };
    ledc_channel_config(&channel_conf);
    
    ESP_LOGI(TAG, "✓ Buzzer+LED (PWM MAX) initialized on GPIO %d @ 2.5kHz", BUZZER_PIN);
}

void buzzer_beep(int times)
{
    for (int i = 0; i < times; i++) {
        // Duty cycle 1023 = 100% với 10-bit = TO NHẤT + SÁNG NHẤT
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 1023);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
        vTaskDelay(pdMS_TO_TICKS(BUZZER_BEEP_TIME));
        
        // TẮT
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
        if (i < times - 1) {
            vTaskDelay(pdMS_TO_TICKS(200)); // Ngắt giữa các tiếng beep
        }
    }
}

void alert_intruder(const char *reason)
{
    ESP_LOGW(TAG, "CANH BAO: %s", reason);
    
    // Kêu buzzer 3 lần
    buzzer_beep(3);
    
    // Gửi alert đến RainMaker app
    if (switch_device) {
        char alert_msg[128];
        snprintf(alert_msg, sizeof(alert_msg), "CANH BAO: %s", reason);
        esp_rmaker_raise_alert(alert_msg);
        ESP_LOGI(TAG, "Alert sent to RainMaker app");
    }
}
// =========================================

// ========== NÚT NHẤN VẬT LÝ - EXIT BUTTON ==========
void init_exit_button(void)
{
    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << EXIT_BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,  // Pull-up vì active LOW
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn_conf);
    
    ESP_LOGI(TAG, "✓ Exit button initialized on GPIO %d (Press to open door)", EXIT_BUTTON_PIN);
}

bool is_exit_button_pressed(void)
{
    return (gpio_get_level(EXIT_BUTTON_PIN) == BUTTON_ACTIVE_LEVEL);
}
// ==================================================

// ========== HÀM ĐIỀU KHIỂN RELAY ==========
// Relay đã được khởi tạo bởi app_driver_init() trong app_driver.c
// Không cần init_relay() riêng nữa

void open_door(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, " MỞ CỬA ");
    
    oled_show_access_granted();
    
    // ===== BƯỚC 1: TẮT RC522 TRƯỚC KHI BẬT RELAY =====
    ESP_LOGI(TAG, ">>> Tạm dừng RC522...");
    
    // Dừng giao tiếp với thẻ
    PICC_HaltA(handle);
    PCD_StopCrypto1(handle);
    
    // Tắt antenna để giảm nhiễu
    PCD_WriteRegister(handle, TxControlReg, 0x00);
    
    // Đưa RC522 vào chế độ idle
    PCD_WriteRegister(handle, CommandReg, PCD_Idle);
    
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // ===== BƯỚC 2: BẬT RELAY =====
    gpio_set_level(RELAY_PIN, RELAY_ON_LEVEL);
    ESP_LOGI(TAG, ">>> Relay ON - Khóa cửa đã mở <<<");
    
    // Giữ mở trong DOOR_OPEN_TIME
    vTaskDelay(pdMS_TO_TICKS(DOOR_OPEN_TIME));
    
    // ===== BƯỚC 3: TẮT RELAY =====
    gpio_set_level(RELAY_PIN, !RELAY_ON_LEVEL);
    ESP_LOGI(TAG, ">>> Relay OFF - Khóa cửa đã đóng <<<");
    
    // Đợi relay ổn định
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // ===== BƯỚC 4: KHỞI ĐỘNG LẠI RC522 =====
    ESP_LOGI(TAG, ">>> Khoi dong lai RC522...");
    
    // Hiển thị đang đợi hệ thống
    oled_show_message("Dang doi", "he thong...", NULL, NULL);
    
    reset_rc522_reader();
    
    // Đợi thêm để RC522 hoàn toàn ổn định
    vTaskDelay(pdMS_TO_TICKS(200));
    
    ESP_LOGI(TAG, "He thong san sang quet the tiep theo");
    ESP_LOGI(TAG, "");
    
    // Hiển thị sẵn sàng quét
    oled_show_ready();
}

// ========== HÀM RESET RC522 CẢI TIẾN ==========
void reset_rc522_reader(void)
{
    ESP_LOGI(TAG, " Đang reset RC522 (full reset)...");
    
    // 1. HARD RESET - Reset hoàn toàn phần cứng
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));  // Tăng từ 50ms lên 100ms
    
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));  // Tăng từ 50ms lên 100ms
    
    // 2. Dừng mọi giao tiếp với thẻ
    PICC_HaltA(handle);
    PCD_StopCrypto1(handle);
    
    // 3. SOFT RESET - Reset logic bên trong chip
    PCD_WriteRegister(handle, CommandReg, PCD_SoftReset);
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // 4. Đợi soft reset hoàn tất
    uint8_t timeout = 0;
    do {
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout++;
        if (timeout > 50) {
            ESP_LOGE(TAG, "Soft reset timeout!");
            break;
        }
    } while (PCD_ReadRegister(handle, CommandReg) & (1 << 4));
    
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // 5. Khởi tạo lại các thanh ghi - QUAN TRỌNG!
    PCD_WriteRegister(handle, TxModeReg, 0x00);
    PCD_WriteRegister(handle, RxModeReg, 0x00);
    PCD_WriteRegister(handle, ModWidthReg, 0x26);
    
    // 6. Cấu hình Timer
    PCD_WriteRegister(handle, TModeReg, 0x80);
    PCD_WriteRegister(handle, TPrescalerReg, 0xA9);
    PCD_WriteRegister(handle, TReloadRegH, 0x03);
    PCD_WriteRegister(handle, TReloadRegL, 0xE8);
    
    // 7. Cấu hình Transmission
    PCD_WriteRegister(handle, TxASKReg, 0x40);
    PCD_WriteRegister(handle, ModeReg, 0x3D);
    
    // 8. Clear tất cả các interrupt flags
    PCD_WriteRegister(handle, ComIrqReg, 0x7F);
    PCD_WriteRegister(handle, DivIrqReg, 0x7F);
    
    // 9. Clear FIFO buffer
    PCD_WriteRegister(handle, FIFOLevelReg, 0x80);
    
    // 10. Clear collision bit
    PCD_ClearRegisterBitMask(handle, CollReg, 0x80);
    
    // 11. Bật lại antenna
    PCD_AntennaOn(handle);
    vTaskDelay(pdMS_TO_TICKS(50));
    
}
// =========================================

// ========== HÀM HIỂN THỊ OLED ==========
void oled_show_message(const char *line1, const char *line2, const char *line3, const char *line4)
{
    oled_clear_screen();
    if (line1) oled_write_string(line1, 0, 0);
    if (line2) oled_write_string(line2, 0, 16);
    if (line3) oled_write_string(line3, 0, 32);
    if (line4) oled_write_string(line4, 0, 48);
    oled_update_screen();
}

void oled_show_card_detected(const char *name)
{
    char name_line[32];
    snprintf(name_line, sizeof(name_line), "Card: %.16s", name);
    oled_show_message("RFID", name_line, NULL, NULL);
}

void oled_show_password_prompt(void)
{
    oled_show_message("Nhap mat khau", "Nhan # de ENTER", NULL, NULL);
}

void oled_show_access_granted(void)
{
    oled_show_message("Mat Khau Dung", "Cua Mo", NULL, NULL);
}

void oled_show_access_denied(void)
{
    oled_show_message("Mat Khau Sai", "Buzzer!...", NULL, NULL);
}

void oled_show_admin_menu(void)
{
    oled_show_message("ADMIN MODE", "Check Monitor", "for Menu", NULL);
}

void oled_show_ready(void)
{
    oled_show_message("San sang quet", "Quet the...", NULL, NULL);
}
// =========================================

// Hàm kiểm tra xem có phải thẻ admin không
bool is_admin_card(uint8_t *uid, uint8_t uid_size)
{
    if (uid_size != ADMIN_UID_SIZE) return false;
    
    for (int i = 0; i < ADMIN_UID_SIZE; i++) {
        if (uid[i] != ADMIN_CARD_UID[i]) {
            return false;
        }
    }
    return true;
}

// Hàm kiểm tra card hợp lệ
bool is_card_allowed(uint8_t *uid, uint8_t uid_size, char *card_name, char *card_password)
{
    if (uid_size != UID_SIZE) return false;
    
    for (int i = 0; i < num_allowed_cards; i++) {
        bool match = true;
        for (int j = 0; j < UID_SIZE; j++) {
            if (uid[j] != allowed_cards[i].uid[j]) {
                match = false;
                break;
            }
        }
        
        if (match) {
            strcpy(card_name, allowed_cards[i].name);
            strcpy(card_password, allowed_cards[i].password);
            return true;
        }
    }
    
    return false;
}

// Hàm hiển thị danh sách thẻ
void display_card_list(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== DANH SÁCH THẺ ===");
    for (int i = 0; i < num_allowed_cards; i++) {
        printf("%d. %-20s | UID: ", i + 1, allowed_cards[i].name);
        for (int j = 0; j < UID_SIZE; j++) {
            printf("%02X ", allowed_cards[i].uid[j]);
        }
        printf("| Pass: %s\n", allowed_cards[i].password);
    }
    ESP_LOGI(TAG, "=====================");
    ESP_LOGI(TAG, "");
}

// Hàm xem tất cả mật khẩu (chế độ admin)
void admin_view_all_passwords(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "*** CHE DO ADMIN - XEM MAT KHAU ***");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== DANH SACH THE VA MAT KHAU ===");
    
    // Hiển thị từng thẻ trên OLED với delay để người dùng xem
    for (int i = 0; i < num_allowed_cards; i++) {
        // Hiển thị trên Monitor
        printf("-------------------------------------\n");
        printf("  %d. %-20s\n", i + 1, allowed_cards[i].name);
        printf("     UID      : ");
        for (int j = 0; j < UID_SIZE; j++) {
            printf("%02X ", allowed_cards[i].uid[j]);
        }
        printf("\n");
        printf("     MAT KHAU : %s\n", allowed_cards[i].password);
        
        // Hiển thị trên OLED
        char line1[32], line2[32], line3[32];
        snprintf(line1, sizeof(line1), "%d. %.13s", i + 1, allowed_cards[i].name);
        snprintf(line2, sizeof(line2), "UID: %02X%02X%02X%02X", 
                 allowed_cards[i].uid[0], allowed_cards[i].uid[1],
                 allowed_cards[i].uid[2], allowed_cards[i].uid[3]);
        snprintf(line3, sizeof(line3), "Pass: %s", allowed_cards[i].password);
        
        oled_show_message(line1, line2, line3, "Press key to next");
        
        // Chờ người dùng nhấn phím bất kỳ hoặc timeout 5 giây
        char dummy[2];
        keypad_read_password(dummy, sizeof(dummy), 5000);
    }
    
    printf("-------------------------------------\n");
    ESP_LOGI(TAG, "");
    
    // Hiển thị kết thúc
    oled_show_message("View Complete", "Total cards:", NULL, NULL);
    vTaskDelay(pdMS_TO_TICKS(2000));
}

// Hàm đổi mật khẩu (chế độ admin)
void admin_change_password_mode(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "*** CHE DO ADMIN - DOI MAT KHAU ***");
    
    oled_show_message("Change Password", "Select card", "number...", NULL);
    
    display_card_list();
    
    ESP_LOGI(TAG, "Nhap so thu tu the can doi (1-%d), nhan #:", num_allowed_cards);
    
    char input_num[4] = {0};
    int len = keypad_read_password(input_num, sizeof(input_num), 15000);
    
    if (len <= 0) {
        ESP_LOGW(TAG, "Timeout! Thoat che do admin.");
        oled_show_message("Timeout!", NULL, NULL, NULL);
        vTaskDelay(pdMS_TO_TICKS(1500));
        return;
    }
    
    int card_index = atoi(input_num) - 1;
    
    if (card_index < 0 || card_index >= num_allowed_cards) {
        ESP_LOGE(TAG, "So thu tu khong hop le!");
        oled_show_message("Invalid number!", NULL, NULL, NULL);
        vTaskDelay(pdMS_TO_TICKS(1500));
        return;
    }
    
    ESP_LOGI(TAG, "Ban chon: %s", allowed_cards[card_index].name);
    ESP_LOGI(TAG, "Mat khau hien tai: %s", allowed_cards[card_index].password);
    
    // Hiển thị thẻ được chọn
    char line1[32], line2[32];
    snprintf(line1, sizeof(line1), "%.16s", allowed_cards[card_index].name);
    snprintf(line2, sizeof(line2), "Old: %s", allowed_cards[card_index].password);
    oled_show_message(line1, line2, "Enter new pass", NULL);
    
    char old_password[16];
    strcpy(old_password, allowed_cards[card_index].password);
    
    ESP_LOGI(TAG, "Nhap mat khau moi (4-15 ky tu), nhan #:");
    
    char new_password[16] = {0};
    len = keypad_read_password(new_password, sizeof(new_password), 20000);
    
    if (len < 4) {
        ESP_LOGE(TAG, "Mat khau qua ngan! Toi thieu 4 ky tu.");
        oled_show_message("Error!", "Pass too short", "Min 4 chars", NULL);
        vTaskDelay(pdMS_TO_TICKS(2000));
        return;
    }
    
    ESP_LOGI(TAG, "Nhap lai mat khau moi de xac nhan:");
    oled_show_message("Confirm", "Re-enter pass", NULL, NULL);
    
    char confirm_password[16] = {0};
    len = keypad_read_password(confirm_password, sizeof(confirm_password), 20000);
    
    if (strcmp(new_password, confirm_password) != 0) {
        ESP_LOGE(TAG, "Mat khau xac nhan khong khop!");
        oled_show_message("Error!", "Password", "not match!", NULL);
        vTaskDelay(pdMS_TO_TICKS(2000));
        return;
    }
    
    oled_show_message("Saving...", NULL, NULL, NULL);
    
    esp_err_t ret = change_card_password(allowed_cards, num_allowed_cards, 
                                         allowed_cards[card_index].uid, new_password);
    
    if (ret == ESP_OK) {
        save_password_change_log(allowed_cards[card_index].name, old_password, new_password);
        
        ESP_LOGI(TAG, "Doi mat khau thanh cong!");
        ESP_LOGI(TAG, "The: %s", allowed_cards[card_index].name);
        ESP_LOGI(TAG, "Mat khau moi: %s", new_password);
        
        char result[32];
        snprintf(result, sizeof(result), "New: %s", new_password);
        oled_show_message("Success!", allowed_cards[card_index].name, result, NULL);
    } else {
        ESP_LOGE(TAG, "Loi khi luu mat khau vao Flash!");
        oled_show_message("Error!", "Save failed", NULL, NULL);
    }
    
    vTaskDelay(pdMS_TO_TICKS(2000));
}

// Hàm reset tất cả mật khẩu về mặc định
void admin_reset_all_passwords(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "*** RESET TAT CA MAT KHAU VE MAC DINH ***");
    ESP_LOGI(TAG, "");
    ESP_LOGW(TAG, "CANH BAO: Tat ca mat khau se duoc reset!");
    ESP_LOGI(TAG, "Nhap '1234' de xac nhan, nhan #:");
    
    oled_show_message("!!! WARNING !!!", "Reset all pass", "Enter 1234", "to confirm");
    
    char confirm[16] = {0};
    int len = keypad_read_password(confirm, sizeof(confirm), 15000);
    
    if (len <= 0 || strcmp(confirm, "1234") != 0) {
        ESP_LOGE(TAG, "Huy bo reset!");
        oled_show_message("Cancelled", NULL, NULL, NULL);
        vTaskDelay(pdMS_TO_TICKS(1500));
        return;
    }
    
    AllowedCard default_cards[] = {
        {{0x41, 0x89, 0x3b, 0x16}, "Quy Nguyen", "0403"},
        {{0x23, 0xe5, 0x64, 0xa9}, "Tam Ho", "0407"},
        {{0x43, 0xe0, 0x9c, 0xa5}, "Nhat Trinh", "0608"},
        {{0xb7, 0x22, 0xc2, 0x01}, "Nhan Cot", "0309"},
    };
    
    memcpy(allowed_cards, default_cards, sizeof(default_cards));
    
    oled_show_message("Resetting...", NULL, NULL, NULL);
    
    esp_err_t ret = save_cards_to_flash(allowed_cards, num_allowed_cards);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Reset thanh cong!");
        ESP_LOGI(TAG, "Tat ca mat khau da ve mac dinh.");
        oled_show_message("Success!", "All passwords", "reset to default", NULL);
        vTaskDelay(pdMS_TO_TICKS(2000));
        display_card_list();
    } else {
        ESP_LOGE(TAG, "Loi khi reset!");
        oled_show_message("Error!", "Reset failed", NULL, NULL);
    }
    
    vTaskDelay(pdMS_TO_TICKS(2000));
}

//Admin thêm thẻ (với quét RFID)
void admin_add_new_card(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "*** CHE DO ADMIN - THEM THE MOI ***");
    
    if (num_allowed_cards >= MAX_ALLOWED_CARDS) {
        ESP_LOGE(TAG, "Danh sach the da day! (Max: %d the)", MAX_ALLOWED_CARDS);
        oled_show_message("Error!", "Card list full", NULL, NULL);
        vTaskDelay(pdMS_TO_TICKS(2000));
        return;
    }
    
    ESP_LOGI(TAG, "Buoc 1: Nhap ten the (dung so de dai dien)");
    ESP_LOGI(TAG, "VD: 1234 = User1234");
    ESP_LOGI(TAG, "Nhap ten (4-8 so), nhan #:");
    
    oled_show_message("Add New Card", "Step 1/3", "Enter name", "(numbers only)");
    
    char name_input[16] = {0};
    int len = keypad_read_password(name_input, sizeof(name_input), 20000);
    
    if (len < 1) {
        ESP_LOGE(TAG, "Timeout hoac ten qua ngan!");
        oled_show_message("Timeout!", NULL, NULL, NULL);
        vTaskDelay(pdMS_TO_TICKS(1500));
        return;
    }
    
    char full_name[32];
    snprintf(full_name, sizeof(full_name), "User_%s", name_input);
    
    ESP_LOGI(TAG, "Ten the: %s", full_name);
    
    char line[32];
    snprintf(line, sizeof(line), "Name: User_%s", name_input);
    oled_show_message("Step 2/3", line, "Enter password", "(min 4 chars)");
    
    ESP_LOGI(TAG, "Buoc 2: Nhap mat khau cho the (4-15 ky tu), nhan #:");
    
    char password_input[16] = {0};
    len = keypad_read_password(password_input, sizeof(password_input), 20000);
    
    if (len < 4) {
        ESP_LOGE(TAG, "Mat khau qua ngan! Toi thieu 4 ky tu.");
        oled_show_message("Error!", "Pass too short", NULL, NULL);
        vTaskDelay(pdMS_TO_TICKS(1500));
        return;
    }
    
    ESP_LOGI(TAG, "Mat khau: %s", password_input);
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Buoc 3: Quet the RFID moi...");
    ESP_LOGI(TAG, "Hay quet the ban muon them vao he thong");
    
    oled_show_message("Step 3/3", "Scan new card", "now...", NULL);
    
    // Reset RC522 trước khi quét
    reset_rc522_reader();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    uint8_t new_uid[UID_SIZE] = {0};
    bool card_scanned = false;
    
    for (int i = 0; i < 150; i++) {
        if (PICC_IsNewCardPresent(handle)) {
            if (PICC_Select(handle, &uid, 0) == STATUS_OK) {
                memcpy(new_uid, uid.uidByte, UID_SIZE);
                card_scanned = true;
                
                printf("✓ Đã quét thẻ! UID: ");
                for (int j = 0; j < UID_SIZE; j++) {
                    printf("%02X ", new_uid[j]);
                }
                printf("\n");
                
                PICC_HaltA(handle);
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    if (!card_scanned) {
        ESP_LOGE(TAG, "Timeout! Khong quet duoc the.");
        oled_show_message("Error!", "No card scanned", NULL, NULL);
        vTaskDelay(pdMS_TO_TICKS(1500));
        return;
    }
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== XAC NHAN THONG TIN ===");
    ESP_LOGI(TAG, "Ten: %s", full_name);
    ESP_LOGI(TAG, "Mat khau: %s", password_input);
    printf("UID: ");
    for (int j = 0; j < UID_SIZE; j++) {
        printf("%02X ", new_uid[j]);
    }
    printf("\n");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Nhap '1' de xac nhan, '0' de huy, nhan #:");
    
    char uid_line[32];
    snprintf(uid_line, sizeof(uid_line), "UID:%02X%02X%02X%02X", new_uid[0], new_uid[1], new_uid[2], new_uid[3]);
    oled_show_message("Confirm?", uid_line, "1=Yes 0=No", NULL);
    
    char confirm[4] = {0};
    len = keypad_read_password(confirm, sizeof(confirm), 10000);
    
    if (len > 0 && confirm[0] == '1') {
        AllowedCard new_card;
        memcpy(new_card.uid, new_uid, UID_SIZE);
        strncpy(new_card.name, full_name, sizeof(new_card.name) - 1);
        new_card.name[sizeof(new_card.name) - 1] = '\0';
        strncpy(new_card.password, password_input, sizeof(new_card.password) - 1);
        new_card.password[sizeof(new_card.password) - 1] = '\0';
        
        allowed_cards[num_allowed_cards] = new_card;
        num_allowed_cards++;
        
        oled_show_message("Saving...", NULL, NULL, NULL);
        
        esp_err_t ret = save_cards_to_flash(allowed_cards, num_allowed_cards);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "THEM THE THANH CONG!");
            ESP_LOGI(TAG, "Tong so the: %d/%d", num_allowed_cards, MAX_ALLOWED_CARDS);
            
            char total[32];
            snprintf(total, sizeof(total), "Total: %d/%d", num_allowed_cards, MAX_ALLOWED_CARDS);
            oled_show_message("Success!", "Card added", total, NULL);
            
            vTaskDelay(pdMS_TO_TICKS(2000));
            display_card_list();
        } else {
            ESP_LOGE(TAG, "Loi khi them the!");
            oled_show_message("Error!", "Save failed", NULL, NULL);
        }
    } else {
        ESP_LOGI(TAG, "Da huy them the.");
        oled_show_message("Cancelled", NULL, NULL, NULL);
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
    
    vTaskDelay(pdMS_TO_TICKS(2000));
}

void admin_delete_card_menu(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "*** CHE DO ADMIN - XOA THE ***");
    
    if (num_allowed_cards == 0) {
        ESP_LOGE(TAG, "Danh sach the trong! Khong co the de xoa.");
        oled_show_message("Error!", "No cards", "to delete", NULL);
        vTaskDelay(pdMS_TO_TICKS(2000));
        return;
    }
    
    oled_show_message("Delete Card", "Select number", NULL, NULL);
    
    display_card_list();
    
    ESP_LOGI(TAG, "Nhap so thu tu the can XOA (1-%d), nhan #:", num_allowed_cards);
    
    char input_num[4] = {0};
    int len = keypad_read_password(input_num, sizeof(input_num), 15000);
    
    if (len <= 0) {
        ESP_LOGW(TAG, "Timeout! Thoat che do xoa the.");
        oled_show_message("Timeout!", NULL, NULL, NULL);
        vTaskDelay(pdMS_TO_TICKS(1500));
        return;
    }
    
    int card_index = atoi(input_num) - 1;
    
    if (card_index < 0 || card_index >= num_allowed_cards) {
        ESP_LOGE(TAG, "So thu tu khong hop le!");
        oled_show_message("Invalid number!", NULL, NULL, NULL);
        vTaskDelay(pdMS_TO_TICKS(1500));
        return;
    }
    
    ESP_LOGI(TAG, "");
    ESP_LOGW(TAG, "=== CANH BAO: BAN SAP XOA THE ===");
    ESP_LOGI(TAG, "Ten: %s", allowed_cards[card_index].name);
    printf("UID: ");
    for (int j = 0; j < UID_SIZE; j++) {
        printf("%02X ", allowed_cards[card_index].uid[j]);
    }
    printf("\n");
    ESP_LOGI(TAG, "Mat khau: %s", allowed_cards[card_index].password);
    ESP_LOGI(TAG, "====================================");
    ESP_LOGI(TAG, "");
    
    char warn_line[32];
    snprintf(warn_line, sizeof(warn_line), "%.16s", allowed_cards[card_index].name);
    oled_show_message("!!! WARNING !!!", warn_line, "Delete?", "1=Yes 0=No");
    
    ESP_LOGI(TAG, "Nhap '1' de XAC NHAN XOA, '0' de huy, nhan #:");
    
    char confirm[4] = {0};
    len = keypad_read_password(confirm, sizeof(confirm), 10000);
    
    if (len > 0 && confirm[0] == '1') {
        oled_show_message("Deleting...", NULL, NULL, NULL);
        
        esp_err_t ret = admin_delete_card(allowed_cards, &num_allowed_cards, card_index);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "XOA THE THANH CONG!");
            ESP_LOGI(TAG, "Tong so the con lai: %d/%d", num_allowed_cards, MAX_ALLOWED_CARDS);
            
            char total[32];
            snprintf(total, sizeof(total), "Remaining: %d", num_allowed_cards);
            oled_show_message("Success!", "Card deleted", total, NULL);
            
            vTaskDelay(pdMS_TO_TICKS(2000));
            if (num_allowed_cards > 0) {
                display_card_list();
            } else {
                ESP_LOGI(TAG, "Danh sach the da trong!");
            }
        } else {
            ESP_LOGE(TAG, "Loi khi xoa the!");
            oled_show_message("Error!", "Delete failed", NULL, NULL);
            vTaskDelay(pdMS_TO_TICKS(1500));
        }
    } else {
        ESP_LOGI(TAG, "Da huy xoa the.");
        oled_show_message("Cancelled", NULL, NULL, NULL);
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
    
    vTaskDelay(pdMS_TO_TICKS(2000));
}

// ADMIN_MENU
void admin_menu(void)
{
    while (1)   
    {
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "THE ADMIN duoc phat hien!");
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "=== MENU ADMIN ===");
        ESP_LOGI(TAG, "1. Xem tat ca mat khau");
        ESP_LOGI(TAG, "2. Doi mat khau the");
        ESP_LOGI(TAG, "3. Reset mat khau ve mac dinh");
        ESP_LOGI(TAG, "4. Xem lich su thay doi");
        ESP_LOGI(TAG, "5. Them the moi");  
        ESP_LOGI(TAG, "6. Xoa the");  
        ESP_LOGI(TAG, "7. Thoat");   
        ESP_LOGI(TAG, "==================");
        ESP_LOGI(TAG, "Nhap so (1-7), nhan #:");
        
        // Hiển thị menu trên OLED
        oled_show_message("=== ADMIN MENU ===", "1.View 2.Change", "3.Reset 4.History", "5.Add 6.Del 7.Exit");

        char choice[4] = {0};
        int len = keypad_read_password(choice, sizeof(choice), -1);  // -1 = không timeout

        if (len <= 0) {
            // Không có timeout, chỉ thoát khi nhấn 7
            continue;
        }

        int menu_choice = atoi(choice);

        switch(menu_choice) {
            case 1:
                admin_view_all_passwords();
                break;

            case 2:
                admin_change_password_mode();
                break;

            case 3:
                admin_reset_all_passwords();
                break;

            case 4:
                oled_show_message("Loading", "History...", NULL, NULL);
                view_password_change_history();
                break;
                
            case 5:
                admin_add_new_card();
                break;

            case 6:
                admin_delete_card_menu();
                break;

            case 7:
                ESP_LOGI(TAG, "Đã thoát chế độ admin.");
                return;

            default:
                ESP_LOGE(TAG, "Lựa chọn không hợp lệ!");
                break;
        }

        ESP_LOGI(TAG, "Quay lại Menu Admin...");
        vTaskDelay(pdMS_TO_TICKS(800));
    }
}

// CẤU HÌNH VÀ KHỞI TẠO UART (SERIAL)
#define SERIAL_RX_BUF_SIZE 256

void init_uart()
{
    // Kiểm tra xem UART đã được khởi tạo chưa
    if (uart_is_driver_installed(UART_NUM_0)) {
        ESP_LOGI(TAG, "UART driver already installed, skipping initialization");
        return;
    }
    
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, SERIAL_RX_BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    ESP_LOGI(TAG, "UART initialized successfully");
}


void app_main()
{
    /* Initialize Application specific hardware drivers and
     * set initial state.
     */
    esp_rmaker_console_init();
    app_driver_init();
    app_driver_set_state(DEFAULT_POWER);

    /* Initialize NVS. */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    /* Initialize Wi-Fi. Note that, this should be called before esp_rmaker_node_init()
     */
    app_network_init();

    /* Register an event handler to catch RainMaker events */
    ESP_ERROR_CHECK(esp_event_handler_register(RMAKER_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(RMAKER_COMMON_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(APP_NETWORK_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(RMAKER_OTA_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));

    /* Initialize the ESP RainMaker Agent.
     * Note that this should be called after app_network_init() but before app_nenetworkk_start()
     * */
    esp_rmaker_config_t rainmaker_cfg = {
        .enable_time_sync = false,
    };
    esp_rmaker_node_t *node = esp_rmaker_node_init(&rainmaker_cfg, "ESP RainMaker Device", "Switch");
    if (!node) {
        ESP_LOGE(TAG, "Could not initialise node. Aborting!!!");
        vTaskDelay(5000/portTICK_PERIOD_MS);
        abort();
    }

    /* Create a Switch device.
     * You can optionally use the helper API esp_rmaker_switch_device_create() to
     * avoid writing code for adding the name and power parameters.
     */
    switch_device = esp_rmaker_device_create("Switch", ESP_RMAKER_DEVICE_SWITCH, NULL);

    /* Add the write callback for the device. We aren't registering any read callback yet as
     * it is for future use.
     */
    esp_rmaker_device_add_cb(switch_device, write_cb, NULL);

    /* Add the standard name parameter (type: esp.param.name), which allows setting a persistent,
     * user friendly custom name from the phone apps. All devices are recommended to have this
     * parameter.
     */
    esp_rmaker_device_add_param(switch_device, esp_rmaker_name_param_create(ESP_RMAKER_DEF_NAME_PARAM, "Switch"));

    /* Add the standard power parameter (type: esp.param.power), which adds a boolean param
     * with a toggle switch ui-type.
     */
    esp_rmaker_param_t *power_param = esp_rmaker_power_param_create(ESP_RMAKER_DEF_POWER_NAME, DEFAULT_POWER);
    esp_rmaker_device_add_param(switch_device, power_param);

    /* Assign the power parameter as the primary, so that it can be controlled from the
     * home screen of the phone apps.
     */
    esp_rmaker_device_assign_primary_param(switch_device, power_param);

    /* Add this switch device to the node */
    esp_rmaker_node_add_device(node, switch_device);
    
    /* ========== TẠO DEVICE THỐNG KÊ ========== */
    stats_device_global = esp_rmaker_device_create("Thống Kê", NULL, NULL);
    esp_rmaker_device_add_param(stats_device_global, 
        esp_rmaker_param_create("Tổng Số Lần Mở", NULL, esp_rmaker_int(0), PROP_FLAG_READ));
    esp_rmaker_device_add_param(stats_device_global,
        esp_rmaker_param_create("RFID Thành Công", NULL, esp_rmaker_int(0), PROP_FLAG_READ));
    esp_rmaker_device_add_param(stats_device_global,
        esp_rmaker_param_create("RFID Thất Bại", NULL, esp_rmaker_int(0), PROP_FLAG_READ));
    esp_rmaker_device_add_param(stats_device_global,
        esp_rmaker_param_create("Cảnh Báo Xâm Nhập", NULL, esp_rmaker_int(0), PROP_FLAG_READ));
    esp_rmaker_device_add_param(stats_device_global,
        esp_rmaker_param_create("Mở Từ App", NULL, esp_rmaker_int(0), PROP_FLAG_READ));
    esp_rmaker_device_add_param(stats_device_global,
        esp_rmaker_param_create("Người Dùng Nhiều Nhất", NULL, esp_rmaker_str("Chưa có"), PROP_FLAG_READ));
    esp_rmaker_node_add_device(node, stats_device_global);
    ESP_LOGI(TAG, "✓ Đã tạo device thống kê");
    /* ========================================== */

    /* Enable OTA */
    esp_rmaker_ota_enable_default();

    /* Enable timezone service which will be require for setting appropriate timezone
     * from the phone apps for scheduling to work correctly.
     * For more information on the various ways of setting timezone, please check
     * https://rainmaker.espressif.com/docs/time-service.html.
     */
    esp_rmaker_timezone_service_enable();

    /* Enable scheduling. */
    esp_rmaker_schedule_enable();

    /* Enable Scenes */
    esp_rmaker_scenes_enable();

    /* Enable Insights. Requires CONFIG_ESP_INSIGHTS_ENABLED=y */
    app_insights_enable();

    /* Start the ESP RainMaker Agent */
    esp_rmaker_start();
    
    // Khởi động task WiFi trong background
    xTaskCreate(wifi_connect_task, "wifi_connect", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "WiFi task started in background");

    // ========== KHỞI ĐỘNG RFID TRƯỚC - KHÔNG ĐỢI WIFI ==========
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "  INITIALIZING RFID SYSTEM...");
    ESP_LOGI(TAG, "  (WiFi will connect in background)");
    ESP_LOGI(TAG, "===========================================");
    
    esp_err_t ret;
    vTaskDelay(pdMS_TO_TICKS(500));
    
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "  SYSTEM STARTING...");
    ESP_LOGI(TAG, "===========================================");
    
    // Khởi tạo NVS
    ESP_LOGI(TAG, "Initializing NVS...");
    ret = card_storage_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init NVS!");
        return;
    }
    
    // Load danh sách thẻ từ Flash
    ret = load_cards_from_flash(allowed_cards, &num_allowed_cards);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "No saved data, using default cards");
        
        AllowedCard default_cards[] = {
            {{0x41, 0x89, 0x3b, 0x16}, "Quy Nguyen", "0403"},
            {{0x23, 0xe5, 0x64, 0xa9}, "Tam Ho", "0407"},
            {{0x43, 0xe0, 0x9c, 0xa5}, "Nhat Trinh", "0608"},
            {{0xb7, 0x22, 0xc2, 0x01}, "Nhan Cot", "0309"},
        };
        
        num_allowed_cards = 4;
        memcpy(allowed_cards, default_cards, sizeof(default_cards));
        
        save_cards_to_flash(allowed_cards, num_allowed_cards);
    }
    
    ESP_LOGI(TAG, "Loaded %d cards from storage", num_allowed_cards);
    
    display_card_list();
    ESP_LOGI(TAG, "Chọn 1 trong 4 thẻ trên làm ADMIN");
    ESP_LOGI(TAG, "Sau đó cập nhật ADMIN_CARD_UID trong code");
    ESP_LOGI(TAG, "");
    
    // Relay đã được khởi tạo bởi app_driver_init() ở trên
    ESP_LOGI(TAG, "✓ Relay GPIO %d ready (controlled by RainMaker & RFID)", RELAY_PIN);
    
    // ========== KHỞI TẠO BUZZER ==========
    init_buzzer();
    ESP_LOGI(TAG, "");
    // ====================================
    
    // ========== KHỞI TẠO NÚT THOÁT KHẨN CẤP ==========
    init_exit_button();
    ESP_LOGI(TAG, "");
    // =================================================
    
    // ========== KHỞI TẠO I2C VÀ KEYPAD TRƯỚC ==========
    ESP_LOGI(TAG, "System start - RC522 + Keypad (PCF8574 addr 0x20)");
    keypad_start_task(); 
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "");
    // ==================================================
    
    // ========== KHỞI TẠO OLED ==========
    ESP_LOGI(TAG, "Initializing OLED...");
    oled_config_t oled_cfg = {
        .i2c_port = OLED_I2C_PORT,
        .i2c_addr = OLED_I2C_ADDRESS,
        .sda_pin = OLED_SDA_PIN,
        .scl_pin = OLED_SCL_PIN,
        .clk_speed = OLED_CLK_SPEED,
    };
    
    ret = oled_init(&oled_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init OLED: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "✓ OLED initialized");
        oled_show_message("System", "Initializing...", NULL, NULL);
    }
    ESP_LOGI(TAG, "");
    // ===================================
    
    // Cấu hình chân RST
    gpio_config_t rst_conf = {
        .pin_bit_mask = (1ULL << PIN_NUM_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&rst_conf);
    
    gpio_set_level(PIN_NUM_RST, 0);
    
    gpio_config_t cs_conf = {
        .pin_bit_mask = (1ULL << PIN_NUM_CS),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cs_conf);
    gpio_set_level(PIN_NUM_CS, 1);
    
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Cấu hình SPI bus
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 5 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7,
        .flags = 0,
    };

    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "SPI initialized successfully!");
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Hiển thị đang chờ hệ thống
    oled_show_message("Dang doi", "he thong...", NULL, NULL);
    
    ESP_LOGI(TAG, "Initializing RC522...");
    PCD_Init(handle);
    vTaskDelay(pdMS_TO_TICKS(200));

    init_uart();

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "  HE THONG SAN SANG!");
    ESP_LOGI(TAG, "  - Quet the thuong: Xac thuc + nhap mat khau");
    ESP_LOGI(TAG, "  - Quet the ADMIN: Menu quan ly");
    ESP_LOGI(TAG, "  - Relay GPIO %d se dieu khien khoa cua", RELAY_PIN);
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "");
    
    // ========== BẮT ĐẦU KẾT NỐI WIFI TRONG TASK RIÊNG (KHÔNG CHẶN) ==========
    ESP_LOGI(TAG, "WiFi will connect in background (non-blocking)");
    ESP_LOGI(TAG, "RFID system is ready to use NOW");
    ESP_LOGI(TAG, "");
    // Không gọi app_network_start() ở đây vì nó block
    // WiFi sẽ tự động kết nối trong task của esp_rmaker
    // =========================================================================
    
    // Hiển thị sẵn sàng sau khi antenna đã bật
    oled_show_ready();
    
    // Đếm để debug
    uint32_t scan_count = 0;
    
    // Vòng lặp chính
    while (1)
    {
        // Kiểm tra xem có cần reset RC522 không (sau khi điều khiển relay từ app)
        if (need_rc522_reset) {
            need_rc522_reset = false;
            ESP_LOGI(TAG, "==> Resetting RC522 after relay operation from app...");
            vTaskDelay(pdMS_TO_TICKS(100)); // Đợi relay ổn định
            reset_rc522_reader();
            vTaskDelay(pdMS_TO_TICKS(200));
            ESP_LOGI(TAG, "==> RC522 reset completed");
        }
        
        char card_name[32] = {0};
        char card_password[16] = {0};
        
        // Debug mỗi 10 giây
        if (scan_count % 100 == 0) {
            ESP_LOGI(TAG, "Scanning for cards... (loop %lu)", scan_count);
        }
        scan_count++;

        if (PICC_IsNewCardPresent(handle))
        {
            ESP_LOGI(TAG, "Card detected!");
            
            if (PICC_Select(handle, &uid, 0) == STATUS_OK)
            {
                ESP_LOGI(TAG, "");
                printf("📇 UID thẻ vừa quét: ");
                for (int i = 0; i < uid.size; i++) {
                    printf("%02X ", uid.uidByte[i]);
                }
                printf("\n");
                
                char uid_str[32];
                snprintf(uid_str, sizeof(uid_str), "%02X %02X %02X %02X", 
                         uid.uidByte[0], uid.uidByte[1], uid.uidByte[2], uid.uidByte[3]);
                oled_show_message("Card Detected", uid_str, NULL, NULL);
                vTaskDelay(pdMS_TO_TICKS(1000));
                
                // Kiểm tra thẻ ADMIN
                if (is_admin_card(uid.uidByte, uid.size))
                {
                    ESP_LOGI(TAG, "The ADMIN duoc xac nhan!");
                    
                    // Hiển thị ADMIN trên OLED
                    oled_show_message("ADMIN CARD", "Detected!", "Opening Menu...", NULL);
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    
                    oled_show_admin_menu();
                    
                    // Reset RC522 trước khi vào menu admin
                    reset_rc522_reader();
                    vTaskDelay(pdMS_TO_TICKS(500));
                    
                    admin_menu();
                    
                    ESP_LOGI(TAG, "Thoat che do admin.");
                    ESP_LOGI(TAG, "===========================================");
                    
                    // Hiển thị đang đợi hệ thống
                    oled_show_message("Dang doi", "he thong...", NULL, NULL);
                    
                    // Reset RC522 sau khi thoát menu admin
                    reset_rc522_reader();
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    
                    // Hiển thị sẵn sàng quét
                    oled_show_ready();
                }
                // Kiểm tra thẻ thường
                else if (is_card_allowed(uid.uidByte, uid.size, card_name, card_password))
                {
                    ESP_LOGI(TAG, ">>> TRUY CẬP ĐƯỢC PHÉP <<<");
                    ESP_LOGI(TAG, "Xin chào %s!", card_name);
                    ESP_LOGI(TAG, "Nhập mật khẩu trên keypad (# để ENTER):");
                    
                    oled_show_card_detected(card_name);
                    vTaskDelay(pdMS_TO_TICKS(1500));
                    oled_show_password_prompt();

                    char input_pass[16] = {0};
                    int len = keypad_read_password(input_pass, sizeof(input_pass), 10000);

                    if (len > 0)
                    {
                        ESP_LOGI(TAG, "Mật khẩu nhập: %s", input_pass);

                        if (strcmp(input_pass, card_password) == 0)
                        {
                            ESP_LOGI(TAG, "✓✓✓ Mật khẩu đúng. Mở CỬA ✓✓✓");
                            
                            // ========== Cập nhật thống kê ==========
                            stats.total_door_opens++;
                            stats.rfid_success++;
                            strncpy(stats.most_frequent_user, card_name, sizeof(stats.most_frequent_user)-1);
                            update_stats_to_rainmaker();
                            // =======================================
                            
                            // ========== Mở CỬA BẰNG RELAY ==========
                            open_door();
                            // ========================================
                        }
                        else
                        {
                            ESP_LOGE(TAG, "XXX Mật khẩu sai! XXX");
                            oled_show_access_denied();
                            
                            // Cập nhật thống kê
                            stats.rfid_failed++;
                            stats.intruder_alerts++;
                            update_stats_to_rainmaker();
                            
                            // Cảnh báo mật khẩu sai
                            char alert[100];
                            snprintf(alert, sizeof(alert), "Mat khau sai! The: %s", card_name);
                            alert_intruder(alert);
                            
                            vTaskDelay(pdMS_TO_TICKS(2000));
                        }
                    }
                    else
                    {
                        ESP_LOGW(TAG, "Hết thời gian nhập mật khẩu.");
                        oled_show_message("Het thoi gian", "Quet lai the", NULL, NULL);
                        vTaskDelay(pdMS_TO_TICKS(1500));
                    }
                    
                    // ========== RESET RC522 SAU KHI XỬ LÝ THẺ ==========
                    ESP_LOGI(TAG, "");
                    ESP_LOGI(TAG, "Vui long di chuyen the ra khoi dau doc...");
                    
                    // Hiển thị đang đợi hệ thống
                    oled_show_message("Dang doi", "he thong...", NULL, NULL);
                    
                    // Reset RC522 để sẵn sàng đọc thẻ tiếp theo
                    reset_rc522_reader();
                    
                    // Chờ người dùng di chuyển thẻ ra
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    
                    ESP_LOGI(TAG, "He thong san sang cho the tiep theo.");
                    ESP_LOGI(TAG, "-------------------------------------");
                    
                    // Hiển thị sẵn sàng quét
                    oled_show_ready();
                    // ==================================================
                }
                else
                {
                    ESP_LOGE(TAG, "XXX TRUY CẬP BỊ TỪ CHỐI XXX");
                    ESP_LOGE(TAG, "Thẻ không nằm trong danh sách cho phép.");
                    
                    oled_show_message("UNAUTHORIZED", "Card Not", "Registered", NULL);
                    
                    // Cập nhật thống kê
                    stats.rfid_failed++;
                    stats.intruder_alerts++;
                    update_stats_to_rainmaker();
                    
                    // Cảnh báo thẻ không hợp lệ
                    char uid_str[50];
                    snprintf(uid_str, sizeof(uid_str), "The khong hop le! UID: %02X%02X%02X%02X", 
                             uid.uidByte[0], uid.uidByte[1], uid.uidByte[2], uid.uidByte[3]);
                    alert_intruder(uid_str);
                    
                    vTaskDelay(pdMS_TO_TICKS(1500));
                    
                    // Hiển thị đang đợi hệ thống
                    oled_show_message("Dang doi", "he thong...", NULL, NULL);
                    
                    // Reset RC522 ngay cả khi thẻ không hợp lệ
                    reset_rc522_reader();
                    vTaskDelay(pdMS_TO_TICKS(500));
                    
                    // Hiển thị sẵn sàng quét
                    oled_show_ready();
                    
                    ESP_LOGI(TAG, "-------------------------------------");
                }
            }
            else
            {
                // Nếu không select được thẻ, reset RC522
                ESP_LOGW(TAG, "Không thể đọc thẻ. Đang reset...");
                reset_rc522_reader();
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
