#include <string.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "card_storage.h"
#include "oled_i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// External function declarations
extern esp_err_t oled_show_message(const char *line1, const char *line2, const char *line3, const char *line4);
extern int keypad_read_password(char *buffer, int max_len, int timeout_ms);

static const char *TAG = "CARD_STORAGE";
static const char *NVS_NAMESPACE = "card_data";
bool admin_mode = false;

// Khởi tạo NVS
esp_err_t card_storage_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition was truncated, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "NVS initialized successfully");
    }
    return ret;
}

// Lưu danh sách thẻ vào Flash
esp_err_t save_cards_to_flash(AllowedCard *cards, int count)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret;
    
    ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Lưu số lượng thẻ
    ret = nvs_set_i32(nvs_handle, "card_count", (int32_t)count);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save card count");
        nvs_close(nvs_handle);
        return ret;
    }
    
    // Lưu từng thẻ
    for (int i = 0; i < count; i++) {
        char key[16];
        
        // Lưu UID
        snprintf(key, sizeof(key), "uid_%d", i);
        ret = nvs_set_blob(nvs_handle, key, cards[i].uid, UID_SIZE);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save UID %d", i);
            nvs_close(nvs_handle);
            return ret;
        }
        
        // Lưu tên
        snprintf(key, sizeof(key), "name_%d", i);
        ret = nvs_set_str(nvs_handle, key, cards[i].name);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save name %d", i);
            nvs_close(nvs_handle);
            return ret;
        }
        
        // Lưu mật khẩu
        snprintf(key, sizeof(key), "pass_%d", i);
        ret = nvs_set_str(nvs_handle, key, cards[i].password);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save password %d", i);
            nvs_close(nvs_handle);
            return ret;
        }
    }
    
    // Commit changes
    ret = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Saved %d cards to Flash successfully", count);
    }
    
    return ret;
}

// Đọc danh sách thẻ từ Flash
esp_err_t load_cards_from_flash(AllowedCard *cards, int *count)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret;
    
    ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "No saved data found, using defaults");
        return ret;
    }
    
    // Đọc số lượng thẻ
    int32_t saved_count = 0;
    ret = nvs_get_i32(nvs_handle, "card_count", &saved_count);
    if (ret != ESP_OK || saved_count <= 0 || saved_count > MAX_ALLOWED_CARDS) {
        ESP_LOGW(TAG, "Invalid card count");
        nvs_close(nvs_handle);
        return ESP_ERR_NOT_FOUND;
    }
    
    *count = (int)saved_count;
    
    // Đọc từng thẻ
    for (int i = 0; i < (int)saved_count; i++) {
        char key[16];
        size_t size;
        
        // Đọc UID
        snprintf(key, sizeof(key), "uid_%d", i);
        size = UID_SIZE;
        ret = nvs_get_blob(nvs_handle, key, cards[i].uid, &size);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to load UID %d", i);
            nvs_close(nvs_handle);
            return ret;
        }
        
        // Đọc tên
        snprintf(key, sizeof(key), "name_%d", i);
        size = sizeof(cards[i].name);
        ret = nvs_get_str(nvs_handle, key, cards[i].name, &size);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to load name %d", i);
            nvs_close(nvs_handle);
            return ret;
        }
        
        // Đọc mật khẩu
        snprintf(key, sizeof(key), "pass_%d", i);
        size = sizeof(cards[i].password);
        ret = nvs_get_str(nvs_handle, key, cards[i].password, &size);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to load password %d", i);
            nvs_close(nvs_handle);
            return ret;
        }
    }
    
    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "Loaded %d cards from Flash successfully", (int)saved_count);
    
    return ESP_OK;
}

// Đổi mật khẩu của 1 thẻ
esp_err_t change_card_password(AllowedCard *cards, int count, uint8_t *target_uid, const char *new_password)
{
    for (int i = 0; i < count; i++) {
        bool match = true;
        for (int j = 0; j < UID_SIZE; j++) {
            if (cards[i].uid[j] != target_uid[j]) {
                match = false;
                break;
            }
        }
        
        if (match) {
            strncpy(cards[i].password, new_password, sizeof(cards[i].password) - 1);
            cards[i].password[sizeof(cards[i].password) - 1] = '\0';
            
            ESP_LOGI(TAG, "Changed password for card: %s", cards[i].name);
            return save_cards_to_flash(cards, count);
        }
    }
    
    ESP_LOGE(TAG, "Card UID not found");
    return ESP_ERR_NOT_FOUND;
}

// Lưu log thay đổi mật khẩu
esp_err_t save_password_change_log(const char *card_name, const char *old_pass, const char *new_pass)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret;
    
    ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Đọc số lượng log hiện có
    int32_t log_count = 0;
    nvs_get_i32(nvs_handle, "log_count", &log_count);
    
    // Giới hạn 10 log gần nhất
    if (log_count >= 10) {
        log_count = 0; // Reset về 0
    }
    
    // Lưu log mới
    char key[32];
    char log_entry[128];
    
    snprintf(log_entry, sizeof(log_entry), "%s: %s -> %s", card_name, old_pass, new_pass);
    snprintf(key, sizeof(key), "log_%d", (int)log_count);
    
    ret = nvs_set_str(nvs_handle, key, log_entry);
    if (ret == ESP_OK) {
        log_count++;
        nvs_set_i32(nvs_handle, "log_count", log_count);
        nvs_commit(nvs_handle);
    }
    
    nvs_close(nvs_handle);
    return ret;
}

// Xem lịch sử thay đổi
esp_err_t view_password_change_history(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret;
    
    ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        printf("Khong co lich su thay doi.\n");
        oled_show_message("History", "No records", "found", NULL);
        vTaskDelay(pdMS_TO_TICKS(2000));
        return ret;
    }
    
    int32_t log_count = 0;
    ret = nvs_get_i32(nvs_handle, "log_count", &log_count);
    if (ret != ESP_OK || log_count == 0) {
        printf("Khong co lich su thay doi.\n");
        oled_show_message("History", "No records", "found", NULL);
        vTaskDelay(pdMS_TO_TICKS(2000));
        nvs_close(nvs_handle);
        return ESP_ERR_NOT_FOUND;
    }
    
    printf("\n=== LICH SU THAY DOI MAT KHAU ===\n");
    
    // Hiển thị từng bản ghi trên OLED
    for (int i = 0; i < (int)log_count; i++) {
        char key[32];
        char log_entry[128];
        size_t size = sizeof(log_entry);
        
        snprintf(key, sizeof(key), "log_%d", i);
        ret = nvs_get_str(nvs_handle, key, log_entry, &size);
        
        if (ret == ESP_OK) {
            printf("%d. %s\n", i + 1, log_entry);
            
            // Phân tích log_entry để hiển thị trên OLED
            // Format: "Card: [name] | Old: [old_pass] -> New: [new_pass]"
            char display_line1[32], display_line2[32], display_line3[32];
            
            // Tìm vị trí của "Card: ", "Old: ", "New: "
            char *card_ptr = strstr(log_entry, "Card: ");
            char *old_ptr = strstr(log_entry, "Old: ");
            char *new_ptr = strstr(log_entry, "New: ");
            
            if (card_ptr && old_ptr && new_ptr) {
                // Trích xuất tên thẻ
                card_ptr += 6; // Skip "Card: "
                char *pipe = strstr(card_ptr, " |");
                int name_len = pipe ? (pipe - card_ptr) : 12;
                if (name_len > 12) name_len = 12;
                
                char card_name[16] = {0};
                strncpy(card_name, card_ptr, name_len);
                card_name[name_len] = '\0';
                
                // Trích xuất mật khẩu cũ
                old_ptr += 5; // Skip "Old: "
                char *arrow = strstr(old_ptr, " ->");
                int old_len = arrow ? (arrow - old_ptr) : 8;
                if (old_len > 8) old_len = 8;
                
                char old_pass[10] = {0};
                strncpy(old_pass, old_ptr, old_len);
                old_pass[old_len] = '\0';
                
                // Trích xuất mật khẩu mới
                new_ptr += 5; // Skip "New: "
                char new_pass[10] = {0};
                int new_len = strlen(new_ptr);
                if (new_len > 8) new_len = 8;
                strncpy(new_pass, new_ptr, new_len);
                new_pass[new_len] = '\0';
                
                // Hiển thị đơn giản trên OLED
                snprintf(display_line1, sizeof(display_line1), "History [%d/%d]", i + 1, (int)log_count);
                snprintf(display_line2, sizeof(display_line2), "%.18s", card_name);
                snprintf(display_line3, sizeof(display_line3), "%s -> %s", old_pass, new_pass);
                
                oled_show_message(display_line1, display_line2, display_line3, "See Monitor");
            } else {
                // Nếu không parse được, hiển thị đơn giản
                snprintf(display_line1, sizeof(display_line1), "Rec [%d/%d]", i + 1, (int)log_count);
                oled_show_message(display_line1, "See Monitor", NULL, NULL);
            }
            
            // Tự động chuyển sau 2 giây (không cần nhấn phím)
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
    
    printf("==================================\n\n");
    
    // Hiển thị kết thúc
    char total_line[32];
    snprintf(total_line, sizeof(total_line), "Total: %d", (int)log_count);
    oled_show_message("History", "Complete", total_line, NULL);
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    nvs_close(nvs_handle);
    return ESP_OK;
}

esp_err_t admin_exit(void)
{
    admin_mode = false;
    ESP_LOGI("ADMIN", "Admin mode exited");
    return ESP_OK;
}


esp_err_t admin_delete_card(AllowedCard *cards, int *count, int card_index)
{
    if (card_index < 0 || card_index >= *count) {
        ESP_LOGE("ADMIN", "Chỉ số thẻ không hợp lệ!");
        return ESP_ERR_INVALID_ARG;
    }

    // Lưu tên thẻ bị xóa để log
    char deleted_name[32];
    strncpy(deleted_name, cards[card_index].name, sizeof(deleted_name) - 1);
    deleted_name[sizeof(deleted_name) - 1] = '\0';
    
    // Dịch chuyển các thẻ phía sau lên trước
    for (int i = card_index; i < *count - 1; i++) {
        cards[i] = cards[i + 1];
    }
    
    // Giảm số lượng thẻ
    (*count)--;
    
    ESP_LOGI("ADMIN", "Đã xóa thẻ: %s", deleted_name);
    
    // Lưu vào Flash
    return save_cards_to_flash(cards, *count);
}

