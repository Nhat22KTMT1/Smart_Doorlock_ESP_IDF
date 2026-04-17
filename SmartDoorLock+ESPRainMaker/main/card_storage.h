#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UID_SIZE 4
#define MAX_ALLOWED_CARDS 10  // Giới hạn 10 thẻ

typedef struct {
    uint8_t uid[UID_SIZE];
    char name[32];
    char password[16];
} AllowedCard;

// Khởi tạo NVS
esp_err_t card_storage_init(void);

// Lưu danh sách thẻ vào Flash
esp_err_t save_cards_to_flash(AllowedCard *cards, int count);

// Đọc danh sách thẻ từ Flash
esp_err_t load_cards_from_flash(AllowedCard *cards, int *count);

// Đổi mật khẩu của 1 thẻ
esp_err_t change_card_password(AllowedCard *cards, int count, uint8_t *target_uid, const char *new_password);

// Lưu log thay đổi mật khẩu
esp_err_t save_password_change_log(const char *card_name, const char *old_pass, const char *new_pass);

// Xem lịch sử thay đổi
esp_err_t view_password_change_history(void);

// Thoát chế độ admin
esp_err_t admin_exit(void);

// Xóa thẻ (admin mode) - THÊM MỚI
esp_err_t admin_delete_card(AllowedCard *cards, int *count, int card_index);

#ifdef __cplusplus
}
#endif