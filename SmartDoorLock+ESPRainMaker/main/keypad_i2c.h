#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Khởi tạo I2C và bắt đầu task keypad
void keypad_start_task(void);

// Lấy 1 key từ keypad, timeout_ms = 0 tức non-blocking
char keypad_get_key_timeout(uint32_t timeout_ms);

// Đọc password từ keypad (dùng '*' để xóa, '#' để ENTER)
int keypad_read_password(char* out_pass, int max_len, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
