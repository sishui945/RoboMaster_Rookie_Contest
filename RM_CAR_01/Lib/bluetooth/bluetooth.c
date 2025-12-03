//
// Created by bilib on 2025/11/27.
//
// 蓝牙通信配置
#include "bluetooth.h"
// 发送数据到蓝牙模块（改进版）
bool send_data_to_bluetooth(const char* data) {
    if (data == NULL || strlen(data) == 0) {
        return false;
    }
    uint16_t data_len = strlen(data);
    HAL_StatusTypeDef status = HAL_UART_Transmit_IT(&huart1, (uint8_t*)data, data_len);
    return (status == HAL_OK);
}

// 发送格式化数据到蓝牙模块
bool send_formatted_data(const char* format, ...) {
    char buffer[BLUETOOTH_BUFFER_SIZE];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    if (len < 0 || len >= sizeof(buffer)) {
        return false;
    }
    return send_data_to_bluetooth(buffer);
}

// 接收蓝牙模块的数据（非阻塞方式）
uint16_t receive_data_from_bluetooth(uint8_t* buffer, uint16_t buffer_size) {
    if (buffer == NULL || buffer_size == 0) {
        return 0;
    }
    HAL_StatusTypeDef status = HAL_UART_Receive_IT(&huart1, buffer, buffer_size);
    if (status == HAL_OK) {
        // 返回实际接收到的数据长度
        return buffer_size;
    } else if (status == HAL_TIMEOUT) {
        // 返回实际接收到的数据长度（可能小于buffer_size）
        return BLUETOOTH_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
    }

    return 0;
}

// 带数据处理的蓝牙接收函数
typedef void (*bluetooth_data_handler)(const uint8_t* data, uint16_t length);

void process_bluetooth_data(bluetooth_data_handler handler, uint16_t timeout_ms) {
    static uint8_t rx_buffer[BLUETOOTH_BUFFER_SIZE];
    static uint8_t rx_data[BLUETOOTH_BUFFER_SIZE];

    uint16_t received_len = receive_data_from_bluetooth(rx_buffer, sizeof(rx_buffer));

    if (received_len > 0 && handler != NULL) {
        // 复制数据到安全缓冲区进行处理
        memcpy(rx_data, rx_buffer, received_len);
        handler(rx_data, received_len);
    }
}

// 示例：蓝牙数据处理器
void example_bluetooth_handler(const uint8_t* data, uint16_t length) {
    // 确保数据以null结尾
    char temp_buffer[BLUETOOTH_BUFFER_SIZE + 1];
    memcpy(temp_buffer, data, length);
    temp_buffer[length] = '\0';

    // 处理接收到的数据

    if (strstr((char*)temp_buffer, "LED_ON")) {
        send_data_to_bluetooth("LED turned ON\r\n");
    }
    else if (strstr((char*)temp_buffer, "LED_OFF")) {
        send_data_to_bluetooth("LED turned OFF\r\n");
    }
    else {
        // 回显接收到的数据
        send_formatted_data("Echo: %s\r\n", temp_buffer);

    }

}

// 使用示例：
void example_usage(void) {
    // 发送数据
    send_data_to_bluetooth("Hello Bluetooth!\r\n");

    // 发送格式化数据
    send_formatted_data("Temperature: %.2f, Humidity: %.2f\r\n", 25.5f, 60.0f);

    // 处理接收数据（非阻塞）
    process_bluetooth_data(example_bluetooth_handler, 100);
}

// 中断接收方式（如果需要更高效率）
#ifdef USE_BLUETOOTH_INTERRUPT
static uint8_t bt_rx_buffer[BLUETOOTH_BUFFER_SIZE];
static volatile uint16_t bt_rx_index = 0;
static volatile bool bt_data_ready = false;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == huart1.Instance) {
        bt_rx_index++;
        if (bt_rx_index >= BLUETOOTH_BUFFER_SIZE || bt_rx_buffer[bt_rx_index-1] == '\n') {
            bt_data_ready = true;
        } else {
            // 继续接收下一个字节
            HAL_UART_Receive_IT(&huart1, &bt_rx_buffer[bt_rx_index], 1);
        }
    }
}

void start_bluetooth_receive_it(void) {
    bt_rx_index = 0;
    bt_data_ready = false;
    HAL_UART_Receive_IT(&huart1, bt_rx_buffer, 1);
}

bool is_bluetooth_data_ready(void) {
    return bt_data_ready;
}

uint16_t get_bluetooth_data(uint8_t* buffer, uint16_t buffer_size) {
    if (!bt_data_ready) return 0;

    uint16_t copy_size = (bt_rx_index < buffer_size) ? bt_rx_index : buffer_size;
    memcpy(buffer, bt_rx_buffer, copy_size);
    bt_data_ready = false;

    return copy_size;
}
#endif