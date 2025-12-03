#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H
#include "main.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "usart.h"
#include <stdarg.h>
#define BLUETOOTH_BUFFER_SIZE 256
#define BLUETOOTH_TIMEOUT 1000  // 1秒超时
bool send_data_to_bluetooth(const char* data);
bool send_formatted_data(const char* format, ...);
uint16_t receive_data_from_bluetooth(uint8_t* buffer, uint16_t buffer_size);
#endif