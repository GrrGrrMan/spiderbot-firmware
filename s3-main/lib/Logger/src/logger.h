#pragma once
#include <Arduino.h>

extern bool g_logEnabled;
void logPrintf(const char* tag, const char* fmt, ...);

#define LOG_NET(fmt, ...) logPrintf("Net", fmt, ##__VA_ARGS__)
#define LOG_MOT(fmt, ...) logPrintf("Motion", fmt, ##__VA_ARGS__)
#define LOG_SYS(fmt, ...) logPrintf("System", fmt, ##__VA_ARGS__)
#define LOG_ERR(fmt, ...) logPrintf("ERROR", fmt, ##__VA_ARGS__)