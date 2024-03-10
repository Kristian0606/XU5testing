//   _____ _____ _____
//  |   | |__   |   | | cpu0/src/utility/log.h
//  | | | |   __| | | | Nikolai Nymo
//  |_|___|_____|_|___| 09-03-19
#ifndef SRC_UTILITY_LOG_H_
#define SRC_UTILITY_LOG_H_

// #define LOG_INFO
// #define LOG_ERROR
// #define LOG_ISR
// #define LOG_INIT
// #define LOG_HSM
// #define LOG_CAN
// #define LOG_TELEMETRY
// #define LOG_TELEMETRY_ERROR

#ifdef LOG_INFO
#include "xil_printf.h"
#define log_info(...) xil_printf(__VA_ARGS__)
#else
#define log_info(...)
#endif

#ifdef LOG_ERROR
#include "xil_printf.h"
#define log_error(...) xil_printf(__VA_ARGS__)
#else
#define log_error(...)
#endif

#ifdef LOG_HSM
#include "xil_printf.h"
#define log_hsm(...) xil_printf(__VA_ARGS__)
#else
#define log_hsm(...)
#endif

#ifdef LOG_ISR
#include "xil_printf.h"
#define log_isr(...) xil_printf(__VA_ARGS__)
#else
#define log_isr(...)
#endif

#ifdef LOG_INIT
#include "xil_printf.h"
#define log_init(...) xil_printf(__VA_ARGS__)
#else
#define log_init(...)
#endif

#ifdef LOG_CAN
#include "xil_printf.h"
#define log_can(...) xil_printf(__VA_ARGS__)
#else
#define log_can(...)
#endif

#ifdef LOG_TELEMETRY
#include "xil_printf.h"
#define log_telemetry(...) xil_printf(__VA_ARGS__)
#else
#define log_telemetry(...)
#endif

#if defined(LOG_TELEMETRY_ERROR) || defined(LOG_ERROR)
#include "xil_printf.h"
#define log_telemetry_error(...) xil_printf(__VA_ARGS__)
#else
#define log_telemetry_error(...)
#endif

#endif /* SRC_UTILITY_LOG_H_ */
