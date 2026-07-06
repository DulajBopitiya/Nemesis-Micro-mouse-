/**
 ******************************************************************************
 * @file    dbg.h
 * @brief   Tiny debug-print wrapper over SEGGER RTT.
 *
 * RTT streams text over the existing SWD/J-Link connection (no UART, no extra
 * pins). View it live with SEGGER's "J-Link RTT Viewer" while the target runs.
 *
 * Usage:
 *     #include "dbg.h"
 *     DBG_Init();                       // once at startup
 *     LOG("clock = %lu Hz\r\n", SystemCoreClock);
 *     LOG("color %d\r\n", i);
 *
 * Define DBG_DISABLE (e.g. -DDBG_DISABLE in build_flags) to compile all the
 * logging out for a release build.
 ******************************************************************************
 */
#ifndef DBG_H
#define DBG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "SEGGER_RTT.h"

#ifndef DBG_DISABLE
  #define DBG_Init()        SEGGER_RTT_Init()
  #define DBG_Printf(...)   SEGGER_RTT_printf(0, __VA_ARGS__)
  #define LOG(...)          SEGGER_RTT_printf(0, __VA_ARGS__)
#else
  #define DBG_Init()        ((void)0)
  #define DBG_Printf(...)   ((void)0)
  #define LOG(...)          ((void)0)
#endif

#ifdef __cplusplus
}
#endif

#endif /* DBG_H */
