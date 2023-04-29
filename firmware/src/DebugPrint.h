#ifndef DEBUG_PRINT_H
#define DEBUG_PRINT_H

#ifdef DEBUG_PORT
  #define _CONCAT(x,y) x##y
  #define CONCAT(x,y) _CONCAT(x,y)
  #define DEBUG_FUNC CONCAT(Serial,DEBUG_PORT)
  #ifdef DEBUG_PRINT_H
    #include <Arduino.h>
    const SemaphoreHandle_t debugPrintMutex = xSemaphoreCreateMutex();
    #define DEBUG_PRINT(x) xSemaphoreTake(debugPrintMutex,1000); DEBUG_FUNC.println("DEBUG: " + (String)x); xSemaphoreGive(debugPrintMutex)
  #else
    #define DEBUG_PRINT(x) DEBUG_FUNC.println("DEBUG: " + (String)x)
  #endif
  #else
    #define DEBUG_PRINT(x) x
  #endif
#endif