#ifndef STM32_FLASH_DEBUG_H
#define STM32_FLASH_DEBUG_H

#define STM32_FLASH_DEBUG           1

#if STM32_FLASH_DEBUG
    extern void AddLog(uint32_t loglevel, PGM_P formatP, ...);
    enum LoggingLevels {LOG_LEVEL_NONE, LOG_LEVEL_ERROR, LOG_LEVEL_INFO, LOG_LEVEL_DEBUG, LOG_LEVEL_DEBUG_MORE};
    #define DEBUG_MSG(...) AddLog(LOG_LEVEL_DEBUG, __VA_ARGS__)
#else
    #define DEBUG_MSG(...)
#endif

#endif
