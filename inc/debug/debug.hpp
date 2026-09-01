#ifndef DEBUG_H
#define DEBUG_H

#ifdef DEBUG
    #include <stdio.h>
    #define DEBUG_PRINTF(fmt, ...) printf("[DEBUG] " fmt, ##__VA_ARGS__)
    #define DEBUG_PRINTF_LOC(fmt, ...) printf("[DEBUG] (%s:%d) " fmt, __func__, __LINE__, ##__VA_ARGS__)
#else
    #define DEBUG_PRINTF(fmt, ...)        do {} while(0)
    #define DEBUG_PRINTF_LOC(fmt, ...)    do {} while(0)
#endif

#endif // DEBUG_H