#ifndef global_h
#define global_h

#include <stdint.h>

extern volatile int g_pendulumEncA;
extern volatile int g_motorEnc;
extern const uint8_t g_leftLimit;
extern const uint8_t g_rightLimit;
extern int g_motorDir;

#endif