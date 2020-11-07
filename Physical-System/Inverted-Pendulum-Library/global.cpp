#include "global.h"
#include <stdint.h>

volatile int g_pendulumEncA = 0;
volatile int g_motorEnc = 0;
const uint8_t g_leftLimit = 7;
const uint8_t g_rightLimit = 8;
int g_motorDir = 0;

//note to self: come back and clean up global variables.