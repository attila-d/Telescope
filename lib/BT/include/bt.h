#pragma once

//#define __BT

#include <Arduino.h>

// #include "nexstar.h"

#ifdef __BT

void tickBT();
void initBT(NexstarProtocol &protocol, Stream *stream);

#endif // __BT
