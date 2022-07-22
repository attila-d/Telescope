#include <Arduino.h>

#include "debug.h"
#include "nexstar.h"
#include "bt.h"

static NexstarProtocol *btProtocol = NULL;
static Stream *btStream = NULL;

void initBT(NexstarProtocol &protocol, Stream * stream)
{
    btProtocol = &protocol;
    // stream.begin(9600); // bluetooth
    btStream = stream;
}

long lastRead = 0;

void tickBT()
{
    if (btStream != NULL)
    {
        long now = millis();

        if (btStream->available() > 0)
        {
            int c = btStream->read();
            // DEBUG("0x");
            // DEBUG2(c, HEX);
            // DEBUGW('=');
            // DEBUGW(c);
            if (btProtocol != NULL)
            {
                // DEBUGW('[');
                // DEBUG2(btProtocol->cnt,DEC);
                // DEBUGW(']');
                btProtocol->process(c);
            }
            lastRead = now;
            // DEBUGLN();
        }
        if (btProtocol != NULL && (now > (lastRead + 5 * 1000)) && btProtocol->isPending())
        {
            DEBUG("Timeout, resetting protocol.");
            DEBUGLN();
            btProtocol->reset();
            lastRead = now;
        }
    }
}