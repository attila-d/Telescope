#pragma once

// #define _DEBUG

#ifdef _DEBUG
// #define DEBUG(x) Serial3.print(x)
// #define DEBUG2(x,z) Serial3.print(x,z)
// #define DEBUGLN() Serial3.println("")
#define DEBUG(x) Serial.print(x)
#define DEBUG2(x, z) Serial.print((x), (z))
#define DEBUG3(x, y, z)         \
    {                           \
        Serial.print((x));      \
        Serial.print((y), (z)); \
    }
#define DEBUG3LN(x, y, z)         \
    {                             \
        Serial.print((x));        \
        Serial.println((y), (z)); \
    }
#define DEBUGLN(x) Serial.println(x)
#define DEBUG2LN(x, z) Serial.println((x), (z))
#define DEBUGW(x) Serial.write(x)
#define DEBUG4(x, y, z, dec)         \
    {                           \
        Serial.print((x));      \
        Serial.print((y), (dec)); \
        Serial.print(",");  \
        Serial.print((z), (dec)); \
    }
#define DEBUG4LN(x, y, z, dec)         \
    {                           \
        Serial.print((x));      \
        Serial.print((y), (dec)); \
        Serial.print(",");  \
        Serial.println((z), (dec)); \
    }


#else

#define DEBUG(x)
#define DEBUG2(x, z)
#define DEBUGLN(x)
#define DEBUG2LN(x, z)
#define DEBUG3(x, y, z)
#define DEBUG3LN(x, y, z)
#define DEBUG4(x, y, z, dec)
#define DEBUG4LN(x, y, z, dec)
#define DEBUGW(x)
#endif