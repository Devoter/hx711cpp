#include <cstdio>

#include "string_to_double.h"


double stringToDouble(const char *input)
{
    const int bytesInDouble = 8;

    union {
        double value;
        unsigned char bytes[bytesInDouble];
    } u;

    unsigned char *output = u.bytes;

    for(auto i = 0; i < bytesInDouble; ++i) {
        sscanf(input, "%02hhX", output);
        input += 2;
        ++output;
    }

    return u.value;
}
