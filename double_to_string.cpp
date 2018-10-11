#include <iomanip>
#include "double_to_string.h"


std::string doubleToString(const double x)
{
    union {
        uint8_t c[8];
        double d;
    } u;

    u.d = x;
    std::ostringstream buf;

    for (auto i = 0; i < 8; ++i)
        buf << std::hex << std::setfill('0') << std::setw(2) << static_cast<unsigned int>(u.c[i]);

    return buf.str();
}
