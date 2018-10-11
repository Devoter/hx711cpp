#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include "hx711.h"
#include "string_to_double.h"


int main(int argc, char *argv[])
{
    if (argc != 7) {
        return 1;
    }

    const double offset = atof(argv[1]);
    const char *alignmentString = argv[2];
    const int movingAverage = atoi(argv[3]);
    const int times = atoi(argv[4]);
    const int dout = atoi(argv[5]);
    const int sck = atoi(argv[6]);
    double k, b;

    k = stringToDouble(alignmentString);
    b = stringToDouble(alignmentString + 16);

    auto hx = new HX711(dout, sck, offset, movingAverage, times, k, b);

    hx->setGain(1);
    hx->read();

    while (hx->once())
        usleep(100);
    sleep(1);
    
    hx->reset();
    hx->start();

    while (true)
        usleep(100000);

    return 0;
}
