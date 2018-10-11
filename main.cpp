#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <signal.h>
#include "hx711.h"
#include "string_to_double.h"

bool sigTerm = false;

void onTerminate(int signum, siginfo_t *info, void *ptr)
{
    sigTerm = true;
}

void catchSigterm()
{
    static struct sigaction _sigact;

    memset(&_sigact, 0, sizeof(_sigact));
    _sigact.sa_sigaction = on_terminate;
    _sigact.sa_flags = SA_SIGINFO;

    sigaction(SIGTERM, &_sigact, NULL);
    sigaction(SIGINT, &_sigact, NULL);
}

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

    while (sigterm)
        usleep(100000);

    delete hx;

    return 0;
}
