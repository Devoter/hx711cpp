#include <iostream>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <signal.h>
#include "hx711.h"
#include "string_to_double.h"

bool sigTerm = false;

void onTerminate(int signum, siginfo_t *info, void *ptr)
{
    std::cerr << "TERM signal received" << std::endl;
    sigTerm = true;
}

void catchSigterm()
{
    static struct sigaction sigact;

    memset(&sigact, 0, sizeof(sigact));
    sigact.sa_sigaction = onTerminate;
    sigact.sa_flags = SA_SIGINFO;

    sigaction(SIGTERM, &sigact, NULL);
    sigaction(SIGINT, &sigact, NULL);
}

int main(int argc, char *argv[])
{
    if (argc != 10) {
        return 1;
    }

    catchSigterm();

    const double offset = atof(argv[1]);
    const char *alignmentString = argv[2];
    const int movingAverage = atoi(argv[3]);
    const int times = atoi(argv[4]);
    const int dout = atoi(argv[5]);
    const int sck = atoi(argv[6]);
    const int deviationFactor = atoi(argv[7]);
    const int deviationValue = atoi(argv[8]);
    const int retries = atoi(argv[8]);

    double k, b;

    k = stringToDouble(alignmentString);
    b = stringToDouble(alignmentString + 16);

    auto hx = new HX711(dout, sck, offset, movingAverage, times, k, b, deviationFactor, deviationValue, retries);

    hx->setGain(1);
    hx->read();

    while (hx->once() && !sigTerm)
        usleep(100000);

    if (!sigTerm) {
        sleep(1);
    
        hx->reset();
        hx->start();

        while (!sigTerm)
            usleep(100000);
    }

    delete hx;

    return 0;
}
