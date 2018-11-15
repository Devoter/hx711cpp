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
    if (argc != 16) {
        return 1;
    }

    catchSigterm();

    const bool humanMode = static_cast<bool>(atoi(argv[1]));
    const double offset = atof(argv[2]);
    const char *alignmentString = argv[3];
    const int movingAverage = atoi(argv[4]);
    const int times = atoi(argv[5]);
    const int dout = atoi(argv[6]);
    const int sck = atoi(argv[7]);
    const int deviationFactor = atoi(argv[8]);
    const int deviationValue = atoi(argv[9]);
    const int retries = atoi(argv[10]);
    const double kalmanQ = humanMode ? atof(argv[11]) : stringToDouble(argv[11]);
    const double kalmanR = humanMode ? atof(argv[12]) : stringToDouble(argv[12]);
    const double kalmanF = humanMode ? atof(argv[13]) : stringToDouble(argv[13]);
    const double kalmanH = humanMode ? atof(argv[14]) : stringToDouble(argv[14]);
    const bool debug = static_cast<bool>(atoi(argv[15]));

    double k, b;

    k = stringToDouble(alignmentString);
    b = stringToDouble(alignmentString + 16);

    auto hx = new HX711(dout, sck, offset, movingAverage, times, k, b, deviationFactor, deviationValue, retries,
            kalmanQ, kalmanR, kalmanF, kalmanH, debug);

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
