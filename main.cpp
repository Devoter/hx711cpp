#include <iostream>
#include <cstdlib>
#include <sstream>
#include <cstring>
#include <unistd.h>
#include <signal.h>
#include "hx711.h"
#include "string_to_double.h"
#include "config.h"


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
    std::stringstream welcome;

    welcome << "HX711 driver, version " << applicationVersion << "\n\tCopyright 2018 Tulpa Automatics";

    std::cerr << welcome << std::endl;

    if (argc != 21) {
        std::cerr << "No enough parameters" << std::endl;
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
    const bool useTAFilter = static_cast<bool>(atoi(argv[11]));
    const bool useKalmanFilter = static_cast<bool>(atoi(argv[12]));
    const double kalmanQ = humanMode ? atof(argv[13]) : stringToDouble(argv[13]);
    const double kalmanR = humanMode ? atof(argv[14]) : stringToDouble(argv[14]);
    const double kalmanF = humanMode ? atof(argv[15]) : stringToDouble(argv[15]);
    const double kalmanH = humanMode ? atof(argv[16]) : stringToDouble(argv[16]);
    const char *temperatureFilename = argv[17];
    const double temperatureFactor = humanMode ? atof(argv[18]) : stringToDouble(argv[18]);
    const int baseTemperature = atoi(argv[19]);
    const bool debug = static_cast<bool>(atoi(argv[20]));
    const double k = stringToDouble(alignmentString), b = stringToDouble(alignmentString + 16);

    if (debug) {
        std::stringstream debugInfo;

        debugInfo << "dout: " << dout << ", sck: " << sck << '\n' <<
                  "offset: " << offset << '\n' <<
                  "k: " << k << ", b: " << b << '\n' <<
                  "moving average: " << movingAverage << '\n' <<
                  "TA filter:: use: " << useTAFilter << ", times: " << times <<
                  ", deviation:: factor: " << deviationFactor << ", deviation value: " << deviationValue << ", retries: " << retries << '\n' <<
                  "Kalman filter:: use: " << useKalmanFilter <<  ", Q: " << kalmanQ << ", R: " <<
                  kalmanR << ", F: " << kalmanF << ", H: " << kalmanH << '\n' <<
                  "debug: " << debug << '\n' <<
                  "human mode: " << humanMode << '\n' <<
                  "temperature filename: " << temperatureFilename << '\n' <<
                  "temperature factor: " << temperatureFactor << ", base: " << baseTemperature;

        std::cerr << debugInfo << std::endl;
    }

    auto hx = new HX711(dout, sck, offset, movingAverage, times, k, b, useTAFilter, deviationFactor, deviationValue, retries,
            useKalmanFilter, kalmanQ, kalmanR, kalmanF, kalmanH, debug, humanMode, temperatureFilename, temperatureFactor, baseTemperature);

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
