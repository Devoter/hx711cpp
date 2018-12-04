#include <iostream>
#include <cstdlib>
#include <sstream>
#include <string>
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

std::string help()
{
    const char b[] = "\033[1;36m"; // bold cyan
    const char wb[] = "\033[1;37m"; // while bold
    const char c[] = "\033[0m"; // clear format
    const char u[] = "\033[4;32m"; // green underline
    const char cu[] = "\033[0m \033[4;32m"; // clear + space + green underline
    const char tb[] = "\t\033[1;36m"; // tab + bold cyan

    return std::string("\n\nhx711 <human_mode> <offset> <alignment_string> <moving_average> <times> <dout> <sck> <deviation_factor> <deviation_value> "
                       "<retries> <use_ta_filter> <use_kalman_filter> <kalman_q> <kalman_r> <kalman_f> <kalman_h> <temperature_filename> <temperature_factor> "
                       "<base_temperature> <debug>\n\n") +
           tb + "int" + cu + "human mode" + c + " - " + wb + '0' + c + " - Normal mode, " + wb + '1' + c +
           " - Human mode (input and output all values as decimal except alignment string)\n" +
           tb + "double" + cu + "offset" + c + " - result offset, appends to a result value\n" +
           tb + "char *" + cu + "alignment string" + c + " - ascii-coded 16 bytes of " + b + "double" + ' ' + wb + 'k' + c + "and" +
           wb + 'b' + c + " factors from " + wb + "y = k * x + b" + c + '\n' +
           tb + "unsigned int" + cu + "times" + c + " - count of consecutive measurements, which are used to reduce the value volatility\n" +
           tb + "unsigned int" + cu + "dout" + c + " - _data out_ pin number (BCM)\n" +
           tb + "unsigned int" + cu + "sck" + c + " - _serial clock_ pin number (BCM)\n" +
           tb + "int" + cu + "deviation factor" + c + " - tolerance percentage\n" +
           tb + "int" + cu + "deviation value" + c + " - tolerance\n" +
           tb + "unsigned int" + cu + "retries" + c + " - count of retries before agree invalid values\n" +
           tb + "int" + cu + "use TA filter" + c + " - " + wb + '0' + c + "- disable Tulpa Automatics filter, " + wb + '1' + c + " - enable\n" +
           tb + "int" + cu + "use Kalman filter" + c + " - " + wb + '0' + c + " - disable Kalman filter, " + wb + '1' + c + " - enable\n" +
           tb + "double" + c + " (Human mode) " + b + "string" + cu + "Kalman Q" + c + " - the covariance of the process noise\n" +
           tb + "double" + c + " (Human mode) " + b + "string" + cu + "Kalman R" + c + " - the covariance of the observation noise\n" +
           tb + "double" + c + " (Human mode) " + b + "string" + cu + "Kalman F" + c + " - the state-transition model (set to " + wb + '1' + c + ")\n" +
           tb + "double" + c + " (Human mode) " + b + "string" + cu + "Kalman H" + c + " - the observation model (set to " + wb + '1' + c + ")\n" +
           tb + "string" + cu + "temperature filename" + c + " - a name of file contains temperature value\n" +
           tb + "double" + c + " (Human mode) " + b + "string" + cu + "temperature factor" + c + " - temperature compensation factor\n" +
           tb + "int" + cu + "base temperature" + c + " - reference temperature value (in thousandths of degrees Celsius)\n" +
           tb + "int" + cu + "debug" + c + " - " + wb + '0' + c + " - disable debug, " + wb + '1' + c + " - enable (debug messages outputs to stderr)\n";
}

int main(int argc, char *argv[])
{
    std::stringstream welcome;

    welcome << "HX711 driver, version " << applicationVersion << "\n\tCopyright 2018 Tulpa Automatics\n";

    std::cerr << welcome.str() << std::endl;

    if (argc != 21) {
        std::cerr << "No enough parameters" << help() << std::endl;
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
                  "Kalman filter:: use: " << useKalmanFilter << ", Q: " << kalmanQ << ", R: " <<
                  kalmanR << ", F: " << kalmanF << ", H: " << kalmanH << '\n' <<
                  "debug: " << debug << '\n' <<
                  "human mode: " << humanMode << '\n' <<
                  "temperature filename: " << temperatureFilename << '\n' <<
                  "temperature factor: " << temperatureFactor << ", base: " << baseTemperature;

        std::cerr << debugInfo.str() << std::endl;
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
