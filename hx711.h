#ifndef HX711_H
#define HX711_H

#include <cmath>
#include <atomic>
#include <memory>
#include <thread>
#include <mutex>
#include "moving_average.h"
#include "simple_kalman_filter.h"


class HX711 {
    int m_dout;
    int m_sck;
    unsigned char m_gain;

    bool m_reading;
    bool m_once;

    bool m_debug;
    std::atomic_bool m_working;

    bool m_useTAFilter;
    bool m_useKalmanFilter;
    bool m_humanMode;

    double m_k;
    double m_b;

    unsigned int m_retries;
    unsigned int m_tries;
    unsigned int m_fails;

    double m_deviationFactor;
    double m_deviationValue;

    std::mutex m_mutex;

    std::atomic_int m_temperature;
    std::atomic_bool m_temperatureReadFail;
    double m_temperatureFactor;
    int m_baseTemperature;

    std::shared_ptr<MovingAverage<double, double>> m_movingAverage;
    std::shared_ptr<MovingAverage<int32_t, double>> m_timed;
    std::shared_ptr<SimpleKalmanFilter> m_kalman;
    std::shared_ptr<std::thread> m_temperatureReader;

public:
    HX711(const int dout, const int sck, const double correctionFactor, const double offset,
          const unsigned int movingAverageSize, const unsigned int times, const double k, const double b,
          const bool useTAFilter, const int deviationFactor, const int deviationValue, const unsigned int retries,
          const bool useKalmanFilter, const double kalmanQ, const double kalmanR, const double kalmanF, const double kalmanH,
          const bool debug, const bool humanMode, const char *filename, const double temperatureFactor,
          const int baseTemperature);
    virtual ~HX711();

    inline int dout() { return m_dout; }
    inline int sck() { return m_sck; }
    inline unsigned char gain() { return m_gain; }
    inline bool reading() { return m_reading; }
    inline bool once() { return m_once; }

    inline void setReading(const bool reading) { m_reading = reading; }
    inline void setOnce(const bool once) { m_once = once; }
    inline void resetFails() { m_fails = 0; }

    void start();
    void stop();
    void read();
    void setGain(const unsigned char gain);
    void powerDown();
    void powerUp();
    void reset();
    void push(const int32_t value);
    void incFails();

protected:
    void pushValue(const double &value);
    bool taFilter(const double &value);
    inline double align(const double &value)
    {
        return (value + (m_temperature - m_baseTemperature) * m_temperatureFactor) * m_k + m_b;
    }
    inline int align(const double &value, const bool integer)
    {
        return std::round((value + (m_temperature - m_baseTemperature) * m_temperatureFactor) * m_k + m_b);
    }
    static void readTemperature(HX711 *instance, const char *filename);
};

#endif // HX711_H
