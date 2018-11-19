#ifndef HX711_H
#define HX711_H

#include <memory>
#include "moving_average.h"
#include "simple_kalman_filter.h"


class HX711 {
    int m_dout;
    int m_sck;
    unsigned char m_gain;
    bool m_reading;
    bool m_once;
    bool m_debug;
    bool m_useTAFilter;
    bool m_useKalmanFilter;
    bool m_humanMode;
    unsigned int m_retries;
    unsigned int m_tries;
    double m_k;
    double m_b;
    unsigned int m_fails;
    double m_deviationFactor;
    double m_deviationValue;
    std::shared_ptr< MovingAverage<double, double> > m_movingAverage;
    std::shared_ptr< MovingAverage<int32_t, double> > m_timed;
    std::shared_ptr<SimpleKalmanFilter> m_kalman;

public:
    HX711(const int dout, const int sck, const double offset, const unsigned int movingAverageSize, const unsigned int times, const double k, const double b,
          const bool useTAFilter, const int deviationFactor, const int deviationValue, const unsigned int retries, const bool useKalmanFilter,
          const double kalmanQ, const double kalmanR, const double kalmanF, const double kalmanH, const bool debug, const bool humanMode);
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
};

#endif // HX711_H
