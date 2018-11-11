#include <iostream>
#include <unistd.h>
#include <wiringPi.h>
#include "double_to_string.h"
#include "hx711.h"

const unsigned int maxFails = 20;
HX711 *instance = nullptr;

void edge()
{
    if (!instance || instance->reading() || digitalRead(instance->dout()))
        return;

    const int dout = instance->dout();
    const int sck = instance->sck();
    instance->setReading(true);

    int32_t data = 0;
               
    for (signed char i = 23; i >= 0; --i) {
        digitalWrite(sck, HIGH);
        digitalRead(dout); // I cannot understand why, but reading doesn't works, if I don't read twice
        digitalWrite(sck, LOW);
        data |= digitalRead(dout) << i;
    }   
                  
    if (data != 0x800000 && data != 0x7fffff && data != 0xffffff) {
        instance->resetFails();

        if (data & 0x800000)
            data |= 0xff << 24; 

        for (unsigned char k = 0; k < instance->gain(); ++k) {
            digitalWrite(sck, HIGH);
            digitalWrite(sck, LOW);
        }

        if (!instance->once())
            instance->push(data);
    }
    else
        instance->incFails();

    usleep(20000);
    if (instance->once())
        instance->setOnce(false);
    else
        instance->setReading(false);
}

HX711::HX711(const int dout, const int sck, const double offset, const unsigned int movingAverageSize,
             const unsigned int times, const double k, const double b, const int deviationFactor,
             const int deviationValue, const unsigned int retries, const bool debug)
{
    m_times = times;
    m_k = k;
    m_b = b + offset;
    m_deviationFactor = deviationFactor ? deviationFactor / 100.0 : 0;
    m_deviationValue = deviationValue;
    m_tries = 0;
    m_retries = retries;
    m_debug = debug;
    m_reading = false;
    m_once = false;
    m_fails = 0;
    wiringPiSetupGpio();
    m_dout = dout;
    m_sck = sck;
    m_gain = 1;
    pinMode(m_dout, INPUT);
    pinMode(m_sck, OUTPUT);
    m_movingAverageSize = movingAverageSize;
    m_movingAverage = std::make_shared< MovingAverage<double, double> >(movingAverageSize);
    m_timed = std::make_shared< MovingAverage<int32_t, double> >(m_times);
}

HX711::~HX711()
{
    m_movingAverage.reset();
    instance = nullptr;
}

void HX711::start()
{
    m_once = false;
    m_reading = false;
    instance = this;
}

void HX711::stop()
{
    instance = nullptr;
}

void HX711::read()
{
    m_once = true;
    m_reading = false;
    instance = this;
    wiringPiISR(m_dout, INT_EDGE_FALLING,  edge);
}

void HX711::setGain(const unsigned char gain)
{
    m_gain = gain;
    digitalWrite(m_sck, LOW);
}

void HX711::powerDown()
{
    digitalWrite(m_sck, LOW);
    digitalWrite(m_sck, HIGH);
    usleep(100);
}

void HX711::powerUp()
{
    digitalWrite(m_sck, LOW);
    usleep(100);
}

void HX711::reset()
{
    powerDown();
    powerUp();
}

void HX711::push(const int32_t value)
{
    if (m_movingAverage->size() < m_movingAverageSize) {
        m_movingAverage->push(value);
        return;
    }
    else if (m_timed->size() < m_times) {
        m_timed->push(value);
        return;
    }
    else {
        auto rawVal = static_cast<double>(m_timed->front());
        auto val = rawVal * m_k + m_b;
        m_timed->push(value);

        auto maValue = m_movingAverage->value() * m_k + m_b;
        auto maFactored = maValue * m_deviationFactor;

        bool filtered = false;
        if (val < (maValue - maFactored - m_deviationValue) || val > (maValue + maFactored + m_deviationValue)) {
            filtered = true;
        }
        else {
            auto deque = *(m_timed->deque());
            for (auto &el : deque) {
                auto t = static_cast<double>(el) * m_k + m_b;
                if (val < (t - maFactored - m_deviationValue) || val > (t + maFactored + m_deviationValue)) {
                    filtered = true;
                    break;
                }
            }
        }

        if (filtered) {
            ++m_tries;
            if (m_debug)
                std::cerr << doubleToString(val) << std::endl;
        }

        if (!filtered) {
            m_tries = 0;
            m_movingAverage->push(rawVal);
        }
        else if (m_tries >= m_retries)
            m_movingAverage->push(rawVal);
    }

    const double result = m_movingAverage->value() * m_k + m_b;
    std::cout << doubleToString(result) << std::endl;
}

void HX711::incFails()
{
    ++m_fails;
    if (m_fails >= maxFails) {
        reset();
        m_fails = 0;
    }
}
