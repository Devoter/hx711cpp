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
             const int deviationValue)
{
    m_times = times;
    m_k = k;
    m_b = b + offset;
    if (deviationFactor) {
        m_deviationFactor = deviationFactor / 100.0;
        m_deviationValue = deviationValue;
    }
    else {
        m_deviationFactor = 0;
        m_deviationValue = (deviationValue - m_b) / k;
    }
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
#ifdef ALTERNATIVE_FILTERING
        auto val = static_cast<double>(m_timed->front());
        m_timed->push(value);

        double factor;
        auto maValue = m_movingAverage->value();
        auto timedValue = m_timed->value();

        if (m_deviationFactor)
            factor = ((maValue * m_k + m_b) * m_deviationFactor + m_deviationValue - m_b) / m_k;
        else
            factor = m_deviationValue;

        if (val < (maValue - factor) || val > (maValue + factor) ||
                val < (timedValue - factor) || val > (timedValue + factor)) {
            return;
        }
#else
        auto val = static_cast<double>(m_timed->front()) * m_k + m_b;
        m_timed->push(value);

        auto maValue = m_movingAverage->value() * m_k + m_b;
        auto maFactored = maValue * m_deviationFactor;
        auto timedValue = m_timed->value() * m_k + m_b;
        auto timedFactored = timedValue * m_deviationFactor;

        if (val < (maValue - maFactored - m_deviationValue) || val > (maValue + maFactored + m_deviationValue) ||
                val < (timedValue - timedFactored - m_deviationValue) ||
                val > (timedValue + timedFactored + m_deviationValue)) {
            return;
        }
#endif
        m_movingAverage->push(val);
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
