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

HX711::HX711(const int dout, const int sck, const double offset, const int movingAverageSize, const unsigned int times, const double k, const double b)
{
    m_offset = offset;
    m_times = times;
    m_k = k;
    m_b = b;
    m_reading = false;
    m_once = false;
    m_fails = 0;
    wiringPiSetupGpio();
    m_dout = dout;
    m_sck = sck;
    m_gain = 1;
    pinMode(m_dout, INPUT);
    pinMode(m_sck, OUTPUT);
    m_movingAverage = std::make_shared< MovingAverage<int32_t, double> >(movingAverageSize);
    m_timed = std::make_shared< MovingAverage<int32_t, int32_t> >(m_times);
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
    m_timed->push(value);

    if (m_timed->size() < m_times)
        return;

    m_movingAverage->push(m_timed->value());
    m_timed->clear();
    const double result = m_movingAverage->value() * m_k + m_b + m_offset;
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