#include <iostream>
#include <chrono>
#include <shared_mutex>
#include <string>
#include <fstream>
#include <thread>
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

HX711::HX711(const int dout, const int sck, const double offset, const unsigned int movingAverageSize, const unsigned int times, const double k, const double b,
             const bool useTAFilter, const int deviationFactor, const int deviationValue, const unsigned int retries, const bool useKalmanFilter,
             const double kalmanQ, const double kalmanR, const double kalmanF, const double kalmanH, const bool debug, const bool humanMode,
             const char *filename)
{
    m_working = true;
    m_k = k;
    m_b = b + offset;

    m_deviationFactor = deviationFactor ? deviationFactor / 100.0 : 0;
    m_deviationValue = deviationValue;
    m_tries = 0;
    m_retries = retries;

    m_debug = debug;
    m_reading = false;
    m_once = false;

    m_humanMode = humanMode;
    m_useTAFilter = useTAFilter;
    m_useKalmanFilter = useKalmanFilter;

    m_fails = 0;

    m_temperature = 0;
    m_temperatureReadFail = true;

    wiringPiSetupGpio();
    m_dout = dout;
    m_sck = sck;
    m_gain = 1;
    pinMode(m_dout, INPUT);
    pinMode(m_sck, OUTPUT);

    m_movingAverage = std::make_shared<MovingAverage<double, double>>(movingAverageSize);
    m_timed = std::make_shared<MovingAverage<int32_t, double>>(times);
    m_kalman = std::make_shared<SimpleKalmanFilter>(kalmanQ, kalmanR, kalmanF, kalmanH);
    m_temperatureReader = std::make_shared<std::thread>(HX711::readTemperature, this, filename);
}

HX711::~HX711()
{
    m_movingAverage.reset();
    instance = nullptr;
    m_working = false;
    if (m_temperatureReader->joinable())
        m_temperatureReader->join();
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
    if (m_movingAverage->size() < m_movingAverage->maxSize()) {
        if (m_useKalmanFilter) {
            m_kalman->initialized() ? m_kalman->correct(align(value)) : m_kalman->setState(align(value), 0.1);
            m_movingAverage->push(m_kalman->state());
        }
        else
            m_movingAverage->push(align(value));
        return;
    }
    else if (m_timed->size() < m_timed->maxSize()) {
        m_timed->push(value);
        return;
    }
    else {
        double val = align(m_timed->front());

        m_timed->push(value);

        bool filtered = !taFilter(val);

        if (filtered) {
            ++m_tries;

            if (m_debug) {
                std::lock_guard<std::shared_mutex> lock(m_mutex);
                if (m_humanMode)
                    std::cerr << "\nFiltered: " << val << std::endl;
                else
                    std::cerr << "Filtered: " << doubleToString(val) << std::endl;
            }
        }

        if (!filtered) {
            m_tries = 0;
            pushValue(val);
        }
        else if (m_tries >= m_retries)
            pushValue(val);
    }

    const double result = m_movingAverage->value();

    if (m_humanMode) {
        for (int i = 0; i < 80; ++i)
            std::cout << '\b';
        std::cout << doubleToString(result) << ' ' << m_temperature << ' ' << result;
    }
    else
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

void HX711::pushValue(const double &value)
{
    if (m_useKalmanFilter) {
        m_kalman->correct(value);
        m_movingAverage->push(m_kalman->state());
    }
    else
        m_movingAverage->push(value);
}

bool HX711::taFilter(const double &value)
{
    if (!m_useTAFilter)
        return true;

    double maValue = m_movingAverage->value();
    double maFactored = maValue * m_deviationFactor;

    if (value < (maValue - maFactored - m_deviationValue) || value > (maValue + maFactored + m_deviationValue))
        return false;
    else {
        auto deque = *(m_timed->deque());

        for (auto &el : deque) {
            double t = static_cast<double>(el) * m_k + m_b;
            if (value < (t - maFactored - m_deviationValue) || value > (t + maFactored + m_deviationValue))
                return false;
        }
    }

    return true;
}

void HX711::readTemperature(HX711 *instance, const char *filename)
{
    std::ifstream inf;
    char yes[] = "YES";
    char tempPrefix[] = "t=";

    while (instance->m_working) {

        inf.open(filename);
        if (!inf.is_open()) {
            std::lock_guard<std::shared_mutex> lock(instance->m_mutex);
            instance->m_temperatureReadFail = true;
            std::cerr << "Could not open sensor device file" << std::endl;
            continue;
        }


        std::string line;
        std::getline(inf, line);
        if (line.find(yes, 0) == std::string::npos) {
            std::lock_guard<std::shared_mutex> lock(instance->m_mutex);
            instance->m_temperatureReadFail = true;
            std::cerr << "Sensor is not ready" << std::endl;
            inf.close();
            continue;
        }

        line.clear();
        std::getline(inf, line);
        inf.close();

        auto found = line.find(tempPrefix, 0);
        if (found == std::string::npos) {
            std::lock_guard<std::shared_mutex> lock(instance->m_mutex);
            instance->m_temperatureReadFail = true;
            std::cerr << "Temperature value is not found" << std::endl;
            continue;
        }

        instance->m_temperature = std::atoi(line.substr(found + 2).c_str());
        instance->m_temperatureReadFail = false;
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}
