#include <iostream>
#include <chrono>
#include <string>
#include <fstream>
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
             const char *filename, const double temperatureFactor, const int baseTemperature)
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
    m_temperatureFactor = temperatureFactor;
    m_baseTemperature = baseTemperature;

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
    m_working = false;
    if (m_temperatureReader->joinable())
        m_temperatureReader->detach();

    instance = nullptr;
    m_movingAverage.reset();
    m_timed.reset();
    m_kalman.reset();
    m_temperatureReader.reset();
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
            m_kalman->initialized() ? m_kalman->correct(value) : m_kalman->setState(value, 0.1);
            m_movingAverage->push(m_kalman->state());
        }
        else
            m_movingAverage->push(value);
        return;
    }
    else if (m_timed->size() < m_timed->maxSize()) {
        m_timed->push(value);
        return;
    }
    else {
        double rawValue = m_timed->front();
        double val = align(rawValue);

        m_timed->push(value);

        bool filtered = !taFilter(val);

        if (filtered) {
            ++m_tries;

            if (m_debug) {
                std::lock_guard<std::mutex> lock(m_mutex);
                if (m_humanMode)
                    std::cerr << "\nFiltered: " << val << std::endl;
                else
                    std::cerr << << val << std::endl;
            }
        }

        if (!filtered) {
            m_tries = 0;
            pushValue(rawValue);
        }
        else if (m_tries >= m_retries)
            pushValue(rawValue);
    }

    const double result = align(m_movingAverage->value());

    if (m_humanMode) {
        for (int i = 0; i < 80; ++i)
            std::cout << '\b';
        std::cout << doubleToString(result) << ' ' << m_temperature << ' ' << m_temperatureReadFail << ' ' << result;
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

    double maValue = align(m_movingAverage->value());
    double maFactored = maValue * m_deviationFactor;

    if (value < (maValue - maFactored - m_deviationValue) || value > (maValue + maFactored + m_deviationValue))
        return false;
    else {
        auto deque = *(m_timed->deque());

        for (auto &el : deque) {
            double t = align(el);
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
        bool failed = false;

        inf.open(filename);
        if (!inf.is_open()) {
            failed = true;

            std::lock_guard <std::mutex> lock(instance->m_mutex);
            if (instance->m_debug)
                std::cerr << "Could not open sensor device file" << std::endl;
        }


        std::string line;

        if (!failed) {
            std::getline(inf, line);

            if (line.find(yes, 0) == std::string::npos) {
                failed = true;

                std::lock_guard <std::mutex> lock(instance->m_mutex);
                if (instance->m_debug)
                    std::cerr << "Sensor is not ready" << std::endl;
            }
        }

        line.clear();
        if (!failed)
            std::getline(inf, line);
        inf.close();

        if (!failed) {
            auto found = line.find(tempPrefix, 0);
            if (found == std::string::npos) {
                failed = true;

                std::lock_guard <std::mutex> lock(instance->m_mutex);
                if (instance->m_debug)
                    std::cerr << "Temperature value is not found" << std::endl;
            }
            else
                instance->m_temperature = std::atoi(line.substr(found + 2).c_str());
        }

        instance->m_temperatureReadFail = failed;
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}
