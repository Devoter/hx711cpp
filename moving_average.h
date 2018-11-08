#ifndef MOVING_AVERAGE_H
#define MOVING_AVERAGE_H

#include <deque>
#include <iostream>


template <typename T, typename D>
class MovingAverage {
    std::size_t m_maxSize;
    std::deque<T> m_deque;
    D m_value;
    bool m_dirty;

public:
    MovingAverage(const std::size_t maxSize);
    inline std::size_t maxSize() { return m_maxSize; }
    inline std::size_t size() { return m_deque.size(); }
    D value();
    void push(const T item);
    T front();
    void clear();
};

template <typename T, typename D>
MovingAverage<T, D>::MovingAverage(const std::size_t maxSize)
{
    m_maxSize = maxSize;
    m_value = 0;
    m_dirty = false;
}

template <typename T, typename D>
D MovingAverage<T, D>::value()
{
    if (m_dirty) {
        T sum = 0;

        for (auto const &el: m_deque)
            sum += el;

        m_value = static_cast<D>(sum) / m_deque.size();
        m_dirty = false;
    }

    return m_value;
}

template <typename T, typename D>
void MovingAverage<T, D>::push(const T item)
{
    if (m_deque.size() >= m_maxSize)
        m_deque.pop_front();

    m_deque.push_back(item);
    m_dirty = true;
}

template <typename T, typename D>
T MovingAverage::front()
{
    return m_deque.front();
}

template <typename T, typename D>
void MovingAverage<T, D>::clear()
{
    m_deque.clear();
    m_value = 0;
    m_dirty = false;
}

#endif // MOVING_AVERAGE_H
