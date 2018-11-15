#include "simple_kalman_filter.h"

SimpleKalmanFilter::SimpleKalmanFilter(const double q, const double r, const double f, const dobule h)
{
    m_q = q;
    m_r = r;
    m_f = f;
    m_h = h;
    m_x0 = m_p0 = m_state = m_covariance = 0;
    m_initialized = false;
}

void SimpleKalmanFilter::setState(const double state, const double covariance)
{
    m_state = state;
    m_covariance = covariance;
    m_initialized = true;
}

void SimpleKalmanFilter::correct(const double data)
{
    // time update - prediction
    m_x0 = m_f * m_state;
    m_p0 = m_f * m_covariance * m_f + m_q;

    // measurement update - correction
    double k = m_h * m_p0 / (m_h * m_p0 * m_h + m_r);
    m_state = m_x0 + k * (data - m_h* m_x0);
    m_covariance = (1 - k * m_h) * m_p0;
}
