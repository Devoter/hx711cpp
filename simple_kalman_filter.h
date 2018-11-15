#ifndef HX711CPP_SIMPLE_KALMAN_FILTER_H
#define HX711CPP_SIMPLE_KALMAN_FILTER_H

class SimpleKalmanFilter {
    double m_x0;
    double m_p0;
    double m_f;
    double m_q;
    double m_h;
    double m_r;
    double m_state;
    double m_covariance;

public:
    SimpleKalmanFilter(const double q, const double r, const double f = 1, const dobule h = 1);

    inline double state() const { return m_state; }
    void setState(const double state, const double covariance);

    inline double covariance() const { return m_covariance; }

    void correct(const double data);
};

#endif //HX711CPP_SIMPLE_KALMAN_FILTER_H
