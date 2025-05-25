#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H



/**
 *  Low pass filter class
 */
class LowPassFilter
{
public:
    /**
     * @param Tf - Low pass filter time constant
     * @param current  - current time in microseconds
     */
    LowPassFilter(float Tf, unsigned long timestamp);
    ~LowPassFilter() = default;

    float operator() (float x, unsigned long timestamp);
    float Tf; //!< Low pass filter time constant

protected:
    unsigned long timestamp_prev;  //!< Last execution timestamp
    float y_prev; //!< filtered value in previous execution step 
};

#endif // LOWPASS_FILTER_H