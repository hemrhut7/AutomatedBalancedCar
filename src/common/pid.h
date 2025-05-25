#ifndef PID_H
#define PID_H
#define constrain_pid(val, min, max) ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))


class PID {
    public:
        PID(){};
        PID(float kp, float ki, float kd, float limit=99999.9f) : 
        kp(kp), ki(ki), kd(kd), limit(limit) {};
        ~PID(){};

        void setTunings(float kp, float ki, float kd);
        void reset() { lastError = 0; integral = 0; lastTime = 0; }
        
        /**
        @param current_time - current time in microseconds
        */
        float compute(unsigned long current_time, float target, float current);

        /**
        @param dt - current time in second
        */
        float compute(float dt, float target, float current);

        /**
        @param dt - current time in second
        */
        float compute(float dt, float target, float current, float derivative);

    private:
        float kp = 1, ki = 1/200, kd = 0.1;
        float lastError = 0;
        float integral = 0;
        float limit = 99999.0;
        unsigned long lastTime = 0;
};

#endif