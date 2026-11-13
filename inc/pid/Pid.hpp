#ifndef PID_HPP
#define PID_HPP


// PID controller used by the flight loop
// D is taken from the measurement (no jump when the target moves)
// a limit of 0 means no limit

class Pid
{

    public:

        struct Settings
        {
            float proportional_gain = 0.0f;
            float integral_gain = 0.0f;
            float derivative_gain = 0.0f;
            float integral_limit = 0.0f; // max size of the I part
            float output_limit = 0.0f; // max size of the output
            float derivative_filter_hz = 0.0f; // low pass on the D part, 0 = off
        };

        Pid() = default;
        explicit Pid(const Settings &new_settings) : settings(new_settings) {}

        // D from the change in the measured value
        float calculate(float target, float measured, float time_step);

        // D from a rate we already have (gyro)
        float calculate_with_rate(float target, float measured, float measured_rate, float time_step);

        void reset();
        void hold_integral(bool hold) { integral_on_hold = hold; }

        void set_gains(float proportional_gain, float integral_gain, float derivative_gain);
        void set_settings(const Settings &new_settings);
        const Settings &get_settings() const { return settings; }

        float get_proportional_part() const { return proportional_part; }
        float get_integral_part() const { return integral_part; }
        float get_derivative_part() const { return derivative_part; }
        float get_output() const { return output; }


    private:

        float run(float error, float measured_rate, float time_step);
        static float limit_value(float value, float limit);

        Settings settings;

        float proportional_part = 0.0f;
        float integral_part = 0.0f;
        float derivative_part = 0.0f;
        float output = 0.0f;

        float previous_measured = 0.0f;
        float filtered_derivative = 0.0f;
        bool has_previous_measured = false;
        bool integral_on_hold = false;

};

#endif // PID_HPP
