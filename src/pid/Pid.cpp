#include "pid/Pid.hpp"


static constexpr float TWO_PI = 6.28318531f;


float Pid::limit_value(float value, float limit)
{
    if (limit <= 0.0f) return value;
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

void Pid::set_gains(float proportional_gain, float integral_gain, float derivative_gain)
{
    settings.proportional_gain = proportional_gain;
    settings.integral_gain = integral_gain;
    settings.derivative_gain = derivative_gain;

    // integral_part already has the old I gain in it, start it again
    integral_part = 0.0f;
}

void Pid::set_settings(const Settings &new_settings)
{
    settings = new_settings;
    reset();
}

void Pid::reset()
{
    proportional_part = 0.0f;
    integral_part = 0.0f;
    derivative_part = 0.0f;
    output = 0.0f;
    filtered_derivative = 0.0f;
    previous_measured = 0.0f;
    has_previous_measured = false;
}

float Pid::calculate(float target, float measured, float time_step)
{
    if (!(time_step > 0.0f)) return output; // also catches NaN

    float measured_rate = 0.0f;
    if (has_previous_measured)
    {
        measured_rate = (measured - previous_measured) / time_step;
    }
    previous_measured = measured;
    has_previous_measured = true;

    return run(target - measured, measured_rate, time_step);
}

float Pid::calculate_with_rate(float target, float measured, float measured_rate, float time_step)
{
    if (!(time_step > 0.0f)) return output;

    previous_measured = measured;
    has_previous_measured = true;

    return run(target - measured, measured_rate, time_step);
}

float Pid::run(float error, float measured_rate, float time_step)
{
    proportional_part = settings.proportional_gain * error;

    if (!integral_on_hold && settings.integral_gain != 0.0f)
    {
        integral_part += settings.integral_gain * error * time_step;
        integral_part = limit_value(integral_part, settings.integral_limit);
    }

    // D on the measurement, so moving the stick does not kick it
    float raw_derivative = -settings.derivative_gain * measured_rate;
    if (settings.derivative_filter_hz > 0.0f)
    {
        float time_constant = 1.0f / (TWO_PI * settings.derivative_filter_hz);
        float filter_amount = time_step / (time_step + time_constant);
        filtered_derivative += filter_amount * (raw_derivative - filtered_derivative);
        derivative_part = filtered_derivative;
    }
    else
    {
        derivative_part = raw_derivative;
    }

    float total = proportional_part + integral_part + derivative_part;

    // NaN never goes to the motors
    if (total != total)
    {
        reset();
        return 0.0f;
    }

    output = limit_value(total, settings.output_limit);
    return output;
}
