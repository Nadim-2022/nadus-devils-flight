#ifndef HEIGHT_FILTER_HPP
#define HEIGHT_FILTER_HPP


#include <stdint.h>


// Kalman filter for the height, 4 states
// height (m), climb speed (m/s), accel bias (m/s^2), BMP581 offset (m)
// range sees the height, BMP581 sees height - offset
// so the BMP581 drift is learned while the range is good
// and kept when the range is gone
//
// Range jumps the accel and BMP581 don't agree with (table, wall, step)
// are thrown away, if the jump stays it becomes the new floor level

class HeightFilter
{

    public:

        struct Settings
        {
            float accel_noise = 0.1f; // m/s^2
            float bias_walk = 0.001f; // accel bias change, m/s^2 per sqrt(s)
            float baro_noise = 0.3f; // m
            float baro_drift = 0.02f; // BMP581 offset change, m per sqrt(s)
            float gate_sigma = 4.0f; // further off than this = rejected, 0 = off
            uint32_t max_baro_rejects = 10; // then taken anyway
            uint32_t floor_step_samples = 20; // range rejected this many in a row = floor moved
            float floor_step_spread_m = 0.15f; // those jumps have to agree this close
        };

        HeightFilter() { reset(0.0f); }
        explicit HeightFilter(const Settings &new_settings) : settings(new_settings) { reset(0.0f); }

        void reset(float start_height);
        void predict(float up_accel, float time_step);

        // false = rejected
        bool update_range(float range_height, float range_noise);
        bool update_baro(float baro_height);

        float get_height() const { return state[0]; }
        float get_climb_speed() const { return state[1]; }
        float get_accel_bias() const { return state[2]; }
        float get_baro_offset() const { return state[3]; }
        float get_floor_offset() const { return floor_offset; } // how much the range floor has moved
        uint32_t get_range_rejects() const { return range_rejects_in_row; }

        void set_settings(const Settings &new_settings) { settings = new_settings; }
        const Settings &get_settings() const { return settings; }


    private:

        float innovation_variance(const float *measure_row, float noise) const;
        void correct(const float *measure_row, float difference, float variance);

        Settings settings;

        float state[4];
        float covariance[4][4];

        float floor_offset = 0.0f;
        uint32_t range_rejects_in_row = 0;
        float lowest_jump = 0.0f;
        float highest_jump = 0.0f;
        float jump_sum = 0.0f;
        uint32_t baro_rejects_in_row = 0;

};

#endif // HEIGHT_FILTER_HPP
