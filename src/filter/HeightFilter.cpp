#include "filter/HeightFilter.hpp"
#include <math.h>


void HeightFilter::reset(float start_height)
{
    state[0] = start_height;
    state[1] = 0.0f;
    state[2] = 0.0f;
    state[3] = 0.0f;

    for (int row = 0; row < 4; row++)
    {
        for (int column = 0; column < 4; column++)
        {
            covariance[row][column] = 0.0f;
        }
    }
    covariance[0][0] = 1.0f; // 1 m
    covariance[1][1] = 1.0f; // 1 m/s
    covariance[2][2] = 0.1f; // about 0.3 m/s^2
    covariance[3][3] = 0.25f; // 0.5 m

    floor_offset = 0.0f;
    range_rejects_in_row = 0;
    baro_rejects_in_row = 0;
}

void HeightFilter::predict(float up_accel, float time_step)
{
    if (time_step <= 0.0f || time_step > 0.1f)
    {
        return; // bad time step
    }

    float half_step_squared = 0.5f * time_step * time_step;
    float accel = up_accel - state[2];
    state[0] += state[1] * time_step + half_step_squared * accel;
    state[1] += accel * time_step;

    // how each state moves with the others
    float move[4][4] =
    {
        { 1.0f, time_step, -half_step_squared, 0.0f },
        { 0.0f, 1.0f, -time_step, 0.0f },
        { 0.0f, 0.0f, 1.0f, 0.0f },
        { 0.0f, 0.0f, 0.0f, 1.0f },
    };

    // covariance = move * covariance * move turned
    float moved[4][4];
    for (int row = 0; row < 4; row++)
    {
        for (int column = 0; column < 4; column++)
        {
            float sum = 0.0f;
            for (int k = 0; k < 4; k++)
            {
                sum += move[row][k] * covariance[k][column];
            }
            moved[row][column] = sum;
        }
    }
    for (int row = 0; row < 4; row++)
    {
        for (int column = 0; column < 4; column++)
        {
            float sum = 0.0f;
            for (int k = 0; k < 4; k++)
            {
                sum += moved[row][k] * move[column][k];
            }
            covariance[row][column] = sum;
        }
    }

    // noise added each step
    float accel_variance = settings.accel_noise * settings.accel_noise;
    covariance[0][0] += accel_variance * half_step_squared * half_step_squared;
    covariance[0][1] += accel_variance * half_step_squared * time_step;
    covariance[1][0] += accel_variance * half_step_squared * time_step;
    covariance[1][1] += accel_variance * time_step * time_step;
    covariance[2][2] += settings.bias_walk * settings.bias_walk * time_step;
    covariance[3][3] += settings.baro_drift * settings.baro_drift * time_step;
}

float HeightFilter::innovation_variance(const float *measure_row, float noise) const
{
    float variance = noise * noise;
    for (int row = 0; row < 4; row++)
    {
        for (int column = 0; column < 4; column++)
        {
            variance += measure_row[row] * covariance[row][column] * measure_row[column];
        }
    }
    return variance;
}

void HeightFilter::correct(const float *measure_row, float difference, float variance)
{
    // covariance times the measure row
    float spread[4];
    for (int row = 0; row < 4; row++)
    {
        spread[row] = 0.0f;
        for (int k = 0; k < 4; k++)
        {
            spread[row] += covariance[row][k] * measure_row[k];
        }
    }

    float gain[4];
    for (int row = 0; row < 4; row++)
    {
        gain[row] = spread[row] / variance;
        state[row] += gain[row] * difference;
    }

    for (int row = 0; row < 4; row++)
    {
        for (int column = 0; column < 4; column++)
        {
            covariance[row][column] -= gain[row] * spread[column];
        }
    }

    // keep it symmetric
    for (int row = 0; row < 4; row++)
    {
        for (int column = row + 1; column < 4; column++)
        {
            float average = 0.5f * (covariance[row][column] + covariance[column][row]);
            covariance[row][column] = average;
            covariance[column][row] = average;
        }
    }
}

bool HeightFilter::update_range(float range_height, float range_noise)
{
    // range sees the height from the floor it started on
    static const float measure_row[4] = { 1.0f, 0.0f, 0.0f, 0.0f };

    float measured = range_height + floor_offset;
    float difference = measured - state[0];
    float variance = innovation_variance(measure_row, range_noise);

    if (settings.gate_sigma > 0.0f && difference * difference > settings.gate_sigma * settings.gate_sigma * variance)
    {
        // accel and BMP581 don't agree with this jump
        if (range_rejects_in_row == 0)
        {
            lowest_jump = difference;
            highest_jump = difference;
            jump_sum = 0.0f;
        }
        if (difference < lowest_jump) lowest_jump = difference;
        if (difference > highest_jump) highest_jump = difference;
        jump_sum += difference;
        range_rejects_in_row++;

        if (range_rejects_in_row >= settings.floor_step_samples)
        {
            if (highest_jump - lowest_jump <= settings.floor_step_spread_m)
            {
                // same jump every time, the floor under it moved
                floor_offset -= jump_sum / range_rejects_in_row;
            }
            range_rejects_in_row = 0;
        }
        return false;
    }

    range_rejects_in_row = 0;
    correct(measure_row, difference, variance);
    return true;
}

bool HeightFilter::update_baro(float baro_height)
{
    // BMP581 sees height - offset
    static const float measure_row[4] = { 1.0f, 0.0f, 0.0f, -1.0f };

    float difference = baro_height - (state[0] - state[3]);
    float variance = innovation_variance(measure_row, settings.baro_noise);

    if (settings.gate_sigma > 0.0f && difference * difference > settings.gate_sigma * settings.gate_sigma * variance &&
        baro_rejects_in_row < settings.max_baro_rejects)
    {
        baro_rejects_in_row++;
        return false;
    }

    baro_rejects_in_row = 0;
    correct(measure_row, difference, variance);
    return true;
}
