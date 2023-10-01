/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  simple model of a servo. Model is:

    - time delay for transport protocol delay
    - slew limit
    - 2-pole butterworth
*/

#include "ServoModel.h"
#include "SITL.h"

float ServoModel::apply_filter(float v)
{
    const auto *sitl = AP::sitl();
    if (!sitl) {
        return v;
    }

    // apply delay
    if (sitl->servo_delay > 0) {
        uint32_t delay_len = MAX(1,sitl->servo_delay * sitl->loop_rate_hz);
        if (!delay) {
            delay = new ObjectBuffer<float>();
        }
        if (delay->get_size() != delay_len) {
            delay->set_size(delay_len);
        }
        while (delay->space() > 0) {
            delay->push(v);
        }
        IGNORE_RETURN(delay->pop(v));
    }

    // apply slew limit
    if (sitl->servo_speed > 0) {
        uint32_t now = AP_HAL::micros();
        float dt = (now - last_update_us) * 1.0e-6;
        last_update_us = now;
        // assume SIM_SERVO_SPEED is time for 60 degrees
        float time_per_degree = sitl->servo_speed / 60.0;
        float proportion_per_second = 1.0 / (angle_deg * time_per_degree);
        delta_max = proportion_per_second * dt;
        v = constrain_float(v, last_v-delta_max, last_v+delta_max);
        v = constrain_float(v, -1, 1);
        last_v = v;
    }

    // apply filter
    if (sitl->servo_filter > 0) {
        filter.set_cutoff_frequency(sitl->loop_rate_hz, sitl->servo_filter);
        v = filter.apply(v);
    }

    return v;
}

float ServoModel::filter_range(uint16_t pwm)
{
    const float v = constrain_float((pwm - pwm_min)/float(pwm_max - pwm_min), 0, 1);
    return apply_filter(v);
}

float ServoModel::filter_angle(uint16_t pwm)
{
    const float v = constrain_float((pwm - 0.5*(pwm_max+pwm_min))/(0.5*float(pwm_max - pwm_min)), -1, 1);
    return apply_filter(v);
}

void ServoModel::set_pwm_range(uint16_t _pwm_min, uint16_t _pwm_max)
{
    pwm_min = _pwm_min;
    pwm_max = _pwm_max;
}

void ServoModel::set_deflection(float _angle_deg)
{
    angle_deg = fabsf(_angle_deg);
}
