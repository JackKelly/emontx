/*
 * Manager.cpp
 *
 *  Created on: 22 Dec 2012
 *      Author: jack
 */

#ifdef TESTING
#include <tests/FakeArduino.h>
#else
#include <Arduino.h>
#endif // TESTING

#include "Manager.h"
#include <vcc.h>
#include <utils.h>

Manager::Manager()
: v(234.26, 2, 1.7), i(110.0, 3),
  both_calibrations(i.get_calibration() * v.get_calibration())
{}


void Manager::init()
{
    i.prime();
    v.prime();
}


void Manager::run()
{
    float sum_p = 0, vcc_ratio, real_power, apparent_power, v_rms, i_rms;
    const uint32_t deadline = millis() + 1000;

    while (utils::in_future(deadline)) {
        v.take_sample();
        i.take_sample();

        v.process();
        i.process();

        sum_p += v.get_phase_shifted() * i.get_filtered();

        // TODO: check for RF data
    }

    Serial.print("N=");
    Serial.print(i.get_num_samples());

    vcc_ratio = get_vcc_ratio();
    real_power = both_calibrations * vcc_ratio * vcc_ratio * sum_p / v.get_num_samples();
    if (real_power < 0) real_power = 0;
    Serial.print(" real=");
    Serial.print(real_power);

    v_rms = v.get_rms_and_reset();
    i_rms = i.get_rms_and_reset();
    Serial.print(" vRMS=");
    Serial.print(v_rms);
    Serial.print(" iRMS=");
    Serial.print(i_rms);

    Serial.print(" ZO=");
    Serial.print(i.get_zero_offset());

    apparent_power = v_rms * i_rms;
    Serial.print(" apparent=");
    Serial.print(apparent_power);

    Serial.print(" PF=");
    Serial.println(apparent_power > 0 ? real_power / apparent_power : 1.0);
}
