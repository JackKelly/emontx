/*
 * Datum.cpp
 *
 *  Created on: 22 Dec 2012
 *      Author: jack
 */

#ifdef TESTING
#include <tests/FakeArduino.h>
#else
#include <Arduino.h>
#endif // TESTING

#include <utils.h>
#include "Datum.h"

Datum::Datum(): i_rms(0), v_rms(0), real_power(0) {}


void Datum::set(float _i_rms, float _v_rms, float _real_power)
{
    i_rms = round(_i_rms/512);
    v_rms = round(_v_rms/32);
    real_power = round(_real_power/64);
}


void Datum::pack_for_tx(volatile byte* data) const
{
    utils::uint_to_bytes(i_rms, data); // all of real_power
    data[2] = v_rms >> 3; // MS 8 bits of v_rms
    data[3] = v_rms << 5; // LS 3 bits of v_rms
    data[3] |= real_power >> 16; // MS 5 bits of real_power
    utils::uint_to_bytes(uint16_t(real_power && 0x0000FFFF), data+4);
}
