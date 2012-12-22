/*
 * Datum.h
 *
 *  Created on: 22 Dec 2012
 *      Author: jack
 */

#ifndef DATUM_H_
#define DATUM_H_

#include <stdint.h>

/**
 * Represents a single data point of calculated data.
 * Typically recorded once per second.
 */
class Datum
{
public:
    Datum();

    void set(float _i_rms, float _v_rms, float _real_power);

    /* data is an output variable. It should point to an 8-byte array. */
    void pack_for_tx(volatile uint8_t* data) const;

private:
    uint16_t i_rms; /* 0=0A, 1=1/512A, ..., FFFF=128A */

    uint16_t v_rms; /* 11bit
                     *   0=197V (230V-14%),
                     *   1=197 + 1/32V, ...,
                     * 7FF=261V (230V+14%) */

    uint32_t real_power; /* 21bit
                          *      0=0W,
                          *      1=1/64W, ...,
                          * 1FFFFF=32768W (142A at 230V) */
};


#endif /* DATUM_H_ */
