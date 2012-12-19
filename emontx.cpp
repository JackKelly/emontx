/*
 * emontx.cpp
 *
 *  Created on: 14 Dec 2012
 *      Author: jack
 */

#include <Arduino.h>
#include <utils.h>

void setup()
{
    Serial.begin(115200);
    Serial.println(F("EmonTX"));
}


void loop()
{
}


/* @return Calibration parameter based on VCC
 *
 * Adapted from EmonLib EnergyMonitor::readVcc()
 *
 * Thanks to http://hacking.majenko.co.uk/making-accurate-adc-readings-on-arduino
 * and Jérôme who alerted us to
 * http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
 */
float get_vcc_ratio() {

    // Read 1.1V reference against AVcc
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    delay(2); // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Convert
    while (bit_is_set(ADCSRA,ADSC))
        ; // measuring...

    uint16_t result = ADCL; // read least significant byte
    result |= ADCH << 8; // read most significant byte
    
    // 1.1V reference
    return 1.1 / result;
}


/**
 * @return VCC in volts
 */
float get_vcc() {
    // 1024 ADC steps http://openenergymonitor.org/emon/node/1186
    return get_vcc_ratio() * 1024;
}

/**
 * A class which abstracts the functionality required for sampling and processing
 * current and voltage waveforms.
 */
class Waveform
{
public:
    Waveform(float _calibration, byte _pin)
    : calibration(_calibration), sum(0), phasecal(1), num_samples(0),
      num_consecutive_equal(0), pin(_pin)
    {
        for (uint8_t i=0; i<3; i++) {
            zero_crossing_times[i] = 0;
        }
    }

    void set_phasecal(float _phasecal) { phasecal=_phasecal; }

    /**
     * Call this if we haven't taken a sample for a while.
     * Prime the LTAD algorithm.
     */
    void prime()
    {
        Serial.print(F("priming LTAD..."));
        sum = 0;
        num_samples = 0;

        /* Use average of 1 second worth of samples to "boot strap" the LTAD algorithm */
        zero_offset = calc_zero_offset();

        /* Figure out which lobe we're in at the moment */
        sample = analogRead(pin);
        in_positive_lobe = last_sample > zero_offset;

        /* "Dry run" LTAD for half a second to let it stabilise */
        const uint32_t deadline = millis() + 500;
        while (utils::in_future(deadline)) {
            take_sample();
            process();
        }

        /* Reset counters and accumulators after dry run */
        reset();

        Serial.println(F(" done priming."));
    }

    void take_sample()
    {
        last_sample = sample;
        sample = analogRead(pin);
        num_samples++;
    }

    void process()
    {
        if (just_crossed_zero()) {
            update_zero_offset();
        }

        /* If we're consuming no current then the CT clamp's
         * zero offset might drift but, because we're consuming no
         * current, we never cross zero so update_zero_offset()
         * will not be called to correct zero_offset.  So we need
         * to explicitly check if we're consuming no current by
         * checking if multiple consecutive raw samples have the
         * same value.  If they do then adjust zero_offset so
         * we correctly report a filtered value of zero. */
        if (sample == last_sample) {
            num_consecutive_equal++;

            if (num_consecutive_equal > 50 &&
                    !utils::roughly_equal<float>(zero_offset, float(sample), 1.0)) {
                zero_offset += (sample - zero_offset);
            }
        } else {
            num_consecutive_equal = 0;
        }

        /* Remove zero offset */
        last_filtered = filtered;
        filtered = sample - zero_offset;

        /* Save samples for printing to serial port later */
        if (num_samples < MAX_NUM_SAMPLES) {
            samples[num_samples] = sample;
        }

        /* Update variables used in RMS calculation */
        sum += (filtered * filtered);

        /* TODO: I should experiment with integer maths
         * http://openenergymonitor.org/emon/node/1629
         * and look into calypso_rae's other ideas:
         * http://openenergymonitor.org/emon/node/841 */
    }

    const float& get_filtered() { return filtered; }

    float get_phase_shifted()
    {
        return last_filtered + phasecal * (filtered - last_filtered);
    }

    float get_rms_and_reset()
    {
        const float ratio = calibration * get_vcc_ratio();
        const float rms = ratio * sqrt(sum / num_samples);
        reset();
        return rms;
    }

    /* Reset counters and accumulators. */
    void reset()
    {
        sum = 0;
        num_samples = 0;
        num_consecutive_equal = 0;
    }

    const uint16_t& get_num_samples() { return num_samples; }

    const float& get_calibration() { return calibration; }

    void print_samples()
    {
        for (uint16_t i=0; i<MAX_NUM_SAMPLES; i++) {
            Serial.println(samples[i]);
        }
    }

    const float& get_zero_offset() { return zero_offset; }

private:
    float calibration, filtered, last_filtered, zero_offset, sum, phasecal;
    bool in_positive_lobe;
    int sample, last_sample;
    uint16_t num_samples, num_consecutive_equal;
    uint32_t zero_crossing_times[3]; /* [0] is two zero crossings ago, [1] is prev zero crossing */
    byte pin;
    static const uint16_t MAX_NUM_SAMPLES = 200; /* Just used for printing raw samples */
    float samples[MAX_NUM_SAMPLES]; /* Just used for printing raw samples */

    /**
     * Calculate the zero offset by averaging over 1 second of raw samples.
     */
    float calc_zero_offset()
    {
        const uint8_t SECONDS_TO_SAMPLE = 1;
        const uint32_t deadline = millis() + (1000 * SECONDS_TO_SAMPLE);
        uint32_t accumulator = 0;
        uint16_t num_samples = 0;

        while (utils::in_future(deadline)) {
            num_samples++;
            accumulator += analogRead(pin);
        }

        return float(accumulator) / num_samples;
    }

    /**
     * Return true if we've just crossed zero_offset.
     * Also sets in_positive_lobe according to the lobe
     * we're just entering.
     */
    bool just_crossed_zero()
    {
        bool just_crossed_zero = false;

        if (last_sample <= zero_offset && sample > zero_offset) {
            just_crossed_zero = true;
            in_positive_lobe = true;
        } else if (last_sample >= zero_offset && sample < zero_offset) {
            just_crossed_zero = true;
            in_positive_lobe = false;
        }

        return just_crossed_zero;
    }

    /**
     * Call this just after crossing zero to update zero_offset.
     */
    void update_zero_offset()
    {
        zero_crossing_times[2] = micros();

        if (zero_crossing_times[0] && zero_crossing_times[1]) { // check these aren't zero
            uint32_t lobe_durations[2]; // TODO: don't need to recalc penultimate lobe duration every time
            lobe_durations[0] = zero_crossing_times[1] - zero_crossing_times[0]; // penultimate lobe
            lobe_durations[1] = zero_crossing_times[2] - zero_crossing_times[1]; // previous lobe

            if (!utils::roughly_equal<uint32_t>(lobe_durations[0], lobe_durations[1], 10)) {
                if (in_positive_lobe) { // just entered positive lobe
                    if (lobe_durations[0] > lobe_durations[1]) { // [0] is +ve, [1] is -ve
                        zero_offset+= 0.1;
                    } else {
                        zero_offset-= 0.1;
                    }
                } else { // just entered negative lobe
                    if (lobe_durations[0] > lobe_durations[1]) { // [0] is -ve, [1] is +ve
                        zero_offset-= 0.1;
                    } else {
                        zero_offset+= 0.1;
                    }

                }
            }
        }

        /* Shuffle zero_crossing_times back 1 */
        for (uint8_t i=0; i<2; i++) {
            zero_crossing_times[i] = zero_crossing_times[i+1];
        }
    }

};

int main(void)
{
    init();
    setup();

    float sum_p = 0, vcc_ratio, real_power, apparent_power, v_rms, i_rms;

    Serial.println(get_vcc());

    /*Waveform v(234.26, 2);
    v.set_phasecal(1.7);
    v.prime();*/

    Waveform i(110.0, 3);
    i.prime();

    //const float both_calibrations = i.get_calibration() * v.get_calibration();

    uint32_t deadline = millis() + 1000;
    while(true) {
        if (utils::in_future(deadline)) {
            // v.take_sample();
            i.take_sample();

            // v.process();
            i.process();

            // sum_p += v.get_phase_shifted() * i.get_filtered();
        } else {
            i.print_samples();

            Serial.print("N=");
            Serial.print(i.get_num_samples());

            /*vcc_ratio = get_vcc_ratio();
            real_power = both_calibrations * vcc_ratio * vcc_ratio * sum_p / v.get_num_samples();
            if (real_power < 0) real_power = 0;
            Serial.print(" real=");
            Serial.print(real_power);

            v_rms = v.get_rms_and_reset(); */
            i_rms = i.get_rms_and_reset();
            /*Serial.print(" vRMS=");
            Serial.print(v_rms);*/
            Serial.print(" iRMS=");
            Serial.print(i_rms);

            Serial.print(" ZO=");
            Serial.println(i.get_zero_offset());
/*
            apparent_power = v_rms * i_rms;
            Serial.print(" apparent=");
            Serial.print(apparent_power);

            Serial.print(" PF=");
            Serial.println(apparent_power > 0 ? real_power / apparent_power : 1.0);
*/
            deadline = millis() + 1000;
            sum_p = 0;
        }
    }

    Serial.println(F("done"));
    Serial.end();
}
