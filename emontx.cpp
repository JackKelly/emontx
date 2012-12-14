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


class Waveform
{
public:
    Waveform(float _calibration, float _phasecal, byte _pin)
    : calibration(_calibration), sum(0), phasecal(_phasecal), num_samples(0), pin(_pin)
    {}

    void init()
    {
        zero_offset = get_zero_offset();
        prime();
    }

    /**
     * Call this if we haven't taken a sample for a while.
     * Prime the high pass filter.
     */
    void prime()
    {
        Serial.print(F("priming high pass filter..."));
        sum = 0;
        num_samples = 0;

        /* Use zero_offset to "prime" the HPF parameters */
        last_sample = analogRead(pin);
        last_filtered = last_sample - zero_offset;

        /* "Dry run" the HPF for half a second to let it stabilise */
        const uint32_t deadline = millis() + 500;
        while (utils::in_future(deadline)) {
            take_sample();
            filter();
        }
        Serial.println(F(" done priming."));
    }

    void take_sample() { sample = analogRead(pin); }

    void process()
    {
        filter();

        /* Update variables used in RMS calculation */
        sum += (filtered * filtered);
        num_samples++;
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
        sum = 0;
        num_samples = 0;
        return rms;
    }

private:
    float calibration, filtered, last_filtered, zero_offset, sum, phasecal;
    int sample, last_sample;
    uint16_t num_samples;
    byte pin;

    /**
     * Calculate the zero offset by averaging over 1 second of raw samples.
     */
    float get_zero_offset()
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

    void filter()
    {
        filtered = 0.996*(last_filtered + (sample - last_sample));
        /* line above takes ~0.2 milliseconds
         * I should experiment with integer maths
         * http://openenergymonitor.org/emon/node/932
         * and look into calypso_rae's other ideas:
         * http://openenergymonitor.org/emon/node/841 */

        last_sample = sample;
        last_filtered = filtered;
    }

};

int main(void)
{
    init();
    setup();

    Serial.println(get_vcc());

    Waveform v(234.26, 1.7, 2);
    v.init();

    uint32_t deadline = millis() + 1000;
    while(true) {
        if (utils::in_future(deadline)) {
            v.take_sample();
            v.process();
        } else {
            Serial.println(v.get_rms_and_reset());
            deadline = millis() + 1000;
        }
    }

    Serial.println(F("done"));
    Serial.end();
}
