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


/* @return Arduino's VCC voltage in volts.
 *
 * Taken from EmonLib EnergyMonitor::readVcc()
 *
 * Thanks to http://hacking.majenko.co.uk/making-accurate-adc-readings-on-arduino
 * and Jérôme who alerted us to
 * http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
 */
float read_vcc() {

    // Read 1.1V reference against AVcc
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    delay(2); // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Convert
    while (bit_is_set(ADCSRA,ADSC))
        ; // measuring...

    uint16_t result = ADCL; // read least significant byte
    result |= ADCH << 8; // read most significant byte
    
    // Back-calculate AVcc in volts
    // 1.1V*1024 ADC steps http://openenergymonitor.org/emon/node/1186
    return 1126.4 / result;
}

float get_zero_offset(const float old_offset=517.1)
{
    // Find zero offset by taking an average over 1 second
    uint64_t accumulator = 0;
    unsigned int num_samples = 0,
                 num_wavelengths = 0;
    const uint32_t deadline = millis() + (1200 * 10);
    const unsigned int NUM_WAVELENGTHS_TO_SAMPLE = 50 * 10;

    float last_sample, sample;

    last_sample = analogRead(2);
    while (utils::in_future(deadline) && num_wavelengths < NUM_WAVELENGTHS_TO_SAMPLE) {

        sample = analogRead(2);

        // Test if we've just crossed zero in a positive-heading direction
        if (last_sample <= old_offset && sample > old_offset) {
            num_wavelengths++;
        }

        if (num_wavelengths) { // don't sample prior to passing the first zero-crossing
            num_samples++;
            accumulator += sample;
        }

        last_sample = sample;
    }

    const float mean = float(accumulator) / num_samples;
    Serial.print(mean);
    Serial.print(" ");
    Serial.print(num_samples);
    Serial.print(" ");
    Serial.println(num_wavelengths);

    return mean;
}

int main(void)
{
    init();
    setup();

    const float VCAL = 234.26;
    const float V_RATIO = VCAL * (read_vcc() / 1024.0);

    float filtered_v, last_filtered_v, sample_v;

    float zero_offset = get_zero_offset();

    int i_sample_v, last_i_sample_v;

    last_filtered_v = last_i_sample_v = analogRead(2) - zero_offset;

    // sample one complete wavelength
    while(true) {
        loop();

        i_sample_v = analogRead(2); // analogRead is blocking and takes ~0.1 milliseconds

        sample_v = i_sample_v - zero_offset;

        //filtered_v = 0.996*(last_filtered_v + (i_sample_v - last_i_sample_v));

        //last_filtered_v = filtered_v;
        //last_i_sample_v = i_sample_v;

        Serial.println(sample_v);
        //Serial.print(" ");
        //Serial.print(filtered_v);
        //Serial.print(" ");
        //Serial.println(sample_v - filtered_v);

    /*    if (sample_v < 10 && sample_v > -10) {
            break;
        }
        */
    }

    Serial.println(F("done"));
    Serial.end();
}
