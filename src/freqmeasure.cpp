#include "freqmeasure.h"
/**
 * @brief set imd duty cycle 
 * 
 * @param duty
 */
void FreqReading::set_duty_cycle(const float duty)
{
    this->duty_cycle = duty;
}
/**
 * @brief set imd pwm frequency (hertz)
 * 
 * @param frequency 
 */
void FreqReading::set_imd_frequency(const float frequency)
{
    this->imd_frequency = frequency;
}
/**
 * @brief get the current IMD pwm duty cycle 
 * 
 * @return float 
 */
float FreqReading::get_duty_cycle()
{
    return this->duty_cycle;
}
/**
 * @brief get the current imd pwm frequency in Hz
 * 
 * @return float 
 */
float FreqReading::get_imd_frequency()
{
    return this->imd_frequency;
}
/**
 * @brief main task to reading IMD sense pin and update measurements
 *
 */
void FreqReading::update_readings()
{
    // Get IMD read pin status and invert (the signal is inverted when ACU reads it)
    // Serial.println("Entering IMD reading check");
    // Only start doing things after a few secs of ACU ontime
    uint8_t reading = (digitalRead(this->imd_read_pin));
    // If rising edge:
    if (reading == 1 && this->last_reading == 0)
    {
        this->low_ms = this->lowtime;
        // Capture total ontime
        this->total_ms = this->ontime;
        // Calculate duty cycle
        // Duty cycle is inverted because the ACU reads it through an inverting buffer
        this->set_duty_cycle(1-(this->high_ms / this->total_ms));

        // Calculate period
        float total_s = total_ms / 1000;
        float calculated_frequency = 1 / total_s;
        this->set_imd_frequency(calculated_frequency);
        // reset stats
        this->ontime = 0;
        this->hightime = 0;
        this->last_reading = reading;
    }
    // If no change
    if (reading == 1 && this->last_reading == 1)
    {
        // Do nothing when state is the same
    }
    // If falling edge
    if (reading == 0 && this->last_reading == 1)
    {

        // Capture total high time
        this->high_ms = this->hightime;
        this->lowtime = 0;
        this->last_reading = reading;
    }
    if (reading == 0 && this->last_reading == 0)
    {
        // Do nothing when state the same
    }
}