#include "imd.h"
void imd::updateInsulationReading(const uint8_t dutyCycle)
{
    uint8_t duty = dutyCycle;
    if (duty < 5)
    {
        duty = 5;
    }
    else if (duty > 95)
    {
        duty = 95;
    }
    uint16_t rf = 0;
    if (duty == 5)
    {
        rf = 50000; // kiloOhms, equals 50Mohm
    }
    else if (duty == 95)
    {
        rf = 0;
    }
    else
    {
        rf = ((0.9 * 1200) / (duty - 5)) - 1200;
    }
    this->insulationOhms = rf;
    
};
void imd::updateState(const uint8_t frequency)
{
    float freq = round(static_cast<float>(frequency/10.0)) * 10;
    uint8_t roundedFreq = static_cast<uint8_t>(freq);
    this->state = roundedFreq/10;
};

uint16_t imd::getInsulationgReading()
{
    return this->insulationOhms;
}
uint8_t imd::getState()
{
    return this->state;
}