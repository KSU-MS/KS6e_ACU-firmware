#ifndef IMD_H
#define IMD_H
#include <stdint.h>
#include <cmath>
class imd
{
    private:
        uint8_t state; 
        uint16_t insulationOhms;
    public:
        void updateInsulationReading(const uint8_t dutyCycle);
        void updateState(const uint8_t frequency);
        uint16_t getInsulationgReading();
        uint8_t getState();
};
#endif