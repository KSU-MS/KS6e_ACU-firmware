#include <Metro.h>
#include <FreqMeasureMulti.h>
class FreqReading
{

private:
    elapsedMillis ontime;
    float total_ms;
    elapsedMillis hightime;
    float high_ms;
    elapsedMillis lowtime;
    float low_ms;
    float duty_cycle;
    float imd_frequency;
    const uint8_t imd_read_pin = 7;
    uint8_t last_reading;

public:
    void set_duty_cycle(const float duty);
    void set_imd_frequency(const float frequency);
    float get_duty_cycle();
    float get_imd_frequency();
    void update_readings();
};
