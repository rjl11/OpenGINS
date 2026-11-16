#include "die_temp.h"

#include "hardware/adc.h"

#define SAMPLES 16u /* about 32 us in all */

void die_temp_init(void)
{
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(ADC_TEMPERATURE_CHANNEL_NUM);
}

float die_temp_read(void)
{
    adc_select_input(ADC_TEMPERATURE_CHANNEL_NUM);
    uint32_t sum = 0;
    for (uint32_t i = 0; i < SAMPLES; i++) {
        sum += adc_read();
    }
    const float v = (float)sum / (float)SAMPLES * (3.3f / 4096.0f);
    return 27.0f - (v - 0.706f) / 0.001721f; /* RP2350 datasheet, ADC temperature sensor */
}
