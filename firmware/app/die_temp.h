/* RP2350 die temperature from the on-chip sensor (internal ADC input), for
 * the status report. One sensor for the whole chip, not one per core.
 *
 * Uncalibrated: the conversion uses the datasheet's typical figures (0.706 V
 * at 27 C, -1.721 mV/C) and takes the ADC reference as exactly 3.3 V, while
 * on the Pico 2 W it is the 3.3 V rail. Expect an offset of a few degrees;
 * good for trends and overheating, not as a thermometer.
 *
 * Core 0 only. Nothing else uses the ADC. Never select ADC input 3 (GP29):
 * on the Pico 2 W it is shared with the CYW43439's SPI.
 */
#ifndef DIE_TEMP_H
#define DIE_TEMP_H

void die_temp_init(void);

/* Mean of a few conversions (about 2 us each), in degrees C. */
float die_temp_read(void);

#endif
