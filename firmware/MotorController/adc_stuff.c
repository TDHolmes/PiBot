#include <stdint.h>

#include "MKL03Z4.h"
#include "fsl_adc16.h"


void init_adc(void)
{
    adc16_config_t adc_configuration;
    adc16_channel_config_t channel_config;

    /* Default Configuration:
    *     config->referenceVoltageSource     = kADC16_ReferenceVoltageSourceVref;
    *     config->clockSource                = kADC16_ClockSourceAsynchronousClock;
    *     config->enableAsynchronousClock    = true;
    *     config->clockDivider               = kADC16_ClockDivider8;
    *     config->resolution                 = kADC16_ResolutionSE12Bit;
    *     config->longSampleMode             = kADC16_LongSampleDisabled;
    *     config->enableHighSpeed            = false;
    *     config->enableLowPower             = false;
    *     config->enableContinuousConversion = false;
    */
    ADC16_GetDefaultConfig(&adc_configuration);
    ADC16_Init(ADC0, &adc_configuration);
    ADC16_SetChannelConfig(ADC0, 0, (const adc16_channel_config_t *)&channel_config);
}
