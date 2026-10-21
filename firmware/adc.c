#include "adc.h"
#include "board.h"
#include "clock.h"
#include "config.h"
#include "conv.h"

void adc_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    ADC1->CR1 = 0;
    /* 239.5 cycle sampling on every channel used: high source impedance
     * on PA4/PA5, and Vrefint needs >= 17.1 us. */
    ADC1->SMPR2 = (7u << ADC_SMPR2_SMP4_Pos) | (7u << ADC_SMPR2_SMP5_Pos);
    ADC1->SMPR1 = (7u << ADC_SMPR1_SMP17_Pos);
    ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_TSVREFE;
    delay_us(20);
    ADC1->CR2 |= ADC_CR2_RSTCAL;
    while (ADC1->CR2 & ADC_CR2_RSTCAL) { }
    ADC1->CR2 |= ADC_CR2_CAL;
    while (ADC1->CR2 & ADC_CR2_CAL) { }
    ADC1->CR2 |= ADC_CR2_EXTSEL | ADC_CR2_EXTTRIG;   /* EXTSEL = 111: SWSTART */
}

static uint32_t read_avg(uint32_t ch, uint32_t n)
{
    uint32_t sum = 0;
    ADC1->SQR3 = ch;
    for (uint32_t i = 0; i < n; i++) {
        ADC1->CR2 |= ADC_CR2_SWSTART;
        while (!(ADC1->SR & ADC_SR_EOC)) { }
        sum += ADC1->DR;
    }
    return (sum + n / 2u) / n;
}

void adc_update(adc_values_t *v)
{
    uint32_t vref = read_avg(ADC_CH_VREFINT, 4);
    v->vdd_mv = conv_vdd_from_vrefint(vref);
    int32_t scale = USE_VREFINT_FOR_VDD ? v->vdd_mv : VDD_NOMINAL_MV;

    v->uvlo_pin_mv = conv_adc_to_mv(read_avg(ADC_CH_VIN, 8), scale);
    v->vin_mv      = conv_vin_from_pin(v->uvlo_pin_mv);
    v->icm_ma      = conv_icm_from_pin(conv_adc_to_mv(read_avg(ADC_CH_ICM, 8), scale));
}
