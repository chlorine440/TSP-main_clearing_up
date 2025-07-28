#ifndef TSP_ADC_H
#define TSP_ADC_H

#include "ti_msp_dl_config.h"
#include "tsp_gpio.h"

void ADC_Init(void);
int CCD1_Get_AO(uint16_t *value);
int CCD2_Get_AO(uint16_t *value);
int CCD3_Get_AO(uint16_t *value);
int CCD4_Get_AO(uint16_t *value);


#endif /* TSP_ADC_H */