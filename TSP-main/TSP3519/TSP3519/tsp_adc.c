#include "tsp_adc.h"

volatile bool ADC1_flag = false, ADC2_flag = false; // ADC conversion flag
volatile uint16_t ADC1_value = 0, ADC2_value = 0; // ADC value
/**
 * @brief Initialize ADC for CCD channels
 */
void ADC_Init(void) {//已经在SYSCFG_DL_init中调用，不需要单独调用
    // Initialize ADC sequence sampling for CCD
    
}

/**
 * @brief Read and return the averaged ADC value from CCD channels.
 *        Triggers a conversion, waits for completion, retrieves the 4-channel sequence,
 *        and returns their average.
 * @return Averaged 12-bit ADC value (0 - 4095)
//  */

int CCD1_Get_AO(uint16_t *value){
    DL_ADC12_startConversion(CCD_INST); // Start ADC conversion
    
    while(!ADC1_flag); // Wait for ADC conversion to complete
    ADC1_flag = false; // Reset flag
    *value = DL_ADC12_getMemResult(CCD_INST,CCD_ADCMEM_CCD1_AO);    //12bit, 0-4095
    DL_ADC12_enableConversions(CCD_INST); // Enable further conversions
}
int CCD2_Get_AO(uint16_t *value){
    DL_ADC12_startConversion(CCD_INST); // Start ADC conversion
    
    while(!ADC2_flag); // Wait for ADC conversion to complete
    ADC2_flag = false; // Reset flag
    *value = DL_ADC12_getMemResult(CCD_INST,CCD_ADCMEM_CCD2_AO);    //12bit, 0-4095
    DL_ADC12_enableConversions(CCD_INST); // Enable further conversions
}



void ADC1_IRQHandler(void){
    // ADC1_IRQHandler();
    switch(DL_ADC12_getPendingInterrupt(CCD_INST)) {
        case DL_ADC12_IIDX_MEM0_RESULT_LOADED:
            // Sequence conversion complete
            ADC1_flag = true;
            DL_ADC12_clearInterruptStatus(CCD_INST, DL_ADC12_IIDX_MEM0_RESULT_LOADED);
            break;
        case DL_ADC12_IIDX_MEM1_RESULT_LOADED:
            // Memory conversion complete
            ADC2_flag = true;
            DL_ADC12_clearInterruptStatus(CCD_INST, DL_ADC12_IIDX_MEM1_RESULT_LOADED);
            break;
        default:
            // Other interrupts
            break;
    }
    // Clear interrupt flag
    
}