#include "tsp_battery.h"
#include "ti_msp_dl_config.h"
volatile uint16_t ADC0_flag = 0;



void ADC0_init(void) {
    // Initialize ADC for battery voltage measurement
    NVIC_EnableIRQ(ADC0_INT_IRQn); // Enable ADC interrupt
}
float tsp_battery_voltage(void)
{
    // 启动ADC转换
    DL_ADC12_startConversion(BATTERY_INST); // Start ADC conversion
    while(!ADC0_flag); // 等待ADC转换完成
    ADC0_flag = false; // 重置标志
    uint16_t adc_value = DL_ADC12_getMemResult(BATTERY_INST, BATTERY_ADCMEM_Vbat);
    DL_ADC12_enableConversions(BATTERY_INST); // 启用进一步的转换
    return adc_value/4096.0f * 3.3f * 5.0f;
}











void ADC0_IRQHandler(void){
    // ADC1_IRQHandler();
    switch(DL_ADC12_getPendingInterrupt(BATTERY_INST)) {
        case DL_ADC12_IIDX_MEM0_RESULT_LOADED:
            // Sequence conversion complete
            ADC0_flag = true;
            DL_ADC12_clearInterruptStatus(BATTERY_INST, DL_ADC12_IIDX_MEM0_RESULT_LOADED);
            break;
        default:
            // Other interrupts
            break;
    }
    // Clear interrupt flag
    
}