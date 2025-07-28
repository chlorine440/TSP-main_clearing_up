#include "tsp_isr.h"

volatile uint32_t sys_tick_counter=0;
volatile static uint32_t delay;
extern uint8_t flag_20_ms;
extern uint8_t rx_buffer[];
extern uint16_t rx_idx;
extern uint8_t rx_flag;
extern uint8_t RES_value;
extern uint8_t change;
extern uint16_t count_20ms;
extern int16_t encoder_pulse_qei1;
extern int16_t encoder_pulse_qei2;
extern int16_t encoder_speed_motor1;
extern int16_t encoder_speed_motor2;
void delay_1ms(uint32_t count)
{
	delay = count;
	while(0U != delay) {}
}

void SysTick_Handler()
{
	sys_tick_counter++;
	count_20ms++;
	if(0U != delay) {
		delay--;
	}
	if (!(sys_tick_counter % 20))
	{
		// LED_R_TOGGLE();
		flag_20_ms = 1;
		encoder_pulse_qei1 = TSP_QEI1_GetCount();
		encoder_pulse_qei2 = TSP_QEI2_GetCount();
		encoder_speed_motor1 = encoder_pulse_qei1 - 5000;
		encoder_speed_motor2 = encoder_pulse_qei2 - 5000;
		DL_Timer_setCounterValueAfterEnable(QEI_1_INST, 5000);
		DL_Timer_setCounterValueAfterEnable(QEI_2_INST, 5000);
	}
}
uint32_t get_systick_counter(void)
{
	return sys_tick_counter;
}
int mspm0_get_clock_ms(unsigned long *count)
{
    if (!count)
        return 1;
    count[0] = sys_tick_counter;
    return 0;
}
/*
void UART0_IRQHandler (void)
{
	switch(DL_UART_getPendingInterrupt(UART0))
	{
		case DL_UART_IIDX_TX:
		case DL_UART_IIDX_RX:
		default:
			break;
	}
	DL_UART_clearInterruptStatus(UART0, UART0->CPU_INT.RIS);
}

void UART1_IRQHandler (void)
{
	switch(DL_UART_getPendingInterrupt(UART1))
	{
		case DL_UART_IIDX_TX:
		case DL_UART_IIDX_RX:
		default:
			break;
	}
	DL_UART_clearInterruptStatus(UART1, UART1->CPU_INT.RIS);
}

void UART2_IRQHandler (void)
{
	switch(DL_UART_getPendingInterrupt(UART2))
	{
		case DL_UART_IIDX_TX:
		case DL_UART_IIDX_RX:
		default:
			break;
	}
	DL_UART_clearInterruptStatus(UART2, UART2->CPU_INT.RIS);
}

void UART3_IRQHandler (void)
{
	switch(DL_UART_getPendingInterrupt(UART3))
	{
		case DL_UART_IIDX_TX:
		case DL_UART_IIDX_RX:
		default:
			break;
	}
	DL_UART_clearInterruptStatus(UART3, UART3->CPU_INT.RIS);
}
*/

void UART6_IRQHandler(void)
{
    uint8_t data;
	static uint8_t uart_start = 0;
    if (!DL_UART_isRXFIFOEmpty(K230_INST))
    {
        data = (uint8_t)DL_UART_Main_receiveData(K230_INST);

        if (data == '<') {
            uart_start = 1;
			return ;
        } 
		if(uart_start == 1) {
			if (data == '>') {
				if (rx_idx < 255) {
					rx_buffer[rx_idx] = '\0';
					rx_flag = 1;
					uart_start = 0;
				}
				rx_idx = 0;
			} 
			else if (rx_idx < 255) {
				rx_buffer[rx_idx++] = data;
			}
		}
    }
}

void GROUP1_IRQHandler(void)
{
    if(DL_GPIO_getEnabledInterruptStatus(PORTA_PORT, PORTA_PHA0_PIN)) {
		if(PHA0()){      // rising edge on PHA0
			if(!PHB0()) 
				{RES_value++;change = 1;}// low on PHB0 -> CW
			else 
				{if(RES_value>0)RES_value--;change = 2;} // high on PHB0 -> CCW
		}
		else{             // falling edge on PHA0
			if(PHB0())
				{RES_value++;change = 1;}// high on PHB0 -> CW
			else 
				{if(RES_value>0)RES_value--;change = 2;} // low on PHB0 -> CCW
		}
		DL_GPIO_clearInterruptStatus(PORTA_PORT, PORTA_PHA0_PIN);
	}
}