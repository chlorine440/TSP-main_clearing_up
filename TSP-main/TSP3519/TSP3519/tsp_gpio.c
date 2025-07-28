#include "tsp_gpio.h"

//使能GPIO中断
void Enable_GPIOA_INT(void)
{
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
}
void Enable_GPIOB_INT(void)
{
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
}
void Enable_GPIOC_INT(void)
{
    NVIC_EnableIRQ(GPIOC_INT_IRQn);
}