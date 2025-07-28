// TSP_CCD.c
#include "TSP_CCD.h"

ccd_t ccd_data_raw, ccd_data_old; // CCD 数据缓存
uint8_t ccd_index;
// 简单空循环延时，80 次大约 1μs（80MHz 时钟）
void tsp_ccd_delay_1us(uint8_t us)
{
    volatile uint16_t cnt = 80 * us; // 1μs 延时
    while (cnt--) { asm("NOP"); }
}


void tsp_ccd_init(void)
{
    NVIC_EnableIRQ(ADC1_INT_IRQn); // enable ADC interrupt
}

/** 仅拉一次 SI 脉冲，复位电荷 */
static void tsp_ccd_trigger_SI(void)
{
    // SI = 1
    CCD_SI1_HIGH;
    CCD_SI2_HIGH;
    tsp_ccd_delay_1us(1);
    // SI = 0
    CCD_SI1_LOW;
    CCD_SI2_LOW;
}

/** 拉一次 CLK 脉冲，用于读出/迭代像素 */
static void tsp_ccd_pulse_CLK(void)
{
    CCD_CLK1_HIGH;
    CCD_CLK2_HIGH;
    tsp_ccd_delay_1us(1);
    CCD_CLK1_LOW;
    CCD_CLK2_LOW;
    tsp_ccd_delay_1us(1);
}

void tsp_ccd_flush(void)
{
    // 1. 复位脉冲
    CCD_CLK1_HIGH;CCD_CLK2_HIGH;
    tsp_ccd_delay_1us(1);
    CCD_SI1_LOW;CCD_SI2_LOW; // 若使用双 SI，则同时拉高 SI2
    tsp_ccd_delay_1us(2);

    CCD_CLK1_LOW;CCD_CLK2_LOW; // 若使用双 TAP，则同时拉低 CLK2
    tsp_ccd_delay_1us(1);
    CCD_SI1_HIGH;CCD_SI2_HIGH; // 若使用双 SI，则同时拉高 SI2
    tsp_ccd_delay_1us(30);

    CCD_CLK1_HIGH;CCD_CLK2_HIGH; // 若使用双 TAP，则同时拉低 CLK2
    tsp_ccd_delay_1us(1);
    CCD_SI1_HIGH;CCD_SI2_HIGH; // 若使用双 SI，则同时拉高 SI2
    tsp_ccd_delay_1us(2);
    
    CCD_CLK1_HIGH;CCD_CLK2_HIGH; // 若使用双 TAP，则同时拉低 CLK2
    tsp_ccd_delay_1us(1);
    CCD_SI1_LOW;CCD_SI2_LOW; // 若使用双 SI，则同时拉高 SI2
    tsp_ccd_delay_1us(2);
    
    // 3. 再来 128 个时钟，丢弃管脚上的电荷输出
    for (uint16_t i = 0; i < CCD_PIXEL_COUNT; i++) {
        tsp_ccd_pulse_CLK();
    }
}

bool tsp_ccd_snapshot(ccd_t buf)
{
    uint16_t val;

    // 1) 清空上一帧
    tsp_ccd_flush();

    // 2) 等待新帧积分（根据环境光强度调整 ms）
    delay_1ms(10);

    // 3) 触发 SI, CLK 初始脉冲，开始读第 0 像素
    CCD_CLK1_HIGH;CCD_CLK2_HIGH;
    tsp_ccd_delay_1us(1);
    CCD_SI1_LOW;CCD_SI2_LOW; // 若使用双 SI，则同时拉高 SI2
    tsp_ccd_delay_1us(2);

    CCD_CLK1_LOW;CCD_CLK2_LOW; // 若使用双 TAP，则同时拉低 CLK2
    tsp_ccd_delay_1us(1);
    CCD_SI1_HIGH;CCD_SI2_HIGH; // 若使用双 SI，则同时拉高 SI2
    tsp_ccd_delay_1us(3);

    CCD_CLK1_HIGH;CCD_CLK2_HIGH; // 若使用双 TAP，则同时拉低 CLK2
    tsp_ccd_delay_1us(1);
    CCD_SI1_HIGH;CCD_SI2_HIGH; // 若使用双 SI，则同时拉高 SI2
    tsp_ccd_delay_1us(2);
    
    CCD_CLK1_HIGH;CCD_CLK2_HIGH; // 若使用双 TAP，则同时拉低 CLK2
    tsp_ccd_delay_1us(1);
    CCD_SI1_LOW;CCD_SI2_LOW; // 若使用双 SI，则同时拉高 SI2
    tsp_ccd_delay_1us(2);

    // 4) 依次读出 CCD_PIXEL_COUNT 个像素
    for (uint16_t i = 0; i < CCD_PIXEL_COUNT; i++) {
        // 下降沿
        CCD_CLK1_LOW;CCD_CLK2_LOW;
        tsp_ccd_delay_1us(1);
        // 读 ADC
        CCD1_Get_AO(&val);
        buf[i] = val;

        // 上升沿出电荷,为下一像素做准备
        CCD_CLK1_HIGH;CCD_CLK2_HIGH;
        tsp_ccd_delay_1us(1);
    }

    // 5) 额外再打一拍 CLK 以终止输出
    tsp_ccd_pulse_CLK();

    return true;
}


void tsp_ccd_show(ccd_t data)
{
    uint8_t i=0;

    for(i=0; i<CCD_PIXEL_COUNT; i++)
	{
        tsp_tft18_draw_pixel(32+i, 128-(ccd_data_old[i]>>6), GRAY1);
        tsp_tft18_draw_pixel(32+i, 128-(data[i]>>6), BLUE);
        ccd_data_old[i] = data[i];
	}
}

void tsp_demo_frame_ccd(void)
{
    //tsp_tft18_show_str_color(1, 0, "ExpT:       Max:    ", WHITE, BLACK);
    //tsp_tft18_show_str_color(1, 1, "Mode:       Min:    ", WHITE, BLACK);
    //tsp_tft18_show_str_color(1, 2, "            Avg:    ", WHITE, BLACK);

    // window for TSL1401 waveform
    tsp_tft18_draw_frame(31, 64, 128, 64, BLUE);
    tsp_tft18_draw_block(32, 65, 128, 63, GRAY1);
}


void CCD_test(void)
{

  	// initialize LCD
	// hsp_spi_init();
	// hsp_tft18_init();
	tsp_tft18_clear(BLACK);
	
	// initialize ADC/CCD
	tsp_ccd_init();
	tsp_demo_frame_ccd();
    for(ccd_index=0; ccd_index<128; ccd_index++)
		ccd_data_raw[ccd_index] = (ccd_index<<5);
	tsp_ccd_show(ccd_data_raw);
	while(1)
	{
		if(tsp_ccd_snapshot(ccd_data_raw)){
			tsp_ccd_show(ccd_data_raw);
        }
        delay_1ms(100);
        if(S0()) break;
    }
    while(S0()) {}
}