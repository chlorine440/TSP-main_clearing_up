#include "tsp_ccd_image.h"


extern ccd_t ccd_data_raw, ccd_data_old; // CCD 数据缓存
extern uint8_t ccd_index; // CCD 数据索引
ccd_t ccd_data_gray, ccd_data_binary; // 灰度图和二值图缓存
ccd_t ccd_data_show, ccd_data_temp;
extern uint8_t RES_value; // 旋转编码器的值
void tsp_show1_ccd_gray(const ccd_t data)
{
    uint16_t temp = 0;
    uint16_t color = 0;
    for (uint16_t j = 0; j < 10; j++){
        for (uint16_t i = 0; i < CCD_PIXEL_COUNT; i++) {
            temp = data[i];
            color = GRAY2RGB16(temp);
            tsp_tft18_draw_pixel(32 + i, 100 - j, color);
        }
    }
}
void tsp_show2_ccd_gray(const ccd_t data)
{
    uint16_t temp = 0;
    uint16_t color = 0;
    for (uint16_t j = 0; j < 10; j++){
        for (uint16_t i = 0; i < CCD_PIXEL_COUNT; i++) {
            temp = data[i];
            color = GRAY2RGB16(temp);
            tsp_tft18_draw_pixel(32 + i, 80 - j, color);
        }
    }
}
void tsp_ccd_data2Gray(const ccd_t data, ccd_t gray_data)
{
    for(uint16_t i = 0; i < CCD_PIXEL_COUNT; i++){
        gray_data[CCD_PIXEL_COUNT - 1 - i] = data[i] >> 4; // 取高 8 位作为灰度值
    }
}
void tsp_ccd_binary(const ccd_t data, ccd_t out, uint16_t threshold)
{
    for (uint16_t i = 0; i < CCD_PIXEL_COUNT; i++) {
        out[i] = (data[i] > threshold) ? 0x0FF : 0x00;
    }
}
uint8_t tsp_ccd_threshold_otsu(const ccd_t data)
{
    // 1) 计算灰度直方图（256 级）
    uint32_t hist[256] = {0};
    for (uint16_t i = 0; i < CCD_PIXEL_COUNT; i++) {
        uint8_t v = data[i];  // 0..255
        hist[v]++;
    }
    uint32_t total = CCD_PIXEL_COUNT;
    // 2) 计算全图像素灰度和
    uint32_t sumAll = 0;
    for (uint16_t t = 0; t < 256; t++) {
        sumAll += (uint32_t)t * hist[t];
    }
    // 3) Otsu 寻最佳阈值
    uint32_t sumB = 0, wB = 0;
    float maxVar = 0.0f;
    uint8_t bestT = 0;
    for (uint16_t t = 0; t < 256; t++) {
        wB += hist[t];
        if (wB == 0) continue;
        uint32_t wF = total - wB;
        if (wF == 0) break;
        sumB += (uint32_t)t * hist[t];
        float mB = (float)sumB / wB;
        float mF = (float)(sumAll - sumB) / wF;
        float varBetween = (float)wB * wF * (mB - mF) * (mB - mF);
        if (varBetween > maxVar) {
            maxVar = varBetween;
            bestT = (uint8_t)t;
        }
    }
    return bestT;
}
uint8_t tsp_ccd_threshold_mean(const ccd_t data)
{
    // 简单取平均灰度作为阈值
    uint32_t sum = 0;
    for (uint16_t i = 0; i < CCD_PIXEL_COUNT; i++) {
        sum += (data[i]);
    }
    return (uint8_t)(sum / CCD_PIXEL_COUNT);
}

//param input: 输入图像数据，已经二值化过
//param output: 输出图像数据
void tsp_ccd_image_erode(const ccd_t input, ccd_t output)
{
    output[0] = input[0];
    for (uint16_t i = 1; i < CCD_PIXEL_COUNT - 1; i++) {
        if(input[i-1] == 0x0 && input[i+1] == 0x0)
            output[i] = 0x0;
        else if(input[i-1] == 0xFF && input[i+1] == 0xFF)
            output[i] = 0xFF;
        else output[i] = input[i]; // 保留原像素值

    }
    output[CCD_PIXEL_COUNT - 1] = input[CCD_PIXEL_COUNT - 1];
}

//param input: 输入图像数据，已经二值化过
//param output: 输出图像数据
void tsp_ccd_image_dilate(const ccd_t input, ccd_t output)
{
    // 三点结构元：[i-1, i, i+1] 最大值
    output[0] = input[0];
    for (uint16_t i = 1; i < CCD_PIXEL_COUNT - 1; i++) {
        if(input[i-1] == 0xFF && input[i+1] == 0xFF)
            output[i] = 0xFF;
        else output[i] = input[i]; // 保留原像素值
    }
    output[CCD_PIXEL_COUNT - 1] = input[CCD_PIXEL_COUNT - 1];
}

void tsp_ccd_detect_scene(const ccd_t input){
    int16_t first_black = -1, last_black = -1;
    uint8_t gte_l, gte_r;
    // 找第一和最后一个黑像素
    gte_l = RESET;
    gte_r = RESET;
    for(uint16_t i=5; i<(CCD_PIXEL_COUNT-5); i++)
    {
        if((gte_l == RESET) && ((255 == input[i])||i==5) && (0 == input[i+1]))
        {
            gte_l= SET; // 左边缘找到
            first_black = i;
        }
        else if((gte_l == SET) && (0 == input[i]) && ((255 == input[i+1])|| i == (CCD_PIXEL_COUNT - 6)))
        {
            gte_r = SET; // 右边缘找到
            last_black = i;
            break; // 找到最后一个黑像素后退出循环
        }
    }
    // // 没扫到线，当成直线/小弯道
    // if (first_black < 0 || last_black < 0) {

    //     return ;
    // }
    if(first_black < 0 || last_black == (CCD_PIXEL_COUNT - 6)) {
        BUZZ_ON();
        //tsp_tft18_show_str(0, 2, "Zhijiao");
        return ;
    }
    uint16_t width = last_black - first_black + 1;
    float pct = (float)width / (float)CCD_PIXEL_COUNT;
    if (pct >= 0.8) {
        //tsp_tft18_show_str(0, 2, "Shizi");
        return ;
    } 
    // else if (pct >= 0.35 && pct < 0.8) {
    //     tsp_tft18_show_str(0, 4, "Zhijiao");
    //     return ;}
     else {
        //tsp_tft18_show_str(0, 2, "Zhixian");
        return ;
    }
}



uint16_t tloss = 0;
void tsp_find_mid_line(const ccd_t input, uint8_t *mid_idx){
    static uint8_t mid_index = 0, last_mid_index = 0;
    uint8_t gte_l, gte_r, gte_ok;				// guide tape edge flag
	uint8_t gte_l_idx, gte_r_idx, gte_c_idx;		// guide tape index
    gte_l = RESET;
    gte_r = RESET;
    gte_ok = RESET;
    for(uint16_t i=5; i<(CCD_PIXEL_COUNT-5); i++)
	{
		if(RESET == gte_l)
		{
			if((255 == input[i]) && (0 == input[i+1]))	// left edge found
			{
				gte_l = SET;
				gte_l_idx = i;									// left edge index
			}
		}
		if((SET == gte_l) && (RESET == gte_r))
		{
			if((0 == input[i]) && (255 == input[i+1]))	// right edge found
			{
				gte_r = SET;
				gte_r_idx = i;									// right edge index
			}
		}
		if((SET == gte_l) && (SET == gte_r) && (RESET == gte_ok))		// both edges found
		{
			if(((gte_r_idx - gte_l_idx) > 10) && ((gte_r_idx - gte_l_idx) < 50))		// proper tape width
			{
				gte_ok = SET;
				tloss = 0;
				gte_c_idx = (gte_r_idx + gte_l_idx) >> 1;	// tape center index
			}
			else
			{
				gte_l = RESET;
				gte_r = RESET;
				gte_ok = RESET;
				tloss += 1;
			}
		}
	}
	
	if(SET == gte_ok)
		mid_index = gte_c_idx;
	else
		mid_index = last_mid_index;
    *mid_idx = mid_index;
    last_mid_index = mid_index; // 更新上次的中线索引

}

void tsp_img_test(void){
    ccd_t ccd_data;
    tsp_tft18_clear(BLACK);
    tsp_ccd_init();
	tsp_demo_frame_ccd();
    while(1)
	{
		if(tsp_ccd_snapshot(ccd_data)){
			tsp_ccd_data2Gray(ccd_data, ccd_data_gray);
            uint16_t threshold = tsp_ccd_threshold_mean(ccd_data_gray);
            //threshold = RES_value; // 使用旋转编码器的值作为阈值
            //if(threshold > 200) {threshold = 20; RES_value = 20;} // 限制阈值范围
			tsp_ccd_binary(ccd_data_gray, ccd_data_binary, threshold);
            tsp_tft18_show_uint16(0, 1, threshold);
            tsp_ccd_image_erode(ccd_data_binary, ccd_data_temp);
            tsp_ccd_detect_scene(ccd_data_temp);
            tsp_ccd_image_dilate(ccd_data_binary, ccd_data_show);
			tsp_show1_ccd_gray(ccd_data_temp);
            //tsp_show2_ccd_gray(ccd_data_show);
        }
        delay_1ms(100);
        if(S0()) break;
    }
    while(S0()) {}
}