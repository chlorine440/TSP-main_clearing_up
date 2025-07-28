#ifndef TSP_CCD_IMAGE_H
#define TSP_CCD_IMAGE_H

#include <stdint.h>
#include <stdbool.h>
#include "tsp_common_headfile.h"
#include "TSP_CCD.h"
/** 线阵 CCD 像素数 */
#define CCD_PIXEL_COUNT   128

/** 线阵 CCD 一行数据类型 */
typedef uint16_t ccd_t[CCD_PIXEL_COUNT];
void tsp_show1_ccd_gray(const ccd_t data);
void tsp_show2_ccd_gray(const ccd_t data);
void tsp_ccd_data2Gray(const ccd_t data, ccd_t gray_data);
void tsp_ccd_binary(const ccd_t data, ccd_t out, uint16_t threshold);
uint8_t tsp_ccd_threshold_otsu(const ccd_t data);
uint8_t tsp_ccd_threshold_mean(const ccd_t data);
void tsp_ccd_image_erode(const ccd_t input, ccd_t output);
void tsp_ccd_image_dilate(const ccd_t input, ccd_t output);
void tsp_find_mid_line(const ccd_t input, uint8_t *mid_idx); // find the mid line of the image
#endif // TSP_CCD_IMAGE_H