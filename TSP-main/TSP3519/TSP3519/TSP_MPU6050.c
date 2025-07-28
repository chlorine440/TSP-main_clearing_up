#include "TSP_MPU6050.h"
#include "tsp_isr.h"
#include "TSP_TFT18.h"

//TODO： 需要根据实际采样频率调整
const float DT = 0.02f; // 20ms采样周期

static float yaw = 0;
static float pitch = 0;
static float roll = 0;
static float gyro_bias[3] = {8.0f, -18.0f, 35.0f};  // 零偏估计值
static float acc_bias[3] = {420.0f, -606.0f, 0.0f};  // 零偏估计值

float gz_filtered = 0;
float prev_input = 0, prev_output = 0;
float alpha = 0.98f; // 越小保留高频越多
uint32_t current_time = 0, last_time = 0;
float HighPassFilter(float input)
{
    float output = alpha * (prev_output + input - prev_input);
    prev_input = input;
    prev_output = output;
    return output;
}


// 写数据到MPU6050寄存器
// 函数：MPU6050_WriteReg
// 功能：向MPU6050写入一个寄存器
// 参数：reg_add：寄存器地址；reg_dat：要写入的数据
// 返回值：返回写入结果
int MPU6050_WriteReg(uint8_t reg_add, uint8_t reg_dat)
{
    return mspm0_i2c_write( MPU6050_ADDRESS >> 1, reg_add, 1, &reg_dat);
}

// 函数：MPU6050_ReadReg
// 功能：从MPU6050读取一个寄存器的值
// 参数：reg_add：寄存器地址
// 返回值：返回读取到的数据
uint8_t MPU6050_ReadReg(uint8_t RegAddress) {
    uint8_t value;
    // 内部会用两次传输：写地址，然后重复 START + 读出数据
    mspm0_i2c_read(MPU6050_ADDRESS >> 1,  // 7 位地址
                       RegAddress,
                       1,
                       &value);
    return value;
}

// 从MPU6050寄存器读取数据
// 函数：MPU6050_ReadData
// 功能：从MPU6050读取多个寄存器的值
// 参数：reg_add：寄存器地址；Read：读取的数据；num：读取的寄存器个数
// 返回值：返回读取结果
int MPU6050_ReadData(uint8_t reg_add, uint8_t* Read, uint8_t num)
{
    return mspm0_i2c_read(MPU6050_ADDRESS >> 1, reg_add, num, Read);
}

// 函数：MPU6050ReadID
// 功能：读取MPU6050的ID
// 参数：无
// 如果读取失败，则返回0
uint8_t MPU6050ReadID(void)
{
   uint8_t Re = MPU6050_ReadReg(MPU6050_WHO_AM_I);		//返回WHO_AM_I寄存器的值
   if (Re != 0x68) {
         tsp_tft18_show_str(0,7,"MPU6050 Not Found");
         return 0;
   } else {
        return 1;
   }
}

// 函数：MPU6050_Init
// 功能：初始化MPU6050
// 参数：无
// 返回值：无
void MPU6050_Init(void)
{
    if (!MPU6050ReadID()) {
        return ; // 如果没有找到 MPU6050，直接返回
    }
    delay_1ms(100);
    /*MPU6050寄存器初始化，需要对照MPU6050手册的寄存器描述配置，此处仅配置了部分重要的寄存器*/
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);		//电源管理寄存器1，取消睡眠模式，选择时钟源为X轴陀螺仪
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);		//电源管理寄存器2，保持默认值0，所有轴均不待机
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);		//采样率分频寄存器，配置采样率
	MPU6050_WriteReg(MPU6050_CONFIG, 0x06);			//配置寄存器，配置DLPF
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);	//陀螺仪配置寄存器，选择满量程为±2000°/s
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);	//加速度计配置寄存器，选择满量程为±16g
    delay_1ms(200);
    for(int i = 0; i < 100; i++) {
        short gyro[3],acc[3];
        MPU6050ReadGyro(gyro);
        MPU6050ReadAcc(acc);
        gyro_bias_update(gyro);
        acc_bias_update(acc);
        delay_1ms(10);
    }
}

void MPU6050ReadAcc(short *accData)
{
    uint8_t buf[6];
    MPU6050_ReadData(MPU6050_ACCEL_XOUT_H, buf, 6);
    accData[0] = ((buf[0] << 8) | buf[1]) ; 
    accData[1] = ((buf[2] << 8) | buf[3]) ;
    accData[2] = ((buf[4] << 8) | buf[5]) ;
}

void Get_Acc(float *accData){
    short acc[3];
    MPU6050ReadAcc(acc);
    float accx = (acc[0] - acc_bias[0])/ ACCSENSITIVITY;
    float accy = (acc[1] - acc_bias[1])/ ACCSENSITIVITY;
    float accz = (acc[2])/ ACCSENSITIVITY;
    accData[0] = accx * ACC_SCALE;
    accData[1] = accy * ACC_SCALE;
    accData[2] = accz * ACC_SCALE;
}
// 函数：MPU6050ReadGyro
// 功能：读取MPU6050的陀螺仪数据
// 参数：gyroData：陀螺仪数据
// 返回值：无
void MPU6050ReadGyro(short *gyroData)
{
    uint8_t buf[6];
    MPU6050_ReadData(MPU6050_GYRO_XOUT_H, buf, 6);
    gyroData[0] = ((buf[0] << 8) | buf[1]);
    gyroData[1] = ((buf[2] << 8) | buf[3]);
    gyroData[2] = ((buf[4] << 8) | buf[5]); 
}
void Get_Gyro(float *gyroData){
    short gyro[3];
    MPU6050ReadGyro(gyro);
    float gyrox = (gyro[0] - gyro_bias[0])/ GYROSENSITIVITY;
    float gyroy = (gyro[1] - gyro_bias[1])/ GYROSENSITIVITY;
    float gyroz = (gyro[2] - gyro_bias[2])/ GYROSENSITIVITY;
    gyroData[0] = gyrox * GYRO_SCALE;
    gyroData[1] = gyroy * GYRO_SCALE;
    gyroData[2] = gyroz * GYRO_SCALE;

}
void gyro_bias_update(short *gyroData){
    
    gyro_bias[0] += ((float)gyroData[0] - gyro_bias[0]) / 20;
    gyro_bias[1] += ((float)gyroData[1] - gyro_bias[1]) / 20;
    gyro_bias[2] += ((float)gyroData[2] - gyro_bias[2]) / 20;

}
void acc_bias_update(short *accData){
    
    acc_bias[0] += ((float)accData[0] - acc_bias[0]) / 20;
    acc_bias[1] += ((float)accData[1] - acc_bias[1]) / 20;
    //acc_bias[2] += ((float)accData[2] - acc_bias[2]) / 20;//z轴不需要更新，以此为基准9.8g

}
void RPY_Update(void)
{
    //使用积分来更新rpy
    float gyrodata[3];
    Get_Gyro(gyrodata);
    float gx = HighPassFilter(gyrodata[0]);
    float gy = HighPassFilter(gyrodata[1]);
    //float gz = HighPassFilter(gyrodata[2]);
    float gz = gyrodata[2];
    current_time = get_systick_counter();
    uint8_t dur = (current_time - last_time);
    last_time = current_time;
    // 3) 纯积分更新
    roll  += gx * dur /1000.0f;
    pitch += gy * dur /1000.0f;
    yaw   += gz * dur /1000.0f;
    //tsp_tft18_show_uint8(0, 7, dur);
}

void MPU6050GetRPY(float *Roll, float *Pitch, float *Yaw)
{
    RPY_Update(); // 更新RPY
    *Roll  = roll;
    *Pitch = pitch;
    *Yaw   = yaw;
}


void mpu_test(void){
    short gyro_origin[3],acc_origin[3];
    float gyro[3],acc[3];
    while(1){
        MPU6050ReadGyro(gyro_origin);
        MPU6050ReadAcc(acc_origin);
        Get_Acc(acc);
        Get_Gyro(gyro);
        char buf[50];
        sprintf(buf,"gyro: %d %d %d",gyro_origin[0],gyro_origin[1],gyro_origin[2]);
        tsp_tft18_show_str(0,0,buf);
        sprintf(buf,"acc: %d %d %d",acc_origin[0],acc_origin[1],acc_origin[2]);
        tsp_tft18_show_str(0,1,buf);
        sprintf(buf,"gyro: %.1f %.1f %.1f",gyro[0],gyro[1],gyro[2]);
        tsp_tft18_show_str(0,2,buf);
        sprintf(buf,"acc: %.2f %.2f %.2f",acc[0],acc[1],acc[2]);
        tsp_tft18_show_str(0,3,buf);
        if(S0()) break;
    }
    while(S0()){};
}