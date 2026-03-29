#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

void mpu6050_init(void);
void mpu6050_read(void);
void mpu6050_calibrate(void);
void mpu6050_request_data(void);

#define MPU6050_ADDRESS 0x68
#define FS_GYRO_250 0
#define FS_GYRO_500 8
#define FS_GYRO_1000 9
#define FS_GYRO_2000 10

#define FS_ACC_2G 0
#define FS_ACC_4G 8
#define FS_ACC_8G 9
#define FS_ACC_16G 10

#define DATA_RDY_EN 1

#define REG_INT_CONFG 55
#define REG_INT_ENABLE 56
#define REG_CONFIG_GYRO 27
#define REG_CONFIG_ACC 28
#define REG_USR_CTRL 107
#define REG_DATA 59

#endif