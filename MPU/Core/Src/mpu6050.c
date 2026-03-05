#include "mpu6050.h"
#include "main.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_def.h"
#include "stm32l4xx_hal_i2c.h"
#include "stm32l4xx_hal_uart.h"
#include <stdio.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

int16_t accel_offset_x, accel_offset_y, accel_offset_z;
int16_t gyro_offset_x, gyro_offset_y, gyro_offset_z;

extern uint8_t i2c_rx_buffer[6];
extern volatile uint8_t i2c_ready_flag;

void mpu6050_init(void) {

  HAL_StatusTypeDef status =
      HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDRESS << 1, 5, 1000);

  if (status != HAL_OK) {
    return;
  }
  uint8_t buf[32];

  uint8_t reg_data = 0;

  status = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS << 1, REG_USR_CTRL, 1,
                             &reg_data, 1, 100);

  if (status == HAL_OK) {
    sprintf((char *)buf, "exited from sleep mode \r\n");
    HAL_UART_Transmit(&huart2, buf, strlen((char *)buf), HAL_MAX_DELAY);
  } else {
    sprintf((char *)buf, "still snoozing \r\n");
    HAL_UART_Transmit(&huart2, buf, strlen((char *)buf), HAL_MAX_DELAY);
  }

  status = HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDRESS << 1, 1, 50);
  if (status == HAL_OK) {
    sprintf((char *)buf, "the device is ready \r\n");
    HAL_UART_Transmit(&huart2, buf, strlen((char *)buf), HAL_MAX_DELAY);
  } else {
    sprintf((char *)buf, "Failed to connect \r\n");
    HAL_UART_Transmit(&huart2, buf, strlen((char *)buf), HAL_MAX_DELAY);
  }

  // gyro config
  reg_data = FS_GYRO_500;
  status = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS << 1, REG_CONFIG_GYRO, 1,
                             &reg_data, 1, 100);

  if (status == HAL_OK) {
    sprintf((char *)buf, "gyro configured \r\n");
    HAL_UART_Transmit(&huart2, buf, strlen((char *)buf), HAL_MAX_DELAY);
  } else {
    sprintf((char *)buf, "failed to configure gyro \r\n");
    HAL_UART_Transmit(&huart2, buf, strlen((char *)buf), HAL_MAX_DELAY);
  }

  // accelerometer config
  reg_data = FS_ACC_4G;

  status = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS << 1, REG_CONFIG_ACC, 1,
                             &reg_data, 1, 100);
  if (status == HAL_OK) {
    sprintf((char *)buf, "acc configured \r\n");
    HAL_UART_Transmit(&huart2, buf, strlen((char *)buf), HAL_MAX_DELAY);
  } else {
    sprintf((char *)buf, "failed to configure acc \r\n");
    HAL_UART_Transmit(&huart2, buf, strlen((char *)buf), HAL_MAX_DELAY);
  }

  reg_data =
      0x10; // INT_RD_CLEAR = 1: clear interrupt on any data register read

  status = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS << 1, REG_INT_CONFG, 1,
                             &reg_data, 1, 100);

  if (status == HAL_OK) {
    sprintf((char *)buf, "configured interrupt \r\n");
    HAL_UART_Transmit(&huart2, buf, strlen((char *)buf), HAL_MAX_DELAY);
  } else {
    sprintf((char *)buf, "failed to configure interrupt\r\n");
    HAL_UART_Transmit(&huart2, buf, strlen((char *)buf), HAL_MAX_DELAY);
  }

  reg_data = DATA_RDY_EN;
  status = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS << 1, REG_INT_ENABLE, 1,
                             &reg_data, 1, 100);

  if (status == HAL_OK) {
    sprintf((char *)buf, "data Interrupt enabled \r\n");
    HAL_UART_Transmit(&huart2, buf, strlen((char *)buf), HAL_MAX_DELAY);
  } else {
    sprintf((char *)buf, "failed to enable data int\r\n");
    HAL_UART_Transmit(&huart2, buf, strlen((char *)buf), HAL_MAX_DELAY);
  }
}

void mpu6050_read() {
  uint8_t data[6];
  uint8_t buff[64];
  int16_t x_raw, y_raw, z_raw;
  int32_t x_mg, y_mg, z_mg;

  // MPU6050 Accelerometer Data Registers:
  // 0x3B: ACCEL_XOUT_H, 0x3C: ACCEL_XOUT_L
  // 0x3D: ACCEL_YOUT_H, 0x3E: ACCEL_YOUT_L
  // 0x3F: ACCEL_ZOUT_H, 0x40: ACCEL_ZOUT_L

  // Read 6 bytes starting from ACCEL_XOUT_H (0x3B)
  if (HAL_I2C_Mem_Read(&hi2c1, (MPU6050_ADDRESS << 1), 0x3B, 1, data, 6, 100) ==
      HAL_OK) {

    // Combine bytes for each axis
    x_raw = (int16_t)(data[0] << 8 | data[1]);
    y_raw = (int16_t)(data[2] << 8 | data[3]);
    z_raw = (int16_t)(data[4] << 8 | data[5]);

    // Convert to mg (assuming +/- 2g range, sensitivity = 16384 LSB/g)
    x_mg = ((int32_t)x_raw * 1000) / 16384;
    y_mg = ((int32_t)y_raw * 1000) / 16384;
    z_mg = ((int32_t)z_raw * 1000) / 16384;

    // Format for Serial Plotter: "x,y,z\r\n"
    sprintf((char *)buff, "%ld,%ld,%ld\r\n", x_mg, y_mg, z_mg);
    HAL_UART_Transmit(&huart2, buff, strlen((char *)buff), HAL_MAX_DELAY);
  }

  HAL_Delay(50); // Adjust for desired sample rate
}

// Global variables to store offsets
int16_t accel_offset_x, accel_offset_y, accel_offset_z;
int16_t gyro_offset_x, gyro_offset_y, gyro_offset_z;

void mpu6050_calibrate(void) {
  uint8_t data[14];
  int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;
  int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
  int num_samples = 200;

  uint8_t buf[24];

  sprintf((char *)buf, "CalibKeepdevice level\r\n");
  HAL_UART_Transmit(&huart2, buf, strlen((char *)buf), HAL_MAX_DELAY);

  for (int i = 0; i < num_samples; i++) {
    // Read all 14 sensor registers (Accel, Temp, Gyro)
    if (HAL_I2C_Mem_Read(&hi2c1, (MPU6050_ADDRESS << 1), 0x3B, 1, data, 14,
                         100) == HAL_OK) {
      // Accumulate Accelerometer raw values
      sum_ax += (int16_t)(data[0] << 8 | data[1]);
      sum_ay += (int16_t)(data[2] << 8 | data[3]);
      sum_az += (int16_t)(data[4] << 8 | data[5]);

      // Accumulate Gyroscope raw values
      sum_gx += (int16_t)(data[8] << 8 | data[9]);
      sum_gy += (int16_t)(data[10] << 8 | data[11]);
      sum_gz += (int16_t)(data[12] << 8 | data[13]);
    }
    HAL_Delay(5); // Small delay between samples
  }

  // Calculate averages
  accel_offset_x = (int16_t)(sum_ax / num_samples);
  accel_offset_y = (int16_t)(sum_ay / num_samples);
  // For Z, subtract 1g (16384 LSB) because gravity is active
  accel_offset_z = (int16_t)((sum_az / num_samples) - 8192);

  gyro_offset_x = (int16_t)(sum_gx / num_samples);
  gyro_offset_y = (int16_t)(sum_gy / num_samples);
  gyro_offset_z = (int16_t)(sum_gz / num_samples);

  sprintf((char *)buf, "Calibration Complete!\r\n");
  HAL_UART_Transmit(&huart2, buf, strlen((char *)buf), HAL_MAX_DELAY);
}

void mpu6050_request_data(void) {
  HAL_I2C_Mem_Read_IT(&hi2c1, MPU6050_ADDRESS << 1, 0x3B, 1, i2c_rx_buffer, 6);

  // if (status != HAL_OK) {
  //   uint8_t buf[24];
  //   sprintf((char *)buf, "failed request\r\n");
  //   HAL_UART_Transmit(&huart2, buf, strlen((char *)buf), HAL_MAX_DELAY);
  // }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c->Instance == I2C1) {
    i2c_ready_flag = 1; // Signal to the main loop that data is ready
  }
}