#include "hal_conf_extra.h"
#include <Arduino.h>
#include "Roboty_sensor.h"
#include "stm32f4xx_hal.h"

#define LED PC3
FMPI2C_HandleTypeDef hfmpi2c1;

LIS2MDLSensor Mag_sensor(&hfmpi2c1);
LSM6DS3Sensor IMU(&hfmpi2c1);

int32_t magnetometer[3];
int8_t data = 0;
void sendOrientation();

//int _write(int file, char *ptr, int len);

void setup()
{

  pinMode(LED, OUTPUT);
  Mag_sensor.begin();
  Mag_sensor.Enable();
  
  IMU.begin();
  IMU.Enable_X();
}

void loop()
{
  sendOrientation();
  digitalToggle(LED);
  delay(1000);

  Mag_sensor.GetAxes(magnetometer);
  Serial.printf("Error=%i\n\r",data);
  Serial.println();
  Serial.printf("X_mag = %d", magnetometer[0]);
  Serial.println();
  Serial.printf("Y_mag = %d", magnetometer[1]);
  Serial.println();
  Serial.printf("Z_mag = %d", magnetometer[2]);
  Serial.println();
  

  
}

void sendOrientation()
{
  int32_t accell[3];
  int32_t gyro[3];

  IMU.Get_G_Axes(gyro);
  IMU.Get_X_Axes(accell);

  Serial.printf("X = %d", accell[0]);
  Serial.println();
}
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 13;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}


