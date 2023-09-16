#include "hal_conf_extra.h"
#include <Arduino.h>
#include "LSM6DS3Sensor_roboty.h"
#include "LIS2MDLSensor.h"
#include "stm32f4xx_hal.h"

#define LED PC3
FMPI2C_HandleTypeDef hfmpi2c1;

LIS2MDLSensor Mag_sensor(&hfmpi2c1);
LSM6DS3Sensor IMU(&hfmpi2c1);

int32_t magnetometer[3];
uint8_t XH;
void sendOrientation();

int _write(int file, char *ptr, int len);

void setup() {

pinMode(LED, OUTPUT);

delay(1000);

//Mag_sensor.begin();
//Mag_sensor.Enable();
IMU.begin();
IMU.Enable_X();

uint8_t data[8];
//HAL_FMPI2C_Mem_Read(&hfmpi2c1,((0x1E) << 1),0x4F,FMPI2C_MEMADD_SIZE_8BIT,data,1,10000);

}

void loop() {
digitalToggle(LED);
delay(1000);
/*
Mag_sensor.GetAxes(magnetometer);
Serial.printf("X = %d",magnetometer[0]);
Serial.println();
Serial.printf("Y = %d",magnetometer[1]);
Serial.println();
Serial.printf("Z = %d",magnetometer[2]);
Serial.println();
*/
sendOrientation();

}
/*
int _write(int file, char *ptr, int len)
{
 
  int i=0;
  for(i=0 ; i<len ; i++)
    ITM_SendChar((*ptr++));
  return len;
}*/
void sendOrientation()
{
 int32_t accell[3];
int32_t gyro[3];

  
  IMU.Get_G_Axes(gyro);
  IMU.Get_X_Axes(accell);


  Serial.printf("X = %d",accell[0]);
  Serial.println();
}