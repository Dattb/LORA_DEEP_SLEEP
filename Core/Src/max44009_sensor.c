/*
 * max44009_sensor.c
 *
 *  Created on: Jul 18, 2021
 *      Author: PC5
 */


#include "max44009_sensor.h"
#include "math.h"
I2C_HandleTypeDef hi2c3;
extern 	uint32_t data_max44009;

uint32_t RD_I2C_Read_light (){
	
	unsigned char i2c_SHT30_rx_buff1[2] = {0};	// control command
	unsigned char i2c_SHT30_rx_buff2[2] = {0};	// control command

	HAL_I2C_Mem_Read(&hi2c3,MAX44009_I2C_ADDR,0x03,1,(uint8_t *)i2c_SHT30_rx_buff1,2,100);
	HAL_I2C_Mem_Read(&hi2c3,MAX44009_I2C_ADDR,0x04,1,(uint8_t *)i2c_SHT30_rx_buff2,2,100);
	
	return Max44009_calculate_data(*i2c_SHT30_rx_buff1,*i2c_SHT30_rx_buff2);
	
}



uint32_t Max44009_calculate_data (unsigned char reg_hi,unsigned char reg_low) {
	Max44009_data_tdef S_data ;
	S_data.raw_data = reg_hi<<8|reg_low;
	
	return lux_calculate(S_data);
}




uint32_t lux_calculate (Max44009_data_tdef data){
	unsigned int mantissa = data.mantissa_hi<<4|data.mantissa_low;
	return (unsigned long)(pow(2,data.exponent)*0.045*mantissa);
}


 void RD_MX_I2C3_Init(void)
{
  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x20303E5D;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(i2cHandle->Instance==I2C3)
  {

  /** Initializes the peripherals clocks
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C3;
    PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB7     ------> I2C1_SDA
    PB8     ------> I2C1_SCL
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C3_CLK_ENABLE();

  }
}
void light_sensor_read_data(){
	
	RD_MX_I2C3_Init();
	HAL_I2C_MspInit(&hi2c3);
	data_max44009  =  RD_I2C_Read_light();
	
}
