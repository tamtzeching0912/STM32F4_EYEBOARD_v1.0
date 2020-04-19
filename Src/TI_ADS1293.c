#include "TI_ADS1293.h"
#include "stdint.h"
#include "stdbool.h"

void Ads1293_Init(void)
{
  HAL_GPIO_WritePin(GPIOA, ADS1293_DRDY_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, ADS1293_CSB_PIN, GPIO_PIN_SET);
	HAL_Delay(10);
}

void Ads1293_Config(bool* is_init, SPI_HandleTypeDef hspi1)
{
    *is_init = false;
    Disable_Start(hspi1);
		Reg_Write(FLEX_CH1_CN, 0x19, hspi1); // INP to IN3 and INN to IN1
		Reg_Write(FLEX_CH2_CN, 0x2E, hspi1); // INP to IN5 and INN to IN6
    Reg_Write(FLEX_PACE_CN, 0, hspi1);
    Reg_Write(FLEX_VBAT_CN, 0, hspi1);
    Reg_Write(LOD_CN, 0x04, hspi1); // Disable Lead-Off detection
    Reg_Write(LOD_EN, 0, hspi1);
    Reg_Write(CMDET_EN, 0x1E, hspi1); // enable common-mode-detector from IN1,IN2,IN3
    Reg_Write(RLD_CN, 0x02, hspi1); // connect RLD to IN2
    Reg_Write(REF_CN, 0, hspi1); // Turn on internal REF
    Reg_Write(OSC_CN, 0x04, hspi1); // use external clock
    Reg_Write(AFE_RES, 0x18, hspi1); // 204800Hz ADC
    Reg_Write(AFE_SHDN_CN, 0x24, hspi1); // Shut down ADC CH3
    Reg_Write(AFE_PACE_CN, 0, hspi1); // turn off PACE
    Reg_Write(R1_RATE, 0, hspi1); // standard DATA rate, R1 = 4
    Reg_Write(R2_RATE, 0x02, hspi1); // R2 = 5
    Reg_Write(R3_RATE_CH1, 0x80, hspi1); // R3 = 8 
    Reg_Write(R3_RATE_CH2, 0x80, hspi1); // R3 = 8, 1280 Hz, ADCMax = 0xC35000, Bandwidth: 260Hz
    Reg_Write(DRDYB_SRC, 0x08, hspi1); // connect DRDY to CH1
    Reg_Write(CH_CNFG, 0x30, hspi1); // loop read-back mode, for CH1 CH2
    Enable_Start(hspi1); 
    *is_init = true;
}

void Reg_Write (uint8_t READ_WRITE_ADDRESS, uint8_t DATA, SPI_HandleTypeDef hspi1)
{                              
		uint8_t writeData[2] = {READ_WRITE_ADDRESS & 0x7f, DATA};
    HAL_GPIO_WritePin(GPIOC, ADS1293_CSB_PIN, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, &writeData[0], 1, 10);
		HAL_SPI_Transmit(&hspi1, &writeData[1], 1, 10);
    HAL_GPIO_WritePin(GPIOC, ADS1293_CSB_PIN, GPIO_PIN_SET);
}

uint8_t Reg_Read (uint8_t  READ_WRITE_ADDRESS, SPI_HandleTypeDef hspi1)
{
    uint8_t tmp;
		READ_WRITE_ADDRESS |= 0x80;
		HAL_GPIO_WritePin(GPIOC, ADS1293_CSB_PIN, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, &READ_WRITE_ADDRESS, 1, 10);
		HAL_SPI_Receive(&hspi1, &tmp, 1, 10);
    HAL_GPIO_WritePin(GPIOC, ADS1293_CSB_PIN, GPIO_PIN_SET);
    return tmp;
}

void Read_Data_Stream(uint8_t* data, int length, SPI_HandleTypeDef hspi1)
{
		uint8_t READ_WRITE_ADDRESS = DATA_LOOP | 0x80;
		HAL_GPIO_WritePin(GPIOC, ADS1293_CSB_PIN, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, &READ_WRITE_ADDRESS, 1, 10);
    for (int i=0; i<length; i++) {
				HAL_SPI_Receive(&hspi1, &data[i], 1, 10);
    }
    HAL_GPIO_WritePin(GPIOC, ADS1293_CSB_PIN, GPIO_PIN_SET);
}

void Disable_Start(SPI_HandleTypeDef hspi1)
{
    Reg_Write(CONFIG, 0x00, hspi1);
}

void Enable_Start(SPI_HandleTypeDef hspi1)
{
    Reg_Write(CONFIG, 0x01, hspi1);
    HAL_Delay(10);
}
