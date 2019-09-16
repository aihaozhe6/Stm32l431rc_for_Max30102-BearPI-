#include "max30102.h"
#include "i2c.h"
#include <stdbool.h>
HAL_StatusTypeDef maxim_max30102_write_reg(uint8_t uch_addr, uint8_t uch_data)
/**
* \brief        Write a value to a MAX30102 register
* \par          Details
*               This function writes a value to a MAX30102 register
*
* \param[in]    uch_addr    - register address
* \param[in]    uch_data    - register data
*
* \retval       true on success
*/
{
    return HAL_I2C_Mem_Write(&hi2c1,I2C_WRITE_ADDR,uch_addr,1,&uch_data,1,1000);
}
uint8_t maxim_max30102_read_reg(uint8_t uch_addr, uint8_t *puch_data)
/**
* \brief        Read a MAX30102 register
* \par          Details
*               This function reads a MAX30102 register
*
* \param[in]    uch_addr    - register address
* \param[out]   puch_data    - pointer that stores the register data
*
* \retval       true on success
*/
{
  uint8_t ch_i2c_data;
  ch_i2c_data=uch_addr;

	
  if( HAL_I2C_Master_Transmit(&hi2c1, I2C_WRITE_ADDR, &ch_i2c_data, 1, 1000)!=HAL_OK)
    return 0;
  if(HAL_I2C_Master_Receive(&hi2c1, I2C_READ_ADDR,&ch_i2c_data,1,1000)==HAL_OK)
  {
    *puch_data=(uint8_t) ch_i2c_data;
    return 1;
  }
  else
    return 0;
}

uint8_t maxim_max30102_init()
{
	
	if(maxim_max30102_write_reg(REG_INTR_ENABLE_1,0xc0)!=HAL_OK) // INTR setting
    return 0;
  if(maxim_max30102_write_reg(REG_INTR_ENABLE_2,0x00)!=HAL_OK)
    return 0;
  if(maxim_max30102_write_reg(REG_FIFO_WR_PTR,0x00)!=HAL_OK)  //FIFO_WR_PTR[4:0]
    return 0;
  if(maxim_max30102_write_reg(REG_OVF_COUNTER,0x00)!=HAL_OK)  //OVF_COUNTER[4:0]
    return 0;
  if(maxim_max30102_write_reg(REG_FIFO_RD_PTR,0x00)!=HAL_OK)  //FIFO_RD_PTR[4:0]
    return 0;
  if(maxim_max30102_write_reg(REG_FIFO_CONFIG,0x0f)!=HAL_OK)  //sample avg = 1, fifo rollover=false, fifo almost full = 17
    return 0;
  if(maxim_max30102_write_reg(REG_MODE_CONFIG,0x03)!=HAL_OK)   //0x02 for Red only, 0x03 for SpO2 mode 0x07 multimode LED
    return 0;
  if(maxim_max30102_write_reg(REG_SPO2_CONFIG,0x27)!=HAL_OK)  // SPO2_ADC range = 4096nA, SPO2 sample rate (100 Hz), LED pulseWidth (400uS)
    return 0;
  
  if(maxim_max30102_write_reg(REG_LED1_PA,0x24)!=HAL_OK)   //Choose value for ~ 7mA for LED1
    return 0;
  if(maxim_max30102_write_reg(REG_LED2_PA,0x24)!=HAL_OK)   // Choose value for ~ 7mA for LED2
    return 0;
  if(maxim_max30102_write_reg(REG_PILOT_PA,0x7f)!=HAL_OK)   // Choose value for ~ 25mA for Pilot LED
    return 0;
  return 1;  

}



uint8_t maxim_max30102_reset()
/**
* \brief        Reset the MAX30102
* \par          Details
*               This function resets the MAX30102
*
* \param        None
*
* \retval       true on success
*/
{
    if(maxim_max30102_write_reg(REG_MODE_CONFIG,0x40)!=HAL_OK)
        return 0;
    else
        return 1;    
}

uint8_t maxim_max30102_read_fifo(uint32_t *pun_red_led, uint32_t *pun_ir_led)
/**
* \brief        Read a set of samples from the MAX30102 FIFO register
* \par          Details
*               This function reads a set of samples from the MAX30102 FIFO register
*
* \param[out]   *pun_red_led   - pointer that stores the red LED reading data
* \param[out]   *pun_ir_led    - pointer that stores the IR LED reading data
*
* \retval       true on success
*/
{
  uint32_t un_temp;
  unsigned char uch_temp;
  *pun_red_led=0;
  *pun_ir_led=0;
  uint8_t ach_i2c_data[6];
  
  //read and clear status register
  maxim_max30102_read_reg(REG_INTR_STATUS_1, &uch_temp);
  maxim_max30102_read_reg(REG_INTR_STATUS_2, &uch_temp);
  
  ach_i2c_data[0]=REG_FIFO_DATA;
	 
  if(HAL_I2C_Master_Transmit(&hi2c1, I2C_WRITE_ADDR, ach_i2c_data, 1, 1000)!=HAL_OK)
    return 0;
  if(HAL_I2C_Master_Receive(&hi2c1, I2C_READ_ADDR,ach_i2c_data,6,1000)!=HAL_OK)
  {
    return 0;
  }
  un_temp=(unsigned char) ach_i2c_data[0];
  un_temp<<=16;
  *pun_red_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[1];
  un_temp<<=8;
  *pun_red_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[2];
  *pun_red_led+=un_temp;
  
  un_temp=(unsigned char) ach_i2c_data[3];
  un_temp<<=16;
  *pun_ir_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[4];
  un_temp<<=8;
  *pun_ir_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[5];
  *pun_ir_led+=un_temp;
  *pun_red_led&=0x03FFFF;  //Mask MSB [23:18]
  *pun_ir_led&=0x03FFFF;  //Mask MSB [23:18]
  return 1;
}
