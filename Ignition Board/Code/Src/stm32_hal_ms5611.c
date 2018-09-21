//includes *-----------------------------------------------------*
#include <stm32_hal_ms5611.h>

/** @defgroup MS5611 private function definitions
  * @{
  */

MS_Status Read_Prom(MS5611_Type *device)
{
	for(int i=1;i<7;i++) //iterate through coefficient addresses.
		{
			if(HAL_I2C_Mem_Read(device->i2c_bus,DEV_ADD,CMD_PROM_RD+2*i,1,device->prom_read_buf,2,20)!=HAL_OK)// save it to the prom buffer.
			{
				return MS_ERROR;
			}
			device->C[i-1] = (device->prom_read_buf[0] << 8) + device->prom_read_buf[1]; // calculate the coefficient from two seperate bytes.
		}
		
		return MS_OK;
}

void Get_Raw_Temp(MS5611_Type *device)
{
	uint8_t D2_adc_cmd_4096 = 0x58; //D2 ADC conversion command with 4096 oversampling rate
	
	HAL_I2C_Master_Transmit(device->i2c_bus,DEV_ADD,&D2_adc_cmd_4096,1,20); //send adc command
	HAL_Delay(50); //wait for the conversion to complete.
	HAL_I2C_Mem_Read(device->i2c_bus,DEV_ADD,CMD_ADC_READ,1,device->adc_read_buf,3,20); //read the adc result.
	
	device->D2 = (device->adc_read_buf[0] << 16) + (device->adc_read_buf[1] << 8) + device->adc_read_buf[2]; //calculate D2
}

void Calculate_Actual_Temp(MS5611_Type *device)
{
	device->dT = device->D2 - device->C[4]*pow(2,8);
	device->TEMP = (2000+device->dT*device->C[5]/pow(2,23))/100;
}

void Get_Raw_Pres(MS5611_Type *device)
{
	uint8_t D1_adc_cmd_4096 = 0x48; //D1 ADC conversion command with 4096 oversampling rate
	
	HAL_I2C_Master_Transmit(device->i2c_bus,DEV_ADD,&D1_adc_cmd_4096,1,20); //send adc command
	HAL_Delay(50); //wait for the conversion to complete.
	HAL_I2C_Mem_Read(device->i2c_bus,DEV_ADD,CMD_ADC_READ,1,device->adc_read_buf,3,20); //read the adc result.
	
	device->D1 = (device->adc_read_buf[0] << 16) + (device->adc_read_buf[1] << 8) + device->adc_read_buf[2]; //calculate D1
}

void Calculate_Actual_Pres(MS5611_Type *device)
{
	device->OFF = device->C[1]*pow(2,16) + (device->C[3]*device->dT)/pow(2,7);
	device->SENS = device->C[0]*pow(2,15) + (device->C[2]*device->dT)/pow(2,8);
	
	device->P = (device->D1*device->SENS/pow(2,21)-device->OFF)/pow(2,15); //pressure in pascal. divide by 100 to get mbar.
}

/**
  * @}
  */

/** @defgroup MS5611 public function definitions
  * @{
  */
MS_Status MS5611_Init(MS5611_Type *device, I2C_HandleTypeDef* i2c_handle)
{
	device->i2c_bus = i2c_handle;
	return Read_Prom(device);
}

float Return_Temp(MS5611_Type *device)
{
	Get_Raw_Temp(device);
	Calculate_Actual_Temp(device);
	return device->TEMP;
}
float Return_Pres(MS5611_Type *device)
{
	Get_Raw_Pres(device);
	Calculate_Actual_Pres(device);
	return device->P;
}
double Return_Altitude(MS5611_Type *device)
{
	Return_Temp(device);
	Return_Pres(device);
	return 4946.55*(8.96196-pow(device->P,0.190263));
}


/**
  * @}
  */
