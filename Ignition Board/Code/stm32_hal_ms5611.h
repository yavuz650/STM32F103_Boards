#ifndef STM32_HAL_MS5611
#define STM32_HAL_MS5611

//includes *-----------------------------------------------------*
#include <math.h>
#include <stm32f1xx_hal.h>


/** @defgroup MS5611 device commands
  * @{
  */
#define DEV_ADD (0x77<<1) 	 //MS5611 device address
#define CMD_PROM_RD 0xA0 		 //Prom read command 
#define CMD_ADC_READ 0x00		 //ADC read command
#define CMD_RESET 0x1E 	  	 //ADC reset command. Not used in this library
/**
  * @}
  */
	
	
/**
  * @brief  Return enumaration for functions.
  * @note   MS_ERROR indicates something went wrong.  
  */
typedef enum
{
	MS_ERROR,
	MS_OK
}MS_Status;


/**
	* @brief MS5611 device struct. All the variables are private, and
	*	abstracted from the user. They are NOT to be modified from outside.
	*/
typedef struct
{
	I2C_HandleTypeDef* i2c_bus; //I2C bus which the device is connected to.
	
	uint32_t D1; //digital pressure value
	uint32_t D2; //digital temperature value
	
	float dT; 	 //difference between actual and reference temperature
	float TEMP;  //actual temperature
	
	double OFF;  //offset at actual temperature
	double SENS; //sensitivity at actual temperature
	float P; 		 //temperature compensated pressure
	
	uint16_t C[6]; //PROM coefficients
	
	uint8_t prom_read_buf[2]; //buffer for reading two bytes(16-bit) of data from PROM.
	uint8_t adc_read_buf[3];	//buffer for reading three bytes(24-bit) conversion result.
	
}MS5611_Type;


/** @defgroup private functions
  * @{
  */

MS_Status Read_Prom(MS5611_Type *device);

void Get_Raw_Temp(MS5611_Type *device);
void Calculate_Actual_Temp(MS5611_Type *device);

void Get_Raw_Pres(MS5611_Type *device);
void Calculate_Actual_Pres(MS5611_Type *device);

/**
  * @}
  */


/** @defgroup public functions
  * @{
  */
MS_Status MS5611_Init(MS5611_Type *device, I2C_HandleTypeDef* i2c_handle);

float Return_Temp(MS5611_Type *device);
float Return_Pres(MS5611_Type *device);
double Return_Altitude(MS5611_Type *device);

/**
  * @}
  */

#endif
