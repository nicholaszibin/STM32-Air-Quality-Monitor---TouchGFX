#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>
#include "main.h"
#include <string.h>
#include <stdio.h>


extern "C"
{
	extern I2C_HandleTypeDef hi2c1;
	extern UART_HandleTypeDef huart2;

	PMSA003I aqi; //Define the struct to store the AQI data
	int n = 0; //Tick counter

	//Function to read data from the PMSA003I air quality sensor
	void PMSA003I_Read(PMSA003I *aqi_data, I2C_HandleTypeDef *i2cHandle) {

		HAL_StatusTypeDef ret;      //Confirms whether I2C data transfer was successful
		uint8_t buf[32];			//Create a buffer with length of 32 as this matches the PMSA003I detached
		uint16_t buf_u16[15];
		buf[0] = PMSA003I_TEMP;

		ret = HAL_I2C_Master_Transmit(i2cHandle, PMSA003I_ADDR, buf, 1, HAL_MAX_DELAY);
		if ( ret != HAL_OK ) {
		  strcpy((char*)buf, "Error Tx\r\n");

		} else {
			ret = HAL_I2C_Master_Receive(i2cHandle, (0x12 << 1), buf, 32, HAL_MAX_DELAY);

	        if ( ret != HAL_OK || buf[0] != 0x42) {
			  strcpy((char*)buf, "Error Tx\r\n");

			} else {

				for (uint8_t i = 0; i < 15; i++) {
					buf_u16[i] = (buf[2 + i * 2] << 8) | buf[2 + i * 2 + 1];
				}

	            memcpy(aqi_data, buf_u16, sizeof(buf_u16));
			}
	  }
	  return;
	}

	// Function to calculate scaled AQI for a given pollutant
	float calculate_scaled_aqi(float concentration, const float* pollutant_levels, const float* aqi_levels, int size) {
	    int i = 1;

	    // Find the appropriate interval
	    while (i < size && concentration >= pollutant_levels[i]) {
	        i++;
	    }

	    // Linear interpolation for AQI
	    float aqi = (aqi_levels[i] - aqi_levels[i-1]) / (pollutant_levels[i] - pollutant_levels[i-1]) *
	                (concentration - pollutant_levels[i-1]) + aqi_levels[i-1];

	    // Scale AQI to a maximum of 10
	    return aqi / 100 * 2;
	}

	// Function to calculate overall AQI from PM2.5 and PM10 concentrations
	void calculate_aqi(uint16_t pm25_standard, uint16_t pm10_standard, float* aqi_val, float* aqi25_calc, float* aqi10_calc) {
	    // Define PM2.5 and PM10 data
	    const float pm25[] = {0, 12.1, 35.5, 55.5, 150.5, 250.5, 500};
	    const float aqi25[] = {0, 50, 100, 150, 200, 300, 500};
	    const int pm25_size = sizeof(pm25) / sizeof(pm25[0]);

	    const float pm10[] = {0, 55, 155, 255, 355, 425, 604};
	    const float aqi10[] = {0, 50, 100, 150, 200, 300, 500};
	    const int pm10_size = sizeof(pm10) / sizeof(pm10[0]);

	    // Calculate AQI values
	    *aqi25_calc = calculate_scaled_aqi(pm25_standard, pm25, aqi25, pm25_size);
	    *aqi10_calc = calculate_scaled_aqi(pm10_standard, pm10, aqi10, pm10_size);

	    // Determine the overall AQI value
	    *aqi_val = (*aqi25_calc > *aqi10_calc) ? *aqi25_calc : *aqi10_calc;

	    // Return the overall AQI

	    // Output or further processing
		char print_buffer[30];
		sprintf((char*)print_buffer, "PM10: %i ug_m3 \r\n", pm10_standard);
		HAL_UART_Transmit(&huart2, (uint8_t*)&print_buffer, strlen(print_buffer),1000);// Sending in normal mode (uint8_t*)&

		sprintf((char*)print_buffer, "PM2.5: %i ug_m3 \r\n", pm25_standard);
		HAL_UART_Transmit(&huart2, (uint8_t*)&print_buffer, strlen(print_buffer),1000);// Sending in normal mode (uint8_t*)&

		sprintf((char*)print_buffer, "AQI: %i.%i\r\n", (int)*aqi_val, (int)(*aqi_val * 10) % 10); //-u _printf_float not working...
		HAL_UART_Transmit(&huart2, (uint8_t*)&print_buffer, strlen(print_buffer),1000);// Sending in normal mode (uint8_t*)&
	}

}


Model::Model() : modelListener(0), aqi_val(0), aqi25_calc(0), aqi10_calc(0)
{

}

void Model::tick()
{

	if (n == 60) {

		PMSA003I_Read(&aqi, &hi2c1);
		calculate_aqi(aqi.pm25_standard, aqi.pm10_standard, &aqi_val, &aqi25_calc, &aqi10_calc);
		n = 0; //reset the tick
	}

	modelListener->setAQI (aqi_val, aqi25_calc, aqi10_calc);
	n++;

}
