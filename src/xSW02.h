/**
 * Copyright (C) 2017 - 2018 Bosch Sensortec GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of the
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 *
 * @file	bme680_defs.h
 * @date	20 Nov 2017
 * @version	3.5.5
 * @brief
 *
 */
/*
	This is a library for the SW02
	DIGITAL HUMIDITY, PRESSURE AND TEMPERATURE SENSOR

	The board uses I2C for communication.
	
	The board communicates with the following I2C device:
	- 	BME680
	
	Data Sheets:
	BME680 - https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BME280_DS001-11.pdf
*/

#ifndef xSW02_h
#define xSW02_h

// System Includes
#include <inttypes.h>
#include <Arduino.h>
#include "xCore.h"
#include "math.h"
#include "SW02_REG.h"

/*=========================================================================*/

class xSW02: public xCoreClass
{
    public:
		/**
		* Constructor
		* Creates a new instance of Sensor class.
		*/	
		xSW02(void);
		xSW02(uint8_t addr);
		/*
		* Runs the setup of the sensor. 
		* Call this in setup(), before reading any sensor data.
		*
		* @return true if setup was successful.
		*/
		bool 	begin();
		
		/*
		* Used to set a calibration offset for the temperature sensor. 
		* Call this in setup(), after begin().
		*
		* @param offset. Float value of temperature off.
		* @return none
		*/		
		void 	setTempCal(float offset);				
		
		/*
		* Used read all raw sensors data and convert to usefull results. 
		* Call this in loop(). Used to call sensor data reads.
		*
		* @return none
		*/
		void 	poll(void);                      
		
		/*
		* Used to get the temperature value in degress celcuis.
		* Call this in loop(). Used to get sensor temperature.
		*
		* @return temperature. Returns float value of temperature.
		*/		
		float 	getTempC(void);				
		
		/*
		* Used to get the temperature value in degress farenhied.
		* Call this in loop(). Used to get sensor temperature.
		*
		* @return temperature. Returns float value of temperature.
		*/		
		float 	getTempF(void);			
		
		/*
		* Used to get the relative humidity percantage
		* Call this in loop(). Used to get sensor humidity value.
		*
		* @return humidity. Returns float value in percentage form.
		*/		
		float 	getHumidity(void);
		
		/*
		* Used to get the pressure in pascals
		* Call this in loop(). Used to get sensor temperature.
		*
		* @return pressure. Returns flost value in pascals
		*/		
		float 	getPressure(void);               	// pressure in pascals
		
		/*
		* Used to get Gas resistance in ohms.
		* Call this in loop(). Used to sensor gas resistance value.
		*
		* @return altitude. Returns float value in ohms.
		*/
	        float 	getGasRes(void);               	// pressure in pascals
		
		/*
		* Used to get Altitude based on standard sea-level pressure
		* Call this in loop(). Used to get sensor altitude value.
		*
		* @return altitude. Returns float value in meters.
		*/	
		float 	getQNE(void);
		
		/*
		* Used to get the approxiamte altitude in meters
		* Call this in loop(). Used to get sensor altitude value.
		*
		* @param sea_level_pressure, input sea level pressure.
		* @return altitude. Returns float value in meters.
		*/		
		float 	getAltitude(float sea_level_pressure);
		
		/*
		* Used to get dew point in celcuis
		* Call this in loop(). Used to get sensor dew point value.
		*
		* @return dewpoint. Returns float value in celcuis.
		*/				
		float 	getDewPoint(void);	

		/*
		* Used to reset the sensor
		*
		* @return true. If reset completed succesfully
		*/	
		void	reset(void);
		
		float getTVOC(void);
		
		float getTVOCFiltered(void);

		/*
		* Get the current IAQ index.
		*
		* @return IAQ, value of current index
		*/	
		uint16_t getIAQ(void);

		struct bme680_dev config;
		
	private:		
    
		/*
		* Reads RAW temperature data.
		*
		* @return none
		*/		
		void readTemperature(uint32_t adc_temp);
		
		/*
		* Reads RAW pressure data.
		*
		* @return none
		*/			
		void readPressure(uint32_t adc_pres);
		
		/*
		* Reads RAW humidity data.
		*
		* @return none
		*/			
		void readHumidity(uint16_t adc_hum);

		/*
		* Reads RAW Gas data.
		*
		* @return none
		*/			
		void readGas(uint16_t resVal);

		/*
		* Reads RAW Gas Resistance value.
		*
		* @return gas_resistance
		*/

		bool readVOC(void);
		
		uint32_t bme680Abc(uint32_t r, float a);
		
		float absHum(float temp, float hum);
		
		uint16_t readGasResistance(void);

		/*
		* Set defaults value for sensor
		*
		* @return none
		*/
		void init_BME680(void);

		/*
		* Configure the Gas Sensor
		*
		* @return none
		*/
		void initGasSensor(uint8_t resHeat);

		/*
		* Set the temperature oversampling
		*/
		void	setTemperatureOversampling(void);

		/*
		* Set the humidity oversampling
		*/		
  		void	setHumidityOversampling(void);

		/*
		* Set the pressure oversampling
		*/		  
  		void	setPressureOversampling(void);

		/*
		* Set the IRR Filter Size
		*/
  		void	setIIRFilterSize(void);

		/*
		* Set the Gas heater temperature and duration
		*/		  
  		uint8_t	setGasHeater(uint8_t set_point);

		/*
		* Trigger Forced Reading
		*/		
		void 	triggerForced(void);
		/*
		* Read a block of data 
		* Put that data in destination pointer
		*
		* @return none
		*/
		void 	readBlock(uint8_t addr, uint8_t reg, uint8_t count, uint8_t * dest);
		
		float   tempcal;	
		float   temperature;              
		float   humidity;                       
		float   pressure;                      
		float 	altitude;			
		float 	dewpoint;
		float 	gas;
		float 	gas_res;
		float   voc;
		float   vocEst;
		bool	isValidIAQ;
		
		int32_t t_fine;
		
		//Device I2C Address
		uint8_t BME680_I2C_ADDRESS;
};

#endif
