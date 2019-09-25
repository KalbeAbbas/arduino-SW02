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
	-	BME680
	
	Data Sheets:
	BME680 - https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BME680_DS001-11.pdf
*/

#include "xSW02.h"
#include <math.h>
#include <SimpleKalmanFilter.h>

SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.01);
float aF = 0;
bool bme680VocValid = false;
uint8_t bDelay = 0;
uint32_t bme680_baseC = 0;
float bme680_baseH = 0;
float resFiltered;
float tVoc = 0;
unsigned long timer = 0;

struct 
{ 
        float         t_offset            = -.5;                 // offset temperature sensor
        float         h_offset            = 1.5;                 // offset humitidy sensor  
        uint32_t      vocBaseR            = 0;                   // base value for VOC resistance clean air, abc 
        uint32_t      vocBaseC            = 0;                   // base value for VOC resistance clean air, abc  
        float         vocHum              = 0;                   // reserved, abc
        uint32_t      signature           = 0x49415143;          // 'IAQC'
}param;     

/*--Public Class Function--*/
/********************************************************
 	Constructor
*********************************************************/
xSW02::xSW02(void)
{
	BME680_I2C_ADDRESS = 0x76;
}

xSW02::xSW02(uint8_t addr)
{
	BME680_I2C_ADDRESS = addr;
}
	
/********************************************************
 	Configure Sensor
*********************************************************/
bool xSW02::begin(void)
{
	uint8_t DEV_ID;
	DEV_ID = xCore.read8(BME680_I2C_ADDRESS, BME680_REG_ID);
	
	sum = 0.0;
	index = 0.0;

	if(DEV_ID != 0x61){
		return false;
	} else {
		reset();
		init_BME680();
		setHumidityOversampling();
		setTemperatureOversampling();
  		setPressureOversampling();
  		setIIRFilterSize();
		initGasSensor(setGasHeater(250));
		xCore.write8(BME680_I2C_ADDRESS, BME680_REG_CNTL_MEAS, config.mode);
		return true;
	}
	
	timer = millis();
}

bool xSW02::readVOC(void)
{
	float t, h, r;
	t = getTempC();
	h = getHumidity();
	float a = absHum(t, h);
	aF = (aF == 0 || a < aF)?a:aF + 0.2 * (a - aF);
	
	/*if(!bme680VocValid)
	{*/
		//isValidIAQ = false;
		voc = 0.0;
		vocEst = 0.0;
		
	//}
	
	r = getGasRes();
	
	uint32_t base = bme680Abc(r, a);
	
	/*if (!bme680VocValid && (millis() > 300000)) 
    {*/       
        //--- allow 300 sec (5 min) to warm-up sensor for stable voc resistance (300000ms)
        resFiltered = r;        // preload low pass filter
        bme680VocValid = true;           
    /*}else{
		isValidIAQ = false;
    }*/
	
	/*if (!bme680VocValid ) 
    {
      return false;
    }*/
	
	isValidIAQ = true;

	resFiltered += 0.1 * (r - resFiltered);    

    float ratio = (float)base / (r * aF * 7.0F);

    float tV  = ( 1250 * log(ratio) ) + 125;                     // approximation    

    float tV2 = tVoc + 0.1 * (tV - tVoc);

    tVoc = (tVoc == 0 || isnan(tVoc) || isinf(tVoc) ) ? tV : tV2;       // global tVoc

    voc = tVoc; 

	float tvoc_estimated_value;

	if(tVoc > 0)
	{
		tvoc_estimated_value = simpleKalmanFilter.updateEstimate(tVoc);
	}else{
		tvoc_estimated_value = 0;		
	}
	

    vocEst = tvoc_estimated_value;	
	
	return true;
	
}

void xSW02::readeCO2()
{
	eco2 = voc * 0.2927 + 392.054;
}

float xSW02::geteCO2()
{
	return eco2;
}

float xSW02::getTVOC(void)
{
	return voc ;
}

float xSW02::getTVOCFiltered(void)
{
	return vocEst ;
}


uint32_t xSW02::bme680Abc(uint32_t r, float a) 
{   
    //--- automatic baseline correction
    uint32_t b = r * a * 7.0F;
    if (b > bme680_baseC && bDelay > 5) 
    {     
      //--- ensure that new baseC is stable for at least >5*10sec (clean air)
      bme680_baseC = b;
      bme680_baseH = a;
    } else if (b > bme680_baseC) 
    {
      bDelay++;
      //return b;
    } else{
      bDelay = 0;
    }
    
    return (param.vocBaseC > bme680_baseC)?param.vocBaseC:bme680_baseC;
}

float xSW02::absHum(float temp, float hum)
{
    double sdd, dd;
    sdd=6.1078 * pow(10,(7.5*temp)/(237.3+temp));
    dd=hum/100.0*sdd;
    return (float)216.687*dd/(273.15+temp);
}  

/********************************************************
 	Read Data from BME680 Sensor
*********************************************************/
void xSW02::reset(void)
{
	xCore.write8(BME680_I2C_ADDRESS, BME680_REG_RESET, 0xB6);
	delay(100);
}

/********************************************************
 	Read Data from BME680 Sensor
*********************************************************/
void xSW02::poll(void)
{
	uint8_t status = xCore.read8(BME680_I2C_ADDRESS, BME680_REG_FIELD0_ADDR);

	if(status & 0x80){
		triggerForced();

		uint8_t rawData[3];

		readBlock(BME680_I2C_ADDRESS, BME680_REG_TEMP_MSB, 3, &rawData[0]);
		readTemperature((((uint32_t) rawData[0] << 16 | (uint32_t) rawData[1] << 8 | rawData[2]) >> 4));
			
		readBlock(BME680_I2C_ADDRESS, BME680_REG_PRES_MSB, 3, &rawData[0]);
		readPressure((((uint32_t) rawData[0] << 16 | (uint32_t) rawData[1] << 8 | rawData[2]) >> 4));

		readBlock(BME680_I2C_ADDRESS, BME680_REG_HUM_MSB, 2, &rawData[0]);
		readHumidity(((uint16_t) (rawData[0] << 8) | rawData[1]));


		readBlock(BME680_I2C_ADDRESS, BME680_REG_GAS_R_MSB, 2, &rawData[0]);
		readGas((((uint16_t)rawData[0] << 2 | (0xC0 & rawData[1]) >> 6)));
		
		readVOC();
		readeCO2();
		readIAQ();
		
		
	}
}

/********************************************************
 	Read Pressure from BME680 Sensor in Pa
*********************************************************/
float xSW02::getPressure(void)
{
	return pressure;
}

/********************************************************
 	Read Altitude based on standard sea-level pressure
*********************************************************/
float xSW02::getQNE(void)
{
	float atmospheric = pressure / 100.0;
	altitude = 44330.0 * (1.0 - pow((atmospheric/1013.25), 1/5.255));
	return altitude;
}

/********************************************************
 	Read Altitude from BME680 Sensor in meters
*********************************************************/
float xSW02::getAltitude(float sea_level_pressure)
{
	float atmospheric = pressure / 100.0;
	altitude = 44330.0 * (1.0 - pow((atmospheric/(sea_level_pressure/100.0)), 1/5.255));
	return altitude;
}

/********************************************************
 	Temperature from BME680 Sensor in Celcuis
*********************************************************/
float xSW02::getTempC(void)
{
    temperature = temperature + tempcal;
    return temperature;	
}

/********************************************************
 	Convert Temperature from BME680 Sensor to Farenhied
*********************************************************/
float xSW02::getTempF(void)
{
	temperature = temperature + tempcal;
  	return temperature * 1.8 + 32;	
}

/********************************************************
 	Read Humidity from BME680 Sensor 
*********************************************************/
float xSW02::getHumidity(void)
{
	return humidity;
}

/********************************************************
 	Set temperature calibration data
*********************************************************/
void xSW02::setTempCal(float offset)
{
	tempcal = offset;
}

/********************************************************
 	Read Dew Point from BME680 Sensor in Celcuis
*********************************************************/
float xSW02::getDewPoint(void)
{
    dewpoint = 243.04 * (log(humidity/100.0) + ((17.625 * temperature)/(243.04 + temperature)))
    /(17.625 - log(humidity/100.0) - ((17.625 * temperature)/(243.04 + temperature)));
	
	return dewpoint;
}

/********************************************************
 	Read Gas Resistance from BME680 Sensor in Ohms
*********************************************************/
float xSW02::getGasRes(void)
{	
	return gas_res;
}

/********************************************************
 	Get the Current IAQ Score
*********************************************************/
float xSW02::getIAQ(void)
{
	return IAQ;
}

/*--Private Class Function--*/
/********************************************************
 	Read Temperature from BME680 Sensor 
*********************************************************/
void xSW02::readTemperature(uint32_t adc_temp)
{
  	int32_t var1 = 0, var2 = 0, var3 = 0, T = 0;
  	var1 = ((int32_t) adc_temp >> 3) - ((int32_t)config.calib.par_t1 << 1);
  	var2 = (var1 * (int32_t)config.calib.par_t2) >> 11;
  	var3 = ((((var1 >> 1) * (var1 >> 1)) >> 12) * ((int32_t)config.calib.par_t3 << 4)) >> 14;
	config.calib.t_fine = var2 + var3;
  	temperature = ((config.calib.t_fine * 5 + 128) >> 8)/100.0;
}

/********************************************************
 	Read Pressure from BME680 Sensor 
*********************************************************/
void xSW02::readPressure(uint32_t adc_pres)
{
	int32_t var1 = 0;
	int32_t var2 = 0;
	int32_t var3 = 0;
	int32_t var4 = 0;
	int32_t pressure_comp = 0;

	var1 = (((int32_t)config.calib.t_fine) >> 1) - 64000;
	var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) *
		(int32_t)config.calib.par_p6) >> 2;
	var2 = var2 + ((var1 * (int32_t)config.calib.par_p5) << 1);
	var2 = (var2 >> 2) + ((int32_t)config.calib.par_p4 << 16);
	var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13) *
		((int32_t)config.calib.par_p3 << 5)) >> 3) +
		(((int32_t)config.calib.par_p2 * var1) >> 1);
	var1 = var1 >> 18;
	var1 = ((32768 + var1) * (int32_t)config.calib.par_p1) >> 15;
	pressure_comp = 1048576 - adc_pres;
	pressure_comp = (int32_t)((pressure_comp - (var2 >> 12)) * ((uint32_t)3125));
	var4 = (1 << 31);
	if (pressure_comp >= var4)
		pressure_comp = ((pressure_comp / (uint32_t)var1) << 1);
	else
		pressure_comp = ((pressure_comp << 1) / (uint32_t)var1);
	var1 = ((int32_t)config.calib.par_p9 * (int32_t)(((pressure_comp >> 3) *
		(pressure_comp >> 3)) >> 13)) >> 12;
	var2 = ((int32_t)(pressure_comp >> 2) *
		(int32_t)config.calib.par_p8) >> 13;
	var3 = ((int32_t)(pressure_comp >> 8) * (int32_t)(pressure_comp >> 8) *
		(int32_t)(pressure_comp >> 8) *
		(int32_t)config.calib.par_p10) >> 17;

	pressure_comp = (int32_t)(pressure_comp) + ((var1 + var2 + var3 +
		((int32_t)config.calib.par_p7 << 7)) >> 4);

	pressure = pressure_comp;
}

/********************************************************
 	Read Humidity from BME680 Sensor 
*********************************************************/
void xSW02::readHumidity(uint16_t adc_hum)
{
	int32_t var1;
	int32_t var2;
	int32_t var3;
	int32_t var4;
	int32_t var5;
	int32_t var6;
	int32_t temp_scaled;
	int32_t calc_hum;

	temp_scaled = (((int32_t)config.calib.t_fine * 5) + 128) >> 8;

	var1 = 	(int32_t) (adc_hum - ((int32_t) ((int32_t)config.calib.par_h1 << 4)))
			-(((temp_scaled * (int32_t)config.calib.par_h3) / ((int32_t) 100)) >> 1);

	var2 = 	((int32_t)config.calib.par_h2 * (((temp_scaled * (int32_t)config.calib.par_h4) /
            ((int32_t)100)) + (((temp_scaled * ((temp_scaled * (int32_t)config.calib.par_h5) /
            ((int32_t)100))) >> 6) / ((int32_t)100)) + (int32_t)(1 << 14))) >> 10;

	var3 = var1 * var2;

	var4 = ((((int32_t)config.calib.par_h6) << 7) + ((temp_scaled * (int32_t) config.calib.par_h7) / ((int32_t)100))) >> 4;

	var5 = ((var3 >> 14) * (var3 >> 14)) >> 10;

	var6 = (var4 * var5) >> 1;

	calc_hum = (((var3 + var6) >> 10) * ((int32_t) 1000)) >> 12;

	if (calc_hum > 102400){
		calc_hum = 102400;
	}else if (calc_hum < 0){
		calc_hum = 0;
	}

	humidity = (calc_hum/1024.0);
}

/********************************************************
 	Read Gas Level from BME680 Sensor 
*********************************************************/
void xSW02::readGas(uint16_t resVal)
{
	float const_array1[16] 	= 	{1, 1, 1, 1, 1, 0.99, 1, 0.992, 1, 1, 0.998, 0.995, 1, 0.99, 1, 1};
	double const_array2[16] = 	{8000000.0, 4000000.0, 2000000.0, 1000000.0, 499500.4995, 248262.1648, 125000.0,
                           		63004.03226, 31281.28128, 15625.0, 7812.5, 3906.25, 1953.125, 976.5625, 488.28125, 244.140625};
	
  	uint8_t gasRange = xCore.read8(BME680_I2C_ADDRESS, BME680_REG_GAS_R_LSB);
	gasRange &= 0x0F;

	uint8_t range_switch_error = xCore.read8(BME680_I2C_ADDRESS, 0x04);
	
  	double var1 = 0;
  	var1 =  (1340.0 + 5.0 * range_switch_error) * const_array1[gasRange];
  	gas_res = var1 * const_array2[gasRange] / (resVal - 512.0 + var1);

	//Serial.println(gas_res);
	//Serial.println(gasRange);
	//Serial.println(range_switch_error);
}

bool xSW02::readIAQ()
{
	sum += getGasRes();
	index += 1;
	
	if(millis() - timer < 360000)
		return false;
	
	gasBase = sum/index;
	
	
	
	
	//hum score
	hum_offset = (getHumidity() - humBase);
	
	if(hum_offset > 0)
	{
		hum_score = 100 - humBase - hum_offset;
		hum_score /= (100 - humBase);
		hum_score *= (hum_weighting * 100);
	}else{
		hum_score = (humBase + hum_offset);
        hum_score /= humBase;
        hum_score *= (hum_weighting * 100);
	}
	
	//gas score
	gas_offset = gasBase - getGasRes();
	if (gas_offset > 0){
		
		gas_score = (getGasRes() / gasBase);
        gas_score *= (100 - (hum_weighting * 100));
		
	}else{
		
		gas_score = 100 - (hum_weighting * 100);
		
	}
	
	IAQ = hum_score + gas_score;
	
	return true;
	
}




/********************************************************
 	Init BM680
*********************************************************/
void xSW02::init_BME680(void)
{
	uint8_t calib_data[41] = {0};

	readBlock(BME680_I2C_ADDRESS, BME680_REG_CALIB_DATA_1, 25, &calib_data[0]);
	readBlock(BME680_I2C_ADDRESS, BME680_REG_CALIB_DATA_2, 16, &calib_data[25]);

	config.calib.par_t1 	= (uint16_t)(((uint16_t) calib_data[34] << 8) | calib_data[33]);
	config.calib.par_t2 	= (int16_t)((( int16_t) calib_data[2] << 8) | calib_data[1]);
	config.calib.par_t3 	= (int8_t)(calib_data[3]);

	config.calib.par_p1		= (uint16_t)(((uint16_t) calib_data[6] << 8) | calib_data[5]);
	config.calib.par_p2		= (int16_t)((( int16_t) calib_data[8] << 8) | calib_data[7]);
	config.calib.par_p3		= (int8_t)(calib_data[9]);
	config.calib.par_p4		= (int16_t)(((int16_t) calib_data[12] << 8) | calib_data[11]);
	config.calib.par_p5		= (int16_t)(((int16_t) calib_data[14] << 8) | calib_data[13]);
	config.calib.par_p6		= (int8_t)(calib_data[16]);
	config.calib.par_p7		= (int8_t)(calib_data[15]);
	config.calib.par_p8		= (int16_t)(((int16_t) calib_data[20] << 8) | calib_data[19]);
	config.calib.par_p9		= (int16_t)(((int16_t) calib_data[22] << 8) | calib_data[21]);
	config.calib.par_p10	= (uint8_t)(calib_data[23]);

	config.calib.par_h1		= (uint16_t)(((uint16_t) calib_data[27] << 4) | (calib_data[26] & 0x0F));
	config.calib.par_h2		= (uint16_t)(((uint16_t) calib_data[25] << 4) | (calib_data[26] >> 4));
	config.calib.par_h3		= (int8_t) calib_data[28];
	config.calib.par_h4		= (int8_t) calib_data[29];
	config.calib.par_h5		= (int8_t) calib_data[30];
	config.calib.par_h6		= (uint8_t) calib_data[31];
	config.calib.par_h7		= (uint8_t) calib_data[32];

	config.calib.par_gh1	= (int8_t) calib_data[37];
	config.calib.par_gh2	= (int16_t)(((int16_t) calib_data[36] << 8) | calib_data[35]);
	config.calib.par_gh3	= (int8_t) calib_data[38];

	config.calib.res_heat_range		= (uint8_t) calib_data[39];
	config.calib.res_heat_val		= (uint8_t) calib_data[40];
	config.calib.range_sw_err		= (uint8_t) calib_data[41];

	config.tph_sett.os_hum	= 0x01;
	config.tph_sett.os_temp	= 0x40;
	config.tph_sett.os_pres	= 0x14;
	config.tph_sett.filter 	= 0x00;

	config.gas_sett.heatr_dur 	= 0x0059;
	config.gas_sett.heatr_temp 	= 0x0000;
	config.gas_sett.nb_conv 	= 0x00;
	config.gas_sett.run_gas 	= 0x00;
	config.gas_sett.heatr_ctrl 	= 0x00;

	config.mode = 0x01;
}

/********************************************************
 	Set Temperature oversampling
*********************************************************/
void xSW02::setTemperatureOversampling(void)
{
	uint8_t var;
	var = xCore.read8(BME680_I2C_ADDRESS, BME680_REG_CNTL_MEAS);

	var |= config.tph_sett.os_temp;
	xCore.write8(BME680_I2C_ADDRESS, BME680_REG_CNTL_MEAS, var);
}

/********************************************************
 	Set Humidity oversampling
*********************************************************/
void xSW02::setHumidityOversampling(void)
{
	xCore.write8(BME680_I2C_ADDRESS, BME680_REG_CNTL_HUM, config.tph_sett.os_hum);
}

/********************************************************
 	Set Pressure oversampling
*********************************************************/	  	  
void xSW02::setPressureOversampling(void)
{
	uint8_t var;
	var = xCore.read8(BME680_I2C_ADDRESS, BME680_REG_CNTL_MEAS);

	var |= config.tph_sett.os_pres;
	xCore.write8(BME680_I2C_ADDRESS, BME680_REG_CNTL_MEAS, var);
}
  		
/********************************************************
 	Set IIR Filter Settings
*********************************************************/	  		  
void xSW02::setIIRFilterSize(void)
{
	uint8_t var;
	var = xCore.read8(BME680_I2C_ADDRESS, BME680_REG_CONFIG);

	var |= config.tph_sett.filter;
	xCore.write8(BME680_I2C_ADDRESS, BME680_REG_CONFIG, var);
}

/********************************************************
 	Init BM680 Gas Sensor
*********************************************************/
void xSW02::initGasSensor(uint8_t resHeat)
{
	// Configure the BME680 Gas Sensor
	xCore.write8(BME680_I2C_ADDRESS, BME680_REG_CNTL_GAS_1, 0x10); 
	// Set gas sampling wait time and target heater resistance
	xCore.write8(BME680_I2C_ADDRESS, (BME680_REG_GAS_WAIT0), 1 | 0x65);
	xCore.write8(BME680_I2C_ADDRESS, (BME680_REG_RES_HEAT0), resHeat);
}

/********************************************************
 	Set Gas heater setpoint
*********************************************************/	  			  
uint8_t xSW02::setGasHeater(uint8_t set_point)
{
	uint8_t res_heat_x = 0;
	double var1 = 0.0, var2 = 0.0, var3 = 0.0, var4 = 0.0, var5 = 0.0;
	uint8_t par_g1 = xCore.read8(BME680_I2C_ADDRESS, 0xED);
	uint16_t  par_g2 = ((uint16_t) xCore.read8(BME680_I2C_ADDRESS, 0xEC) << 8) | xCore.read8(BME680_I2C_ADDRESS, 0xEB);
	uint8_t  par_g3 = xCore.read8(BME680_I2C_ADDRESS, 0xEE);
	uint8_t  res_heat_range = (xCore.read8(BME680_I2C_ADDRESS, 0x02) & 0x30) >> 4;
	uint8_t res_heat_val = xCore.read8(BME680_I2C_ADDRESS, 0x00);
	var1 = ((double) par_g1 / 16.0) + 49.0;
	var2 = (((double)par_g2 / 32768.0) * 0.0005) + 0.00235;
	var3 = (double)par_g3 / 1024.0;
	var4 = var1 * (1.0 + (var2 * (double)set_point));
	var5 = var4 + (var3 * 25.0); // use 25 C as ambient temperature
	res_heat_x = (uint8_t)(3.4 * ((var5 * (4.0 / (4.0 + (double)res_heat_range)) * (1.0/(1.0 +((double)res_heat_val * 0.002)))) - 25));
	return res_heat_x;
}

/*uint8_t xSW02::setGasHeater(uint8_t temp)
{
	uint8_t heatr_res;
	int32_t var1;
	int32_t var2;
	int32_t var3;
	int32_t var4;
	int32_t var5;
	int32_t heatr_res_x100;

	uint8_t par_g1 = xCore.read8(BME680_I2C_ADDRESS, 0xED);
	uint16_t  par_g2 = ((uint16_t) xCore.read8(BME680_I2C_ADDRESS, 0xEC) << 8) | xCore.read8(BME680_I2C_ADDRESS, 0xEB);
	uint8_t  par_g3 = xCore.read8(BME680_I2C_ADDRESS, 0xEE);
	uint8_t  res_heat_range = (xCore.read8(BME680_I2C_ADDRESS, 0x02) & 0x30) >> 4;
	uint8_t res_heat_val = xCore.read8(BME680_I2C_ADDRESS, 0x00);

	if (temp > 400)
		temp = 400;


	var1 = (((int32_t) 25 * par_g3) / 1000) * 256;
	var2 = (par_g1 + 784) * (((((par_g2 + 154009) * temp * 5) / 100) + 3276800) / 10);
	var3 = var1 + (var2 / 2);
	var4 = (var3 / (res_heat_range + 4));
	var5 = (131 * res_heat_val) + 65536;
	heatr_res_x100 = (int32_t) (((var4 / var5) - 250) * 34);
	heatr_res = (uint8_t) ((heatr_res_x100 + 50) / 100);

	return heatr_res;
}*/

/********************************************************
 	Init trigger forced reading
*********************************************************/
void xSW02::triggerForced(void)
{
	uint8_t var;
	var |= config.tph_sett.os_temp;
	var |= config.tph_sett.os_pres;
	var |= config.mode = 0x01;
	xCore.write8(BME680_I2C_ADDRESS, BME680_REG_CNTL_MEAS, var);
}

/********************************************************
 	Read Block of data over I2C Bus
*********************************************************/
void xSW02::readBlock(uint8_t addr, uint8_t reg, uint8_t count, uint8_t * dest)
{
	Wire.beginTransmission(addr);  
	Wire.write(reg);         
	Wire.endTransmission(false);  
	uint8_t i = 0;
    Wire.requestFrom(addr, (size_t) count);  
	while (Wire.available()) {
        dest[i++] = Wire.read(); 
	}  
}
