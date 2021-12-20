#include "main.h"

#ifdef BEM680_ENV_SENSOR
#include <Adafruit_BME680.h> 
Adafruit_BME680 bme;
#endif
#ifdef SHTC3_TEMP_HUM_SENSOR
#include "SparkFun_SHTC3.h" //Click here to get the library: http://librarymanager/All#SparkFun_SHTC3
SHTC3 shtc;	
#endif
#ifdef LIS3DH_ACCEL_SENSOR
#include <SparkFunLIS3DH.h>
LIS3DH imu(I2C_MODE, 0x18);
#endif
#ifdef OPT3001_LIGHT_SENSOR
#include <ClosedCube_OPT3001.h>
ClosedCube_OPT3001 opt3001;
#endif
#ifdef LPS22_PRESSURE_SENSOR
#include <Arduino_LPS22HB.h> // Click here to get the library: http://librarymanager/All#Arduino_LPS22HB
#endif

uint16_t readVBAT(void)
{
    //return 0;
    float raw;

	raw = analogRead(PIN_VBAT);
	// //Serial.print("Battery: ");
	// //Serial.println(raw* REAL_VBAT_MV_PER_LSB);
	return round((raw * REAL_VBAT_MV_PER_LSB));
}

void critical_error(char *issue) {
    while(1){
        digitalWrite(LED_BLUE, HIGH);
        delay(200);
        digitalWrite(LED_BLUE, LOW);
        delay(300);
        #ifdef DEBUG
        Serial.print("[!] ");
        Serial.println(issue);
        #endif
    }
}

void initSensors() {
    Wire.begin();
    Wire.setClock(400000);

    #ifdef BEM680_ENV_SENSOR
    //Serial.print(F("[*] Initializing BME680 sensor..."));
	while (!bme.begin(0x76)) {  // Start BME680 using I2C, use first device found
		//Serial.print(F("FAIL..."));
		delay(5000);
	}  // of loop until device is located
	//Serial.print("DONE...");
	bme.setTemperatureOversampling(BME680_OS_8X);
	bme.setHumidityOversampling(BME680_OS_2X);
	bme.setPressureOversampling(BME680_OS_4X);
	bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
	bme.setGasHeater(320, 150); // 320*C for 150 ms
    delay(5);
    bme.performReading();
	//Serial.println("CONFIGURED");
    #endif

    #ifdef LPS22_PRESSURE_SENSOR
    if (!BARO.begin())
	{
		critical_error("Failed to init pressure sensor");
	}
    #endif

    #ifdef SHTC3_TEMP_HUM_SENSOR
    shtc.begin();
    if (shtc.passIDcrc) // Whenever data is received the associated checksum is calculated and verified so you can be sure the data is true
	{					   // The checksum pass indicators are: passIDcrc, passRHcrc, and passTcrc for the ID, RH, and T readings respectively
		Serial.print("ID Passed Checksum. ");
		Serial.print("Device ID: 0b");
		Serial.println(shtc.ID, BIN); // The 16-bit device ID can be accessed as a member variable of the object
	}
	else
	{
        critical_error("LPS22 ID CHECKSUM FAILURE");
	}
    #endif

    #ifdef OPT3001_LIGHT_SENSOR
    OPT3001_Config newConfig;

	newConfig.RangeNumber = B1100;
	newConfig.ConvertionTime = B0;
	newConfig.Latch = B1;
	newConfig.ModeOfConversionOperation = B11;

	OPT3001_ErrorCode errorConfig = opt3001.writeConfig(newConfig);
	if (errorConfig != NO_ERROR){
		critical_error("Light sensor issue");
    }
    #endif

    #ifdef LIS3DH_ACCEL_SENSOR
    Serial.print("Initializing accelerometer...");
    if(imu.begin()==0){
        Serial.println("DONE");
    }else{
        critical_error("Failure communicating with imu");
    }
    #endif
}

bool populateSensorData(char *buffer) {
    StaticJsonDocument<BUFFER_LEN> doc;
    doc["bat"] = readVBAT();
    
    // Env sensor
    #ifdef BEM680_ENV_SENSOR
    if(bme.performReading()){
        doc["gas"] = bme.gas_resistance;
        #ifndef SHTC3_TEMP_HUM_SENSOR // If we dont have another humidity sensor, use the bme's humidity
        doc["temp"] = bme.temperature;
        doc["humidity"] = bme.humidity;
        #endif
        #ifndef LPS22_PRESSURE_SENSOR
        doc["pressure"] = bme.pressure;
        #endif
    }else{
        // Error and make 0
        critical_error("Failed to read bme sensor");
    }
    #endif

    // Temp humidity sensor
    #ifdef SHTC3_TEMP_HUM_SENSOR
    shtc.update();
    doc["temp"] = shtc.toDegC();
    doc["humidity"] = shtc.toPercent();
    #endif

    // Pressure sensor
    #ifdef LPS22_PRESSURE_SENSOR
    doc["pressure"] = BARO.readPressure();
    #ifndef SHTC3_TEMP_HUM_SENSOR | BEM680_ENV_SENSOR
    doc["temp"] = barometricSensor.readTemperature();
    
    #endif
    #endif

    #ifdef OPT3001_LIGHT_SENSOR
    OPT3001 result = opt3001.readResult();
    #endif

    #ifdef LIS3DH_ACCEL_SENSOR
    // This is a really bad way to do this, its just to demo making an alert if someone moves the device
    doc["accel"] = abs(imu.readFloatAccelX()) + abs(imu.readFloatAccelY()) + abs(imu.readFloatAccelZ());
    #endif
    
    serializeJson(doc, buffer, BUFFER_LEN);
}


