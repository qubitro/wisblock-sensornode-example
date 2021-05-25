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

#define TEMP_DATA_MASK  0x1
#define HUM_DATA_MASK   0x2
#define PRES_DATA_MASK  0x4
#define GAS_DATA_MASK   0x8
#define LIGHT_DATA_MASK 0x10
#define ACCL_DATA_MASK  0x20
#define LOC_DATA_MASK   0x40

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
    
        // Battery
    analogReference(AR_INTERNAL_3_0);
	// Set the resolution to 12-bit (0..4095)
	analogReadResolution(12); // Can be 8, 10, 12 or 14

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

unsigned populateSensorData(uint8_t *buffer, unsigned len) {
    // Packet format
    // 0 Header | 1 Battery | 3 Temp | 7 Humidity | 11 Pressure | 15 Gas | 19 Light | 23 Accel
    buffer[0] = 0x0;

    int16_t bat = readVBAT();
    float temp = 0;
    memcpy(&buffer[1], &bat, 2);
    
    // Env sensor
    #ifdef BEM680_ENV_SENSOR
    if(bme.performReading()){
        buffer[0] ^= ( TEMP_DATA_MASK | HUM_DATA_MASK | PRES_DATA_MASK | GAS_DATA_MASK );
        memcpy(&buffer[15], &bme.gas_resistance, sizeof bme.gas_resistance);
        #ifndef SHTC3_TEMP_HUM_SENSOR // If we dont have another humidity sensor, use the bme's humidity
        memcpy(&buffer[7], &bme.humidity, 4);
        memcpy(&buffer[3], &bme.temperature, 4);
        #endif
        #ifndef LPS22_PRESSURE_SENSOR
        memcpy(&buffer[11], &bme.pressure, 4);
        #endif
    }else{
        // Error and make 0
        critical_error("Failed to read bme sensor");
    }
    #endif

    // Temp humidity sensor
    #ifdef SHTC3_TEMP_HUM_SENSOR
    buffer[0] ^= ( TEMP_DATA_MASK | HUM_DATA_MASK );
    shtc.update();
    temp = shtc.toDegC();
    float hum = shtc.toPercent();
    memcpy(&buffer[7], &hum, 4);
    memcpy(&buffer[3], &temp, 4);
    #endif

    // Pressure sensor
    #ifdef LPS22_PRESSURE_SENSOR
    buffer[0] ^= PRES_DATA_MASK;
    float pressure = BARO.readPressure();
    memcpy(&buffer[11], &pressure, 4);
    #ifndef SHTC3_TEMP_HUM_SENSOR | BEM680_ENV_SENSOR
    buffer[0] ^= PRES_DATA_MASK;
    temp = barometricSensor.readTemperature();
    memcpy(&buffer[11], &pressure, 4);
    #endif
    #endif

    #ifdef OPT3001_LIGHT_SENSOR
    buffer[0] ^= LIGHT_DATA_MASK;
    OPT3001 result = opt3001.readResult();
    memcpy(&buffer[19], &result.lux, 4);
    #endif

    #ifdef LIS3DH_ACCEL_SENSOR
    buffer[0] ^= ACCL_DATA_MASK;
    // This is a really bad way to do this, its just to demo making an alert if someone moves the device
    float accel = imu.readFloatAccelX() + imu.readFloatAccelY() + imu.readFloatAccelZ();
    memcpy(&buffer[23], &accel, 4);
    #endif

    return 27;
}

uint16_t readVBAT(void)
{
    //return 0;
    float raw;

	raw = analogRead(PIN_VBAT);
	// //Serial.print("Battery: ");
	// //Serial.println(raw* REAL_VBAT_MV_PER_LSB);
	return round((raw * REAL_VBAT_MV_PER_LSB));
}


