#include "main.h"

uint8_t nodeDeviceEUI[8] = DEV_EUI;
uint8_t nodeAppEUI[8] = APP_EUI;
uint8_t nodeAppKey[16] = APP_KEY;

DeviceClass_t gCurrentClass = CLASS_A;							  /* class definition*/
lmh_confirm gCurrentConfirm = LMH_UNCONFIRMED_MSG;				  /* confirm/unconfirm packet definition*/
uint8_t gAppPort = LORAWAN_APP_PORT;

static lmh_param_t lora_param_init = {LORAWAN_ADR_ON, LORAWAN_DATERATE, LORAWAN_PUBLIC_NETWORK, JOINREQ_NBTRIALS, LORAWAN_TX_POWER, LORAWAN_DUTYCYCLE_OFF};

static lmh_callback_t lora_callbacks = {GetBatteryLoRaWAN, BoardGetUniqueId, BoardGetRandomSeed,
										lorawan_rx_handler, lorawan_has_joined_handler, lorawan_confirm_class_handler};

#define LORAWAN_APP_DATA_BUFF_SIZE 64
#define LORAWAN_APP_INTERVAL 120000
static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0, 0, 0, 0};

static uint32_t count = 0;
static uint32_t count_fail = 0;

unsigned long long next_transmission = 0;
bool joined = false;

#ifndef WIRE_HAS_END
#error "op"
#endif

void setup()
{
	pinMode(LED_BLUE, OUTPUT);
	pinMode(LED_GREEN, OUTPUT);
	pinMode(34, OUTPUT);

	digitalWrite(LED_GREEN, HIGH);
	digitalWrite(34, HIGH);
	
	
	// Initialize //Serial for debug output
	Serial.begin(115200);
	while (!Serial && millis() < 10000)
	{
		delay(10);
	}
	Serial.print("[*] Starting radio initialization...");
	// Initialize LoRa chip.
	if(lora_rak4630_init()){
		Serial.println("FAIL");
		digitalWrite(LED_BLUE, HIGH);
		while(1);
	}

#if defined(REGION_AS923)
	Serial.println("Region: AS923");
#elif defined(REGION_AU915)
	Serial.println("Region: AU915");
#elif defined(REGION_CN470)
	Serial.println("Region: CN470");
#elif defined(REGION_CN779)
	Serial.println("Region: CN779");
#elif defined(REGION_EU433)
	Serial.println("Region: EU433");
#elif defined(REGION_IN865)
	Serial.println("Region: IN865");
#elif defined(REGION_EU868)
	Serial.println("Region: EU868");
#elif defined(REGION_KR920)
	Serial.println("Region: KR920");
#elif defined(REGION_US915)
	Serial.println("Region: US915");
#elif defined(REGION_US915_HYBRID)
	Serial.println("Region: US915_HYBRID");
#else
	#error "Please define a region in the compiler options."
#endif
	
	initSensors();

	// Setup the EUIs and Keys
	lmh_setDevEui(nodeDeviceEUI);
	lmh_setAppEui(nodeAppEUI);
	lmh_setAppKey(nodeAppKey);

	// Initialize LoRaWan
	uint32_t err_code = lmh_init(&lora_callbacks, lora_param_init, true);
	if (err_code != 0)
	{
		Serial.printf("[!] lmh_init failed - %d\n", err_code);
	}
	
	digitalWrite(LED_GREEN, HIGH);
	Serial.println("[*] Starting join process");
	// Start Join procedure
	lmh_join();
	digitalWrite(34, LOW);
}

void loop() {
    
    Radio.IrqProcess();

    if(next_transmission<millis() && joined){
        #ifdef POWERGATE_SENSORS
		digitalWrite(34, HIGH);
		initSensors();
		delay(500);
		#endif
		
        tx_lora_periodic_handler();
		
        #ifdef POWERGATE_SENSORS
        digitalWrite(34, LOW);
        #endif

        next_transmission = POLL_TIME + millis();
	}
}

void lorawan_has_joined_handler(void)
{
	digitalWrite(LED_BUILTIN, HIGH);
	//Serial.println("[N] LoRaWAN Network Joined!");
	lmh_error_status ret = lmh_class_request(gCurrentClass);
	if (ret == LMH_SUCCESS)
	{
		delay(1000);
		joined = true;
		//TimerSetValue(&appTimer, LORAWAN_APP_INTERVAL);
		//TimerStart(&appTimer);
	}
	digitalWrite(LED_BUILTIN, LOW);
}

void lorawan_rx_handler(lmh_app_data_t *app_data)
{
	digitalWrite(LED_BUILTIN, HIGH);
	//Serial.printf("[N] LoRa Packet received on port %d, size:%d, rssi:%d, snr:%d, data:%s\n",
	//			  app_data->port, app_data->buffsize, app_data->rssi, app_data->snr, app_data->buffer);
	digitalWrite(LED_BUILTIN, LOW);
}

void lorawan_confirm_class_handler(DeviceClass_t Class)
{
	//Serial.printf("[N] switch to class %c done\n", "ABC"[Class]);
	// Informs the server that switch has occurred ASAP
	m_lora_app_data.buffsize = 0;
	m_lora_app_data.port = gAppPort;
	lmh_send(&m_lora_app_data, gCurrentConfirm);
}

void send_lora_frame(void)
{
	digitalWrite(LED_BUILTIN, HIGH);
	if (lmh_join_status_get() != LMH_SET)
	{
		//Not joined, try again later
		return;
	}

	
	memset(m_lora_app_data.buffer, 0, LORAWAN_APP_DATA_BUFF_SIZE);
	m_lora_app_data.port = gAppPort;
	//strncpy(m_lora_app_data.buffer, "",LORAWAN_APP_DATA_BUFF_SIZE );
	
	m_lora_app_data.buffsize = populateSensorData(m_lora_app_data.buffer, 0);

	lmh_error_status error = lmh_send(&m_lora_app_data, gCurrentConfirm);
	if (error == LMH_SUCCESS)
	{
		count++;
		//Serial.printf("lmh_send ok count %d\n", count);
	}
	else
	{
		count_fail++;
		//Serial.printf("lmh_send fail count %d\n", count_fail);
	}
	digitalWrite(LED_BUILTIN, LOW);
}

uint8_t mvToLoRaWanBattVal(uint16_t mvolts)
{ // * 2.55
	if (mvolts < 3300)
		return 0;

	if (mvolts < 3600)
	{
		mvolts -= 3300;
		return mvolts / 30 * 2.55;
	}

	mvolts -= 3600;
	return (10 + (mvolts * 0.15F)) * 2.55;
}

static uint8_t GetBatteryLoRaWAN(void){
	//Serial.println("[N] Network fetched battery status");
	return mvToLoRaWanBattVal(readVBAT());
}

void tx_lora_periodic_handler(void)
{
	Serial.println("[*] Sending interval data...");
	send_lora_frame();
}
