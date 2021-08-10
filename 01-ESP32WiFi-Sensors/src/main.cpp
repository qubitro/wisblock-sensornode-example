#include "main.h"

#if USE_SSL
const int port = 8883;
WiFiClientSecure wifiClient;
#else
const int port = 1883;
WiFiClient wifiClient
#endif

QubitroMqttClient mqttClient(wifiClient);

char ssid[] = "WiFi_ID";
char pass[] = "WiFi_PASSWORD";

char deviceID[] = "PASTE_DEVICE_ID_HERE";
char deviceToken[] = "PASTE_DEVICE_TOKEN_HERE";
char host[] = "broker.qubitro.com";

unsigned long next = 0;

void receivedMessage(int messageSize) 
{
  Serial.print("New message received:");
  while (mqttClient.available()) 
  {
    Serial.print((char)mqttClient.read());
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.print("Attempting to connect to ssid ");
  Serial.print(ssid);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);  
  }
  Serial.println("Connected!");

  #if USE_SSL
  wifiClient.setInsecure();
  #endif

  mqttClient.setId(deviceID);
  mqttClient.setDeviceIdToken(deviceID, deviceToken);

  Serial.print("Connecting to Qubitro...");

  if (!mqttClient.connect(host, port)) 
  {
    Serial.print("Connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    Serial.println("Visit docs.qubitro.com or create a new issue on github.com/qubitro");
    while (1);
  }
  Serial.println("Connected to Qubitro!");

  mqttClient.onMessage(receivedMessage);    
                                      
  mqttClient.subscribe(deviceID);

  initSensors();
}

void loop() {
  mqttClient.poll();
  if(millis() > next) {
    char buffer[1024];
    next = millis() + 20000;

    mqttClient.beginMessage(deviceID);
    // Send value
    Serial.println("Posted: ");
    populateSensorData(buffer);
    Serial.println(buffer);
    mqttClient.print(buffer); 
    mqttClient.endMessage();  
  }
}