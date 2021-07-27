#include "main.h"

WiFiClient wifiClient;

QubitroMqttClient mqttClient(wifiClient);

char ssid[] = "WiFi_ID";
char pass[] = "WiFi_PASSWORD";

char deviceID[] = "PASTE_DEVICE_ID_HERE";
char deviceToken[] = "PASTE_DEVICE_TOKEN_HERE";
char host[] = "broker.qubitro.com";
int port = 1883;

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

  mqttClient.setId(deviceID);
  mqttClient.setDeviceIdToken(deviceID, deviceToken);

  Serial.println("Connecting to Qubitro...");

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
    // Change if possible to have a message over 256 characters
    static char payload[256];
    mqttClient.beginMessage(deviceID);
    // Send value
    populateSensorData(buffer);
    Serial.println(buffer);
    mqttClient.print(buffer); 
    mqttClient.endMessage();  
    Serial.println("Posted!!");
  }
}