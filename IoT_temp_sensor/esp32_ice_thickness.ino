#include "DHT.h" 
//#include <dht11.h>
#include <WiFi.h>
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

// #define WIFI_SSID "REPLACE_WITH_YOUR_SSID"
// #define WIFI_PASSWORD "REPLACE_WITH_YOUR_PASSWORD"

#define WIFI_SSID "miao"
#define WIFI_PASSWORD "lilyESP32"

// Raspberry Pi Mosquitto MQTT Broker
//#define MQTT_HOST IPAddress(192, 168, 22, 77) 172.20.10.13
// #define MQTT_HOST IPAddress(172, 20, 10, 13)172.20.10.10
#define MQTT_HOST IPAddress(172, 20, 10, 10)
// For a cloud MQTT broker, type the domain name
//#define MQTT_HOST "example.com"
#define MQTT_PORT 1883

// Temperature MQTT Topics
#define MQTT_PUB_TEMP "esp32/dht/temperature"
#define MQTT_PUB_HUM  "esp32/dht/humidity"

// Digital pin connected to the DHT sensor
#define DHTPIN 4  

#define DHTTYPE DHT11   

#define USE_MUTEX
SemaphoreHandle_t mutex;

// Initialize DHT sensor
DHT dht(DHTPIN, DHTTYPE); 

// Variables to hold sensor readings, shared_variables
float temp = 0.0;
float hum = 0.0;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 10000;        // Interval at which to publish sensor readings

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  switch(event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt(); // Connect to MQTT after Wi-Fi connection is established
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("WiFi lost connection. Reconnecting...");
      xTimerStop(mqttReconnectTimer, 0); // Stop MQTT reconnect attempts
      xTimerStart(wifiReconnectTimer, 0); // Retry Wi-Fi connection
      break;
    default:
      Serial.printf("[WiFi-event] Unhandled event: %d\n", event);
      break;
  }
}


void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

/*void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}
void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}*/

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void Read(void *pvParameters) {  //Read sensor task
    printf("Task 0          | Task 1\n");
    //reading sensor data
    float new_hum;
    float new_temp;
    unsigned long currentMillis = millis();
    // Every X number of seconds (interval = 10 seconds) 
    // it publishes a new MQTT message
    if (currentMillis - previousMillis >= interval) {
      // Save the last time a new reading was published
      previousMillis = currentMillis;
      // New DHT sensor readings
      new_hum = dht.readHumidity();
      Serial.printf("hum: %.2f \n", hum);
      // Read temperature as Celsius (the default)
      new_temp = dht.readTemperature();
      Serial.printf("temp: %.2f \n", temp);
      // Read temperature as Fahrenheit (isFahrenheit = true)
      //temp = dht.readTemperature(true);

      // Check if any reads failed and exit early (to try again).
      if (isnan(temp) || isnan(hum)) {
        Serial.println(F("Failed to read from DHT sensor!"));
        return; 
      }
    }
    while (1) {
        xSemaphoreTake(mutex, portMAX_DELAY);  // Lock access to shared_variable

        printf(" hum %.2f <- %.2f, temp %.2f <- %.2f     |\n", hum, new_hum, temp, new_temp);
        hum = new_hum;
        temp = new_temp;

        xSemaphoreGive(mutex);  // Unlock access to shared_variable
        vTaskDelay(pdMS_TO_TICKS(1000));  // Simulate work
    }
}

void Send(void *pvParameters) {  // Send data task
    printf("                 | Starting\n");
    while (1) {
        xSemaphoreTake(mutex, portMAX_DELAY);  // Lock access to shared_variable

        printf("                                  | R_hum: %.2f, R_temp: %.2f\n", hum, temp);  // Only reading

        // Publish an MQTT message on topic esp32/dht/temperature
        uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP, 1, true, String(temp).c_str());                            
        Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_TEMP, packetIdPub1);
        Serial.printf("Message: %.2f \n", temp);

        // Publish an MQTT message on topic esp32/dht/humidity
        uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_HUM, 1, true, String(hum).c_str());                            
        Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_HUM, packetIdPub2);
        Serial.printf("Message: %.2f \n", hum);
        xSemaphoreGive(mutex);  // Unlock access to shared_variable
        vTaskDelay(pdMS_TO_TICKS(500));  // Simulate work
    }
}

void setup() {
  Serial.begin(115200);
  Serial.println();

  dht.begin();
  
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  //mqttClient.onSubscribe(onMqttSubscribe);
  //mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  // If your broker requires authentication (username and password), set them below
  //mqttClient.setCredentials("REPlACE_WITH_YOUR_USER", "REPLACE_WITH_YOUR_PASSWORD");
  connectToWifi();

  Serial.printf(" Reading          | Sending\n");

  #ifdef USE_MUTEX
    mutex = xSemaphoreCreateMutex();  // Create the mutex
  #endif

  static int task_number0 = 0;
  xTaskCreate(
    Read, "Reading"  // A name just for humans
    ,
    2048  // The stack size
    ,
    (void *)&task_number0  // Pass reference to a variable describing the task number
    //,  5  // High priority
    ,
    1  // priority
    ,
    NULL  // Task handle is not used here - simply pass NULL
  );

  static int task_number1 = 1;
  xTaskCreate(
    Send, "Sending", 2048  // Stack size
    ,
    (void *)&task_number1  // Pass reference to a variable describing the task number
    ,
    1  // Low priority
    ,
    NULL  // Task handle is not used here - simply pass NULL
  );
}

void loop() {
  // unsigned long currentMillis = millis();
  // // Every X number of seconds (interval = 10 seconds) 
  // // it publishes a new MQTT message
  // if (currentMillis - previousMillis >= interval) {
  //   // Save the last time a new reading was published
  //   previousMillis = currentMillis;
  //   temp = 100;
  //   hum = 200;
  //   // New DHT sensor readings
  //   hum = dht.readHumidity();
  //   Serial.printf("hum: %.2f \n", hum);
  //   // Read temperature as Celsius (the default)
  //   temp = dht.readTemperature();
  //   Serial.printf("temp: %.2f \n", temp);
  //   // Read temperature as Fahrenheit (isFahrenheit = true)
  //   //temp = dht.readTemperature(true);

  //   // Check if any reads failed and exit early (to try again).
  //   if (isnan(temp) || isnan(hum)) {
  //     Serial.println(F("Failed to read from DHT sensor!"));
  //     //return; lily
  //   }
    
  //   // Publish an MQTT message on topic esp32/dht/temperature
  //   uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP, 1, true, String(temp).c_str());                            
  //   Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_TEMP, packetIdPub1);
  //   Serial.printf("Message: %.2f \n", temp);

  //   // Publish an MQTT message on topic esp32/dht/humidity
  //   uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_HUM, 1, true, String(hum).c_str());                            
  //   Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_HUM, packetIdPub2);
  //   Serial.printf("Message: %.2f \n", hum);
  // }
}
