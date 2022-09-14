#include <Arduino.h>
#include <string.h>
#include <cstdio>
#include "ethcomm.h"
#include "main.h"
#include "mqtt.h"
#include "freertos/queue.h"
#include <PubSubClient.h>
#include "WiFi.h"
#include "memory.hpp"

_Memory Memory;

//------------------------------------------------------------------------------
//  MAIN Variables for MQTT
//------------------------------------------------------------------------------
WiFiClient espWiFiClient;
EthernetClient espEthClient;
PubSubClient client(espEthClient);

//------------------------------------------------------------------------------
// Static Function Prototypes
//------------------------------------------------------------------------------
static void subscribeMQTT(char *topic);
static void reSubscribe();

//------------------------------------------------------------------------------
// Local Variables
//------------------------------------------------------------------------------
// MQTT Broker
const char *mqtt_broker = "broker.emqx.io";
const int mqtt_port = 1883;
const char *mqtt_ID = "esp32a02";
// MQTT Credentials
const char *mqtt_username = "remote2";
const char *mqtt_password = "password2";
const char *mqtt_client = "ESP32A02";

//------------------------------------------------------------------------------
// Extern Functions and others
//------------------------------------------------------------------------------
extern QueueHandle_t mqttQueue;
extern struct callbackStruct callback_data;
extern void CallbackTask(void *pvParam);
extern void MQTT_Task(void *pvParam);

//------------------------------------------------------------------------------
//  void MQTT_Task(void *pvParam)
//
//  This is the main MQTT Task function
//------------------------------------------------------------------------------
void MQTT_Task(void *pvParam) {
  unsigned long mqttReconnectionTime;
  Struct_MQTT mqttData;
  //gUartMessage mqttData;
  char dataArray[100];
  char result[100];
  while (1) {
    while ((GetMQTTConnectionStatus() == true) && ((GetIsWiFiConnected() == true) || (GetActiveInterface() == COMMUNICATION_ETHERNET)) ) {
      if (xQueueReceive(mqttQueue, (void *)&mqttData, portMAX_DELAY) == pdTRUE) {
        //        result = (char *)pvPortMalloc(strlen(dataArray) + 1);
        switch ((mqttData.ID)) {
          case IDPUBLISHMQTT:
            Serial.print("ID Publish MQTT");
            memset(dataArray, 0, 100);
            snprintf(dataArray, 100, "%s/%s", mqtt_ID, mqttData.topic);
            client.publish(dataArray, mqttData.payload);
            Serial.print(dataArray);
            Serial.print("/");
            Serial.println(mqttData.payload);
            break;

          case IDSUBSCRIBEMQTT:

            Serial.println("ID Subscribe MQTT");
            memset(dataArray, 0, 100);

            snprintf(dataArray, sizeof dataArray, "%s/%s", mqtt_ID, mqttData.topic);
            client.subscribe(dataArray);
            Serial.println(dataArray);
            break;

          case IDPUBLISH_INPUT:
            break;

          case IDPUBLISH_SYSTEM:
            break;
        }
      }
    }
    // Yielding the task in case of Connection not established
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void MemoryInit()
{
  Memory.begin();
}

//------------------------------------------------------------------------------
//  void CallbackTask(void *pvParam)
//
//  This is CallbackTask main function
//------------------------------------------------------------------------------
void CallbackTask(void *pvParam) {
  while (1) {
    if ((callback_data.dataArrives)) {
      callback_data.dataArrives = false;
      char *payloadId = strtok(callback_data.topic, "/");
      char *payloadFunc = strtok(NULL, "/");
      // Break payload down
      char *payloadName = strtok(callback_data.payload, "/");
      char *payloadData = strtok(NULL, "/");

      Serial.print("ID: ");
      Serial.print(payloadId);
      Serial.print(" Function: ");
      Serial.print(payloadFunc);
      Serial.print(" Name: ");
      Serial.print(payloadName);
      Serial.print(" Data: ");
      Serial.println(payloadData);

      // If topic is a set
      if (strcmp(payloadFunc, "set") == 0) {
        Memory.set(payloadName, payloadData);
        char *reply = Memory.get(payloadName);
        publishMQTTMessage("reply", reply);
      }
      // If topic is a get
      if (strcmp(payloadFunc, "get") == 0) {
        char *reply = Memory.get(payloadName);
        publishMQTTMessage("reply", reply);
      }

      // The set and get works with out a payload
      if (strcmp(payloadFunc, "output") == 0) {
        if (payloadData != NULL) {
          SendMessageToOutputTask(callback_data.payload, payloadData, PROCESS_OUT);
          Serial.println("Callback Data output ");
        }
      }

      if (strcmp(payloadFunc, "timer") == 0) {
        // Timer.start(payloadAsChar);
        return;
      }

      if (strcmp(payloadFunc, "system") == 0) {

        if (strcmp(payloadName, "publish") == 0) {
          bool ret = GetPublishInputMessageEnable();
          SetPublishInputMessageEnable(!ret);
          Serial.print("Publish input messages: ");
          Serial.println(ret);
        }

        if (strcmp(payloadName, "restart") == 0) {
          Serial.println("Resetting ESP32");
          ESP.restart();
          return;
        }

        if (strcmp(payloadName, "save") == 0) {
          Memory.save();
        }

        if (strcmp(payloadName, "erase") == 0) {
          Memory.erase();
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

//------------------------------------------------------------------------------
//  void subscribeMQTT(char *topic)
//
//  This function is main function for subscribing to a topic
//------------------------------------------------------------------------------
static void subscribeMQTT(char *topic) {
  char dataArray[30];
  snprintf(dataArray, sizeof dataArray, "%s/%s", mqtt_ID, topic);
  client.subscribe(dataArray);
  Serial.println(dataArray);
}

//------------------------------------------------------------------------------
//  void reSubscribe()
//
//  This function is used to re subscribe to the MQTT topics
//------------------------------------------------------------------------------
static void reSubscribe() {
  Serial.println("Subscribing...");
  subscribeMQTT("timer");
  subscribeMQTT("output");
  subscribeMQTT("system");
  subscribeMQTT("set");
  subscribeMQTT("get");
}

//------------------------------------------------------------------------------
//  void MQTTreconnect(void)
//
//  This function is used to reconnect to the MQTT broker
//------------------------------------------------------------------------------
int reconnectCounter = 0;

void MQTTreconnect() {
  // Loop until we're reconnected
  Serial.println("Attempting MQTT");
  SendOLEDMessageFromInit("Attempting MQTT");
  // Attempt to connect
  if (client.connect(mqtt_ID, mqtt_username, mqtt_password)) {  //(client.connect(clientId.c_str())) {
    reconnectCounter = 0;  // reset counter
    Serial.println("MQTT Connected");
    SendOLEDMessageFromInit("MQTT Connected");
    SetMQTTConnectionStatus(true);
    reSubscribe();
  }
  else if (reconnectCounter > 500) {
    Serial.println("Resetting ESP32");
    delay(500);
    ESP.restart();
  }
  else {
    reconnectCounter++;
    Serial.print("Attempt: ");
    Serial.print(reconnectCounter);
    Serial.print(" failed, Error: ");
    Serial.print(client.state());
    Serial.println(" Retrying in 5 seconds");
  }
}


void callback(char *topic, byte *payload, unsigned int length) {

  // Clearing the global string buffers
  memset(callback_data.topic, 0, 100);
  memset(callback_data.payload, 0, 500);
  //Conver *byte to char*
  payload[length] = '\0';  //First terminate payload with a NULL
  // Break topic down
  Serial.print("Message arrived: ");
  Serial.print(topic);

  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    callback_data.payload[i] = (char)payload[i];
  }
  Serial.println();

  memcpy(callback_data.payload, payload, length);
  memcpy(callback_data.topic, topic, strlen(topic));

  Serial.print("Payload:");
  Serial.println(callback_data.payload);
  Serial.print(" Topic:");
  Serial.println(callback_data.topic);
  callback_data.dataArrives = true;
}

//------------------------------------------------------------------------------
//  static void InitMQTTClient(void )
//
//  This function is used to initialized the MQTT Client
//------------------------------------------------------------------------------
void InitMQTTClient(void) {
  if (GetActiveInterface() == COMMUNICATION_ETHERNET)
  {
    client.setClient(espEthClient);
    client.setServer(mqtt_broker, 1883);
    client.setCallback(callback);
  }
  else if (GetActiveInterface() == COMMUNICATION_WIFI_DYNAMIC || GetActiveInterface() == COMMUNICATION_WIFI_STATIC)
  {
    client.setClient(espWiFiClient);
    client.setServer(mqtt_broker, 1883);
    client.setCallback(callback);
  }

}

//------------------------------------------------------------------------------
//  bool GetMQTTClientConnectionStatus(void)
//
//  This function is used to Get the status of MQTT Connection
//------------------------------------------------------------------------------
bool GetMQTTClientConnectionStatus(void)
{
  return client.connected();
}


//------------------------------------------------------------------------------
//  void MQTTRefreshConnection(void)
//
//  This function is used to refresh MQTT Connection
//------------------------------------------------------------------------------
void MQTTRefreshConnection(void)
{
  client.loop();
}
