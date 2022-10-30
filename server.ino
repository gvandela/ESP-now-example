#include <ESP8266WiFi.h>
#include <espnow.h>
#include <PubSubClient.h>

#define REPORT_PERIOD 5000
#define ENABLE_WIFI_MQTT

// REPLACE WITH RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Broadcast to all devices
uint8_t broadcastAddress_GreenHouse[]= {0x50, 0x02, 0x91, 0x79, 0x4C, 0xDE}; // GreenHouse 50:02:91:79:4C:DE
const char* MAC_GreenHouse = "50:02:91:79:4c:de";

// CONFIGURE FOR YOUR WIFI NETWORK / MQTT server
const char* ssid = "WIFI_SSID";
const char* wifi_password = "13245678";
const char* mqtt_server = "192.168.0.20";
const char* mqtt_username = "mqtt_username";
const char* mqtt_password = "mqtt_password";
const int mqtt_port = 1883;
// The client id identifies the ESP8266 device. Think of it a bit like a hostname (Or just a name, like Greg).
const char* clientID = "ESPnow server";
const char* mqtt_topic = "SENSORS";
// subscribe to:
const char* REPORT_SENSORS = "SENSORS/readout"; // can be whatever, just needs to line up with info in Node-RED
const char* GREENHOUSE = "greenhouse/readout";

WiFiClient wifiClient;
PubSubClient client(wifiClient);

uint32_t noReceivedMessages = 0;
uint32_t last_timestamp = 0;

//GreenHouse
typedef struct struct_green_sent_message {
  float SHT_Tamb;
  float SHT_RH;
  float SHT_DP;
  float BMP_P;
  float BAT_V;
  float BAT_P;
} struct_green_sent_message;
struct_green_sent_message recGreen;

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0){
    Serial.println("Delivery success");
  }
  else{
    Serial.println("Delivery fail");
  }
}

// Callback when data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.print("Packet received from: ");
  Serial.println(macStr);

  if (strcmp(macStr, MAC_GreenHouse) == 0) {
    Serial.println("Message from GreenHouse");
    memcpy(&recGreen, incomingData, sizeof(recGreen));
    char* str_json = (char*)malloc(256 * sizeof(char));
    sprintf(str_json, "{\"Tamb\":%f, \"RH\":%.5f, \"DP\":%.5f, \"Pres\":%.5f, \"BAT_V\":%.5f, \"BAT_P\":%.5f}",
      recGreen.SHT_Tamb, recGreen.SHT_RH, recGreen.SHT_DP, recGreen.BMP_P, recGreen.BAT_V, recGreen.BAT_P);
    client.publish(GREENHOUSE, str_json);
  }
}

// EXAMPLE OF TRANSMITTING TO SENSOR ON MQTT MESSAGE
void mqtt_callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  char message_str[length];

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
    message_str[i] = (char)message[i];
  }
  Serial.println(".");

  // For primary:
  // if (String(topic) == REQUEST_FAN) {
  //   if (messageTemp == "off") {
  //     setPrim.fanSpeed = 0;
  //     setPrim.autoFanControl = false;
  //     esp_now_send(broadcastAddress_ONTC_primary, (uint8_t *) &setPrim, sizeof(setPrim));
  //     Serial.println("FAN: OFF");
  //   } else {
  //     if (messageTemp == "auto") {
  //       setPrim.autoFanControl = true;
  //       esp_now_send(broadcastAddress_ONTC_primary, (uint8_t *) &setPrim, sizeof(setPrim));
  //       Serial.println("FAN: AUTO");
  //     } else {
  //       // let's hope the only other option is a number
  //       uint32_t pwm_value = atoi(message_str);
  //       if (pwm_value < 256) {
  //         setPrim.fanSpeed = pwm_value;
  //         setPrim.autoFanControl = false;
  //         esp_now_send(broadcastAddress_ONTC_primary, (uint8_t *) &setPrim, sizeof(setPrim));
  //         Serial.printf("FAN: %d\n", pwm_value);
  //       }
  //     }
  //   }
  // }
}
void mqttReconnect(void) {
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect(clientID, mqtt_username, mqtt_password)) {
      Serial.println("connected");

      // subscribe list
      client.subscribe(GREENHOUSE, 1); // QoS 1: THE MESSAGE IS GIVEN AT LEAST ONCE
      client.subscribe(REPORT_SENSORS, 1);

    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(100);
    }
  }
  client.loop();
}

/*************************************************************
    Setup
*************************************************************/
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Set the device as a Station and Soft Access Point simultaneously
  WiFi.mode(WIFI_AP_STA);

  // Set device as a Wi-Fi Station
  WiFi.begin(ssid, wifi_password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Setting as a Wi-Fi Station..");
  }
  Serial.print("Station IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Wi-Fi Channel: ");
  Serial.println(WiFi.channel());

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);  // Set ESP-NOW Role
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0); // Register peer
  esp_now_register_recv_cb(OnDataRecv); // register on receive callback functions
  esp_now_register_send_cb(OnDataSent); // register on transmit callback functions

  // Connect to MQTT Broker
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqtt_callback);
}
/*************************************************************
    Loop
*************************************************************/
void loop() {
  mqttReconnect();

  uint32_t current_timestamp = millis();
  if (current_timestamp > last_timestamp + REPORT_PERIOD) {
    last_timestamp = current_timestamp;

    int wifi_rssi = WiFi.RSSI();
    char* str_json = (char*)malloc(256 * sizeof(char));
    sprintf(str_json, "{\"RSSI\":%d}", wifi_rssi);
    client.publish(REPORT_SENSORS, str_json);
  }
}
