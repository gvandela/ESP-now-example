
#include <stdint.h>
#include <Wire.h>
#include <ESP8266WiFi.h> // Enables the ESP8266 to connect to the local network (via WiFi)
#include <espnow.h>

//sensor libs
#include <Adafruit_BMP280.h>
// #include <Sodaq_SHT2x.h>
#include <Adafruit_SHT31.h>


// uint32_t start_time = millis();

// #define PRINT_DELIVERY_STATUS
// #define AUTO_WIFI_CHANNEL

const char* ssid = "WIFI_SSID"; // replace with your network SSID

static const float fVoltageMatrix[22][2] = {  // battery voltage vs. percentage LUT
    {4.2,  100},
    {4.15, 95},
    {4.11, 90},
    {4.08, 85},
    {4.02, 80},
    {3.98, 75},
    {3.95, 70},
    {3.91, 65},
    {3.87, 60},
    {3.85, 55},
    {3.84, 50},
    {3.82, 45},
    {3.80, 40},
    {3.79, 35},
    {3.77, 30},
    {3.75, 25},
    {3.73, 20},
    {3.71, 15},
    {3.69, 10},
    {3.61, 5},
    {3.27, 0},
    {0, 0}
  };

typedef struct struct_send_message {  // ESP-now message struct
  float SHT_Tamb;
  float SHT_RH;
  float SHT_DP;
  float BMP_P;
  float BAT_V;
  float BAT_P;
} struct_send_message;
struct_send_message sendMsg;

//ESPnow
uint8_t broadcastAddress_server[] = {0xFC, 0xF5, 0xC4, 0x97, 0x1D, 0x48}; // replace with ESP-now server MAC
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) { // Report ESPnow delivery status
#ifdef PRINT_DELIVERY_STATUS
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0){
    Serial.println("Delivery success");
  }
  else{
    Serial.println("Delivery fail");
  }
#endif
}
// WiFi channel needs to match for ESP-now, automatic channel selection by scanning costs a lot of time/energy though
int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
    for (uint8_t i=0; i<n; i++) {
      if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
        return WiFi.channel(i);
      }
    }
  }
  return 0;
}

Adafruit_BMP280 bmp280;                // I2C
Adafruit_SHT31 sht31 = Adafruit_SHT31();

double computeDewPoint(double celsius, double humidity) {
  //https://gist.github.com/Mausy5043/4179a715d616e6ad8a4eababee7e0281
  double RATIO = 373.15 / (273.15 + celsius);
  double SUM = -7.90298 * (RATIO - 1);
  SUM += 5.02808 * log10(RATIO);
  SUM += -1.3816e-7 * (pow(10, (11.344 * (1 - 1/RATIO ))) - 1) ;
  SUM += 8.1328e-3 * (pow(10, (-3.49149 * (RATIO - 1))) - 1) ;
  SUM += log10(1013.246);
  double VP = pow(10, SUM - 3) * humidity;
  double T = log(VP/0.61078);   // temp var
  return (241.88 * T) / (17.558 - T);
}

void start_espnow(void) {
  // ESP-NOW WIFI_OPERATION
  WiFi.mode(WIFI_STA);
  // WiFi.disconnect();
#ifdef AUTO_WIFI_CHANNEL
  int32_t channel = getWiFiChannel(ssid);    // costs over 2 seconds!!
#else
  int32_t channel = 6;  // manually configure correct channel!
#endif
  Serial.printf("\nWiFi Channel %d\n", channel);
  wifi_set_channel(channel);

  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_add_peer(broadcastAddress_server, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
  // esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  bmp280.begin(0x76);
  sht31.begin(0x44);
}

void loop() {
  // BMP280 measurements
  char* str_bmp = (char*)malloc(16 * sizeof(char));
  float temp_bmp = bmp280.readTemperature();
  float baro = bmp280.readPressure() / 100;
  // sprintf(str_bmp, "bmp280, %.2f, %.2f", temp_bmp, baro);
  // Serial.printf("BMP280 - temp: %.2f, baro: %.2f\n", temp_bmp, baro);

  // SHT21 measurements
  float hum = sht31.readHumidity();
  float temp_sht = sht31.readTemperature();
  char* str_sht = (char*)malloc(16 * sizeof(char));
  // sprintf(str_sht, "sht21, %.2f, %.2f", temp_sht, hum);
  // Serial.printf("SHT21 - temp: %.2f, RH: %.2f\n", temp_sht, hum);

  //Battery measurement
  char* str_bat = (char*)malloc(16 * sizeof(char));
  float perc = 100;
  int nVoltageRaw = analogRead(A0);
  float fVoltage = nVoltageRaw * 0.0055078;
  for(int i=20; i>=0; i--) {
    if(fVoltageMatrix[i][0] >= fVoltage) {
      perc = fVoltageMatrix[i + 1][1];
      break;
    }
  }
  // sprintf(str_bat, "BATT, %.3f, %.0f%%", fVoltage, perc);
  // Serial.printf("BAT - voltage: %.2f, %.0f%%\n", fVoltage, perc);

  sendMsg.SHT_Tamb = temp_sht;
  sendMsg.SHT_RH = hum;
  sendMsg.SHT_DP = computeDewPoint(sendMsg.SHT_Tamb, sendMsg.SHT_RH);
  sendMsg.BMP_P = baro;
  sendMsg.BAT_V = fVoltage;
  sendMsg.BAT_P = perc;
  start_espnow();
  esp_now_send(broadcastAddress_server, (uint8_t *) &sendMsg, sizeof(sendMsg));

  // Serial.printf("\nOperation time: %dms\n", millis()-start_time);

  ESP.deepSleep(1*60*1e6);  // time in microseconds
}
