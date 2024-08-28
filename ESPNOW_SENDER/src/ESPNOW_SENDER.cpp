#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_SHT31.h"
#include "Adafruit_SHT4x.h"
#include <Adafruit_BMP280.h>
#include "Adafruit_MAX1704X.h"
#define ARDUINOJSON_USE_DOUBLE 0
#include <ArduinoJson.h>

uint64_t uS_TO_S_FACTOR = 1000000;  // Conversion factor for micro seconds to seconds
uint64_t TIME_TO_SLEEP = 10; //sleep time in seconds

int ledPin = 2 ;
Adafruit_SHT31 sht31 = Adafruit_SHT31();
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
Adafruit_BMP280 bmp;
Adafruit_MAX17048 maxlipo;

// REPLACE WITH YOUR RECEIVER MAC Address
//uint8_t broadcastAddress1[] = {0x7C, 0x9E, 0xBD, 0x53, 0x97, 0xF8}; //Utama
//uint8_t broadcastAddress2[] = {0x0C, 0x8B, 0x95, 0x75, 0xDE, 0xA0}; //Debug
//uint8_t broadcastAddress3[] = {0x40, 0x22, 0xD8, 0x08, 0x83, 0x5C};
//D2-6B-27-1F-58-BE
uint8_t broadcastAddress[] = {0xD2, 0x6B, 0x27, 0x1F, 0x58, 0xBE};
String recv_jsondata;
String send_jsondata;
JsonDocument doc_from_espnow;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  //Inisiasi SHT31
  if (!sht31.begin(0x45)) {   // Set to 0x45 for alternate i2c addr
    Serial.println("Could not find a valid SHT31 sensor, check wiring!");
    while (1);
  }
  //Inisiasi SHT4x
  if (!sht4.begin()) {   // Set to 0x45 for alternate i2c addr
    Serial.println("Could not find a valid SHT4x sensor, check wiring!");
    while (1);
  }
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  sht4.setHeater(SHT4X_NO_HEATER);
  //Inisiasi BMP280
  if (!bmp.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  //setSampling(sensor_mode mode = MODE_NORMAL, sensor_sampling tempSampling = SAMPLING_X16, sensor_sampling pressSampling = SAMPLING_X16, sensor_filter filter = FILTER_OFF, standby_duration duration = STANDBY_MS_1) -> void
  /*bmp.setSampling(Adafruit_BMP280::MODE_FORCED,
                  Adafruit_BMP280::SAMPLING_X1, // temperature
                  Adafruit_BMP280::SAMPLING_X16, // pressure
                  Adafruit_BMP280::FILTER_OFF,
                  Adafruit_BMP280::STANDBY_MS_500);*/
  //Inisiasi MAXLIPO
  if (!maxlipo.begin()) {
    Serial.println("Could not find a valid MAX17048 sensor, check wiring!");
    while (1);
  }
  //Setting batas bawah dan atas voltase
  maxlipo.setAlertVoltages(2.0, 4.2);
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);   
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer Addres 0");
    return;
  }

  sensors_event_t humid, temp;
  sht4.getEvent(&humid, &temp);
  int temperature = (sht31.readTemperature()*100);
  int humidity    = (sht31.readHumidity()*100);
  int temperature2 = (temp.temperature)*100;
  int humidity2    = (humid.relative_humidity)*100;
  //double temperature2 = (bmp.readTemperature());
  int pressure    = (bmp.readPressure());
  int volt = (maxlipo.cellVoltage()*100);
  
  /*double calc = log(humidity / 100.0F) + ((17.625F * temperature) / (243.04F + temperature));
  double dewpoint = (243.04F * calc / (17.625F - calc));

  #define c1 -8.78469475556
  #define c2 1.61139411
  #define c3 2.33854883889
  #define c4 -0.14611605
  #define c5 -0.012308094
  #define c6 -0.0164248277778
  #define c7 0.002211732
  #define c8 0.00072546
  #define c9 -0.000003582
  double heat = (c1 + (c2 * temperature) + (c3 * humidity) + (c4 * temperature * humidity) + (c5 * sq(temperature)) + (c6 * sq(humidity)) + (c7 * sq(temperature) * humidity) + (c8 * temperature * sq(humidity)) + (c9 * sq(temperature) * sq(humidity))); 
  */

  StaticJsonDocument<180> doc_to_espnow;
  doc_to_espnow["t1"] = temperature;
  doc_to_espnow["t2"] = temperature2;
  doc_to_espnow["h1"] = humidity;  
  doc_to_espnow["h2"] = humidity2;
  //doc_to_espnow["sotemp"] = soiltemp;
  doc_to_espnow["p"] = pressure;
  doc_to_espnow["v"] = volt;

  serializeJson(doc_to_espnow, send_jsondata);
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(0, (uint8_t *) send_jsondata.c_str(), send_jsondata.length());
  //esp_now_send(const uint8_t *peer_addr, const uint8_t *data, size_t len);
  //Serial.println(send_jsondata); 
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }
  Serial.flush();
  Serial.end();
  delay(50);
  //timestamp = millis() - timestamp;
  //Serial.println(timestamp);
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Going to sleep now");
  //delay(50);
  digitalWrite(ledPin, LOW);
  esp_deep_sleep_start();  
  Serial.println("Harusnya kagak muncul");
}

void loop() {
  //namanya juga deep sleep ya kagak looping njir
}
