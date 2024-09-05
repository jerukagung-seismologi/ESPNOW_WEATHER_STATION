#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <ArduinoJson.h>
#include <StreamUtils.h>

// Libraries for SD card
#include "FS.h"
#include "SD.h"
#include <SPI.h>

// Libraries for RTC
#include <Wire.h>
#include <RTClib.h>

// Define CS pin for the SD card module
#define SD_CS 5
#define RXD2 16
#define TXD2 17

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t newMACAddress[] = {0xD2, 0x6B, 0x27, 0x1F, 0x58, 0xBE};

String recv_jsondata;
RTC_DS3231 rtc;
String dayStamp, timeStamp;
JsonDocument doc_to_espnow; // JSON Doc for Transmitting data to ESPNOW Devices

void getTimeStamp() {
  DateTime now = rtc.now();
  timeStamp = String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second());
  dayStamp = String(now.day()) + "-" + String(now.month()) + "-" + String(now.year());
}

void writeFile(fs::FS &fs, const char *path, const char *message) {
  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  file.print(message);
  file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message) {
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  file.print(message);
  file.close();
}

void logSDCard(String data) {
  String dataMessage = dayStamp + " " + timeStamp + ", " + data + "\r\n";
  appendFile(SD, "/data.txt", dataMessage.c_str());
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  JsonDocument doc_from_espnow;
  char *buff = (char *)incomingData;
  recv_jsondata = String(buff);
  Serial.print("Received from ESPNOW: ");
  Serial.println(recv_jsondata);

  DeserializationError error = deserializeJson(doc_from_espnow, recv_jsondata);
  if (!error) {
    digitalWrite(2, HIGH);

    float t1 = doc_from_espnow["t1"];
    float t2 = doc_from_espnow["t2"];
    float h1 = doc_from_espnow["h1"];
    float h2 = doc_from_espnow["h2"];
    float press = doc_from_espnow["p"];
    float volt = doc_from_espnow["v"];

    t1 = t1 / 100;
    t2 = t2 / 100;
    h1 = h1 / 100;
    h2 = h2 / 100;
    press = press / 100;
    volt = volt / 100;

    float temperature = (t1 + t2) / 2;
    float humidity = (h1 + h2) / 2;

    double calc = log(humidity / 100.0F) + ((17.625F * temperature) / (243.04F + temperature));
    double dewpoint = (243.04F * calc / (17.625F - calc));
    double heat = (0.8 * temperature) + ((humidity * temperature / 500));

    Serial.println(t1);
    Serial.println(t2);
    Serial.println(temperature);
    Serial.println(h1);
    Serial.println(h2);
    Serial.println(humidity);
    Serial.println(dewpoint);
    Serial.println(press);
    Serial.println(heat);
    Serial.println(volt);

    getTimeStamp();
    String data = String(t1) + ", " + String(t2) + ", " + String(h1) + ", " + String(h2) + ", " + String(dewpoint) + ", " + String(heat) + ", " + String(press) + ", " + String(volt);
    logSDCard(data);

    Serial.println("Data logged to SD card.");

    JsonDocument doc_to_Serial2;
    doc_to_Serial2["temp"] = t1;
    doc_to_Serial2["temp2"] = t2;
    doc_to_Serial2["humi"] = h1;
    doc_to_Serial2["humi2"] = h2;
    doc_to_Serial2["dew"] = dewpoint;
    doc_to_Serial2["heat"] = heat;
    doc_to_Serial2["press"] = press;
    doc_to_Serial2["volt"] = volt;

    WriteBufferingStream bufferedSerial2(Serial2, 64);
    serializeJson(doc_to_Serial2, Serial2);
    bufferedSerial2.flush();

    digitalWrite(2, LOW);
  } else {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA, &newMACAddress[0]);

  pinMode(2, OUTPUT);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  if (!SD.begin(SD_CS)) {
    Serial.println("ERROR - SD card initialization failed!");
    return;
  }

  File file = SD.open("/data.txt");
  if (!file) {
    writeFile(SD, "/data.txt", "Timestamp, Temp1, Temp2, Humidity1, Humidity2, Dew Point, Heat Index, Pressure, Voltage \r\n");
  }
  file.close();
}

void loop() {
  if (Serial2.available()) {
    String recv_str_jsondata = Serial2.readStringUntil('\n');
    serializeJson(doc_to_espnow, recv_str_jsondata);
    Serial.println(recv_str_jsondata);

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)recv_str_jsondata.c_str(), sizeof(recv_str_jsondata) * recv_str_jsondata.length());

    if (result == ESP_OK) {
      Serial.println("Sent with success");
    } else {
      Serial.println(result);
    }
  }
}
