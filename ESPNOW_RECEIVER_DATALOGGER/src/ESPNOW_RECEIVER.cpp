#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <ArduinoJson.h>
#include <StreamUtils.h>

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Set your new MAC Address (Receiver MAC Address)
//uint8_t newMACAddress[] = {0x7C, 0x9E, 0xBD, 0x53, 0x97, 0xF8};
uint8_t newMACAddress[] = {0xD2, 0x6B, 0x27, 0x1F, 0x58, 0xBE};
StaticJsonDocument<256> doc_to_espnow; // JSON Doc for Transmitting data to ESPNOW Devices

String recv_jsondata;

#define RXD2 16
#define TXD2 17

// Create a struct_message called myData

// ESPNOW Send Callback Function
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  StaticJsonDocument<256> doc_from_espnow; // JSON Doc for Receiving data from ESPNOW Devices
  char* buff = (char*) incomingData;
  recv_jsondata = String(buff);
  Serial.print("Recieved from ESPNOW: "); Serial.println(recv_jsondata);
  DeserializationError error = deserializeJson(doc_from_espnow, recv_jsondata);
  if (!error) {
  digitalWrite(2, HIGH);
      float t1 = doc_from_espnow["t1"];
      float t2 = doc_from_espnow["t2"];
      float h1 = doc_from_espnow["h1"];
      float h2 = doc_from_espnow["h2"];
      float press = doc_from_espnow["p"];
      float volt = doc_from_espnow["v"];
  
  t1 = t1/100;
  t2 = t2/100;
  h1 = h1/100;
  h2 = h2/100;
  press  = press/100;
  volt  = volt/100;
  //Real Value
  float temperature = (t1+t2)/2;
  float humidity = (h1+h2)/2;

  double calc = log(humidity / 100.0F) + ((17.625F * temperature) / (243.04F + temperature));
  double dewpoint = (243.04F * calc / (17.625F - calc));

  double heat = (0.8*temperature)+((humidity*temperature/500));

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
  /*Serial.print("Serailizing to Serial2: ");
  Serial.println(recv_jsondata);*/
  Serial.println(len); 
  StaticJsonDocument<256> doc_to_Serial2;
  doc_to_Serial2["temp"] = t1;
  doc_to_Serial2["temp2"] = t2;
  doc_to_Serial2["humi"] = h1; 
  doc_to_Serial2["humi2"] = h2;
  doc_to_Serial2["dew"] = dewpoint;
  doc_to_Serial2["heat"] = heat;
  doc_to_Serial2["press"] = press;
  doc_to_Serial2["volt"] = volt;
  WriteBufferingStream bufferedSerial2(Serial2, 64);        
  serializeJson(doc_to_Serial2, Serial2);            // Writing Data to Serial2
  bufferedSerial2.flush();
  digitalWrite(2, LOW);

  } else {
  Serial.print(F("deserializeJson() failed: "));
  Serial.println(error.f_str());
  return;
  }
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // ESP32 Board add-on after version > 1.0.5
  esp_wifi_set_mac(WIFI_IF_STA, &newMACAddress[0]);

  pinMode(2, OUTPUT);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the send and receive callback functions
  esp_now_register_send_cb(esp_now_send_cb_t(OnDataSent));
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  // Add peer information for broadcast
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  for (int ii = 0; ii < 6; ++ii ) {
    peerInfo.peer_addr[ii] = (uint8_t) broadcastAddress[ii];
  }
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}
 
void loop() {
 if (Serial2.available()) {
    // Recieving data (JSON) from BLYNK ESP
    String recv_str_jsondata = Serial2.readStringUntil('\n');

    //Serializing JSON
    serializeJson(doc_to_espnow, recv_str_jsondata);
    Serial.println(recv_str_jsondata);

    // Broadcasting data (JSON) via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) recv_str_jsondata.c_str(), sizeof(recv_str_jsondata) * recv_str_jsondata.length());

    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println(result);
    }
  }
}