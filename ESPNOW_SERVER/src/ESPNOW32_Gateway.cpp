#define BLYNK_TEMPLATE_ID "TMPLnpPSbLMm"
#define BLYNK_TEMPLATE_NAME "WeatherStationESP32"
//#define BLYNK_AUTH_TOKEN "zbWt2kx6i8-geYh2uMTULrp2YTPQbkxc"

#define BLYNK_FIRMWARE_VERSION "0.3.19"
#define BLYNK_PRINT Serial
#define APP_DEBUG
#define USE_ESP32_DEV_MODULE
#define CONFIG_DEVICE_PREFIX "StaMet-Jerukagung"

#include <ArduinoJson.h>
#include <StreamUtils.h>
#include "BlynkEdgent.h"
//#include <BlynkSimpleEsp32.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClient.h>

//char ssid[] = "JerukagungMeteorologi";
//char pass[] = "meteorologi";

String recv_str_jsondata;

StaticJsonDocument<256> doc_send;

//String apiKey1 = "1CAB7D9FTRFKIJDY"; //API Thingspeak Prototipe
String apiKey1 = "S1VN6AN4R1QT4QMD";  //API Thingspeak Jerukagung Meteorologi

//String wid = "20827bd2c71d5d59";
//String apiKey2 = "bea9f057d1af19cfdd8ba31bb7b367f9"; // Jerukagung Climate

String wid = "e28208f1755c7f97";
String apiKey2 = "151d25fa6b39a591ea9651f1a110e2ad";  // Jerukagung Meteorologi

//Definisi Pin UART
#define RXD2 16
#define TXD2 17

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  pinMode(2, OUTPUT);
  //Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  BlynkEdgent.begin();
}
/*unsigned int checkStatusPeriode = 120000;
unsigned int checkStatusNext;
void connectionstatus() {
  if ((WiFi.status() != WL_CONNECTED) ) {
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.begin(ssid, pass);
    while (WiFi.status() != WL_CONNECTED) {
      delay(100);
      Serial.print(".");
    }
    Serial.println();
    Serial.println(WiFi.localIP());
    //Alternatively, you can restart your board
    //ESP.restart();
  } else {
    Serial.println("wifi OK");
  }

  if (!Blynk.connected() ) {
    Serial.println("Lost Blynk server connection");
    Blynk.connect();
  } else {
    Serial.println("Blynk OK");
  }
}*/

ReadBufferingStream bufferedSerial2(Serial2, 64);
void Data() {
  // Recieving data (JSON) from Coordinator ESP
  if (bufferedSerial2.available()) {
    StaticJsonDocument<256> doc_recv;
    Serial2.setTimeout(5000);
    DeserializationError error = deserializeJson(doc_recv, bufferedSerial2);
    if (error == DeserializationError::Ok) { 
    digitalWrite(2, HIGH);
      float temp1 = doc_recv["temp"];
      float temp2 = doc_recv["temp2"];
      float humi1 = doc_recv["humi"];
      float humi2 = doc_recv["humi2"];
      float dew = doc_recv["dew"];
      float heat = doc_recv["heat"];
      float press = doc_recv["press"];
      float volt = doc_recv["volt"];
      int rssi = WiFi.RSSI();
      int ram = ESP.getFreeHeap();
      // Konversi nilai jika temp adalah 0
      //filter NULL
      if (temp1 == 0) {
        temp1 = NAN; // Set temp menjadi NaN (Not a Number)
      }
      if (temp2 == 0) {
        temp2 = NAN; // Set temp menjadi NaN (Not a Number)
      }
      if (humi1 == 0) {
        humi1 = NAN; // Set temp menjadi NaN (Not a Number)
      }
      if (humi2 == 0) {
        humi2 = NAN; // Set temp menjadi NaN (Not a Number)
      }
      float temperature = (temp1+temp2)/2;
      float humidity = (humi1+humi2)/2;

      Blynk.virtualWrite(V0, temperature);
      Blynk.virtualWrite(V1, humidity);
      Blynk.virtualWrite(V2, press);
      Blynk.virtualWrite(V3, dew);

      if (press == 0) {
        press = NAN; // Set temp menjadi NaN (Not a Number)
      }
      if (dew == 0){
        dew = NAN;
      }
      if (heat == 0){
        heat = NAN;
      }
      Blynk.virtualWrite(V8, volt);
      Blynk.virtualWrite(V9, rssi);
      Blynk.virtualWrite(V10, ram);

      /*Serial.println(t1);
      Serial.println(t2);
      Serial.println(temp);
      Serial.println(h1);
      Serial.println(h2);
      Serial.println(humi);
      Serial.println(press);
      Serial.println(dew);
      Serial.println(he);
      Serial.println(volt);*/ 
     
      WiFiClient client;
      HTTPClient http;
      http.setTimeout(2000);

      String url1 = "http://api.thingspeak.com/update?api_key=" + apiKey1;
      url1 += "&field1=" + String(temperature);
      url1 += "&field2=" + String(humidity);
      url1 += "&field3=" + String(press);
      url1 += "&field4=" + String(dew);
      url1 += "&field8=" + String(volt);
      // Send HTTP POST request
      http.begin(client, url1);
      int httpResponseCode1 = http.GET();
      if (httpResponseCode1 > 0) {
        Serial.print("Thingspeak Response code: ");
        Serial.println(httpResponseCode1);
        String payload = http.getString();
        Serial.println(payload);
      } 
      else {
        Serial.print("Error code Thingspeak: ");
        Serial.println(httpResponseCode1);
      }

      // Free resources
      http.end();

      String url2 = "http://api.weathercloud.net/set/wid/" + wid + "/key/" + apiKey2;
      url2 += "/temp/" + String(temperature * 10);
      url2 += "/hum/" + String(humidity);
      url2 += "/bar/" + String(press * 10);
      url2 += "/dew/" + String(dew * 10);
      url2 += "/heat/" + String(heat * 10);
      //url2 += "/chill/" + String(dewpoint * 10);
      url2 += "/wspd/" + String(0);
      url2 += "/wspdhi/" + String(0);
      url2 += "/wdir/" + String(0);
      url2 += "/rain/" + String(0 * 10);
      url2 += "/rainrate/" + String(0 * 10);
      //url2 += "/solarrad/" + String(0 * 10);
      //url2 += "/et/" + String(0 * 10);
      //url2 += "/uvi/" + String(0 * 10);

      http.begin(client, url2);
      int httpResponseCode2 = http.GET();
      if (httpResponseCode2 > 0) {
        Serial.print("Weathercloud Response code: ");
        Serial.println(httpResponseCode2);
        String payload = http.getString();
        Serial.println(payload);
      } 
      else {
        Serial.print("Error code Weathercloud: ");
        Serial.println(httpResponseCode2);
      }

      http.end();
      digitalWrite(2, LOW);
    } 
    else {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());

      while(bufferedSerial2.available()>0);
        bufferedSerial2.read();
      return;
    }
  }
  if (Serial.available()) {
    Serial2.write(Serial.read());
    }
  }

void loop() {
  /*if (checkStatusNext<=millis() && WiFi.status() !=WL_CONNECTED) {
  connectionstatus();
  checkStatusNext = millis() + checkStatusPeriode;
  }*/
  BlynkEdgent.run();
  Data();
}