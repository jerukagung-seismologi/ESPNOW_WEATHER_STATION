#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_SSD1306.h>

// OLED Display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// RTC
RTC_DS3231 rtc;
String dayStamp, timeStamp;

// ESP-NOW MAC Address
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t newMACAddress[] = {0xD2, 0x6B, 0x27, 0x1F, 0x58, 0xBE};

// JSON Document for incoming ESP-NOW data
String recv_jsondata;

void getTimeStamp() {
  DateTime now = rtc.now();
  timeStamp = String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second());
  dayStamp = String(now.day()) + "-" + String(now.month()) + "-" + String(now.year());
}

void displayData(float temperature, float humidity, float pressure, float dewpoint, float voltage) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  display.print("Waktu: ");
  display.println(timeStamp);
  
  display.print("Temp: ");
  display.print(temperature);
  display.println(" C");

  display.print("Humid: ");
  display.print(humidity);
  display.println(" %");

  display.print("Pressure: ");
  display.print(pressure);
  display.println(" hPa");

  display.print("Dew Point: ");
  display.print(dewpoint);
  display.println(" C");

  display.print("Voltage: ");
  display.print(voltage);
  display.println(" V");

  display.display();
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  JsonDocument doc_from_espnow;
  char *buff = (char *)incomingData;
  recv_jsondata = String(buff);

  Serial.print("Received from ESPNOW: ");
  Serial.println(recv_jsondata);

  DeserializationError error = deserializeJson(doc_from_espnow, recv_jsondata);
  if (!error) {
    float t1 = doc_from_espnow["t1"];
    float t2 = doc_from_espnow["t2"];
    float h1 = doc_from_espnow["h1"];
    float h2 = doc_from_espnow["h2"];
    float p = doc_from_espnow["p"];
    float v = doc_from_espnow["v"];

    // Calculate average temperature and humidity
    float temperature = (t1 + t2) / 2;
    float humidity = (h1 + h2) / 2;
    
    float tempA = temperature/100;
    float humidA = humidity/100;

    float press = p/100;
    float volt = v/100;

    // Calculate dew point and heat index
    double calc = log(tempA / 100.0F) + ((17.625F * tempA) / (243.04F + tempA));
    double dewpoint = (243.04F * calc / (17.625F - calc));

    // Get timestamp from RTC
    getTimeStamp();

    // Display data on OLED
    displayData(tempA, humidA, press, dewpoint, volt);

    Serial.println("Data displayed on OLED.");
  } else {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
  }
}

void setup() {
  Serial.begin(115200);

  // Initialize WiFi and ESP-NOW
  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA, &newMACAddress[0]);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  // Initialize RTC
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Initialize OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  display.clearDisplay();
}

void loop() {
  // No code needed in loop; ESP-NOW will trigger the OnDataRecv callback automatically
}
