#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
//#include <WiFiClientSecure.h>

const char* ssid = "EXPERIMENT";
const char* password = "Xjok23dl!?ifenel";

//Don't use https
String serverName = "http://maker.ifttt.com/trigger/Fall_Detected/with/key/d9OBb_vwlj1VGq7XDfcXkTi0Aq9gARHuLNIq899ev0l";

int fall = 0;
int fall_pin = 2;
int fall_flag = 0;

unsigned long lastTime = 0;
unsigned long timerDelay = 5000;

void setup() {

  Serial.begin(115200);
  delay(10);

  pinMode(fall_pin, OUTPUT);
  digitalWrite(fall_pin, HIGH); //Set the PIN to be rising initially

  WiFi.begin(ssid, password);
  Serial.println("");
  Serial.print("Waiting to connect to WiFi... ");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  delay(1000);

  attachInterrupt(digitalPinToInterrupt(fall_pin), send_info, FALLING);
}

void loop() {
  if (fall_flag == 0) Serial.println("No Fall Detected");
  if (fall_flag) {Serial.println("Fall Detected"); send_to_cloud();}
  if (fall_flag) fall_flag = 0;
}


ICACHE_RAM_ATTR void send_info() {
  fall_flag = 1;
}

void send_to_cloud() {

  //Send an HTTP POST request every 10 seconds
  //if ((millis() - lastTime) > timerDelay) {
  //Check WiFi connection status
  if (WiFi.status() == WL_CONNECTED) {
    WiFiClient client;
    HTTPClient http;

    // Your Domain name with URL path or IP address with path
    http.begin(client, serverName.c_str());
    //http.addHeader("Content-Type", "application/x-www-form-urlencoded");

    // Send HTTP POST request
    int httpResponseCode = http.GET();

    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);

    // Free resources
    http.end();
  } else {
    Serial.println("WiFi Disconnected");
  }
  lastTime = millis();
  //  }
}