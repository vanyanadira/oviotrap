/*
  Development of the Low Cost OviTrap Device
  Maintained by HAM
  V0: (020922) create the base code; publish image to the broker server
  V1: (060922) create the subscribe system; publish image only if commanded;
              add adjustable LED brightness;
  V2: (150223) adjust the MQTT to communicate to real server; add wifi reconnect;
              add global node number variable; subscribe only specific topic;
  V2.1: (100323) add watchdog timer to anticipate ESP hang problem
*/
/* --------------------- library declaration -----------------------*/
#include <WiFi.h>
#include <PubSubClient.h>
#include "esp_camera.h" 
#include "Arduino.h"
#include "soc/soc.h"           // Disable brownout problems
#include "soc/rtc_cntl_reg.h"  // Disable brownout problems
#include "driver/rtc_io.h"
#include "Base64.h"
/* ---------------------- Wifi & Node Configuration ------------------------*/
// Update these with values suitable for your network.
// const char* ssid = "CINOVASI OPS LT 2";
// const char* password = "Wi070707";
// const char* ssid = "AAZ";
// const char* password = "Yumi1812";
const char* ssid = "Pestiplus";
const char* password = "Lebus1168";
const char* mqtt_server = "147.93.87.43";
String node_number = "101"; //Use this format: 01,02,03,...,10,11,12,...
// Topic String, Don't change unless its necessary
String req = "reqdengue/"+node_number;
String isc = "isconnect/"+node_number;
String con = "connected/"+node_number;
String reso = "resdengue/"+node_number;
/* --------------------- Camera Configuration -----------------------*/
// Pin definition for CAMERA_MODEL_AI_THINKER
#define Node_Number     1
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
camera_fb_t * fb = NULL;
int pictureNumber = 0;
/* --------------------- LED Setup -----------------------*/
int freq = 50000;
int ledCHannel = 2;
int res = 8;
const int ledPin = 4;
int brightness = 255; //50;
/* --------------------- Wifi Configuration -----------------------*/
WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE  (2048)
char msg[MSG_BUFFER_SIZE];
int value = 0;
// for wifi reconnecting
unsigned long previousMillis = 0;
unsigned long interval = 5000;
/* --------------------- Watchdog Timer -----------------------*/
#define WATCHDOG_TIMEOUT_S 300  // adjust this, for now it is set at 5 minutes
hw_timer_t * watchDogTimer = NULL;

void IRAM_ATTR watchDogInterrupt()
{
  Serial.println("reboot");
  ESP.restart();
}

void watchDogRefresh()
{
  timerWrite(watchDogTimer, 0); //reset timer (feed watchdog)
}
/* ----------------------- Setup -------------------------*/
void setup() {
  Serial.begin(115200);
  ledcSetup(ledCHannel, freq, res);
  ledcAttachPin(ledPin, ledCHannel);
  //  ledcWrite(ledCHannel, brightness);
  init_camera();
  setup_wifi();
  client.setServer(mqtt_server,1883);
  client.setCallback(callback);

  watchDogTimer = timerBegin(2, 80, true);
  timerAttachInterrupt(watchDogTimer, &watchDogInterrupt, true);
  timerAlarmWrite(watchDogTimer, WATCHDOG_TIMEOUT_S * 1000000, false);
  timerAlarmEnable(watchDogTimer);
}
/* ----------------------- Loop -------------------------*/
void loop() {

  unsigned long currentMillis = millis();
  if (WiFi.status() != WL_CONNECTED)
  {
    if (currentMillis - previousMillis >= interval)
    {
      Serial.print(millis());
      Serial.println("Reconnecting to WiFi...");
//      WiFi.disconnect();
      WiFi.reconnect();
      previousMillis = currentMillis;
    }
  }
  else
  {
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
  }
}
/* ------------------- void function -------------------*/
void setup_wifi()
{
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length)
{
  if (String(topic) == req)
  {
    Serial.println("taking_picture");
    take_picture();
  }
  else if (String(topic) == isc)
  {
    Serial.println("send_node_status");
    snprintf (msg, MSG_BUFFER_SIZE, "connected");
    client.publish(con.c_str(), msg);
    watchDogRefresh();
  }
}

void init_camera()
{
  // WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_VGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
  config.jpeg_quality = 8;
  config.fb_count = 1;

  // Init Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_vflip(s, 1); // flip it back
  s->set_brightness(s, 1); // up the brightness just a bit
  s->set_saturation(s, -2); // lower the saturation
}

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
// Create a random client ID
    String clientId = "ESP_CAM";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str()))
    {
      Serial.println("connected");
      client.subscribe(req.c_str());
      client.subscribe(isc.c_str());
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }

    if (WiFi.status() != WL_CONNECTED){break;}
  }
}

void take_picture()
{
  ledcWrite(ledCHannel, brightness);
  delay(1000);
  // Take Picture with Camera
  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  esp_camera_fb_return(fb); // dispose the buffered image
  fb = NULL; // reset to capture errors
  fb = esp_camera_fb_get();
  if (!fb)
  {
    Serial.println("Camera capture failed");
    return;
  }

  char *input = (char *)fb->buf;
  char output[base64_enc_len(3)];
  String post = "";
  for (int i = 0; i < fb->len; i++)
  {
    base64_encode(output, (input++), 3);
    if (i % 3 == 0) post += String(output);
  }

  int fbLen = post.length();
  Serial.println("publishing");
  Serial.println(fbLen);
  client.beginPublish(reso.c_str(), fbLen, true);
  //  Serial.println(post);

  String str = "";
  for (size_t n = 0; n < fbLen; n = n + 2048)
  {
    Serial.println(n);
    if (n + 2048 < fbLen)
    {
      str = post.substring(n, n + 2048);
      client.write((uint8_t*)str.c_str(), 2048);
    }
    else if (fbLen % 2048 > 0)
    {
      size_t remainder = fbLen % 2048;
      str = post.substring(n, n + remainder);
      client.write((uint8_t*)str.c_str(), remainder);
    }
  }
  Serial.println("end publishing");
  client.endPublish();
  // Serial.println(post);
  // Once you have finished with this image you tell it that this reserved memory can be released
  esp_camera_fb_return(fb);
  ledcWrite(ledCHannel, 0);
}
