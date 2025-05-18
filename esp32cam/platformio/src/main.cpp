#include <WiFi.h>
#include "esp_camera.h"
#include <WiFiUdp.h>
#include <algorithm>

// Network credentials and PC destination
const char* ssid = "Tenda_85DCD8";
const char* password = "antonio1";
const char* pc_ip = "192.168.0.105";  // PC IP address
const uint16_t pc_port = 4210;        // PC listening port

// Camera configuration pins for AI Thinker module
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM       5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// Desired FPS
const int TARGET_FPS = 10;
const int FRAME_DELAY_MS = 1000 / TARGET_FPS;

WiFiUDP Udp;

void setup() {
  Serial.begin(115200);
  
  // Initialize camera
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG; // Use JPEG for compression
  config.frame_size   = FRAMESIZE_QVGA; // 320x240
  config.jpeg_quality = 4;
  config.fb_count     = psramFound() ? 2 : 1; // Double buffer if PSRAM available

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  Udp.begin(pc_port);
}

void loop() {
  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected, reconnecting...");
    WiFi.reconnect();
    delay(5000);
    return;
  }

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    delay(FRAME_DELAY_MS);
    return;
  }

  // Split frame into UDP packets
  const int packetSize = 1400; // Optimized for MTU
  int totalPackets = (fb->len + packetSize - 1) / packetSize;

  for (int i = 0; i < totalPackets; i++) {
    int offset = i * packetSize;
    int size = std::min(packetSize, static_cast<int>(fb->len - offset));

    Udp.beginPacket(pc_ip, pc_port);
    Udp.write((uint8_t*)(fb->buf + offset), size);
    Udp.endPacket();
    yield(); // Allow background tasks
  }

  Serial.printf("Frame sent: %d bytes in %d packets\n", fb->len, totalPackets);

  esp_camera_fb_return(fb); // Free buffer
  delay(FRAME_DELAY_MS);    // Maintain frame rate
}