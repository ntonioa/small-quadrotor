// #include <WiFi.h>
// #include "esp_camera.h"
// #include <WiFiUdp.h>

// // Replace with your network credentials and PC destination
// const char* ssid = "LAPTOP-QES4EU9V 7607";
// const char* password = "12345678";
// const char* pc_ip = "192.168.137.156";  // PC IP address
// const uint16_t pc_port = 4210;         // PC listening port

// // Camera configuration pins for AI Thinker module
// #define PWDN_GPIO_NUM     32
// #define RESET_GPIO_NUM    -1
// #define XCLK_GPIO_NUM     0
// #define SIOD_GPIO_NUM     26
// #define SIOC_GPIO_NUM     27

// #define Y9_GPIO_NUM       35
// #define Y8_GPIO_NUM       34
// #define Y7_GPIO_NUM       39
// #define Y6_GPIO_NUM       36
// #define Y5_GPIO_NUM       21
// #define Y4_GPIO_NUM       19
// #define Y3_GPIO_NUM       18
// #define Y2_GPIO_NUM       5

// #define VSYNC_GPIO_NUM    25
// #define HREF_GPIO_NUM     23
// #define PCLK_GPIO_NUM     22

// // Desired FPS
// const int TARGET_FPS = 10;
// const int FRAME_DELAY_MS = 1000 / TARGET_FPS;

// WiFiUDP Udp;

// void setup() {
//   Serial.begin(115200);
  
//   // Initialize camera
//   camera_config_t config;
//   config.ledc_channel = LEDC_CHANNEL_0;
//   config.ledc_timer   = LEDC_TIMER_0;
//   config.pin_d0       = Y2_GPIO_NUM;
//   config.pin_d1       = Y3_GPIO_NUM;
//   config.pin_d2       = Y4_GPIO_NUM;
//   config.pin_d3       = Y5_GPIO_NUM;
//   config.pin_d4       = Y6_GPIO_NUM;
//   config.pin_d5       = Y7_GPIO_NUM;
//   config.pin_d6       = Y8_GPIO_NUM;
//   config.pin_d7       = Y9_GPIO_NUM;
//   config.pin_xclk     = XCLK_GPIO_NUM;
//   config.pin_pclk     = PCLK_GPIO_NUM;
//   config.pin_vsync    = VSYNC_GPIO_NUM;
//   config.pin_href     = HREF_GPIO_NUM;
//   config.pin_sscb_sda = SIOD_GPIO_NUM;
//   config.pin_sscb_scl = SIOC_GPIO_NUM;
//   config.pin_pwdn     = PWDN_GPIO_NUM;
//   config.pin_reset    = RESET_GPIO_NUM;
//   config.xclk_freq_hz = 20000000;
//   config.pixel_format = PIXFORMAT_RGB565;
//   config.frame_size   = FRAMESIZE_QVGA; // 320x240
//   config.jpeg_quality = 12;
//   config.fb_count     = 1;

//   esp_err_t err = esp_camera_init(&config);
//   if (err != ESP_OK) {
//     Serial.printf("Camera init failed with error 0x%x", err);
//     return;
//   }

//   // Connect to Wi-Fi
//   WiFi.begin(ssid, password);
//   Serial.print("Connecting to WiFi");
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print(".");
//   }
//   Serial.println("\nWiFi connected");

//   Udp.begin(pc_port);
// }

// void loop() {
//   unsigned long start = millis();

//   // Capture frame
//   camera_fb_t * fb = esp_camera_fb_get();
//   if (!fb) {
//     Serial.println("Camera capture failed");
//     return;
//   }

//   int width = fb->width;
//   int height = fb->height;
//   uint16_t * buf = (uint16_t *)fb->buf;

//   // Variables for centroid
//   uint32_t sum_x = 0, sum_y = 0, count = 0;
  
//   // Threshold red channel
//   for (int y = 0; y < height; y++) {
//     for (int x = 0; x < width; x++) {
//       uint16_t pixel = buf[y * width + x];
//       // RGB565 -> extract R, G, B
//       uint8_t r = (pixel >> 11) * 8;
//       uint8_t g = ((pixel >> 5) & 0x3F) * 4;
//       uint8_t b = (pixel & 0x1F) * 8;
//       // Simple red detection
//       if (r > 150 && r > g + 40 && r > b + 40) {
//         sum_x += x;
//         sum_y += y;
//         count++;
//       }
//     }
//   }

//   float cx = 0, cy = 0;
//   if (count > 0) {
//     cx = (float)sum_x / count;
//     cy = (float)sum_y / count;
//   }

//   // Send coordinates
//   char msg[64];
//   snprintf(msg, sizeof(msg), "%.2f,%.2f", cx, cy);
//   Udp.beginPacket(pc_ip, pc_port);
//   Udp.write((uint8_t*)msg, strlen(msg));
//   Udp.endPacket();

//   Serial.printf("Centroid: x=%.2f, y=%.2f, pixels=%u\n", cx, cy, count);

//   esp_camera_fb_return(fb);

//   // Maintain target FPS
//   unsigned long elapsed = millis() - start;
//   if (elapsed < FRAME_DELAY_MS) {
//     delay(FRAME_DELAY_MS - elapsed);
//   }
// }


#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_camera.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "camera_index.h"
#include "esp_http_server.h"

// Wi-Fi
const char* ssid     = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";
// UDP
const char*  pc_ip   = "192.168.1.100";
const uint16_t pc_port = 4210;

// Camera AI-Thinker pins
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

WiFiUDP Udp;
httpd_handle_t stream_httpd = NULL;

// Handler per lo streaming MJPEG
static esp_err_t stream_handler(httpd_req_t *req){
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  char part_buf[64];

  // Header multipart
  res = httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");
  if(res != ESP_OK) return res;

  while(true){
    fb = esp_camera_fb_get();
    if(!fb) {
      Serial.println("Camera capture failed");
      return ESP_FAIL;
    }
    // invio MJPEG
    size_t hlen = snprintf(part_buf, 64,
      "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", fb->len);
    httpd_resp_send_chunk(req, part_buf, hlen);
    httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
    httpd_resp_send_chunk(req, "\r\n", 2);

    esp_camera_fb_return(fb);
    
    // Breve delay per evitare flooding
    if(httpd_req_to_sockfd(req) < 0) break;
  }
  return res;
}

// Avvia server HTTP con endpoint /stream
void startCameraServer(){
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 81;

  httpd_uri_t stream_uri = {
    .uri       = "/stream",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };
  if(httpd_start(&stream_httpd, &config) == ESP_OK){
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}

void setup(){
  Serial.begin(115200);
  // Inizializza camera
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
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size   = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count     = 1;
  if(esp_camera_init(&config) != ESP_OK){
    Serial.println("Camera init failed");
    return;
  }

  // Connetti Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connessione WiFi");
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connesso: " + WiFi.localIP().toString());

  // Inizializza UDP
  Udp.begin(pc_port);

  // Avvia stream MJPEG
  startCameraServer();
}

void loop(){
  // Cattura frame per centroid
  camera_fb_t * fb = esp_camera_fb_get();
  if(!fb){
    Serial.println("Capture failed");
    return;
  }
  int w = fb->width, h = fb->height;
  uint16_t * buf = (uint16_t *)fb->buf;
  uint32_t sx=0, sy=0, cnt=0;

  // Estrai rosso da RGB565
  for(int y=0; y<h; y++){
    for(int x=0; x<w; x++){
      uint16_t p = buf[y*w + x];
      uint8_t r = (p >> 11) * 8;
      uint8_t g = ((p >>5) & 0x3F) *4;
      uint8_t b = (p & 0x1F)*8;
      if(r>150 && r>g+40 && r>b+40){
        sx+=x; sy+=y; cnt++;
      }
    }
  }
  float cx=0, cy=0;
  if(cnt){
    cx = (float)sx/cnt;
    cy = (float)sy/cnt;
  }
  // Invia UDP
  char msg[32];
  snprintf(msg, sizeof(msg), "%.1f,%.1f", cx, cy);
  Udp.beginPacket(pc_ip, pc_port);
  Udp.write((uint8_t*)msg, strlen(msg));
  Udp.endPacket();

  esp_camera_fb_return(fb);
  delay(100);  // ~10 FPS per centroid
}
