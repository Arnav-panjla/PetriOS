/*
 * ESP32-CAM (AI-Thinker) -> USB serial JPEG streamer for the Raspberry Pi.
 *
 * Each frame is sent as:  AA 55 AA 55 | uint32 length (little endian) | JPEG bytes
 * The Pi side (robot_arm/vision/esp32_serial.py) finds the magic, reads the JPEG
 * and serves it in the arm GUI. Nothing else is ever printed on the serial port,
 * so do NOT add Serial.print() debug output here.
 * The Pi can send 'L' / 'l' to switch the flash LED (GPIO4) on / off.
 *
 * Arduino IDE setup:
 *   Board:  "AI Thinker ESP32-CAM"   (esp32 by Espressif board package)
 *   Upload: connect via the USB bridge / FTDI, GPIO0 -> GND while uploading,
 *           then remove the jumper and press reset.
 *   Keep BAUD identical to --camera-baud on the Pi (default 921600).
 */
#include "esp_camera.h"

// ---- settings you may change ----
#define BAUD          460800          // must match --camera-baud on the Pi (default 460800)
#define FRAME_SIZE    FRAMESIZE_QVGA  // QQVGA 160x120, QVGA 320x240, CIF 400x296, VGA 640x480
#define JPEG_QUALITY  12              // 10 (best) .. 63 (smallest); higher number = smaller frames
#define FLIP_VERTICAL 0               // set 1 if the image is upside down
#define MIRROR        0               // set 1 to mirror horizontally

// AI-Thinker ESP32-CAM pins
#define PWDN_GPIO_NUM   32
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM    0
#define SIOD_GPIO_NUM   26
#define SIOC_GPIO_NUM   27
#define Y9_GPIO_NUM     35
#define Y8_GPIO_NUM     34
#define Y7_GPIO_NUM     39
#define Y6_GPIO_NUM     36
#define Y5_GPIO_NUM     21
#define Y4_GPIO_NUM     19
#define Y3_GPIO_NUM     18
#define Y2_GPIO_NUM      5
#define VSYNC_GPIO_NUM  25
#define HREF_GPIO_NUM   23
#define PCLK_GPIO_NUM   22
#define FLASH_LED_GPIO   4

static const uint8_t MAGIC[4] = {0xAA, 0x55, 0xAA, 0x55};

void setup() {
  pinMode(FLASH_LED_GPIO, OUTPUT);
  digitalWrite(FLASH_LED_GPIO, LOW);      // flash LED off
  Serial.begin(BAUD);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;       // 10 MHz is more stable than 20 MHz on the OV2640
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAME_SIZE;
  config.jpeg_quality = JPEG_QUALITY;
  config.fb_count = 1;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;  // always send the newest frame

  if (esp_camera_init(&config) != ESP_OK) {
    // Blink the flash LED forever so a failed init is visible without serial output.
    while (true) {
      digitalWrite(FLASH_LED_GPIO, HIGH); delay(150);
      digitalWrite(FLASH_LED_GPIO, LOW);  delay(150);
    }
  }
  sensor_t *s = esp_camera_sensor_get();
  s->set_vflip(s, FLIP_VERTICAL);
  s->set_hmirror(s, MIRROR);
}

void loop() {
  // Commands from the Pi: 'L' = flash LED on, 'l' = off.
  while (Serial.available() > 0) {
    int c = Serial.read();
    if (c == 'L') digitalWrite(FLASH_LED_GPIO, HIGH);
    else if (c == 'l') digitalWrite(FLASH_LED_GPIO, LOW);
  }
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    delay(10);
    return;
  }
  uint32_t len = fb->len;
  Serial.write(MAGIC, 4);
  Serial.write((const uint8_t *)&len, 4);
  // Send in small chunks and wait for the UART to drain. One huge write from PSRAM
  // was losing bytes in the middle of each frame.
  const uint8_t *p = fb->buf;
  size_t left = len;
  while (left > 0) {
    size_t n = left > 256 ? 256 : left;
    size_t w = Serial.write(p, n);
    p += w;
    left -= w;
  }
  Serial.flush();
  esp_camera_fb_return(fb);
}
