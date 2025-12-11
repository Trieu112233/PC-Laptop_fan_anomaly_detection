#include "driver/i2s.h"
#include <WiFi.h>
#include "esp_wifi.h"


// Cấu hình Wifi
const char* ssid = "SSW2014";
const char* pass = "vanhtrieuduc888@";

// Cấu hình I2S cho INMP441 
#define I2S_WS   8    // GPIO8 - INMP441 WS (L/R Select)
#define I2S_SCK  9    // GPIO9 - INMP441 SCK (Serial Clock)
#define I2S_SD   7    // GPIO7 - INMP441 SD (Serial Data)

#define I2S_PORT I2S_NUM_0
#define I2S_SAMPLE_RATE   16000

const int i2sBufferSize = 512;
int32_t i2s_raw_buffer[i2sBufferSize];
int16_t i2s_16bit_buffer[i2sBufferSize];

// High-pass filter 
float prev_sample = 0;
float prev_filtered = 0;

void applyHighPassFilter(int16_t* buffer, size_t length) {
  const float alpha = 0.95;
  for (int i = 0; i < length; i++) {
    float current = buffer[i];
    float filtered = alpha * (prev_filtered + current - prev_sample);
    prev_sample = current;
    prev_filtered = filtered;
    buffer[i] = (int16_t)constrain(filtered, -32768, 32767);
  }
}

// Hàm tạo chuỗi lặp
String repeatString(String str, int count) {
  String result = "";
  for (int i = 0; i < count; i++) {
    result += str;
  }
  return result;
}

// Khởi tạo I2S
void initI2S() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = I2S_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 128,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };

  esp_err_t result = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (result != ESP_OK) {
    Serial.println("Loi khoi tao I2S driver!");
    return;
  }
  
  result = i2s_set_pin(I2S_PORT, &pin_config);
  if (result != ESP_OK) {
    Serial.println("Loi cau hinh pin I2S!");
    return;
  }
  
  delay(100);
  Serial.println("I2S da khoi tao thanh cong!");
}

void setup() {
  Serial.begin(460800);
  delay(2000);
  
  Serial.println();
  Serial.println(repeatString("=", 40));
  Serial.println("ESP32-S3 + INMP441 Audio Test");
  Serial.println("Platform: Linux");
  Serial.println(repeatString("=", 40));

  WiFi.begin(ssid, pass);
  Serial.print("Đang kết nối WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nĐã kết nối WiFi!");

  // Giảm công suất WiFi xuống 8 dBm (thấp hơn mặc định 20 dBm)
  esp_wifi_set_max_tx_power(8 * 4);  // Đơn vị là 0.25 dBm (8*4 = 32 = 8 dBm)
  Serial.println("Đã giảm công suất WiFi xuống 8 dBm.");
  
  Serial.println("Dang khoi tao I2S...");
  initI2S();
  
  Serial.println("Kiem tra ket noi INMP441:");
  Serial.println("   VDD -> 3.3V");
  Serial.println("   GND -> GND");
  Serial.println("   WS  -> GPIO8");
  Serial.println("   SCK -> GPIO9");
  Serial.println("   SD  -> GPIO7");
  
  delay(1000);
  Serial.println();
  Serial.println("AUDIO_START");  // Signal cho Python
  Serial.println("Bat dau stream audio data...");
}

void loop() {
  size_t bytes_read = 0;
  
  esp_err_t result = i2s_read(I2S_PORT, &i2s_raw_buffer, sizeof(i2s_raw_buffer), &bytes_read, 100);
  
  if (result == ESP_OK && bytes_read > 0) {
    int samples_read = bytes_read / sizeof(int32_t);
    
    // Chuyển đổi 32-bit sang 16-bit
    for (int i = 0; i < samples_read; i++) {
      // INMP441 trả về data ở 24-bit cao nhất của 32-bit
      i2s_16bit_buffer[i] = (int16_t)(i2s_raw_buffer[i] >> 14);
    }
    
    // Áp dụng high-pass filter
    applyHighPassFilter(i2s_16bit_buffer, samples_read);
    
    // Gửi audio data qua Serial
    Serial.write((uint8_t*)i2s_16bit_buffer, samples_read * sizeof(int16_t));
  }
  
}