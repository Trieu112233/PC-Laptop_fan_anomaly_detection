#define BLYNK_TEMPLATE_ID   "TMPL6vkPgopi0"
#define BLYNK_TEMPLATE_NAME "dht22test"
#define BLYNK_AUTH_TOKEN    "IjmdLxBwgtLJoPfjDE_lKmK6d-JS9NLK"
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <cnn1d_fan_inferencing.h> // Thư viện Edge Impulse
#include "driver/i2s.h"
#include "esp_heap_caps.h"
#include "esp_wifi.h"

using namespace ei;

// CẤU HÌNH PHẦN CỨNG (PIN DEFINITIONS)
// --- Cấu hình LED (Xiao ESP32S3) ---
#define LED_PIN         21
#define LED_ON          LOW   
#define LED_OFF         HIGH  
#define BLINK_INTERVAL  200   // Tốc độ nháy (ms)

// --- Cấu hình I2S Microphone (INMP441) ---
#define I2S_WS          8     // Word Select
#define I2S_SCK         9     // Serial Clock
#define I2S_SD          7     // Serial Data
#define I2S_PORT        I2S_NUM_0

// CẤU HÌNH HỆ THỐNG & LOGIC
// --- Thông tin WiFi ---
const char* ssid = "SSW2014";
const char* pass = "vanhtrieuduc888@";

// --- Trạng thái hệ thống ---
enum SystemState {
  STATE_INIT,     
  STATE_NORMAL,   
  STATE_ABNORMAL  
};

SystemState currentState  = STATE_INIT;
SystemState previousState = STATE_INIT;

// --- Biến quản lý LED & Cảnh báo ---
unsigned long previousLedMillis = 0;
bool ledState = false;
unsigned long lastAlertTime = 0;
const unsigned long ALERT_COOLDOWN = 30000; // Thời gian chờ giữa 2 lần gửi thông báo (30s)

// --- Cấu hình Audio & Edge Impulse ---
#define AUDIO_SAMPLE_RATE    EI_CLASSIFIER_FREQUENCY
#define AUDIO_SAMPLES        EI_CLASSIFIER_RAW_SAMPLE_COUNT

// Bộ đệm I2S và Audio
const int i2sBufferSize = 512;
int32_t i2s_raw_buffer[i2sBufferSize];
int16_t *sampleBuffer; // Sẽ được cấp phát trong setup()

// Biến lọc nhiễu (High-pass filter)
float prev_sample = 0;
float prev_filtered = 0;

static bool debug_nn = false; // Bật/tắt debug chi tiết của Neural Network
static int cycle_count = 0;

// 4. CÁC HÀM XỬ LÝ TÍN HIỆU (DSP)
// Áp dụng bộ lọc thông cao để loại bỏ thành phần DC và nhiễu tần số thấp
void applyHighPassFilter(int16_t* buffer, size_t length) {
  const float alpha = 0.95;
  for (int i = 0; i < length; i++) {
    float current = buffer[i];
    float filtered = alpha * (prev_filtered + current - prev_sample);
    prev_sample = current;
    prev_filtered = filtered;
    // Giới hạn giá trị trong khoảng int16
    buffer[i] = (int16_t)constrain(filtered, -32768, 32767);
  }
}

// Callback để Edge Impulse lấy dữ liệu từ buffer
int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
  numpy::int16_to_float(&sampleBuffer[offset], out_ptr, length);
  return 0;
}

// CÁC HÀM HỆ THỐNG (I2S, LED, WIFI)
// Khởi tạo giao tiếp I2S với Microphone
void initI2S() {
  Serial.println("[I2S] Initializing...");
  
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = AUDIO_SAMPLE_RATE,
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

  if (i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL) != ESP_OK) {
    Serial.println("[I2S] Driver install failed!");
  }
  
  if (i2s_set_pin(I2S_PORT, &pin_config) != ESP_OK) {
    Serial.println("[I2S] Pin config failed!");
  }
  
  Serial.println("[I2S] Initialized OK");
}

// Điều khiển LED dựa trên trạng thái hệ thống (Non-blocking)
// - NORMAL: Sáng liên tục
// - ABNORMAL: Nhấp nháy
void handleLedStatus() {
  if (currentState == STATE_NORMAL) {
    digitalWrite(LED_PIN, LED_ON);
  } 
  else if (currentState == STATE_ABNORMAL) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousLedMillis >= BLINK_INTERVAL) {
      previousLedMillis = currentMillis;
      ledState = !ledState; 
      digitalWrite(LED_PIN, ledState ? LED_ON : LED_OFF);
    }
  }
  else {
    digitalWrite(LED_PIN, LED_OFF); // STATE_INIT
  }
}

// Thu âm thanh từ I2S vào buffer
// Trả về true nếu thu âm thành công và âm lượng đủ lớn
bool captureAudio() {
  if (sampleBuffer == NULL) return false;

  size_t samples_captured = 0;
  size_t bytes_read = 0;
  
  memset(sampleBuffer, 0, AUDIO_SAMPLES * sizeof(int16_t));

  while (samples_captured < AUDIO_SAMPLES) {
    // Gọi hàm LED ở đây để đèn vẫn nháy mượt khi đang thu âm
    handleLedStatus();

    // Đọc dữ liệu từ I2S (Non-blocking timeout ngắn)
    i2s_read(I2S_PORT, &i2s_raw_buffer, i2sBufferSize * 4, &bytes_read, 10);
    
    int samples_read = bytes_read / 4;
    for (int i = 0; i < samples_read; i++) {
      if (samples_captured < AUDIO_SAMPLES) {
        // Chuyển đổi 32-bit (INMP441) sang 16-bit
        int32_t sample = i2s_raw_buffer[i] >> 14; 
        sampleBuffer[samples_captured] = (int16_t)sample;
        samples_captured++;
      }
    }
  }
  
  // Lọc nhiễu
  applyHighPassFilter(sampleBuffer, AUDIO_SAMPLES);

  // Tính RMS để kiểm tra độ ồn
  long long sum_squares = 0;
  for (int i = 0; i < AUDIO_SAMPLES; i += 100) {
    sum_squares += (long)sampleBuffer[i] * sampleBuffer[i];
  }
  float rms = sqrt((float)sum_squares / (AUDIO_SAMPLES / 100));
  
  // Nếu quá yên tĩnh, bỏ qua không chạy AI
  if (rms < 10.0f) {
    return false;
  }
  
  return true;
}

// Chạy mô hình AI và xử lý kết quả
void runInference() {
  if (sampleBuffer == NULL) return;

  // 1. Chuẩn bị tín hiệu
  signal_t signal;
  signal.total_length = AUDIO_SAMPLES;
  signal.get_data = &raw_feature_get_data;
  
  ei_impulse_result_t result = { 0 };
  
  // 2. Chạy Classifier
  EI_IMPULSE_ERROR res = run_classifier(&signal, &result, debug_nn);
  if (res != EI_IMPULSE_OK) {
    Serial.printf("ERR: Classifier failed (%d)\n", res);
    return;
  }
  
  // 3. Tìm nhãn có điểm số cao nhất
  float max_score = 0;
  const char* predicted_label = "unknown";
  
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    if (result.classification[ix].value > max_score) {
        max_score = result.classification[ix].value;
        predicted_label = result.classification[ix].label;
    }
  }
  
  Serial.printf("[%d] Pred: %s (%.1f%%)", cycle_count, predicted_label, max_score * 100.0f);
  if (EI_CLASSIFIER_HAS_ANOMALY == 1) {
     Serial.printf(" | Anomaly: %.3f", result.anomaly);
  }

  // 4. Logic xác định trạng thái
  bool isAbnormal = false;
  // Điều kiện: Nhãn là "abnormal" VÀ độ tin cậy > 70%
  if (strcmp(predicted_label, "abnormal") == 0 && max_score > 0.7f) {
    isAbnormal = true;
  }

  // Cập nhật biến trạng thái
  currentState = isAbnormal ? STATE_ABNORMAL : STATE_NORMAL;

  // 5. Gửi Blynk (Chỉ khi trạng thái thay đổi)
  if (currentState != previousState) {
    if (currentState == STATE_ABNORMAL) {
      Serial.println(" -> ABNORMAL DETECTED!");
      Blynk.virtualWrite(V1, "ABNORMAL !!!"); 
      Blynk.virtualWrite(V2, 1); 
      
      // Gửi thông báo đẩy (có giới hạn thời gian)
      if (millis() - lastAlertTime > ALERT_COOLDOWN) {
        Blynk.logEvent("anomaly_alert", "CẢNH BÁO: Quạt bất thường!");
        lastAlertTime = millis();
      }
    } else {
      Serial.println(" -> Normal");
      Blynk.virtualWrite(V1, "Normal"); 
      Blynk.virtualWrite(V2, 0);
    }
    previousState = currentState;
  } else {
    Serial.println();
  }
  
  Serial.printf(" | Time: %dms\n", result.timing.dsp + result.timing.classification);
}

// SETUP & LOOP
void setup() {
  Serial.begin(115200);
  
  // --- Init LED ---
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_OFF);

  while(!Serial && millis() < 2000); // Chờ Serial kết nối

  Serial.println("\n--- Fan Anomaly Detection System (CNN1D) ---");
  
  // --- Init Memory (PSRAM) ---
  if (psramFound()) {
    Serial.println("[MEM] PSRAM found!");
  } else {
    Serial.println("[MEM] PSRAM not found!");
  }

  size_t bufferSize = AUDIO_SAMPLES * sizeof(int16_t);
  sampleBuffer = (int16_t*)heap_caps_malloc(bufferSize, MALLOC_CAP_SPIRAM);
  
  if (sampleBuffer == NULL) {
    Serial.println("[MEM] Failed to allocate in PSRAM, trying internal RAM...");
    sampleBuffer = (int16_t*)malloc(bufferSize);
  }

  if (sampleBuffer == NULL) {
    Serial.println("[ERR] Critical: Failed to allocate sample buffer!");
    while(1);
  }

  // --- Init WiFi ---
  WiFi.begin(ssid, pass);
  Serial.print("[WiFi] Connecting...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n[WiFi] Connected!");

  esp_wifi_set_max_tx_power(8 * 4); // Giảm công suất phát để giảm nhiễu

  // --- Init Blynk ---
  Serial.println("[Blynk] Connecting...");
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  Serial.println("[Blynk] Connected!");

  // --- Init I2S ---
  initI2S();
  
  Serial.println("[SYS] System ready!\n");
  currentState = STATE_NORMAL;
  delay(1000);
}

void loop() {
  Blynk.run();
  
  // Xử lý hiệu ứng LED liên tục
  handleLedStatus();

  cycle_count++;
  Serial.printf("Cycle %d: Capturing... ", cycle_count); 
  
  if (captureAudio()) {
    Serial.print("OK | Inferencing... ");
    runInference();
  } else {
    Serial.println("Audio capture failed");
  }
  delay(100); 
}
