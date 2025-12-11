#include <cnn1d_fan_inferencing.h>
#include "driver/i2s.h"
#include "esp_heap_caps.h"

using namespace ei;

// ---------------------------------
// I2S Configuration 
// ---------------------------------
#define I2S_WS   8
#define I2S_SCK  9  
#define I2S_SD   7
#define I2S_PORT I2S_NUM_0

// Edge Impulse audio settings
#define AUDIO_SAMPLE_RATE    EI_CLASSIFIER_FREQUENCY
#define AUDIO_SAMPLES        EI_CLASSIFIER_RAW_SAMPLE_COUNT

// I2S buffer configuration
const int i2sBufferSize = 512;
int32_t i2s_raw_buffer[i2sBufferSize];
int16_t i2s_16bit_buffer[i2sBufferSize];

// Audio buffer
int16_t *sampleBuffer; 
static bool debug_nn = false; 

// High-pass filter variables
float prev_sample = 0;
float prev_filtered = 0;

static int cycle_count = 0;

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

void initI2S() {
  Serial.println("Initializing I2S...");
  
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

  esp_err_t result = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (result != ESP_OK) {
    Serial.printf("I2S driver install failed: %d\n", result);
    return;
  }
  
  result = i2s_set_pin(I2S_PORT, &pin_config);
  if (result != ESP_OK) {
    Serial.printf("I2S pin config failed: %d\n", result);
    return;
  }
  
  Serial.println("I2S initialized OK");
}

void setup() {
  Serial.begin(115200);

  while(!Serial && millis() < 2000);

  Serial.println("\n--- Fan Anomaly Detection System (CNN1D) ---");
  
  // 1. Kiểm tra PSRAM
  if (psramFound()) {
    Serial.printf("PSRAM Detected! Size: %d bytes\n", ESP.getPsramSize());
  } else {
    Serial.println("WARNING: No PSRAM detected! Model will likely fail.");
  }

  // 2. Cấp phát bộ nhớ cho Audio Buffer vào PSRAM
  size_t bufferSize = AUDIO_SAMPLES * sizeof(int16_t);
  sampleBuffer = (int16_t*)heap_caps_malloc(bufferSize, MALLOC_CAP_SPIRAM);
  
  if (sampleBuffer != NULL) {
    Serial.printf("Allocated %d bytes in PSRAM for audio buffer.\n", bufferSize);
  } else {
    Serial.println("PSRAM allocation failed. Trying Internal RAM...");
    sampleBuffer = (int16_t*)malloc(bufferSize);
  }

  if (sampleBuffer == NULL) {
    Serial.println("CRITICAL ERROR: Failed to allocate sample buffer!");
    while(1) delay(1000);
  }

  Serial.printf("Model: %s v%d\n", EI_CLASSIFIER_PROJECT_NAME, EI_CLASSIFIER_PROJECT_DEPLOY_VERSION);
  Serial.printf("Arena Size defined in Lib: %d bytes\n", EI_CLASSIFIER_TFLITE_LARGEST_ARENA_SIZE);
  initI2S();
  
  if (EI_CLASSIFIER_HAS_ANOMALY == 1) {
    Serial.println("Anomaly detection: ENABLED");
  }
  
  Serial.println("System ready!\n");
  delay(1000);
}

bool captureAudio() {
  if (sampleBuffer == NULL) return false;

  size_t samples_captured = 0;
  size_t bytes_read = 0;
  
  memset(sampleBuffer, 0, AUDIO_SAMPLES * sizeof(int16_t));

  while (samples_captured < AUDIO_SAMPLES) {
    esp_err_t result = i2s_read(I2S_PORT, &i2s_raw_buffer, 
                               sizeof(i2s_raw_buffer), &bytes_read, 100);
    
    if (result == ESP_OK && bytes_read > 0) {
      int samples_read = bytes_read / sizeof(int32_t);
      
      for (int i = 0; i < samples_read && samples_captured < AUDIO_SAMPLES; i++) {
        i2s_16bit_buffer[i] = (int16_t)(i2s_raw_buffer[i] >> 14);
      }
      
      int samples_to_filter = min(samples_read, (int)(AUDIO_SAMPLES - samples_captured));
      applyHighPassFilter(i2s_16bit_buffer, samples_to_filter);
      
      for (int i = 0; i < samples_to_filter; i++) {
        sampleBuffer[samples_captured] = i2s_16bit_buffer[i];
        samples_captured++;
      }
    }
    yield();
  }
  
  long sum_squares = 0;
  for (int i = 0; i < AUDIO_SAMPLES; i += 100) {
    sum_squares += (long)sampleBuffer[i] * sampleBuffer[i];
  }
  float rms = sqrt((float)sum_squares / (AUDIO_SAMPLES / 100));
  
  if (rms < 10.0f) {
    Serial.println("WARNING: Low audio signal");
    return false;
  }
  
  return true;
}

int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
  numpy::int16_to_float(&sampleBuffer[offset], out_ptr, length);
  return 0;
}

void runInference() {
  if (sampleBuffer == NULL) return;

  signal_t signal;
  signal.total_length = AUDIO_SAMPLES;
  signal.get_data = &raw_feature_get_data;
  
  ei_impulse_result_t result = { 0 };
  
  EI_IMPULSE_ERROR res = run_classifier(&signal, &result, debug_nn);
  
  if (res != EI_IMPULSE_OK) {
    Serial.printf("Inference failed: %d\n", res);
    if (res == EI_IMPULSE_ALLOC_FAILED) {
        Serial.println("ERR: Out of Memory! Did you modify ei_classifier_porting.cpp?");
    }
    return;
  }
  
  float max_score = 0;
  const char* predicted_label = "unknown";
  
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    if (result.classification[ix].value > max_score) {
      max_score = result.classification[ix].value;
      predicted_label = result.classification[ix].label;
    }
  }
  
  Serial.printf("[%d] Prediction: %s (%.1f%%)", 
                cycle_count, predicted_label, max_score * 100.0f);
  
  if (EI_CLASSIFIER_HAS_ANOMALY == 1) {
    Serial.printf(" | Anomaly: %.3f", result.anomaly);
    if (result.anomaly > 0.3f) Serial.print(" - ANOMALY!");
  }
  
  Serial.printf(" | Time: %dms\n", result.timing.dsp + result.timing.classification);
}

void loop() {
  cycle_count++;
  Serial.printf("Cycle %d: Capturing... ", cycle_count);
  
  if (captureAudio()) {
    Serial.print("OK | Inferencing... ");
    runInference();
  } else {
    Serial.println("Audio capture failed");
  }
  
  delay(2000);
}
