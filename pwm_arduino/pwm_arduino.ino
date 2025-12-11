/*
 * Điều khiển tốc độ động cơ DC bằng PWM qua Serial Monitor.
 * Nhập giá trị PWM (0–255) từ Serial để thay đổi tốc độ.
 * IN1 là chân PWM, IN2 xác định chiều quay.
 */

#define IN1 9  // Chân PWM để điều khiển tốc độ
#define IN2 7  // Chân điều khiển chiều quay

int pwmValue = 255; // Mặc định chạy ở 100%

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Đặt chiều quay (IN2 = LOW)
  digitalWrite(IN2, LOW);

  // Khởi tạo Serial
  Serial.begin(9600);
  Serial.println("Nhap gia tri PWM (0 - 255) de thay doi toc do dong co:");
  Serial.println("Vi du: 120");
  
  // Đặt tốc độ ban đầu
  analogWrite(IN1, pwmValue);
}

void loop() {
  // Kiểm tra xem có dữ liệu từ Serial chưa
  if (Serial.available() > 0) {
    int newValue = Serial.parseInt(); // Đọc số nguyên từ Serial
    
    // Kiểm tra giá trị hợp lệ
    if (newValue >= 0 && newValue <= 255) {
      pwmValue = newValue;
      analogWrite(IN1, pwmValue);
      Serial.print("Toc do moi (PWM) = ");
      Serial.println(pwmValue);
    } else {
      Serial.println("Gia tri khong hop le! Vui long nhap tu 0 den 255.");
    }
    
    // Xóa bộ đệm còn lại (nếu có ký tự thừa)
    while (Serial.available() > 0) Serial.read();
  }
}
