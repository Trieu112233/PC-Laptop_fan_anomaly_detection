# PC-Laptop Fan Anomaly Detection

Dự án phát hiện bất thường của quạt PC/Laptop sử dụng machine learning và Edge Impulse. 

## Giới thiệu

Hệ thống giám sát và phát hiện các bất thường trong hoạt động của quạt máy tính thông qua phân tích âm thanh, sử dụng mô hình CNN 1D. 

## Cấu trúc thư mục

- **PC_fan_dataset/**: Dataset chứa dữ liệu âm thanh của quạt PC
- **firmware_esp/**: Firmware cho ESP32 chạy mô hình phát hiện bất thường
- **firmware_colect_audio_dataset/**:  Firmware để thu thập dữ liệu âm thanh
- **pwm_arduino/**: Code Arduino điều khiển PWM cho quạt

## Công nghệ sử dụng

- **Edge Impulse**: Huấn luyện mô hình machine learning
- **CNN 1D**: Mô hình phát hiện bất thường
- **ESP32**: Vi điều khiển chạy inference
- **Arduino**: Điều khiển quạt qua PWM

## Cách sử dụng

1. Thu thập dữ liệu âm thanh bằng firmware trong `firmware_colect_audio_dataset/`
2. Huấn luyện mô hình trên Edge Impulse với dataset từ `PC_fan_dataset/`
3. Deploy mô hình lên ESP32 sử dụng firmware trong `firmware_esp/`
4. Điều khiển tốc độ quạt bằng Arduino trong `pwm_arduino/`

## Nhóm thực hiện (Contributors)
Dự án được thực hiện bởi nhóm 2 sinh viên:

| STT | Họ và tên | Mã sinh viên |
|:---:|:---|:---:|
| 1 | Quách Ngọc Quang | 22022132 |
| 2 | Nguyễn Đức Triệu | 22022110 |

