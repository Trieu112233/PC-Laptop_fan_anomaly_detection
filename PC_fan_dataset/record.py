import serial
import time
import wave
import os
import sys
from datetime import datetime
import signal

# ===================================================================
# CHỈNH THÔNG SỐ TẠI ĐÂY
# ===================================================================
CURRENT_FAN_ID = 'id_01'        # Chỉnh thành 'id_01' hoặc 'id_02'
CURRENT_CONDITION = 'abnormal'    # Chỉnh thành 'normal' hoặc 'abnormal'
NUMBER_OF_SAMPLES = 1           # Số mẫu muốn thu (mỗi mẫu 5 giây)
# ===================================================================

# --- CẤU HÌNH SYSTEM ---
SERIAL_PORT = 'COM8'
BAUD_RATE = 460800
SAMPLE_DURATION = 5 # giây
FILENAME_PREFIX = 'sample'

# --- Thông số âm thanh ---
SAMPLE_RATE = 16000
CHANNELS = 1
BYTES_PER_SAMPLE = 2
BYTES_PER_SEGMENT = SAMPLE_RATE * BYTES_PER_SAMPLE * CHANNELS * SAMPLE_DURATION

# --- Cấu trúc thư mục ---
BASE_DIRS = {
    'id_01': {
        'normal': 'id_01/normal',
        'abnormal': 'id_01/abnormal'
    },
    'id_02': {
        'normal': 'id_02/normal', 
        'abnormal': 'id_02/abnormal'
    }
}

# --- Biến global ---
stop_recording = False
sample_counter = 0

class SimpleFanCollector:
    def __init__(self):
        self.serial_connection = None
        self.total_samples = 0
        
    def setup_directories(self):
        """Tạo cấu trúc thư mục"""
        print("📁 Thiết lập thư mục...")
        for fan_id, conditions in BASE_DIRS.items():
            for condition, path in conditions.items():
                os.makedirs(path, exist_ok=True)
                print(f"   ✓ {path}")
        print()

    def connect_serial(self):
        """Kết nối ESP32-S3"""
        print(f"🔌 Kết nối {SERIAL_PORT}...")
        
        if not os.path.exists(SERIAL_PORT):
            print(f"❌ Không tìm thấy {SERIAL_PORT}")
            return False
            
        try:
            self.serial_connection = serial.Serial(
                port=SERIAL_PORT,
                baudrate=BAUD_RATE,
                timeout=1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            print("✓ Kết nối thành công!")
            return True
        except Exception as e:
            print(f"❌ Lỗi kết nối: {e}")
            return False

    def reset_esp32(self):
        """Reset ESP32"""
        print("🔄 Reset ESP32...")
        try:
            self.serial_connection.setDTR(False)
            time.sleep(0.1)
            self.serial_connection.setDTR(True)
            time.sleep(0.1)
            self.serial_connection.setDTR(False)
            
            print("✓ ESP32 đã reset")
            time.sleep(2)
            
            self.serial_connection.flushInput()
            self.serial_connection.flushOutput()
            return True
        except Exception as e:
            print(f"❌ Lỗi reset: {e}")
            return False

    def wait_for_audio_start(self):
        """Đợi AUDIO_START"""
        print("🎯 Đợi tín hiệu AUDIO_START...")
        start_time = time.time()
        buffer = b""
        
        while (time.time() - start_time) < 20:
            try:
                if self.serial_connection.in_waiting > 0:
                    data = self.serial_connection.read(self.serial_connection.in_waiting)
                    buffer += data
                    
                    if b"AUDIO_START" in buffer:
                        print("✓ Nhận AUDIO_START!")
                        time.sleep(1)
                        self.serial_connection.flushInput()
                        return True
                    
                    # Debug info
                    try:
                        text = buffer.decode('utf-8', errors='ignore')
                        lines = text.split('\n')
                        for line in lines:
                            line = line.strip()
                            if line and any(kw in line for kw in ["ESP32", "WiFi", "I2S", "AUDIO"]):
                                print(f"ESP32: {line}")
                        buffer = b""
                    except:
                        pass
                        
            except Exception as e:
                print(f"Lỗi: {e}")
                
            time.sleep(0.1)
            
        print("❌ Timeout AUDIO_START")
        return False

    def save_audio_segment(self, audio_data, fan_id, condition):
        """Lưu file âm thanh"""
        global sample_counter
        sample_counter += 1
        self.total_samples += 1
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{FILENAME_PREFIX}_{timestamp}_{sample_counter:04d}.wav"
        filepath = os.path.join(BASE_DIRS[fan_id][condition], filename)
        
        try:
            with wave.open(filepath, 'wb') as wav_file:
                wav_file.setnchannels(CHANNELS)
                wav_file.setsampwidth(BYTES_PER_SAMPLE)
                wav_file.setframerate(SAMPLE_RATE)
                wav_file.writeframes(audio_data[:BYTES_PER_SEGMENT])
            
            file_size = os.path.getsize(filepath)
            print(f"💾 [{self.total_samples}/{NUMBER_OF_SAMPLES}] {filename} ({file_size} bytes)")
            return True
            
        except Exception as e:
            print(f"❌ Lỗi lưu: {e}")
            return False

    def collect_samples(self):
        """Thu thập số lượng mẫu cố định"""
        global stop_recording
        
        print(f"🎙️  Thu thập {NUMBER_OF_SAMPLES} mẫu...")
        print(f"📍 Cấu hình: {CURRENT_FAN_ID.upper()} - {CURRENT_CONDITION.upper()}")
        print("🎵 Bắt đầu...")
        
        segment_buffer = b""
        samples_collected = 0
        
        while not stop_recording and samples_collected < NUMBER_OF_SAMPLES:
            try:
                # Đọc dữ liệu
                if self.serial_connection.in_waiting > 0:
                    chunk = self.serial_connection.read(min(
                        self.serial_connection.in_waiting,
                        BYTES_PER_SEGMENT - len(segment_buffer)
                    ))
                    segment_buffer += chunk
                    
                    # Hiển thị tiến trình
                    progress = (len(segment_buffer) / BYTES_PER_SEGMENT) * 100
                    print(f"\r🔊 {CURRENT_FAN_ID.upper()}-{CURRENT_CONDITION.upper()} | "
                          f"Sample {samples_collected + 1}/{NUMBER_OF_SAMPLES} | "
                          f"Progress: {progress:.1f}% | "
                          f"Time: {datetime.now().strftime('%H:%M:%S')}", end="", flush=True)
                    
                    # Khi đủ 5 giây
                    if len(segment_buffer) >= BYTES_PER_SEGMENT:
                        print()  # Xuống dòng
                        success = self.save_audio_segment(
                            segment_buffer, 
                            CURRENT_FAN_ID, 
                            CURRENT_CONDITION
                        )
                        
                        if success:
                            samples_collected += 1
                            remaining = NUMBER_OF_SAMPLES - samples_collected
                            if remaining > 0:
                                print(f"✅ Hoàn thành! Còn lại {remaining} mẫu...")
                            else:
                                print("🎉 Đã thu đủ số mẫu!")
                        
                        # Reset buffer
                        segment_buffer = b""
                
                time.sleep(0.001)
                
            except Exception as e:
                print(f"\n❌ Lỗi thu thập: {e}")
                break
                
        print(f"\n⏹️  Hoàn tất thu thập {samples_collected}/{NUMBER_OF_SAMPLES} mẫu")

def signal_handler(sig, frame):
    """Xử lý Ctrl+C"""
    global stop_recording
    print("\n\n🛑 Dừng bởi người dùng...")
    stop_recording = True
    sys.exit(0)

def main():
    global stop_recording
    
    signal.signal(signal.SIGINT, signal_handler)
    
    print("=" * 70)
    print("🎵 SIMPLE FAN AUDIO COLLECTOR - ESP32-S3 + INMP441")
    print("=" * 70)
    print(f"📅 Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} UTC")
    print(f"👤 User: Trieu112233")
    print(f"🔧 Port: {SERIAL_PORT}")
    print("=" * 70)
    print("⚙️  CẤU HÌNH HIỆN TẠI:")
    print(f"   🎯 Fan ID: {CURRENT_FAN_ID}")
    print(f"   📊 Condition: {CURRENT_CONDITION}")
    print(f"   📈 Số mẫu: {NUMBER_OF_SAMPLES}")
    print(f"   ⏱️  Thời gian/mẫu: {SAMPLE_DURATION}s")
    print(f"   🎵 Sample rate: {SAMPLE_RATE}Hz")
    print("=" * 70)
    print("💡 Để thay đổi cấu hình:")
    print("   - Sửa CURRENT_FAN_ID = 'id_01' hoặc 'id_02'")
    print("   - Sửa CURRENT_CONDITION = 'normal' hoặc 'abnormal'")
    print("   - Sửa NUMBER_OF_SAMPLES = số mẫu muốn thu")
    print("=" * 70)
    
    # Xác nhận
    target_dir = BASE_DIRS[CURRENT_FAN_ID][CURRENT_CONDITION]
    print(f"📂 File sẽ lưu vào: {target_dir}")
    
    try:
        confirm = input("Nhấn Enter để bắt đầu, Ctrl+C để thoát...")
    except KeyboardInterrupt:
        print("Thoát.")
        return
    
    collector = SimpleFanCollector()
    
    try:
        # Setup
        collector.setup_directories()
        
        # Connect
        if not collector.connect_serial():
            print("❌ Không thể kết nối ESP32.")
            return
        
        # Reset
        if not collector.reset_esp32():
            print("⚠️  Không reset được, thử tiếp tục...")
        
        # Wait ready
        if not collector.wait_for_audio_start():
            print("❌ ESP32 không sẵn sàng.")
            return
        
        # Collect
        collector.collect_samples()
        
    except Exception as e:
        print(f"\n❌ Lỗi: {e}")
        
    finally:
        if collector.serial_connection:
            collector.serial_connection.close()
        
        print(f"\n📊 Tổng kết:")
        print(f"   ✅ Đã thu: {collector.total_samples} mẫu")
        print(f"   📂 Thư mục: {target_dir}")
        print("✅ Hoàn tất!")

if __name__ == "__main__":
    main()