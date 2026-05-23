#include <Arduino.h>
#include <Servo.h> // Thư viện điều khiển động cơ Servo SG90

// ======================================================
// SYSTEM CONFIGURATION - CẤU HÌNH HỆ THỐNG
// ======================================================
// Tốc độ giao tiếp Serial với máy tính / Module Bluetooth (9600 bps)
const unsigned long SERIAL_BAUD_RATE = 9600;

// Các ngưỡng khoảng cách an toàn để xử lý va chạm (đơn vị: cm)
const int WARNING_DISTANCE = 30; // Dưới 30cm: Bắt đầu cảnh báo và giảm tốc độ
const int DANGER_DISTANCE = 15;  // Dưới 15cm: Nguy hiểm, kích hoạt phanh khẩn cấp

// ======================================================
// MOTOR DRIVER PINS - CẤU HÌNH CHÂN ĐIỀU KHIỂN ĐỘNG CƠ
// ======================================================
const int ENA_PIN = 5;    // Chân điều khiển tốc độ PWM cho Motor A
const int IN1_A_PIN = 6;  // Chân điều hướng 1 cho Motor A
const int IN2_A_PIN = 7;  // Chân điều hướng 2 cho Motor A

const int ENB_PIN = 11;   // Chân điều khiển tốc độ PWM cho Motor B (Né chân 9, 10 để tránh xung đột)
const int IN1_B_PIN = 10; // Chân điều hướng 1 cho Motor B
const int IN2_B_PIN = 9;  // Chân điều hướng 2 cho Motor B

// ======================================================
// ULTRASONIC SENSOR PINS - CHÂN CẢM BIẾN SIÊU ÂM HC-SR04
// ======================================================
const int TRIG_PIN = 12; // Chân phát sóng siêu âm (Output)
const int ECHO_PIN = 2;  // Chân thu sóng siêu âm phản hồi (Input)

// ======================================================
// SERVO CONFIGURATION - CẤU HÌNH ĐỘNG CƠ SERVO SG90
// ======================================================
const int SERVO_PIN = 3; // Chân phát xung PWM điều khiển Servo (Cắm dây Cam/Vàng)
Servo radarServo;        // Tạo đối tượng radarServo từ thư viện Servo

// ======================================================
// SYSTEM STATES - ĐỊNH NGHĨA TRẠNG THÁI HỆ THỐNG
// ======================================================
enum CarState 
{
  IDLE_STATE,           
  FORWARD_STATE,        
  BACKWARD_STATE,       
  LEFT_STATE,           
  RIGHT_STATE,          
  STOP_STATE,           
  EMERGENCY_STOP_STATE, 
};

struct SensorData {
  long distance;    // Khoảng cách đo được thực tế (cm)
  int confidence;   // Độ tin cậy của dữ liệu (%)
  int safetyLevel;  // Mức độ an toàn: 0 = An toàn, 1 = Cảnh báo, 2 = Nguy hiểm
  bool emergency;   // Cờ báo động khẩn cấp (true = nguy hiểm, false = an toàn)
};

// ======================================================
// GLOBAL VARIABLES - BIẾN TOÀN CỤC
// ======================================================
CarState currentState = IDLE_STATE;       
long currentDistance = 0;                 
bool emergencyStopActive = false;         
char bluetoothCommand = 'S';              // Biến lưu lệnh di chuyển nhận được (F, B, L, R, S, U, D)
bool safetyOverrideActive = false;        // Cờ ghi đè an toàn (Khi true, khóa mọi chuyển động)
SensorData currentSensorData;             

int motorSpeed = 150;                     // Tốc độ gốc của xe (0 - 255)
int dynamicSpeed = motorSpeed;            // Tốc độ thay đổi linh hoạt theo khoảng cách
float turnMultiplier = 0.7;               // Hệ số giảm tốc khi rẽ
int turnSpeed = 0;                        

// --- BIẾN BỔ SUNG: PHỤC VỤ THUẬT TOÁN TỐI ƯU TĂNG/GIẢM TỐC THEO CỬ CHỈ ---
char lastDirectionCommand = 'S';          // Biến lưu lại hướng đi gần nhất để tránh xe bị khựng khi đổi tốc độ

// --- BIẾN BỔ SUNG: PHỤC VỤ THUẬT TOÁN QUÉT RADAR LIÊN TỤC KHÔNG CHẶN (NON-BLOCKING) ---
int currentAngle = 60;          // Góc quét bắt đầu (Quét từ 60 độ sang 120 độ trước mặt)
int scanDirection = 1;          // Hướng quay của servo (1: Quay thuận, -1: Quay nghịch)
unsigned long lastRadarTime = 0; // Lưu mốc thời gian của lần dịch góc Servo trước đó
const int RADAR_INTERVAL = 60;  // Khoảng thời gian giãn cách giữa các góc quét (60ms giúp xe chạy mượt, không khựng)

// ======================================================
// MOTOR CONTROLLER - CÁC HÀM ĐIỀU KHIỂN ĐỘNG CƠ
// ======================================================

void stopMotor()
{
  digitalWrite(IN1_A_PIN, LOW);  digitalWrite(IN2_A_PIN, LOW);
  digitalWrite(IN1_B_PIN, LOW);  digitalWrite(IN2_B_PIN, LOW);
  analogWrite(ENA_PIN, 0);       analogWrite(ENB_PIN, 0);
  currentState = STOP_STATE; 
}

void moveForward()
{
  digitalWrite(IN1_A_PIN, HIGH); digitalWrite(IN2_A_PIN, LOW);
  digitalWrite(IN1_B_PIN, HIGH); digitalWrite(IN2_B_PIN, LOW);
  analogWrite(ENA_PIN, dynamicSpeed);
  analogWrite(ENB_PIN, dynamicSpeed);
  currentState = FORWARD_STATE; 
}

void moveBackward()
{
  digitalWrite(IN1_A_PIN, LOW);  digitalWrite(IN2_A_PIN, HIGH);
  digitalWrite(IN1_B_PIN, LOW);  digitalWrite(IN2_B_PIN, HIGH);
  analogWrite(ENA_PIN, motorSpeed);
  analogWrite(ENB_PIN, motorSpeed);
  currentState = BACKWARD_STATE; 
}

void updateMotorSpeeds()
{
  turnSpeed = (int)(dynamicSpeed * turnMultiplier);
}

void turnLeft()
{
  updateMotorSpeeds(); 
  digitalWrite(IN1_A_PIN, LOW);  digitalWrite(IN2_A_PIN, HIGH);
  digitalWrite(IN1_B_PIN, HIGH); digitalWrite(IN2_B_PIN, LOW);
  analogWrite(ENA_PIN, turnSpeed);
  analogWrite(ENB_PIN, turnSpeed);
  currentState = LEFT_STATE; 
}

void turnRight()
{
  updateMotorSpeeds(); 
  digitalWrite(IN1_A_PIN, HIGH); digitalWrite(IN2_A_PIN, LOW);
  digitalWrite(IN1_B_PIN, LOW);  digitalWrite(IN2_B_PIN, HIGH);
  analogWrite(ENA_PIN, turnSpeed);
  analogWrite(ENB_PIN, turnSpeed);
  currentState = RIGHT_STATE; 
}

// ======================================================
// SENSOR FUNCTIONS - CÁC HÀM XỬ LÝ CẢM BIẾN SIÊU ÂM
// ======================================================

/**
 * @brief Đọc khoảng cách siêu âm nhanh (Giới hạn timeout thấp để xe không bị delay đứng im)
 */
long readDistanceCM()
{
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Đặt giới hạn chờ xung phản hồi là 15ms (15000us) nhằm triệt tiêu độ trễ làm khựng động cơ
  long duration = pulseIn(ECHO_PIN, HIGH, 15000);

  if (duration == 0)
    return -1;
  
  return duration * 0.034 / 2;
}

/**
 * @brief Thuật toán lấy trung bình cộng 3 lần đo (Rút ngắn từ 5 xuống 3 lần để đáp ứng tốc độ quét liên tục)
 */
long getFilteredDistance()
{
  long sum = 0;
  int validCount = 0;

  for (int i = 0; i < 3; i++)
  {
    long d = readDistanceCM();
    if (d > 0 && d < 200)
    {
      sum += d;
      validCount++; 
    }
    delay(4); // Giãn cách ngắn giữa các mẫu đo
  }

  if (validCount == 0) return -1; 
  return sum / validCount;        
}

// ======================================================
// SENSOR DATA PROCESSOR - BỘ XỬ LÝ VÀ PHÂN LOẠI DỮ LIỆU
// ======================================================

SensorData getSensorData()
{
  SensorData data;
  long distance = getFilteredDistance(); 
  data.distance = distance;

  if (distance == -1)
  {
    data.confidence = 0;    
    data.safetyLevel = 0;   // Nếu mất tín hiệu tạm thời khi đang quét góc, coi như an toàn để tránh xe bị phanh giật cục
    data.emergency = false;  
    dynamicSpeed = motorSpeed;
    return data;
  }

  data.confidence = map(min(distance, 200), 0, 200, 100, 20);

  // LOGIC PHÂN PHỐI AN TOÀN THEO KHOẢNG CÁCH GẦN VẬT CẢN
  if (distance <= DANGER_DISTANCE)
  {
    data.safetyLevel = 2;  // Mức nguy hiểm
    data.emergency = true; // Kích hoạt phanh khẩn cấp
    dynamicSpeed = 0;      
  }
  else if (distance <= WARNING_DISTANCE){
    data.safetyLevel = 1;   // Mức cảnh báo
    data.emergency = false; 
    dynamicSpeed = constrain((int)(motorSpeed * 0.6), 0, motorSpeed); // Giảm bớt tốc độ nền khi mấp mé vật cản
  }
  else {
    data.safetyLevel = 0;   // Vùng an toàn hoàn toàn
    data.emergency = false; 
    dynamicSpeed = motorSpeed; 
  }

  if (dynamicSpeed < 0) dynamicSpeed = 0;
  if (dynamicSpeed > 255) dynamicSpeed = 255;

  if (data.safetyLevel == 2){
    emergencyStopActive = true;
  }
  return data; 
}

// ======================================================
// NEW RADAR CONTROL - HÀM ĐIỀU KHIỂN RADAR QUÉT LIÊN TỤC CHỦ ĐỘNG
// ======================================================

/**
 * @brief HÀM BỔ SUNG: Điều khiển Servo tự động gạt qua gạt lại liên tục từ 60 đến 120 độ.
 * Hàm này sử dụng hàm millis() thay vì hàm delay() nên xe sẽ VỪA CHẠY VỪA QUÉT mượt mà.
 */
void runContinuousRadar()
{
  // Kiểm tra nếu chưa đủ thời gian trễ (60ms) thì thoát hàm để nhường luồng xử lý cho động cơ
  if (millis() - lastRadarTime < RADAR_INTERVAL) {
    return; 
  }
  lastRadarTime = millis(); // Cập nhật lại mốc thời gian mới

  // 1. Ra lệnh cho Servo quay đến góc hiện tại trong chu kỳ quét
  radarServo.write(currentAngle);

  // 2. Chạy thuật toán kiểm tra khoảng cách an toàn ngay tại góc Servo đang đứng
  currentSensorData = getSensorData(); 

  // Nếu bộ xử lý báo trạng thái khẩn cấp (có vật cản nguy hiểm lọt vào vùng quét)
  if (currentSensorData.emergency)
  {
    safetyOverrideActive = true; // Kích hoạt cờ ghi đè an toàn hệ thống
    stopMotor();                 // Ép xe phanh dừng lại ngay lập tức để bảo vệ phần cứng
  }
  else
  {
    safetyOverrideActive = false; // Giải phóng cờ ghi đè nếu góc quét hiện tại thoáng
  }

  // 3. Tính toán góc quay tiếp theo cho chu kỳ sau (mỗi bước dịch 5 độ cho mượt)
  currentAngle += (scanDirection * 5); 

  // Nếu Servo chạm biên giới hạn quét phải (120 độ), đảo hướng quét sang trái
  if (currentAngle >= 120) {
    currentAngle = 120;
    scanDirection = -1; // Đổi hướng quay ngược về
  }
  // Nếu Servo chạm biên giới hạn quét trái (60 độ), đảo hướng quét sang phải
  else if (currentAngle <= 60) {
    currentAngle = 60;
    scanDirection = 1;  // Đổi hướng quay tiến lên
  }
}

// ======================================================
// INPUT LỆNH - ĐỌC TÍN HIỆU ĐIỀU KHIỂN
// ======================================================

void readBluetooth(){
  if (Serial.available() > 0){
    char cmd = Serial.read(); 
    // ĐÃ CẬP NHẬT: Thêm 'U' (Tăng tốc) và 'D' (Giảm tốc) vào danh sách ký tự hợp lệ
    if (cmd == 'F' || cmd == 'B' || cmd == 'L' || cmd == 'R' || cmd == 'S' || cmd == 'U' || cmd == 'D'){
      bluetoothCommand = cmd; 
    }
  }
}

void executeBluetoothCommand()
{
  // ⚡ KIỂM TRA ĐIỀU KIỆN AN TOÀN TRƯỚC (SAFETY OVERRIDE FIRST)
  // Nếu mắt quét radar phát hiện nguy hiểm, ép dừng động cơ và phớt lờ lệnh tay
  if (safetyOverrideActive)
  {
    stopMotor();
    return;
  }

  // ======================================================
  // KHU VỰC XỬ LÝ LỆNH TỐC ĐỘ (MỚI BỔ SUNG & TỐI ƯU CHỐNG KHỰNG XE)
  // ======================================================
  if (bluetoothCommand == 'U') {
    motorSpeed += 25; // Mỗi lần nhận lệnh từ Python sẽ tăng tốc lên 1 nấc
    if (motorSpeed > 255) motorSpeed = 255; // Khống chế tối đa 255
    dynamicSpeed = motorSpeed; 
    
    // Đảo biến lệnh về hướng đi trước đó để xe giữ nguyên trạng thái chạy, không bị phanh giật cục
    bluetoothCommand = lastDirectionCommand; 
  } 
  else if (bluetoothCommand == 'D') {
    motorSpeed -= 25; // Mỗi lần nhận lệnh từ Python sẽ giảm tốc đi 1 nấc
    if (motorSpeed < 100) motorSpeed = 100; // Giữ nấc tối thiểu 100 để xe không bị nghẹt cơ
    dynamicSpeed = motorSpeed; 
    
    bluetoothCommand = lastDirectionCommand; 
  }

  // Thực thi các chuyển động dựa theo ký tự hướng đi
  switch (bluetoothCommand)
  {
    case 'F': 
      moveForward();  
      lastDirectionCommand = 'F'; // Ghi nhớ trạng thái xe đang đi thẳng
      break;
    case 'B': 
      moveBackward(); 
      lastDirectionCommand = 'B'; // Ghi nhớ trạng thái xe đang lùi
      break;
    case 'L': 
      turnLeft();     
      // Không lưu hướng rẽ vào lastDirectionCommand để tránh xe tự xoay vòng tròn vô hạn
      break;
    case 'R': 
      turnRight();    
      break;
    case 'S':
      stopMotor();
      lastDirectionCommand = 'S'; // Ghi nhớ trạng thái xe đang đứng im
      break;
    default:  
      stopMotor();    
      break;
  }
}

// ======================================================
// SETUP - KHỞI TẠO PHẦN CỨNG
// ======================================================
void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  // Cấu hình các chân điều khiển linh kiện cầu H là ngõ ra (OUTPUT)
  pinMode(ENA_PIN, OUTPUT);   pinMode(IN1_A_PIN, OUTPUT);   pinMode(IN2_A_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);   pinMode(IN1_B_PIN, OUTPUT);   pinMode(IN2_B_PIN, OUTPUT);

  // Cấu hình các chân cho cảm biến siêu âm HC-SR04
  pinMode(TRIG_PIN, OUTPUT);  pinMode(ECHO_PIN, INPUT);  

  // Khởi tạo cơ cấu động cơ Servo SG90
  radarServo.attach(SERVO_PIN); 
  radarServo.write(90); // Ban đầu cho Servo nhìn thẳng 
  delay(500);
  
  lastRadarTime = millis(); // Cài đặt mốc thời gian chạy nền ban đầu
}

// ======================================================
// LOOP - VÒNG LẶP CHƯƠNG TRÌNH CHÍNH (CHẠY VÔ HẠN)
// ======================================================
void loop()
{
  // Bước 1: Liên tục đọc lệnh điều khiển thủ công (từ cử chỉ ngón tay Python truyền xuống)
  readBluetooth();

  // Bước 2: Kích hoạt radar quét liên tục không chặn (Servo tự gạt để dò chướng ngại vật bảo vệ xe)
  runContinuousRadar();

  // Bước 3: Đưa lệnh điều khiển vào bộ xử lý thực thi động cơ dựa trên trạng thái an toàn
  executeBluetoothCommand();
}