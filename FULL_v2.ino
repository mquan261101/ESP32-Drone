// =====================================================
// 1. Khai báo thư viện
// =====================================================
#include "sbus.h"
#include <Wire.h>
#include <BasicLinearAlgebra.h>
#include <math.h>

using namespace BLA;

// =====================================================
// 2. ĐỊNH NGHĨA CHÂN KẾT NỐI
// =====================================================
#define PIN_MOTOR1 27
#define PIN_MOTOR2 14 
#define PIN_MOTOR3 12 
#define PIN_MOTOR4 13

#define RX_PIN 16        // Chân RX cho SBUS (UART2)
#define TX_PIN -1        // Không sử dụng TX

#define SDA_PIN 21
#define SCL_PIN 22

#define LED_PIN 2

// =====================================================
// 3. SBUS OBJECT
// =====================================================
bfs::SbusRx sbus_rx(&Serial2, RX_PIN, TX_PIN, true, false);
bfs::SbusData data;

// =====================================================
// 4. BIẾN BỘ THU SBUS
// =====================================================
volatile int ReceiverValue[6] = {0, 0, 0, 0, 0, 0};
const float t = 0.004;  // 250Hz (4ms)

// =====================================================
// 6. BIẾN CON QUAY HỒI CHUYỂN
// =====================================================
volatile float RatePitch, RateRoll, RateYaw;
volatile float RateCalibrationRoll = -1.94, RateCalibrationPitch = -0.97, RateCalibrationYaw = -1.92;

const float GYRO_FILTER_ALPHA = 0.75f;
volatile float RateRollFiltered = 0, RatePitchFiltered = 0, RateYawFiltered = 0;

// =====================================================
// 7. BIẾN GIA TỐC KẾ VÀ GÓC
// =====================================================
volatile float AccX, AccY, AccZ;
volatile float AngleRoll, AnglePitch;
volatile float AccXCalibration = 0.05, AccYCalibration = -0.00, AccZCalibration = 0.88;

// =====================================================
// 11. BIẾN BAROMETER (BMP280)
// =====================================================
uint16_t dig_T1, dig_P1;
int16_t  dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5;
int16_t  dig_P6, dig_P7, dig_P8, dig_P9;

float AltitudeBarometer, AltitudeBarometerStartUp;
float AccZInertial;

// =====================================================
// 8. BIẾN KALMAN FILTER 1D (CHO GÓC ROLL/PITCH)
// =====================================================
float KalmanAngleRoll = 0,  KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
float Kalman1DOutput[] = {0, 0};

// =====================================================
// 12. BIẾN KALMAN FILTER 2D (CHO ĐỘ CAO & VẬN TỐC THẲNG ĐỨNG)
// =====================================================
float AltitudeKalman, VelocityVerticalKalman;

BLA::Matrix<2,2> F;  BLA::Matrix<2,1> G;
BLA::Matrix<2,2> P;  BLA::Matrix<2,2> Q;
BLA::Matrix<2,1> S;  BLA::Matrix<1,2> H;
BLA::Matrix<2,2> I;  BLA::Matrix<1,1> Acc;
BLA::Matrix<2,1> K;  BLA::Matrix<1,1> R;
BLA::Matrix<1,1> L;  BLA::Matrix<1,1> M;

// =====================================================
// 8. BIẾN PID VÒNG TRONG (RATE MODE)
// =====================================================
volatile float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
volatile float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
volatile float InputRoll, InputThrottle, InputPitch, InputYaw;
volatile float PrevErrorRateRoll = 0, PrevErrorRatePitch = 0, PrevErrorRateYaw = 0;
volatile float PrevItermRateRoll = 0, PrevItermRatePitch = 0, PrevItermRateYaw = 0;
volatile float PIDReturn[3] = {0, 0, 0};

const float PRateRoll = 0.625;
const float IRateRoll = 2.15;
const float DRateRoll = 0.0085;

const float PRatePitch = PRateRoll;
const float IRatePitch = IRateRoll;
const float DRatePitch = DRateRoll;

const float PRateYaw = 4;
const float IRateYaw = 2.85;  
const float DRateYaw = 0;

volatile float PtermRateRoll, ItermRateRoll, DtermRateRoll, PIDOutputRateRoll;
volatile float PtermRatePitch, ItermRatePitch, DtermRatePitch, PIDOutputRatePitch;
volatile float PtermRateYaw, ItermRateYaw, DtermRateYaw, PIDOutputRateYaw;

// =====================================================
// 10. BIẾN PID VÒNG NGOÀI (ANGLE CONTROL)
// =====================================================
volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float PrevErrorAngleRoll = 0, PrevErrorAnglePitch = 0;
volatile float PrevItermAngleRoll = 0, PrevItermAnglePitch = 0;

const float PAngleRoll = 2;
const float PAnglePitch = PAngleRoll;
const float IAngleRoll = 0.525;
const float IAnglePitch = IAngleRoll;
const float DAngleRoll = 0.0075; 
const float DAnglePitch = DAngleRoll;

volatile float PtermAngleRoll, ItermAngleRoll, DtermAngleRoll, PIDOutputAngleRoll;
volatile float PtermAnglePitch, ItermAnglePitch, DtermAnglePitch, PIDOutputAnglePitch;

// =====================================================
// 13. BIẾN PID VẬN TỐC THẲNG ĐỨNG (ALTITUDE HOLD)
// =====================================================
float DesiredVelocityVertical, ErrorVelocityVertical;
float PVelocityVertical = 0.75;
float IVelocityVertical = 0.0001;
float DVelocityVertical = 0.0035;
float PrevErrorVelocityVertical = 0, PrevItermVelocityVertical = 0;

// =====================================================
// 14. HÀM ĐỌC GYRO & ACCEL (MPU-6050)
// =====================================================
void gyro_signals(void) {
    // // Cấu hình DLPF
    // Wire.beginTransmission(0x68);
    // Wire.write(0x1A);
    // Wire.write(0x05);
    // Wire.endTransmission();

    // // Cấu hình accel ±8g
    // Wire.beginTransmission(0x68);
    // Wire.write(0x1C);
    // Wire.write(0x10);
    // Wire.endTransmission();

    // Đọc accel
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);
    int16_t AccXLSB = Wire.read() << 8 | Wire.read();
    int16_t AccYLSB = Wire.read() << 8 | Wire.read();
    int16_t AccZLSB = Wire.read() << 8 | Wire.read();

    // // Cấu hình gyro ±500°/s
    // Wire.beginTransmission(0x68);
    // Wire.write(0x1B);
    // Wire.write(0x8);
    // Wire.endTransmission();

    // Đọc gyro
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);
    int16_t GyroX = Wire.read() << 8 | Wire.read();
    int16_t GyroY = Wire.read() << 8 | Wire.read();
    int16_t GyroZ = Wire.read() << 8 | Wire.read();

    RateRoll = (float)GyroX / 65.5;
    RatePitch = (float)GyroY / 65.5;
    RateYaw = (float)GyroZ / 65.5;
    AccX = (float)AccXLSB / 4096;
    AccY = (float)AccYLSB / 4096;
    AccZ = (float)AccZLSB / 4096;

    // AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 1 / (3.142 / 180);
    // AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 1 / (3.142 / 180);
}

void barometer_signals(void) {
    Wire.beginTransmission(0x76);
    Wire.write(0xF7);
    Wire.endTransmission();
    Wire.requestFrom(0x76, 6);
    uint32_t press_msb = Wire.read();
    uint32_t press_lsb = Wire.read();
    uint32_t press_xlsb = Wire.read();
    uint32_t temp_msb = Wire.read();
    uint32_t temp_lsb = Wire.read();
    uint32_t temp_xlsb = Wire.read();
    unsigned long int adc_P = (press_msb << 12) | (press_lsb << 4) | (press_xlsb >> 4);
    unsigned long int adc_T = (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >> 4);
    signed long int var1, var2;
    var1 = ((((adc_T >> 3) - ((signed long int)dig_T1 << 1))) * ((signed long int)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) * ((adc_T >> 4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;
    signed long int t_fine = var1 + var2;
    unsigned long int p;
    var1 = (((signed long int)t_fine) >> 1) - (signed long int)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((signed long int)dig_P6);
    var2 = var2 + ((var1 * ((signed long int)dig_P5)) << 1);
    var2 = (var2 >> 2) + (((signed long int)dig_P4) << 16);
    var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((signed long int)dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((signed long int)dig_P1)) >> 15);
    if (var1 == 0) { p = 0; }
    p = (((unsigned long int)(((signed long int)1048576) - adc_P) - (var2 >> 12))) * 3125;
    if (p < 0x80000000) { p = (p << 1) / ((unsigned long int)var1); }
    else { p = (p / (unsigned long int)var1) * 2; }
    var1 = (((signed long int)dig_P9) * ((signed long int)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((signed long int)(p >> 2)) * ((signed long int)dig_P8)) >> 13;
    p = (unsigned long int)((signed long int)p + ((var1 + var2 + dig_P7) >> 4));
    double pressure = (double)p / 100;
    AltitudeBarometer = 44330 * (1 - pow(pressure / 1013.25, 1 / 5.255)) * 100;
}


// =====================================================
// 20. HÀM PID
// =====================================================
void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm) {
    float Pterm = P * Error;
    float Iterm = PrevIterm + I * (Error + PrevError) * 0.004 / 2;
    if (Iterm > 400) Iterm = 400;
    else if (Iterm < -400) Iterm = -400;

    float Dterm = D * (Error - PrevError) / 0.004;
    float PIDOutput = Pterm + Iterm + Dterm;
    if (PIDOutput > 400) PIDOutput = 400;
    else if (PIDOutput < -400) PIDOutput = -400;

    PIDReturn[0] = PIDOutput;
    PIDReturn[1] = Error;
    PIDReturn[2] = Iterm;
}

// =====================================================
// 22. HÀM RESET PID
// =====================================================
void reset_pid(void) {
    PrevErrorRateRoll = 0;  PrevErrorRatePitch = 0;  PrevErrorRateYaw = 0;
    PrevItermRateRoll = 0;  PrevItermRatePitch = 0;  PrevItermRateYaw = 0;
    PrevErrorAngleRoll = 0; PrevErrorAnglePitch = 0;
    PrevItermAngleRoll = 0; PrevItermAnglePitch = 0;
    PrevErrorVelocityVertical = 0;
    PrevItermVelocityVertical = 0;
}

// =====================================================
// 17. HÀM KALMAN FILTER 1D
// =====================================================
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
    KalmanState = KalmanState + 0.004 * KalmanInput;
    KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
    float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);
    KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
    KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;

    Kalman1DOutput[0] = KalmanState;
    Kalman1DOutput[1] = KalmanUncertainty;
}

// =====================================================
// 18. HÀM KALMAN FILTER 2D
// =====================================================
void kalman_2d(void) {
    Acc = {AccZInertial};
    S = F * S + G * Acc;
    P = F * P * ~F + Q;
    L = H * P * ~H + R;
    K = P * ~H * Invert(L);
    M = {AltitudeBarometer};
    S = S + K * (M - H * S);
    AltitudeKalman = S(0, 0);
    VelocityVerticalKalman = S(1, 0);
    P = (I - K * H) * P;
}

// =====================================================
// 10. ĐẦU VÀO CHO 4 ĐỘNG CƠ
// =====================================================
volatile float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
const int ThrottleIdle = 1180;
const int ThrottleCutOff = 1000;

// =====================================================
// 17. HÀM ĐỌC SBUS
// =====================================================
void read_receiver(void) {
    if (sbus_rx.Read()) {
        data = sbus_rx.data();
        if (!data.failsafe) {
            uint16_t ch1_raw = data.ch[0];
            uint16_t ch2_raw = data.ch[1];
            uint16_t ch3_raw = data.ch[2];
            uint16_t ch4_raw = data.ch[3];

            ReceiverValue[0] = constrain(map(ch1_raw, 172, 1811, 1000, 2000), 1000, 2000);
            ReceiverValue[1] = constrain(map(ch2_raw, 172, 1811, 1000, 2000), 1000, 2000);
            ReceiverValue[2] = constrain(map(ch3_raw, 172, 1811, 1000, 2000), 1000, 2000);
            ReceiverValue[3] = constrain(map(ch4_raw, 172, 1811, 1000, 2000), 1000, 2000);
        } else {
            ReceiverValue[0] = 1500;
            ReceiverValue[1] = 1500;
            ReceiverValue[2] = 1000;
            ReceiverValue[3] = 1500;
        }
    }
}

// =====================================================
// 11. DEBUG
// =====================================================
volatile uint32_t LoopTimer;
uint32_t last_debug_time = 0;
const uint32_t DEBUG_INTERVAL = 1000000;  // 1 giây
uint32_t max_loop_time_us = 0;            // theo dõi loop overrun

// =====================================================
// 12. SETUP - KHỞI TẠO
// =====================================================
void setup() {
    // LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    // Serial debug
    Serial.begin(115200);
    while (!Serial) {}
    Serial.println("\n=== FLIGHT CONTROLLER - SBUS ===");

    // I2C
    Wire.setClock(400000);
    Wire.begin(SDA_PIN, SCL_PIN);
    delay(250);

    // MPU-6050 wake up
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();

    Wire.beginTransmission(0x68);
    Wire.write(0x1A);
    Wire.write(0x05);
    Wire.endTransmission();

    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();

    Wire.beginTransmission(0x68);
    Wire.write(0x1B);
    Wire.write(0x08);
    Wire.endTransmission();

    // BMP280 config
    Wire.beginTransmission(0x76);
    Wire.write(0xF4);
    Wire.write(0x57);
    Wire.endTransmission();
    Wire.beginTransmission(0x76);
    Wire.write(0xF5);
    Wire.write(0x14);
    Wire.endTransmission();

    // Đọc calibration data BMP280
    uint8_t calib[24], i = 0;
    Wire.beginTransmission(0x76);
    Wire.write(0x88);
    Wire.endTransmission();
    Wire.requestFrom(0x76, 24);
    while (Wire.available()) {
        calib[i] = Wire.read();
        i++;
    }
    dig_T1 = (calib[1] << 8) | calib[0];
    dig_T2 = (calib[3] << 8) | calib[2];
    dig_T3 = (calib[5] << 8) | calib[4];
    dig_P1 = (calib[7] << 8) | calib[6];
    dig_P2 = (calib[9] << 8) | calib[8];
    dig_P3 = (calib[11] << 8) | calib[10];
    dig_P4 = (calib[13] << 8) | calib[12];
    dig_P5 = (calib[15] << 8) | calib[14];
    dig_P6 = (calib[17] << 8) | calib[16];
    dig_P7 = (calib[19] << 8) | calib[18];
    dig_P8 = (calib[21] << 8) | calib[20];
    dig_P9 = (calib[23] << 8) | calib[22];
    delay(250);

    // Calibration gyro & barometer
    for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
        barometer_signals();
        AltitudeBarometerStartUp += AltitudeBarometer;
        delay(1);
    }
    AltitudeBarometerStartUp /= 2000;

    // Khởi tạo Kalman 2D
    F = {1, 0.004,
         0, 1};
    G = {0.5 * 0.004 * 0.004,
         0.004};
    H = {1, 0};
    I = {1, 0,
         0, 1};
    Q = G * ~G * 10 * 10;
    R = {30 * 30};
    P = {0, 0,
         0, 0};
    S = {0,
         0};

    // PWM cho ESC (LEDC)
    ledcAttach(PIN_MOTOR1, 250, 12);
    ledcAttach(PIN_MOTOR2, 250, 12);
    ledcAttach(PIN_MOTOR3, 250, 12);
    ledcAttach(PIN_MOTOR4, 250, 12);

    // SBUS
    sbus_rx.Begin();

    // An toàn: chờ cần ga về vị trí thấp
    while (ReceiverValue[2] < 1020 || ReceiverValue[2] > 1050) {
        read_receiver();
        delay(4);
    }

    digitalWrite(LED_PIN, LOW);
    Serial.println("Ready to fly!");

    LoopTimer = micros();
}

// =====================================================
// 19. LOOP CHÍNH
// =====================================================
void loop() {
    // --- Đọc gyro & accel ---
    gyro_signals();
    RateRoll -= RateCalibrationRoll;
    RatePitch -= RateCalibrationPitch;
    RateYaw -= RateCalibrationYaw;

    //Lọc thông thấp IIR cho gyro
    RateRollFiltered  = GYRO_FILTER_ALPHA * RateRollFiltered  + (1 - GYRO_FILTER_ALPHA) * RateRoll;
    RatePitchFiltered = GYRO_FILTER_ALPHA * RatePitchFiltered + (1 - GYRO_FILTER_ALPHA) * RatePitch;
    RateYawFiltered   = GYRO_FILTER_ALPHA * RateYawFiltered   + (1 - GYRO_FILTER_ALPHA) * RateYaw;

    // --- Kalman 1D cho góc ---
    kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRollFiltered, AngleRoll);
    KalmanAngleRoll = Kalman1DOutput[0];
    KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitchFiltered, AnglePitch);
    KalmanAnglePitch = Kalman1DOutput[0];
    KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

    // --- Tính AccZ inertial ---
    AccZInertial = -sin(AnglePitch * (3.142 / 180)) * AccX
                 + cos(AnglePitch * (3.142 / 180)) * sin(AngleRoll * (3.142 / 180)) * AccY
                 + cos(AnglePitch * (3.142 / 180)) * cos(AngleRoll * (3.142 / 180)) * AccZ;
    AccZInertial = (AccZInertial - 1) * 9.81 * 100;

    // --- Barometer & Kalman 2D ---
    barometer_signals();
    AltitudeBarometer -= AltitudeBarometerStartUp;
    kalman_2d();

    // --- Đọc SBUS ---
    read_receiver();

    // --- Tính giá trị mong muốn ---
    DesiredAngleRoll = 0.10 * (ReceiverValue[0] - 1500);
    DesiredAnglePitch = 0.10 * (ReceiverValue[1] - 1500);
    DesiredRateYaw = 0.15 * (ReceiverValue[3] - 1500);
    DesiredVelocityVertical = 0.3 * (ReceiverValue[2] - 1500);

    // --- PID Velocity Vertical (Altitude Hold) ---
    ErrorVelocityVertical = DesiredVelocityVertical - VelocityVerticalKalman;
    pid_equation(ErrorVelocityVertical, PVelocityVertical, IVelocityVertical, DVelocityVertical,
                 PrevErrorVelocityVertical, PrevItermVelocityVertical);
    InputThrottle = 1500 + PIDReturn[0];
    PrevErrorVelocityVertical = PIDReturn[1];
    PrevItermVelocityVertical = PIDReturn[2];

    // --- PID Angle Roll ---
    ErrorAngleRoll = DesiredAngleRoll - KalmanAngleRoll;
    pid_equation(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll,
                 PrevErrorAngleRoll, PrevItermAngleRoll);
    DesiredRateRoll = PIDReturn[0];
    PrevErrorAngleRoll = PIDReturn[1];
    PrevItermAngleRoll = PIDReturn[2];

    // --- PID Angle Pitch ---
    ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch;
    pid_equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch,
                 PrevErrorAnglePitch, PrevItermAnglePitch);
    DesiredRatePitch = PIDReturn[0];
    PrevErrorAnglePitch = PIDReturn[1];
    PrevItermAnglePitch = PIDReturn[2];

    // --- PID Rate ---
    ErrorRateRoll  = DesiredRateRoll  - RateRollFiltered;
    ErrorRatePitch = DesiredRatePitch - RatePitchFiltered;
    ErrorRateYaw   = DesiredRateYaw   - RateYawFiltered;

    pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll,
                 PrevErrorRateRoll, PrevItermRateRoll);
    InputRoll = PIDReturn[0];
    PrevErrorRateRoll = PIDReturn[1];
    PrevItermRateRoll = PIDReturn[2];

    pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch,
                 PrevErrorRatePitch, PrevItermRatePitch);
    InputPitch = PIDReturn[0];
    PrevErrorRatePitch = PIDReturn[1];
    PrevItermRatePitch = PIDReturn[2];

    pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw,
                 PrevErrorRateYaw, PrevItermRateYaw);
    InputYaw = PIDReturn[0];
    PrevErrorRateYaw = PIDReturn[1];
    PrevItermRateYaw = PIDReturn[2];

    // --- Giới hạn ga ---
    if (InputThrottle > 1800) InputThrottle = 1800;

    // --- Motor mixing ---
    MotorInput1 = InputThrottle - InputPitch - InputRoll - InputYaw;
    MotorInput2 = InputThrottle + InputPitch - InputRoll + InputYaw;
    MotorInput3 = InputThrottle + InputPitch + InputRoll - InputYaw;
    MotorInput4 = InputThrottle - InputPitch + InputRoll + InputYaw;

    // --- Giới hạn động cơ ---
    if (MotorInput1 > 2000) MotorInput1 = 1999;
    if (MotorInput2 > 2000) MotorInput2 = 1999;
    if (MotorInput3 > 2000) MotorInput3 = 1999;
    if (MotorInput4 > 2000) MotorInput4 = 1999;

    if (MotorInput1 < ThrottleIdle) MotorInput1 = ThrottleIdle;
    if (MotorInput2 < ThrottleIdle) MotorInput2 = ThrottleIdle;
    if (MotorInput3 < ThrottleIdle) MotorInput3 = ThrottleIdle;
    if (MotorInput4 < ThrottleIdle) MotorInput4 = ThrottleIdle;

    // --- An toàn: cắt ga ---
    if (ReceiverValue[2] < 1030) {
        MotorInput1 = ThrottleCutOff;
        MotorInput2 = ThrottleCutOff;
        MotorInput3 = ThrottleCutOff;
        MotorInput4 = ThrottleCutOff;
        reset_pid();
    }

    // --- Ghi PWM cho ESC ---
    ledcWrite(PIN_MOTOR1, (uint32_t)(1.024 * MotorInput1));
    ledcWrite(PIN_MOTOR2, (uint32_t)(1.024 * MotorInput2));
    ledcWrite(PIN_MOTOR3, (uint32_t)(1.024 * MotorInput3));
    ledcWrite(PIN_MOTOR4, (uint32_t)(1.024 * MotorInput4));

    // --- Duy trì tần số 250Hz + cảnh báo overrun ---
    uint32_t elapsed = micros() - LoopTimer;
    if (elapsed > max_loop_time_us) max_loop_time_us = elapsed;

    if (elapsed < (uint32_t)(t * 1000000)) {
        while (micros() - LoopTimer < (uint32_t)(t * 1000000));
    }
    LoopTimer = micros();
}















