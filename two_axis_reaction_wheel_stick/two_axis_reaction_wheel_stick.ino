#include <Wire.h>
#include <EEPROM.h>
#include <PWM.h> 

#define MPU6050       0x68         // Device address
#define ACCEL_CONFIG  0x1C         // Accelerometer configuration address
#define GYRO_CONFIG   0x1B         // Gyro configuration address
#define PWR_MGMT_1    0x6B
#define PWR_MGMT_2    0x6C

#define BRAKE         8 
#define PWM_X         9
#define PWM_Y         10
#define DIRECTION_X   4
#define DIRECTION_Y   3

#define BUZZER        12
#define VBAT          A7

float K1 = 70;
float K2 = 5.15;
float K3 = 0.035;
float loop_time = 10;

float alpha = 0.4; 

struct OffsetsObj {
  int ID;
  float X;
  float Y;
};

OffsetsObj offsets;

int pwm_X, pwm_Y = 0;
byte brake_t = 1;   // stabdis - '0 stop'

int32_t motor_speed_pwmX; 
int32_t motor_speed_pwmY;

uint32_t timer;
long currentT, previousT_1, previousT_2 = 0;  // laiko periodai

/* IMU Data */
int16_t  AcX, AcY, AcZ;
int32_t GyZ, GyX, GyY, gyroZ, gyroY;

//Sensor output scaling
#define accSens 0             // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define gyroSens 1            // 0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s
#define Gyro_amount 0.996     //percent of gyro in complementary filter

//IMU offset values
int16_t  AcX_offset = -750;
int16_t  AcY_offset = 280;
int16_t  AcZ_offset = 100;
int16_t  GyZ_offset = 0;
int16_t  GyY_offset = 0;
int32_t  GyZ_offset_sum = 0;
int32_t  GyY_offset_sum = 0;

float robot_angleX, robot_angleY;
float angleX, angleY;
float Acc_angleX, Acc_angleY;
float gyroZfilt, gyroYfilt;

bool vertical = false;  
bool calibrating = false;
bool calibrated = false;

uint8_t i2cData[14]; // Buffer for I2C data

void battVoltage(double voltage) {
  //Serial.print("batt: "); Serial.println(voltage); 
  if (voltage > 8 && voltage <= 9.5) {
    digitalWrite(BUZZER, HIGH);
  } else {
    digitalWrite(BUZZER, LOW);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);
  pinMode(BRAKE, OUTPUT);
  pinMode(DIRECTION_X, OUTPUT);
  pinMode(DIRECTION_Y, OUTPUT);
  InitTimersSafe();
  SetPinFrequencySafe(PWM_X, 20000);
  SetPinFrequencySafe(PWM_Y, 20000);
  pwmWrite(PWM_X, 255);
  pwmWrite(PWM_Y, 255);
  digitalWrite(BRAKE, HIGH);
  delay(1000);
  EEPROM.get(0, offsets);
  if (offsets.ID == 11) calibrated = true;
    else calibrated = false;
  Serial.println("Calibrating gyroscope...");
  angle_setup();
}

void loop() {

  currentT = millis();

  if (currentT - previousT_1 >= loop_time) {

    Tuning(); 
    angle_calc();

    if (vertical && calibrated) {
      digitalWrite(BRAKE, HIGH);
      gyroZ = GyZ / 131.0; // Convert to deg/s
      gyroY = GyY / 131.0; // Convert to deg/s

      gyroZfilt = alpha * gyroZ + (1 - alpha) * gyroZfilt;
      gyroYfilt = alpha * gyroY + (1 - alpha) * gyroYfilt;
	  
      pwm_X = constrain(K1 * angleX + K2 * gyroZfilt + K3 * motor_speed_pwmX, -255, 255); 
      pwm_Y = constrain(K1 * angleY + K2 * gyroYfilt + K3 * motor_speed_pwmY, -255, 255); 
      
      if (!calibrating) {
        Motor_controlX(pwm_X);
        motor_speed_pwmX += pwm_X;
        Motor_controlY(pwm_Y);
        motor_speed_pwmY += pwm_Y;
      } else {
          Motor_controlX(0);
          Motor_controlY(0);
      }
      previousT_1 = currentT;
    } else {
      Motor_controlX(0);
      Motor_controlY(0);
      digitalWrite(BRAKE, LOW);
      motor_speed_pwmX = 0;
      motor_speed_pwmY = 0;
    }
  }
  if (currentT - previousT_2 >= 2000) {
    battVoltage((double)analogRead(VBAT) / 38.4); // This is then connected to a 47k-12k voltage divider
	if (!calibrated && !calibrating) {
      Serial.println("first you need to calibrate the balancing point...");
    }
    previousT_2 = currentT;
  }
}

