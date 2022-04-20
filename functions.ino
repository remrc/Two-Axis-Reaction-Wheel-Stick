void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

//setup MPU6050
void angle_setup() {
  Wire.begin();
  delay (100);
  writeTo(MPU6050, PWR_MGMT_1, 0);
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3); // Specifying output scaling of accelerometer
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3); // Specifying output scaling of gyroscope
  delay (100);

  // calc Y gyro offset by averaging 1024 values
  for (int i = 0; i < 1024; i++) {
    angle_calc();
    GyZ_offset_sum += GyZ;
    delay(5);
  }
  GyZ_offset = GyZ_offset_sum >> 10;
  Serial.print("GyZ offset value = "); Serial.println(GyZ_offset);

  digitalWrite(BUZZER, HIGH);
  delay(70);
  digitalWrite(BUZZER, LOW);
  
  for (int i = 0; i < 1024; i++) {
    angle_calc();
    GyY_offset_sum += GyY;
    delay(5);
  }
  GyY_offset = GyY_offset_sum >> 10;
  Serial.print("GyY offset value = "); Serial.println(GyY_offset);
  
  digitalWrite(BUZZER, HIGH);
  delay(70);
  digitalWrite(BUZZER, LOW);
  delay(80);
  digitalWrite(BUZZER, HIGH);
  delay(70);
  digitalWrite(BUZZER, LOW);
}

void angle_calc() {
  
  Wire.beginTransmission(MPU6050);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true);  
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);                  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true);  
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  AcX += AcX_offset;
  AcY += AcY_offset;  
  AcZ += AcZ_offset;
  GyZ -= GyZ_offset;
  GyY -= GyY_offset;

  robot_angleX += GyZ * loop_time / 1000 / 65.536; 
  Acc_angleX = atan2(AcY, -AcX) * 57.2958;               // angle from acc. values  * 57.2958 (deg/rad)
  robot_angleX = robot_angleX * Gyro_amount + Acc_angleX * (1.0 - Gyro_amount);

  robot_angleY += GyY * loop_time / 1000 / 65.536;   
  Acc_angleY = -atan2(AcZ, -AcX) * 57.2958;              //angle from acc. values  * 57.2958 (deg/rad)
  robot_angleY = robot_angleY * Gyro_amount + Acc_angleY * (1.0 - Gyro_amount);
  
  angleX = robot_angleX - offsetX;
  angleY = robot_angleY - offsetY;
  
  if (abs(angleX) > 6 || abs(angleY) > 6) vertical = false;
  if (abs(angleX) < 0.3 && abs(angleY) < 0.3) vertical = true;

  //Serial.print("AngleX: "); Serial.print(angleX); Serial.print(" AngleY: "); Serial.println(angleY);
}

void Motor_controlX(int pwm) {
  if (pwm < 0) {
    digitalWrite(DIRECTION_X, LOW);
    pwm = -pwm;
  } else {
    digitalWrite(DIRECTION_X, HIGH);
  }
  pwmWrite(PWM_X, pwm > 255 ? 255 : 255 - pwm);
}

void Motor_controlY(int pwm) {
  if (pwm < 0) {
    digitalWrite(DIRECTION_Y, LOW);
    pwm = -pwm;
  } else {
    digitalWrite(DIRECTION_Y, HIGH);
  }
  pwmWrite(PWM_Y, pwm > 255 ? 255 : 255 - pwm);
}

int Tuning() {
  if (!Serial.available())  return 0;
  delay(2);
  char param = Serial.read();               // get parameter byte
  if (!Serial.available()) return 0;
  char cmd = Serial.read();                 // get command byte
  Serial.flush();
  switch (param) {
    case 'p':
      if (cmd == '+')    K1 += 1;
      if (cmd == '-')    K1 -= 1;
      printValues();
      break;
	case 'i':
      if (cmd == '+')    K2 += 0.01;
      if (cmd == '-')    K2 -= 0.01;
      printValues();
      break;  
    case 's':
      if (cmd == '+')    K3 += 0.005;
      if (cmd == '-')    K3 -= 0.005;
      printValues();
      break;  
    case 'c':
      if (cmd == '+') {
        calibrating = true;
         Serial.println("calibrating on");
      }
      if (cmd == '-')  {
        calibrating = false;
        offsetX = robot_angleX;
        offsetY = robot_angleY;
        Serial.println("calibrating off");
        Serial.print("X: "); Serial.print(offsetX); Serial.print(" Y: "); Serial.println(offsetY);
      }
      break;         
   }
}

void printValues() {
  Serial.print("K1: "); Serial.print(K1);
  Serial.print(" K2: "); Serial.print(K2);
  Serial.print(" K3: "); Serial.println(K3,3);
}
