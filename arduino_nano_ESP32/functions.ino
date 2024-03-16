#include <cctype> // For tolower()
#include <Arduino.h>

void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

void beep() {
    digitalWrite(BUZZER, HIGH);
    delay(70);
    digitalWrite(BUZZER, LOW);
    delay(80);
}

void save() {
    EEPROM.put(0, offsets);
    EEPROM.commit(); //for ESP32
    EEPROM.get(0, offsets);
    if (offsets.ID1 == 99 && offsets.ID2 == 99 && offsets.ID3 == 99 && offsets.ID4 == 99) calibrated = true;
    calibrating = false;
    Serial.println("calibrating off");
    beep();
}

void angle_setup() {
  Wire.begin();
  delay (100);
  writeTo(MPU6050, PWR_MGMT_1, 0);
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3); // Specifying output scaling of accelerometer
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3); // Specifying output scaling of gyroscope
  delay (100);

  for (int i = 0; i < 1024; i++) {
    angle_calc();
    Serial.println(GyZ);
    GyZ_offset_sum += GyZ;
    delay(3);
  }
  GyZ_offset = GyZ_offset_sum >> 10;
  //Serial.print("GyZ offset value = "); Serial.println(GyZ_offset);
  beep();
  
  for (int i = 0; i < 1024; i++) {
    angle_calc();
    Serial.println(GyY);
    GyY_offset_sum += GyY;
    delay(3);
  }
  GyY_offset = GyY_offset_sum >> 10;
  //Serial.print("GyY offset value = "); Serial.println(GyY_offset);

  // ESP32
    for (int i = 0; i < 1024; i++) {
    angle_calc();
    GyX_offset_sum += GyX;
    delay(3);
  }
  GyX_offset = GyX_offset_sum >> 10;
  //Serial.print("GyX offset value = "); Serial.println(GyX_offset);

  beep();
  beep();
}

void angle_calc() {
  
  // read raw accel/gyro measurements from device
  Wire.beginTransmission(MPU6050);
  //Wire.write(0x45); //maybe 0x43 for ESP32
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true);  
  // GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  // GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  //Maybe for ESP32
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

  GyZ -= GyZ_offset;
  GyY -= GyY_offset;
  GyX -= GyX_offset; //Maybe for ESP32

  robot_angleX += GyZ * loop_time / 1000 / 65.536;
  Acc_angleX = atan2(AcY, -AcX) * 57.2958;
  robot_angleX = robot_angleX * Gyro_amount + Acc_angleX * (1.0 - Gyro_amount);

  robot_angleY += GyY * loop_time / 1000 / 65.536;
  Acc_angleY = -atan2(AcZ, -AcX) * 57.2958; 
  robot_angleY = robot_angleY * Gyro_amount + Acc_angleY * (1.0 - Gyro_amount);

  angleX = robot_angleX;
  angleY = robot_angleY;

  //Serial.print("AngleX: "); Serial.print(angleX); Serial.print( "AngleY: "); Serial.println(angleY);

  if (abs(angleX - offsets.X1) < 0.4 && abs(angleY - offsets.Y1) < 0.4) {
    balancing_point = 1;
    if (!vertical) beep();
    vertical = true;
  } else if (abs(angleX - offsets.X2) < 3 && abs(angleY - offsets.Y2) < 0.6) {
    balancing_point = 2;
    if (!vertical) beep();
    vertical = true;
  } else if (abs(angleX - offsets.X3) < 6 && abs(angleY - offsets.Y3) < 0.6) {
    balancing_point = 3;
    if (!vertical) beep();
    vertical = true;
  } else if (abs(angleX - offsets.X4) < 0.6 && abs(angleY - offsets.Y4) < 3) { //ESP32 = 2
    balancing_point = 4;
    if (!vertical) beep();
    vertical = true;
  }
}

void XY_to_threeWay(float pwm_X, float pwm_Y) {
  
  int16_t m1 = round(0.5 * pwm_X - 0.75 * pwm_Y); 
  int16_t m2 = round(0.5 * pwm_X + 0.75 * pwm_Y);
  //int16_t m3 = pwm_X;  //ESP32 is -pwm_x
  int16_t m3 = -pwm_X;
  m1 = constrain(m1, -255, 255);
  m2 = constrain(m2, -255, 255);
  m3 = constrain(m3, -255, 255);
  
  //Motor1_control(-m1); //ESP32 m1
  Motor1_control(m1);
  //Motor2_control(-m2); //ESP32 m2
  Motor2_control(m2);
  Motor3_control(m3);
}

void battVoltage(double voltage) {
    //Serial.print("batt: "); Serial.println(voltage); //debug
  if (voltage > 8 && voltage <= 9.5) {
    digitalWrite(BUZZER, HIGH);
  } else {
    digitalWrite(BUZZER, LOW);
  }
}

void pwmSet(uint8_t pin, uint32_t value) {
  analogWrite(pin,value);
}

void Motor1_control(int sp) {
  if (sp < 0) {
    digitalWrite(DIR_1, LOW);
    sp = -sp;
  } else {
    digitalWrite(DIR_1, HIGH);
  }
  pwmSet(PWM_1, sp > 255 ? 255 : 255 - sp);
}

void Motor2_control(int sp) {
  if (sp < 0) {
    digitalWrite(DIR_2, LOW);
    sp = -sp;
  } else {
    digitalWrite(DIR_2, HIGH);
  }
  pwmSet(PWM_2, sp > 255 ? 255 : 255 - sp);
}

void Motor3_control(int sp) {
  if (sp < 0) {
    digitalWrite(DIR_3, LOW);
    sp = -sp;
  } else {
    digitalWrite(DIR_3, HIGH);
  }
  pwmSet(PWM_3, sp > 255 ? 255 : 255 - sp);
}


// void Motor1_control(int sp) {
//   if (sp > 0) digitalWrite(DIR_1, LOW);
//     else digitalWrite(DIR_1, HIGH);
//   analogWrite(PWM_1, 255 - abs(sp));
// }

// void Motor2_control(int sp) {
//   if (sp > 0) digitalWrite(DIR_2, LOW);
//     else digitalWrite(DIR_2, HIGH);
//   analogWrite(PWM_2, 255 - abs(sp));
// }

// void Motor3_control(int sp) {
//   if (sp > 0) digitalWrite(DIR_3, LOW);
//     else digitalWrite(DIR_3, HIGH);
//   analogWrite(PWM_3, 255 - abs(sp));
//}

void printAngles(){
  if(calibrating){
    Serial.print("X: "); Serial.print(robot_angleX); Serial.print(" Y: "); Serial.println(robot_angleY);
  }
}

void sendMessageOverBLE(const String &message){
  if(pCharacteristic){
    pCharacteristic->setValue(message.c_str()); // Convert String to char array
    pCharacteristic->notify(); // Notify connected client
  }
}

int Tuning() {
  // if (!Serial.available()){
  //   return 0;
  // }  
  // //delay(2);
  // char param = Serial.read();               // get parameter byte
  // if (!Serial.available()){
  //   return 0;
  // } 
  // char cmd = Serial.read();                 // get command byte
  // if (!newBtDataAvailable){
  //   return 0;
  // }
  if (newBtDataAvailable && btCommand.length() >= 2) {
  char param = tolower(btCommand[0]); //command to lowercase
  char cmd = btCommand[1];

  if(cmd == '+' || cmd == '-'){
    float value = (cmd == '+') ? 1.0 : -1.0;
    switch(param){
      case 'p':
        //float value = (cmd == '+') ? 1.0 : -1.0; 
        pGain += value;
        sendMessageOverBLE(createStringValues());
        break;
      case 'i':
        //float value = (cmd == '+') ? 0.05 : -0.05;
        iGain += (value * 0.05);
        sendMessageOverBLE(createStringValues());
        break;
      case 's':
        //float value = (cmd == '+') ? 0.005 : -0.005;
        sGain += (value * 0.005);
        sendMessageOverBLE(createStringValues());
        break;
      case 'b':
        //float value = (cmd == '+') ? 1 : -1;
        bat_divider += value;
        sendMessageOverBLE(createStringValues());
        break;
      case 'c':
      if (cmd == '+' && !calibrating) {
        calibrating = true;
        sendMessageOverBLE("calibrating on");
        Serial.println("calibrating on");
      }
      if (cmd == '-' && calibrating)  {
        Serial.print("X: "); Serial.print(robot_angleX); Serial.print(" Y: "); Serial.println(robot_angleY);
        String angleMessage = "X: " + String(robot_angleX) + " Y " + String(robot_angleY);
        if (abs(robot_angleX) < 10 && abs(robot_angleY) < 10) {
          offsets.ID1 = 99;
          offsets.X1 = robot_angleX;
          offsets.Y1 = robot_angleY;
          //Serial.println("Vertex OK.");
          angleMessage += String("\nVertex OK.");
          save();
        } else if (robot_angleX > -45 && robot_angleX < -25 && robot_angleY > -30 && robot_angleY < -10) {
          offsets.ID2 = 99;
          offsets.X2 = robot_angleX;
          offsets.Y2 = robot_angleY;
          //Serial.println("First edge OK.");
          angleMessage += String("\nFirst edge OK.");
          save();
        } else if (robot_angleX > 20 && robot_angleX < 40 && robot_angleY > -30 && robot_angleY < -10) {
          offsets.ID3 = 99;
          offsets.X3 = robot_angleX;
          offsets.Y3 = robot_angleY;
          angleMessage += String("\nSecond edge OK.");
          //Serial.println("Second edge OK.");
          save();
        } else if (abs(robot_angleX) < 15 && robot_angleY > 30 && robot_angleY < 50) {
          offsets.ID4 = 99;
          offsets.X4 = robot_angleX;
          offsets.Y4 = robot_angleY;
          angleMessage += String("\nThird edge OK.");
          //Serial.println("Third edge OK.");
          save();
        } else {
          angleMessage += String("\nWrong angles!");
          //Serial.println("The angles are wrong!!!");
          beep();
          beep();
        }
        sendMessageOverBLE(angleMessage);
        angleMessage = "";
      }
      break;    
    }
  } else {
    String valueStr = btCommand.substring(1);
    float value = valueStr.toFloat();

    switch (param){
      case 'p': 
        pGain = value;
        sendMessageOverBLE(createStringValues());
        break;
      case 'i':
        iGain = value;
        sendMessageOverBLE(createStringValues());
        break;
      case 's':
        sGain = value;
        sendMessageOverBLE(createStringValues());
        break;
      case 'b':
        bat_divider += value;
        sendMessageOverBLE(createStringValues());
        break;
    }
  }

  //Serial.flush();
  // switch (param) {
  //   case 'p':
  //     if (cmd == '+')    pGain += 1;
  //     if (cmd == '-')    pGain -= 1;
  //     //printValues();
  //     sendMessageOverBLE(createStringValues());
  //     break;
  //   case 'i':
  //     if (cmd == '+')    iGain += 0.05;
  //     if (cmd == '-')    iGain -= 0.05;
  //     //printValues();
  //     sendMessageOverBLE(createStringValues());
  //     break;
  //   case 's':
  //     if (cmd == '+')    sGain += 0.005;
  //     if (cmd == '-')    sGain -= 0.005;
  //     //printValues();
  //     sendMessageOverBLE(createStringValues());
  //     break;  
  //   case 'b':
  //     if (cmd == '+')    bat_divider += 1;
  //     if (cmd == '-')    bat_divider -= 1;
  //     //printValues();
  //     sendMessageOverBLE(createStringValues());
  //     break;
  //   case 'c':
  //     if (cmd == '+' && !calibrating) {
  //       calibrating = true;
  //       sendMessageOverBLE("calibrating on");
  //       Serial.println("calibrating on");
  //     }
  //     if (cmd == '-' && calibrating)  {
  //       Serial.print("X: "); Serial.print(robot_angleX); Serial.print(" Y: "); Serial.println(robot_angleY);
  //       String angleMessage = "X: " + String(robot_angleX) + " Y " + String(robot_angleY);
  //       if (abs(robot_angleX) < 10 && abs(robot_angleY) < 10) {
  //         offsets.ID1 = 99;
  //         offsets.X1 = robot_angleX;
  //         offsets.Y1 = robot_angleY;
  //         //Serial.println("Vertex OK.");
  //         angleMessage += String("\nVertex OK.");
  //         save();
  //       } else if (robot_angleX > -45 && robot_angleX < -25 && robot_angleY > -30 && robot_angleY < -10) {
  //         offsets.ID2 = 99;
  //         offsets.X2 = robot_angleX;
  //         offsets.Y2 = robot_angleY;
  //         //Serial.println("First edge OK.");
  //         angleMessage += String("\nFirst edge OK.");
  //         save();
  //       } else if (robot_angleX > 20 && robot_angleX < 40 && robot_angleY > -30 && robot_angleY < -10) {
  //         offsets.ID3 = 99;
  //         offsets.X3 = robot_angleX;
  //         offsets.Y3 = robot_angleY;
  //         angleMessage += String("\nSecond edge OK.");
  //         //Serial.println("Second edge OK.");
  //         save();
  //       } else if (abs(robot_angleX) < 15 && robot_angleY > 30 && robot_angleY < 50) {
  //         offsets.ID4 = 99;
  //         offsets.X4 = robot_angleX;
  //         offsets.Y4 = robot_angleY;
  //         angleMessage += String("\nThird edge OK.");
  //         //Serial.println("Third edge OK.");
  //         save();
  //       } else {
  //         angleMessage += String("\nWrong angles!");
  //         //Serial.println("The angles are wrong!!!");
  //         beep();
  //         beep();
  //       }
  //       sendMessageOverBLE(angleMessage);
  //       angleMessage = "";
  //     }
  //     break;                
  //  }
   newBtDataAvailable = false;
   btCommand = "";
  }
   return 1; 
}

void printValues() {
  Serial.print("P: "); Serial.print(pGain);
  Serial.print(" I: "); Serial.print(iGain);
  Serial.print(" S: "); Serial.println(sGain,4);
  Serial.print("Bat_divider: "); Serial.println(bat_divider);
}

String createStringValues(){
  String message = "";
  message += "P: " + String(pGain) + "\n I: " + String(iGain) + "\n S: " + String(sGain,3)
  + "\n Bat_divier: " + String(bat_divider);
  return message;
}
