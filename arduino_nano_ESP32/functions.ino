#include <cctype> // For tolower()
#include <Arduino.h>

/**
 * Writes a byte value to a specific address of a device using I2C communication.
 *
 * @param device The I2C device address.
 * @param address The address to write the value to.
 * @param value The byte value to write.
 */
void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

/**
 * Generates a beep sound using the buzzer.
 * The buzzer is turned on for 70 milliseconds, then turned off for 80 milliseconds.
 */
void beep() {
    digitalWrite(BUZZER, HIGH);
    delay(70);
    digitalWrite(BUZZER, LOW);
    delay(80);
}

/**
 * Saves the offsets to the EEPROM memory and checks if the calibration is complete.
 * If the calibration is complete, sets the 'calibrated' flag to true.
 * Turns off the calibration mode and prints a message to the serial monitor.
 * Also triggers a beep sound.
 */
void save() {
    EEPROM.put(0, offsets);
    EEPROM.commit(); //for ESP32
    EEPROM.get(0, offsets);
    if (offsets.ID1 == 99 && offsets.ID2 == 99 && offsets.ID3 == 99 && offsets.ID4 == 99) calibrated = true;
    calibrating = false;
    Serial.println("calibrating off");
    beep();
}

/**
 * Initializes the MPU6050 sensor and calibrates the gyroscope readings.
 * This function sets up the necessary configurations for the sensor and calculates the gyroscope offset values.
 * It also performs a beep sound at the beginning and end of the calibration process.
 */
void angle_setup() {
  Wire.begin();
  delay (100);
  writeTo(MPU6050, PWR_MGMT_1, 0); // Wake up the MPU-6050
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3); // Specifying output scaling (sensitivity) of accelerometer
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3); // Specifying output scaling of gyroscope
  delay (100);

  // Calibrate the gyroscope z-axis
  for (int i = 0; i < 1024; i++) {
    angle_calc();
    Serial.println(GyZ);
    GyZ_offset_sum += GyZ;
    delay(3);
  }
  GyZ_offset = GyZ_offset_sum >> 10; // Divide by 1024
  beep();
  
  // Calibrate the gyroscope y-axis
  for (int i = 0; i < 1024; i++) {
    angle_calc();
    Serial.println(GyY);
    GyY_offset_sum += GyY;
    delay(3);
  }
  GyY_offset = GyY_offset_sum >> 10;

  // Calibrate the gyroscope x-axis
  for (int i = 0; i < 1024; i++) {
    angle_calc();
    Serial.println(GyX);
    GyX_offset_sum += GyX;
    delay(3);
  }

  beep();
  beep();
}

/**
 * Calculates the angles of the self-balancing cube using accelerometer and gyroscope measurements.
 * Reads raw accelerometer and gyroscope measurements from the MPU6050 device.
 * Calculates the robot's angle in the X and Y directions based on the gyroscope and accelerometer data.
 * Compares the calculated angles with predefined offsets to determine the balancing point of the cube.
 * Updates the `balancing_point` variable based on the calculated angles.
 * If the cube is in a vertical position, it plays a beep sound.
 */
void angle_calc() {
  
  // read raw gyro measurements from device
  Wire.beginTransmission(MPU6050);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true);  // request a total of 6 bytes 

  // Read high and low bytes of the gyro measurements, representing the rate of rotation around the X, Y, and Z axis
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L) 
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  // read raw accelerometer measurements from device
  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);                  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true);  

  // Read high and low bytes of the accelerometer measurements, representing the acceleration in the X, Y, and Z axis
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  // Correct the gyro stationary bias 
  GyZ -= GyZ_offset;
  GyY -= GyY_offset;
  GyX -= GyX_offset; 

  robot_angleX += GyZ * loop_time / 1000 / 65.536; // Integrate over time and divide by the sensitivity of the gyroscope -> More responsive and precise
  Acc_angleX = atan2(AcY, -AcX) * 57.2958; // Tilt angle relative to surface (in degrees) -> More stable (less noise and drift)
  robot_angleX = robot_angleX * Gyro_amount + Acc_angleX * (1.0 - Gyro_amount); // Complementary filter used to combine accelerometer and gyroscope data, filtering out noise and drift that could result from integrating over time

  robot_angleY += GyY * loop_time / 1000 / 65.536;
  Acc_angleY = -atan2(AcZ, -AcX) * 57.2958; 
  robot_angleY = robot_angleY * Gyro_amount + Acc_angleY * (1.0 - Gyro_amount); 

  angleX = robot_angleX;
  angleY = robot_angleY;

  //Serial.print("AngleX: "); Serial.print(angleX); Serial.print( "AngleY: "); Serial.println(angleY);

  // Determine the balancing point based on the calculated angles and predefined offsets (calibration points) 
  // If the current angle is close to a predefined balancing-angle, set the balancing point and play a beep sound
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

/**
 * Converts the X and Y values to three-way motor control signals.
 * 
 * @param pwm_X The PWM value for the X-axis.
 * @param pwm_Y The PWM value for the Y-axis.
 */
void XY_to_threeWay(float pwm_X, float pwm_Y) {
  
  // Calculate control signals for each motor
  int16_t m1 = round(0.5 * pwm_X - 0.75 * pwm_Y); 
  int16_t m2 = round(0.5 * pwm_X + 0.75 * pwm_Y);
  int16_t m3 = -pwm_X;
  m1 = constrain(m1, -255, 255);
  m2 = constrain(m2, -255, 255);
  m3 = constrain(m3, -255, 255);
  
  Motor1_control(m1);
  Motor2_control(m2);
  Motor3_control(m3);
}

/**
 * Sets the buzzer state based on the battery voltage.
 * 
 * @param voltage The battery voltage to be checked.
 */
void battVoltage(double voltage) {
  //Serial.print("batt: "); Serial.println(voltage); //debug
  //sendMessageOverBLE("Batt voltage: " + String(voltage)); // debug via BLE
  if (voltage > 8 && voltage <= 9.5) {
    digitalWrite(BUZZER, HIGH);
  } else {
    digitalWrite(BUZZER, LOW);
  }
}

/**
 * Sets the PWM (Pulse Width Modulation) value for a given pin.
 * 
 * @param pin The pin number to set the PWM value for.
 * @param value The PWM value to set (0-255).
 */
void pwmSet(uint8_t pin, uint32_t value) {
  analogWrite(pin,value);
}

/**
 * Controls Motor 1 based on the given speed.
 * 
 * @param sp The speed value to set for Motor 1. Positive values indicate forward direction, while negative values indicate reverse direction.
 */
void Motor1_control(int sp) {
  if (sp < 0) {
    digitalWrite(DIR_1, LOW);
    sp = -sp;
  } else {
    digitalWrite(DIR_1, HIGH);
  }
  pwmSet(PWM_1, sp > 255 ? 255 : 255 - sp);
}

/**
 * Controls the second motor based on the given speed.
 * 
 * @param sp The speed value to set for the motor. Positive values indicate forward direction, while negative values indicate reverse direction.
 */
void Motor2_control(int sp) {
  if (sp < 0) {
    digitalWrite(DIR_2, LOW);
    sp = -sp;
  } else {
    digitalWrite(DIR_2, HIGH);
  }
  pwmSet(PWM_2, sp > 255 ? 255 : 255 - sp);
}

/**
 * Controls Motor 3 based on the given speed value.
 * 
 * @param sp The speed value to set for Motor 3. Positive values rotate the motor in one direction, while negative values rotate it in the opposite direction.
 */
void Motor3_control(int sp) {
  if (sp < 0) {
    digitalWrite(DIR_3, LOW);
    sp = -sp;
  } else {
    digitalWrite(DIR_3, HIGH);
  }
  pwmSet(PWM_3, sp > 255 ? 255 : 255 - sp);
}

/**
 * Sends a message over BLE (Bluetooth Low Energy).
 * 
 * @param message The message to be sent.
 */
void sendMessageOverBLE(const String &message){
  if(pCharacteristic){
    pCharacteristic->setValue(message.c_str()); // Convert String to char array
    pCharacteristic->notify(); // Notify connected client
  }
}

/**
 * @brief This function handles the tuning of various parameters based on Bluetooth commands.
 * 
 * The function checks if new Bluetooth data is available and sets the tuning parameters based on the received commands.
 * 
 * @return 1 indicating successful execution of the function.
 */
int Tuning() {
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
      }
      if (cmd == '-' && calibrating)  {
        String angleMessage = "X: " + String(robot_angleX) + " Y " + String(robot_angleY);
        if (abs(robot_angleX) < 10 && abs(robot_angleY) < 10) {
          offsets.ID1 = 99;
          offsets.X1 = robot_angleX;
          offsets.Y1 = robot_angleY;
          angleMessage += String("\nVertex OK.");
          save();
        } else if (robot_angleX > -45 && robot_angleX < -25 && robot_angleY > -30 && robot_angleY < -10) {
          offsets.ID2 = 99;
          offsets.X2 = robot_angleX;
          offsets.Y2 = robot_angleY;
          angleMessage += String("\nFirst edge OK.");
          save();
        } else if (robot_angleX > 20 && robot_angleX < 40 && robot_angleY > -30 && robot_angleY < -10) {
          offsets.ID3 = 99;
          offsets.X3 = robot_angleX;
          offsets.Y3 = robot_angleY;
          angleMessage += String("\nSecond edge OK.");
          save();
        } else if (abs(robot_angleX) < 15 && robot_angleY > 30 && robot_angleY < 50) {
          offsets.ID4 = 99;
          offsets.X4 = robot_angleX;
          offsets.Y4 = robot_angleY;
          angleMessage += String("\nThird edge OK.");
          save();
        } else {
          angleMessage += String("\nWrong angles!");
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
   newBtDataAvailable = false;
   btCommand = "";
  }
   return 1; 
}

/**
 * Prints the values of the variables pGain, iGain, sGain, and bat_divider to the Serial monitor.
 * 
 * This function is used to display the current values of the control gains (pGain, iGain, sGain) and the battery voltage divider (bat_divider) on the Serial monitor.
 * The values are printed in the format "P: <pGain> I: <iGain> S: <sGain> Bat_divider: <bat_divider>".
 */
void printValues() {
  Serial.print("P: "); Serial.print(pGain);
  Serial.print(" I: "); Serial.print(iGain);
  Serial.print(" S: "); Serial.println(sGain,4);
  Serial.print("Bat_divider: "); Serial.println(bat_divider);
}

/**
 * Creates a string representation of the PID-values and the battery divider.
 * 
 * @return The string representation of the values.
 */
String createStringValues(){
  String message = "";
  message += "P: " + String(pGain) + "\n I: " + String(iGain) + "\n S: " + String(sGain,3)
  + "\n Bat_divier: " + String(bat_divider);
  return message;
}
