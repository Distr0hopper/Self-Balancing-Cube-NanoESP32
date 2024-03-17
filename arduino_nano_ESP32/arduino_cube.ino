#include "arduino_cube.h"
#include <Wire.h>
#include <EEPROM.h>
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <WiFi.h>

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
// Arduino MAC adress: 34:85:18:7B:F6:41


class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();

      if (value.length() > 0) {
        Serial.println("*********");
        Serial.print("Send value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);
        
        btCommand = ""; //Clear previous command

        for(int i = 0; i < value.length(); i++)
          btCommand += value[i];
        newBtDataAvailable = true;
        Serial.println();
        Serial.println("*********");
      }
    }
};

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      Serial.println("Device connected");
    }

    void onDisconnect(BLEServer* pServer) {
      Serial.println("Device disconnected");
      // Restart advertising after disconnection so connection can be established again
      BLEDevice::startAdvertising(); 
    }
};

void openBluetoothPort(){

  BLEDevice::init("CubliArduino");
  BLEServer *pServer = BLEDevice::createServer();

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );

  pCharacteristic->setCallbacks(new MyCallbacks());
  pServer->setCallbacks(new MyServerCallbacks());

  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
}

void setup() {
  Serial.begin(115200);
  openBluetoothPort();
  EEPROM.begin(EEPROM_SIZE); 

  pinMode(DIR_1, OUTPUT);
  pinMode(DIR_2, OUTPUT);
  pinMode(DIR_3, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  
  Motor1_control(0);
  Motor2_control(0);
  Motor3_control(0);

  EEPROM.get(0, offsets);
  if (offsets.ID1 == 99 && offsets.ID2 == 99 && offsets.ID3 == 99 && offsets.ID4 == 99) calibrated = true;
    else calibrated = false;
  delay(3000);
  beep();
  angle_setup();
  String calibStatus = "false";
  if(calibrated == 1) calibStatus = "true";
  String startingMessage = "Cube ready and calib: " + calibStatus;
  sendMessageOverBLE(startingMessage);
}

void loop() {
  currentT = millis();
  if (currentT - previousT_1 >= loop_time) {
    Tuning();  // derinimui
    angle_calc();
    
    if (balancing_point == 1) {
      angleX -= offsets.X1;
      angleY -= offsets.Y1;
      if (abs(angleX) > 8 || abs(angleY) > 8) vertical = false;
    } else if (balancing_point == 2) {
      angleX -= offsets.X2;
      angleY -= offsets.Y2;
      if (abs(angleY) > 6) vertical = false; //ESP32 > 5, nano 6
    } else if (balancing_point == 3) {
      angleX -= offsets.X3;
      angleY -= offsets.Y3;
      if (abs(angleY) > 6) vertical = false; //ESP32 > 5, nano 6
    } else if (balancing_point == 4) {
      angleX -= offsets.X4;
      angleY -= offsets.Y4;
      if (abs(angleX) > 6) vertical = false; //ESP32 > 5, nano 6
    }

    if (abs(angleX) < 8 || abs(angleY) < 8) {  // fast restore angle
      Gyro_amount = 0.996; 
    } else 
      Gyro_amount = 0.1;
 
    if (vertical && calibrated && !calibrating) {    
      digitalWrite(BRAKE, HIGH);
      gyroZ = GyZ / 131.0; // Convert to deg/s
      gyroY = GyY / 131.0; // Convert to deg/s
      gyroX = GyX / 131.0; // Convert to deg/s (ESP32)

      gyroYfilt = alpha * gyroY + (1 - alpha) * gyroYfilt;
      gyroZfilt = alpha * gyroZ + (1 - alpha) * gyroZfilt;
      
      int pwm_X = constrain(pGain * angleX + iGain * gyroZfilt + sGain * motor_speed_pwmX, -255, 255); // LQR
      int pwm_Y = constrain(pGain * angleY + iGain * gyroYfilt + sGain * motor_speed_pwmY, -255, 255); // LQR
      motor_speed_pwmX += pwm_X; 
      motor_speed_pwmY += pwm_Y;
      
      if (balancing_point == 1) {
        XY_to_threeWay(-pwm_X, -pwm_Y);
      } else if (balancing_point == 2) {
        Motor1_control(pwm_Y);
        //Motor1_control(-pwm_Y); //ESP32 pwm_y
      } else if (balancing_point == 3) {
        Motor2_control(-pwm_Y);
       // Motor2_control(pwm_Y); //ESP32 -pwm_y
      } else if (balancing_point == 4) {
       Motor3_control(pwm_X); 
       // Motor3_control(-pwm_X); //ESP32 pwm_X
      }
    } else {
      //balancing_point = 0; //Not on ESP32 
      XY_to_threeWay(0, 0);
      digitalWrite(BRAKE, LOW);
      motor_speed_pwmX = 0;
      motor_speed_pwmY = 0;
    }
    previousT_1 = currentT;
  }
  
  if (currentT - previousT_2 >= 2000) {    
    battVoltage((double)analogRead(VBAT) / bat_divider); //Bat diveder ESP32 = 207
    if (!calibrated && !calibrating) {
      Serial.println("first you need to calibrate the balancing points...");
    }
    previousT_2 = currentT;
  }

  //printAngles();
}


