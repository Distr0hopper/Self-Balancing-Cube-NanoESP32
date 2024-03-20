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


/**
 * @brief Callbacks for BLE characteristic events.
 * 
 * This class provides callback functions for handling write events on a BLE characteristic.
 * When a write event occurs, the `onWrite` function is called, which retrieves the value
 * written to the characteristic and performs necessary operations.
 */
class MyCallbacks: public BLECharacteristicCallbacks {
    /**
     * This function is called when a write operation is performed on a BLE characteristic.
     * It retrieves the value written to the characteristic and stores it in the btCommand variable.
     * The btCommand variable is then used to process the received data.
     * 
     * @param pCharacteristic The BLE characteristic on which the write operation was performed.
     */
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

/**
 * @brief Callbacks for BLE server events.
 */
class MyServerCallbacks: public BLEServerCallbacks {
    /**
     * @brief Called when a device is connected to the server.
     * 
     * This function prints a message to the serial monitor indicating that a device has connected.
     * 
     * @param pServer Pointer to the BLE server object.
     */
    void onConnect(BLEServer* pServer) {
      Serial.println("Device connected");
    }

    /**
     * @brief Called when a device is disconnected from the server.
     * 
     * This function prints a message to the serial monitor indicating that a device has disconnected.
     * It also restarts advertising after disconnection so that a new connection can be established.
     * 
     * @param pServer Pointer to the BLE server object.
     */
    void onDisconnect(BLEServer* pServer) {
      Serial.println("Device disconnected");
      BLEDevice::startAdvertising(); 
    }
};

/**
 * Initializes and opens the Bluetooth port for communication.
 * This function initializes the Bluetooth device, creates a server, and sets up the necessary characteristics and callbacks.
 * It also starts the Bluetooth service and advertising.
 */
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

/**
 * @brief Initializes the setup for the self-balancing cube.
 * 
 * This function is called once when the Arduino board is powered on or reset.
 * It sets up the serial communication, Bluetooth port, EEPROM, pin modes, motor controls,
 * and checks if the cube is calibrated. It also sends a starting message over BLE.
 */
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
  // Calibrate the IMU and set the offsets
  angle_setup();

  String calibStatus = "false";
  if(calibrated == 1) calibStatus = "true";
  String startingMessage = "Cube ready. Calibrated: " + calibStatus;
  sendMessageOverBLE(startingMessage);
}

/**
 * The main loop function of the program.
 * This function is called repeatedly in an infinite loop.
 * It performs the balancing and control operations for the self-balancing cube.
 */
void loop() {
  currentT = millis();
  if (currentT - previousT_1 >= loop_time) {
    Tuning();  
    angle_calc(); // Calc angle and determine the cubes exact angle relative to its desired balancing point
    
    if (balancing_point == 1) {
      angleX -= offsets.X1; //Checking how far the cube is from the desired balancing point (this is the error signal for the PID controller)
      angleY -= offsets.Y1;
      if (abs(angleX) > 8 || abs(angleY) > 8) vertical = false; // Check if cubes current orientation deviates significantly from the desired balancing point
    } else if (balancing_point == 2) {
      angleX -= offsets.X2;
      angleY -= offsets.Y2;
      if (abs(angleY) > 6) vertical = false; 
    } else if (balancing_point == 3) {
      angleX -= offsets.X3;
      angleY -= offsets.Y3;
      if (abs(angleY) > 6) vertical = false; 
    } else if (balancing_point == 4) {
      angleX -= offsets.X4;
      angleY -= offsets.Y4;
      if (abs(angleX) > 6) vertical = false; 
    }

    // Dynamic adjustment of the gyro amount depending if the cube is close to the balancing point
    if (abs(angleX) < 8 || abs(angleY) < 8) {  
      Gyro_amount = 0.996; 
    } else // Far from balancing point, rely more on accelerometer to quickly correct significant imbalances
      Gyro_amount = 0.1;
 
    if (vertical && calibrated && !calibrating) {    
      digitalWrite(BRAKE, HIGH);

      gyroX = GyX / 131.0; // Convert to deg/s (500 rad/s sensitivity)
      gyroY = GyY / 131.0; // Convert to deg/s
      gyroZ = GyZ / 131.0; // Convert to deg/s 

      // Moving avarage integration in discrete time (smoothing with alpha value) -> Delayed integration for less overshoot 
      gyroYfilt = alpha * gyroY + (1 - alpha) * gyroYfilt;
      gyroZfilt = alpha * gyroZ + (1 - alpha) * gyroZfilt;
      
      // PID-control calculation for the motors. 
      // The I term is not accumulating the error over time, but rather using the filtered gyro data (rate of change in angle) to correct the error.
      // This compensates for persistent angular velocity offsets that could lead to drift.
      // We can use the gyro value because we can directly read out the error in the rate of change of the angle.
      // The S term not looks at the rate of change of the error, but rather multiplies the integrated motor speed with an gain to compensate the error.
      // Motor speed is closely related to system response to control inputs and its subsequent motion and orientation changes.
      int pwm_X = constrain(pGain * angleX + iGain * gyroZfilt + sGain * motor_speed_pwmX, -255, 255); // LQR
      int pwm_Y = constrain(pGain * angleY + iGain * gyroYfilt + sGain * motor_speed_pwmY, -255, 255); // LQR

      // Low pass filter for the motor speed to avoid sudden changes in motor speed (d-term delayed) - Integral in the feedback path
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
      balancing_point = 0; 
      XY_to_threeWay(0, 0);
      digitalWrite(BRAKE, LOW);
      motor_speed_pwmX = 0;
      motor_speed_pwmY = 0;
    }
    previousT_1 = currentT;
  }
  
  // Check battery voltage every 2 seconds
  if (currentT - previousT_2 >= 2000) {    
    battVoltage((double)analogRead(VBAT) / bat_divider); 
    if (!calibrated && !calibrating) {
      Serial.println("first you need to calibrate the balancing points...");
    }
    previousT_2 = currentT;
  }
}


