# Self-Balancing-Cube with Arduino Nano ESP32 using BLE

This Repository contains the files needed for running the PID-Controller code on the Arduino Nano ESP32 and tuning it using **B**luetooth **L**ow **E**nergy (BLE).
For build description refer to the parent repository of RemRC. Use the circuit diagram for the Arduino Nano.


> [!IMPORTANT]
> Instead of soldering the IMU-VCC and the Buzzer to the 5V pin (not existing in the Arduino Nano ESP32, instead it uses a VBUS pin only delivering voltage when plugged in via USB), use the 3.3V pin of the Arduino ESP32 instead!

## Flashing the ESP32
* Open the Arduino IDE or IOPlatform.
* Select the Arduino Nano ESP32 board from Arduino itself.
  * ![image](https://github.com/Distr0hopper/Self-Balancing-Cube-NanoESP32/assets/100717485/9e465b8f-b4ab-4b5a-8a4f-a6d3b9c3a8af)
* Flash the code from the folder 'arduino_nano_ESP32'.

## Connect to Arduino 
### Method 1: BLE Python script (Linux only)
The repository contains a python script that can connect to the Arduino using its MAC address. 

1. Open the folder 'BLEScanner' in a Terminal 
2. Run `sudo python BLEScanner.py` or if using python3 `sudo python3 BLEScanner.py`
3. The Script automatically connects to the Arduino. Wait until the message 'Received: Cube ready and calib: `true/false`' appears!

### Method 2: Using BLE Scanning App 
A good app for connecting via mobile device is 'nRF Connect' as you can send String values and not only hexadecimal numbers.

1. Connect with the "CubliArduino" beacon displayed in the app


## Calibration and Tuning 
**Wait for calibration process being finished (two-beeps)**. When calibration finished the message 'Received: Cube ready and calib: `true/false`' is send via BLE.

* For calibrating send 'c+' to start and 'c-' to end calibration process (not case sensitive since every string is to-lower-case).
* For tuning the PID-Controller send 'p+', 'p-', 'i+', 'i-', 's+' and 's-'. You also can set the specific value by writing the part which you want to tune and a number (e.g. 'p140').

  After sending a message the Arduino sends back a message (only if it is a known command!). The python script prints the received value on the console. **If the received value is the same as the sent value, the command sent is unkown (or no message is sent back by the arduino)!**

  
  For receiving and sending values inside the 'nRF Connect' app, see the picture below:


* ![BLEStart(2)](https://github.com/Distr0hopper/Self-Balancing-Cube-NanoESP32/assets/100717485/d40180e8-2d2c-4fa3-9979-495b62f1ca01)
  * Red Circle: Changing received value to be printed as String (UTF-8 instead of hexadecimal)
  * Blue Circle: Receive message from cubli
  * Green Circle: Send message to cubli
  * Box: Value received (here in Hexadecimal since the value have to be changed with the red circle in UTF-8)

  

