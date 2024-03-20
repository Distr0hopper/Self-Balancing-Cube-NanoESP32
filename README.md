# Self-Balancing-Cube with Arduino Nano ESP32 using BLE

This Repository contains the files needed for running the PID-Controller code on the Arduino Nano ESP32 and tuning it using Bluetooth Low Energy.
For build description refer to the parent repository of RemRC. Use the circuit diagram for the Arduino Nano.


> [!IMPORTANT]
> Instead of soldering the IMU-VCC and the Buzzer to the 5V pin (not existing in the Arduino Nano ESP32, instead it uses a VBUS pin only delivering voltage when plugged in via USB), use the 3.3V pin of the Arduino ESP32 instead!

## Flashing the ESP32
* Open the Arduino IDE or IOPlatform.
* Select the Arduino Nano ESP32 board from Arduino itself.
  * ![image](https://github.com/Distr0hopper/Self-Balancing-Cube-NanoESP32/assets/100717485/9e465b8f-b4ab-4b5a-8a4f-a6d3b9c3a8af)
* Flash the code from the folder 'arduino_nano_ESP32'.

## Calibration and Tuning 
* Wait for calibration process being finished (two-beeps).
* Get an Bluetooth Low Energy beacon scanning app (e.g. i use nRF Connect).
  * **Caution: You need to use an app where you can send strings via UTF8, not only sending hexadecimal numbers!!**
* Connect with the "CubliArduino" beacon!

* ![BLEStart(2)](https://github.com/Distr0hopper/Self-Balancing-Cube-NanoESP32/assets/100717485/d40180e8-2d2c-4fa3-9979-495b62f1ca01)
  * Red Circle: Changing received value to be printed as String (UTF 8 instead of hexadecimal)
  * Blue Circle: Receive message from cubli
  * Green Circle: Send message to cubli
  * Box: Value received

* For calibrating you can send 'c+' to start and 'c-' to end calibration process (not case sensitive since every string is to-lower-case).
* For tuning the PID-Controller you can send 'p+', 'p-', 'i+', 'i-', 's+' and 's-'. You also can set the specific value by writing the part which you want to tune and a number (e.g. 'p140').
* You can also receice messages via the BLE app, for example the cube sends a message if the values are OK after calibrating and it sends the PID-Values after changing the value.
  

