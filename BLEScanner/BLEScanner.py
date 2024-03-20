from bluepy.btle import Peripheral, BTLEDisconnectError
import time
import threading

# UUIDs for the Service and Characteristic
SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"

# The MAC address of the Arduino BLE device
arduino_mac_address = "34:85:18:7B:F6:41"

# Shared variables between threads
last_message = None
message_sent = False
ble_lock = threading.Lock()  # Lock for synchronizing BLE access

def listen_for_messages(characteristic):
    global last_message, message_sent
    while True:
        if characteristic.supportsRead():
            with ble_lock:  # Use the lock when accessing the characteristic
                try:
                    response = characteristic.read()
                    decoded_response = response.decode('utf-8')
                    if decoded_response and (decoded_response != last_message or message_sent):
                        print("\nReceived:\n------------\n", decoded_response, "\n------------\nInput (type 'exit' to disconnect): ")
                        last_message = decoded_response
                        message_sent = False  # Reset the flag after printing
                except BTLEDisconnectError:
                    print("Disconnected from the device")
                    break  # Exit the thread if the device is disconnected
                except Exception as e:
                    print(f"Error during read operation: {e}")
        time.sleep(1)  # Wait for a while before checking for new messages

try:
    arduino_peripheral = Peripheral(arduino_mac_address)
    print("Connected to the Arduino device.")

    arduino_service = arduino_peripheral.getServiceByUUID(SERVICE_UUID)
    arduino_characteristic = arduino_service.getCharacteristics(CHARACTERISTIC_UUID)[0]

    listener_thread = threading.Thread(target=listen_for_messages, args=(arduino_characteristic,))
    listener_thread.daemon = True
    listener_thread.start()

    print("Connection established. Type 'exit' to disconnect.")
    while True:
        data = input()
        if data.lower() == 'exit':
            break

        with ble_lock:  # Use the lock when accessing the characteristic
            arduino_characteristic.write(bytes(data, 'utf-8'))
        print(f"\nSent: {data}")
        message_sent = True

except Exception as e:
    print(f"Failed to connect or communicate with the device: {e}")
finally:
    arduino_peripheral.disconnect()
    print("Disconnected.")
