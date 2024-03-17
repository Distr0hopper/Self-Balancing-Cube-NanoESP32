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

def listen_for_messages(characteristic):
    global last_message, message_sent
    while True:
        if characteristic.supportsRead():
            try:
                response = characteristic.read()
                decoded_response = response.decode('utf-8')
                # Check if this is a new message or a message sent flag is true
                if decoded_response and (decoded_response != last_message or message_sent):
                    print("")
                    print("Received:", decoded_response)
                    last_message = decoded_response
                    message_sent = False  # Reset the flag after printing
                    time.sleep(0.1) # Wait until accepting new input
                    print("Input (type 'exit' to disconnect): ")
                time.sleep(0.5) # Wait for a while before checking for new messages
            except BTLEDisconnectError:
                print("Disconnected from the device")
                break  # Exit the thread if the device is disconnected

try:
    print("Attempting to connect to the device...")
    arduino_peripheral = Peripheral(arduino_mac_address)
    print("Connected to the Arduino device.")

    # Get the specific service by UUID
    arduino_service = arduino_peripheral.getServiceByUUID(SERVICE_UUID)

    # Get the specific characteristic by UUID
    arduino_characteristic = arduino_service.getCharacteristics(CHARACTERISTIC_UUID)[0]

    # Start a background thread to listen for messages
    listener_thread = threading.Thread(target=listen_for_messages, args=(arduino_characteristic,))
    listener_thread.daemon = True
    listener_thread.start()

    print("Connection established. Type 'exit' to disconnect.")
    while True:
        data = input()
        if data.lower() == 'exit':
            print("Disconnecting...")
            break

        arduino_characteristic.write(bytes(data, 'utf-8'))
        print(f"Sent: {data}")
        message_sent = True  # Set the flag indicating a message has been sent

except Exception as e:
    print(f"Failed to connect or communicate with the device: {e}")
finally:
    arduino_peripheral.disconnect()
    print("Disconnected.")

