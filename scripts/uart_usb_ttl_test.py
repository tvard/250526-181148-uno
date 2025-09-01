import serial

PORT = 'COM3'  # Change as needed
BAUD = 9600

with serial.Serial(PORT, BAUD, timeout=2) as ser:
    print("Waiting for data. Type on the device or send something...")
    while True:
        data = ser.readline()
        if data:
            print(f"Received: {data!r}")