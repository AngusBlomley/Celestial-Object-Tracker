import serial
import time

# Setup the serial connection
ser = serial.Serial('COM4', 9600, timeout=1)
time.sleep(2)  # Wait for connection to establish

def parse_mpu_data(line):
    parts = line.split(',')[1:]  # Exclude the 'MPU' part
    return [float(p) for p in parts]

def parse_gps_data(line):
    label, value = line.split(': ')
    return float(value)

def move_and_read_test(pan_angle, tilt_angle):
    # Send the command to move the pan-and-tilt head to the desired angles
    ser.write(f'MOVE,{pan_angle},{tilt_angle}\n'.encode())

    # Wait for the movement to complete (this duration will depend on your setup)
    time.sleep(5)  # Assuming it takes less than 5 seconds to move

    # Request MPU data from Arduino
    ser.write('REQUEST_MPU\n'.encode())
    
    while True:
        line = ser.readline().decode().strip()
        if line:
            print(f"Received line: {line}")  # Debug: print the line received
            if line.startswith("MPU,"):
                ax, ay, az, gx, gy, gz = parse_mpu_data(line)
                print(f"MPU Data for angles ({pan_angle}, {tilt_angle}):")
                print(f"  Accel: ({ax}, {ay}, {az})")
                print(f"  Gyro: ({gx}, {gy}, {gz})")
                break  # Exit the loop after processing MPU data
            elif line.startswith("Lat:") or line.startswith("Lng:") or line.startswith("Altitude:") or line.startswith("Satellites:"):
                value = parse_gps_data(line)
                label = line.split(':')[0]
                print(f"{label} updated: {value}")
            else:
                print(f"Unexpected line format: {line}")

# Testing different angles
for pan_angle in range(0, 180, 45):  # Example range of pan angles
    for tilt_angle in range(0, 90, 45):  # Example range of tilt angles
        move_and_read_test(pan_angle, tilt_angle)
        time.sleep(2)  # Wait before moving to the next angle

# Close the serial connection
ser.close()
