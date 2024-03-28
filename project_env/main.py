import tkinter as tk
from tkinter import font as tkFont
from skyfield.api import Loader, Topos, load
import serial
import time

#Declare Variables
ser = serial.Serial('COM4', 9600, timeout=0.1)
time.sleep(2)
loader = Loader('~/.skyfield-data')
planets = loader('de421.bsp')
ts = loader.timescale()
last_print_time = time.time()
last_print_time = time.time()
root = tk.Tk()
root.title("Celestial Object Tracker")
observer_location = Topos(latitude_degrees=51.5074, longitude_degrees=-0.1278)
current_lat = None
current_lng = None
accel_x, accel_y, accel_z = None, None, None
gyro_x, gyro_y, gyro_z = None, None, None
current_lat, current_lng, current_altitude, current_satellites = None, None, None, None

celestial_objects = {
    "Mercury": planets['mercury'],
    "Venus": planets['venus'],
    "Mars": planets['mars'],
    "Jupiter": planets['jupiter barycenter'],
    "Saturn": planets['saturn barycenter'],
    "Uranus": planets['uranus barycenter'],
    "Neptune": planets['neptune barycenter'],
    "Moon": planets['moon'],
    "Sun": planets['sun']
}

def safe_read_line():
    try:
        return ser.readline().decode('utf-8').strip()
    except Exception as e:
        print(f"Error reading line: {e}")
        return None
    
def allow_mpu_print():
    global print_mpu_data
    print_mpu_data = True  # Set the flag to True to allow printing
    root.after(1000, allow_mpu_print)

def update_data():
    global current_lat, current_lng, current_altitude, current_satellites
    global accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z  # Assuming these are defined globally
    global print_mpu_data
    global last_print_time

    line = safe_read_line()
    if line:
        try:
            if line.startswith("MPU:"):
                # Remove "MPU: " prefix and then split the line
                parts = line.replace("MPU: ", "").split(',')
                if len(parts) == 6:  # Ensure there are exactly 6 parts for the MPU data
                    accel_x, accel_y, accel_z = float(parts[0]), float(parts[1]), float(parts[2])
                    gyro_x, gyro_y, gyro_z = float(parts[3]), float(parts[4]), float(parts[5])

                    # Update the GUI elements with new data
                    mpu_vars['accel_x_var'].set(f"Accel X: {accel_x}")
                    mpu_vars['accel_y_var'].set(f"Accel Y: {accel_y}")
                    mpu_vars['accel_z_var'].set(f"Accel Z: {accel_z}")
                    mpu_vars['gyro_x_var'].set(f"Gyro X: {gyro_x}")
                    mpu_vars['gyro_y_var'].set(f"Gyro Y: {gyro_y}")
                    mpu_vars['gyro_z_var'].set(f"Gyro Z: {gyro_z}")

                else:
                    print("Insufficient MPU data elements.")
            elif "Lat:" in line:
                current_lat = float(line.split("Lat: ")[1])
                gps_vars['lat_var'].set(f"Latitude: {current_lat}")
            elif "Lng:" in line:
                current_lng = float(line.split("Lng: ")[1])
                gps_vars['lng_var'].set(f"Longitude: {current_lng}")
            elif "Altitude:" in line:
                alt_data = line.split("Altitude: ")[1]
                # Remove the " Meters" text to isolate the numeric value
                alt_data = alt_data.replace(" Meters", "")
                try:
                    current_altitude = float(alt_data)
                    gps_vars['alt_var'].set(f"Altitude: {current_altitude} Meters")
                except ValueError:
                    # Handle the case where alt_data cannot be converted to float
                    print(f"Could not convert altitude to float: '{alt_data}'")
                    gps_vars['alt_var'].set("Altitude data is not valid.")
            elif "Satellites:" in line:
                sat_data = line.split("Satellites: ")[1]
                if sat_data != " data is not valid.":
                    current_satellites = int(sat_data)
                    gps_vars['sat_var'].set(f"Satellites: {current_satellites}")
                else:
                    gps_vars['sat_var'].set(sat_data)
            else:
                print(f"Unrecognized line format: {line}")
        except ValueError as e:
            print(f"Value parsing error: {e}")
        except Exception as e:
            print(f"Unexpected error: {e}")

    # Check if all required data (GPS and MPU) have been updated
    if current_lat is not None and current_lng is not None and current_altitude is not None and current_satellites is not None:
        pass
    if all(v is not None for v in [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]):
        pass
    else:
        print("Waiting for complete data...")

    root.after(100, update_data)

def track_celestial_object(body, lat, lng):
    t = ts.now()
    earth = planets['earth']
    observer_location = Topos(latitude_degrees=lat, longitude_degrees=lng)
    observer = earth + observer_location
    astrometric = observer.at(t).observe(body)
    alt, az, dist = astrometric.apparent().altaz()
    alt_deg = int(alt.degrees)
    az_deg = int(az.degrees)
    # Ensure the command is formatted as "MOT,azimuth,elevation\n"
    send_str = f"MOT,{az_deg},{alt_deg}\n"
    ser.write(send_str.encode('utf-8'))

def button_command(body, name):  # Add the 'name' parameter
    if current_lat is not None and current_lng is not None:
        track_celestial_object(body, current_lat, current_lng)
        print(f"Moving towards {name}...")  # Use the 'name' parameter here
    else:
        print("GPS data not available.")

def return_to_home():
    # Sends "RTH\n" to signal the Arduino to return motors to the home position
    send_str = "RTH\n"
    ser.write(send_str.encode('utf-8'))

def main():
    global gps_vars, mpu_vars
    gps_vars = {var: tk.StringVar() for var in ['lat_var', 'lng_var', 'alt_var', 'sat_var']}
    mpu_vars = {var: tk.StringVar() for var in ['accel_x_var', 'accel_y_var', 'accel_z_var', 'gyro_x_var', 'gyro_y_var', 'gyro_z_var']}
    data_frame = tk.Frame(root, width=200)
    data_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=5)
    buttons_frame = tk.Frame(root)
    buttons_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

    return_home_button = tk.Button(root, text="Return to Home", command=return_to_home)
    return_home_button.pack(padx=5, pady=5, fill=tk.X)

    title_font = tkFont.Font(size=10, weight="bold")
    data_font = tkFont.Font(size=9)

    # Create labeled frames for GPS and MPU data with borders
    gps_frame = tk.LabelFrame(data_frame, text="GPS Data", font=title_font, padx=5, pady=5)
    gps_frame.pack(fill="both", expand="yes")
    mpu_frame = tk.LabelFrame(data_frame, text="MPU Data", font=title_font, padx=5, pady=5)
    mpu_frame.pack(fill="both", expand="yes")

    # Populate data_frame with GPS and MPU data labels
    tk.Label(data_frame, text="").pack()
    for var in ['lat_var', 'lng_var', 'alt_var', 'sat_var']:
        tk.Label(gps_frame, textvariable=gps_vars[var], anchor="w", font=data_font).pack(fill="x")
    tk.Label(data_frame, text="").pack()
    for var in ['accel_x_var', 'accel_y_var', 'accel_z_var', 'gyro_x_var', 'gyro_y_var', 'gyro_z_var']:
        tk.Label(mpu_frame, textvariable=mpu_vars[var], anchor="w", font=data_font).pack(fill="x")
    for name, body in celestial_objects.items():
        button = tk.Button(root, text=name, command=lambda body=body, name=name: button_command(body, name))
        button.pack(padx=5, pady=5, fill=tk.X)

    update_data()
    root.mainloop()

if __name__ == "__main__":
    main()
