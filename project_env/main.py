import tkinter as tk
from tkinter import font as tkFont
from tkinter import ttk
from skyfield.api import Loader, Topos, load
import serial
import time
import cv2
from PIL import Image, ImageTk
import os



#Declare Variables and initialize
ser = serial.Serial('COM4', 9600, timeout=0.1)
time.sleep(2)
loader = Loader('~/.skyfield-data')
planets = loader('de421.bsp')
ts = loader.timescale()
last_print_time = time.time()
last_print_time = time.time()
root = tk.Tk()
root.withdraw()
should_update = True  
root.title("Celestial Object Tracker")
observer_location = Topos(latitude_degrees=51.5074, longitude_degrees=-0.1278)
current_lat = None
current_lng = None
accel_x, accel_y, accel_z = None, None, None
gyro_x, gyro_y, gyro_z = None, None, None
current_lat, current_lng, current_altitude, current_satellites = None, None, None, None



#Placeholder data for when a GPS connection is unavailble for testing
use_live_gps_data = True
toggle_button = None
placeholder_lat = 51.5074
placeholder_lng = -0.1278
placeholder_altitude = 50  
placeholder_satellites = 5  



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
    root.after(100, allow_mpu_print)




def update_data():
    global should_update
    global current_lat, current_lng, current_altitude, current_satellites
    global accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
    global print_mpu_data
    global last_print_time

    if should_update:
        if use_live_gps_data:
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
        else:
            # Use placeholder data
            current_lat = placeholder_lat
            current_lng = placeholder_lng
            current_altitude = placeholder_altitude
            current_satellites = placeholder_satellites
            # Update the GUI elements with placeholder data
            gps_vars['lat_var'].set(f"Latitude: {placeholder_lat}")
            gps_vars['lng_var'].set(f"Longitude: {placeholder_lng}")
            gps_vars['alt_var'].set(f"Altitude: {placeholder_altitude} Meters")
            gps_vars['sat_var'].set(f"Satellites: {placeholder_satellites}")
    root.after(1000, update_data)




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




def button_command(body, name):
    if current_lat is not None and current_lng is not None:
        track_celestial_object(body, current_lat, current_lng)
        print(f"Moving towards {name}...")
        capture_and_highlight_object(name)
    else:
        print("GPS data not available.")




def toggle_gps_data():
    global use_live_gps_data
    use_live_gps_data = not use_live_gps_data
    if use_live_gps_data:
        toggle_button.config(text="Placeholder Data")
    else:
        toggle_button.config(text="Use Live GPS Data")



def return_to_home():
    # Sends "RTH\n" to signal the Arduino to return motors to the home position
    send_str = "RTH\n"
    ser.write(send_str.encode('utf-8'))




def on_closing():
    global should_update
    should_update = False  # Stop scheduling new updates
    root.destroy()



# NEEDS UPDATE
def capture_and_highlight_object(object_name):
    # Capture the image
    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()
    cap.release()

    if ret:
        # add your logic to find and highlight the object in the frame
        # For simplicity, draw a rectangle and annotate it
        cv2.rectangle(frame, (100, 100), (200, 200), (0, 255, 0), 2)
        cv2.putText(frame, object_name, (100, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Display the annotated image
        display_image(frame)
    



def display_image(cv2image):
    # Convert the image to RGB (for Tkinter compatibility)
    cv2image = cv2.cvtColor(cv2image, cv2.COLOR_BGR2RGB)
    img = Image.fromarray(cv2image)
    imgtk = ImageTk.PhotoImage(image=img)

    # Display the image in the GUI
    if not hasattr(display_image, 'frame_label'):
        display_image.frame_label = tk.Label(root, image=imgtk)
        display_image.frame_label.image = imgtk  # Keep a reference!
        display_image.frame_label.pack(side=tk.LEFT, padx=10, pady=10)
    else:
        display_image.frame_label.configure(image=imgtk)
        display_image.frame_label.image = imgtk




def center_window(root, width=1280, height=720):
    # Get the screen dimensions
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()

    # Calculate x and y coordinates for the Tk root window
    x = (screen_width - width) // 2
    y = (screen_height - height) // 2

    # Set the dimensions of the screen 
    # and where it is placed
    root.geometry(f'{width}x{height}+{x}+{y}')




def show_loading_screen(root, num_tasks=100):
    loading_screen = tk.Toplevel(root)
    loading_screen.overrideredirect(True)

    # Load the image
    image = tk.PhotoImage(file="images/loading_page.png")  # Load the original image

    # Calculate the desired size for the image
    desired_width = 640
    desired_height = 360

    # Resize the image
    resized_image = image.subsample(round(image.width() / desired_width), round(image.height() / desired_height))

    # Create a label to display the image
    image_label = tk.Label(loading_screen, image=resized_image)
    image_label.image = resized_image  # Keep a reference to the image to prevent garbage collection
    image_label.pack(padx=0, pady=0)

    # Setup the loading bar
    canvas = tk.Canvas(loading_screen, width=400, height=20)
    canvas.place(relx=0.5, rely=0.5, anchor=tk.CENTER, y=100) 
    loading_bar = canvas.create_rectangle(0, 0, 0, 20, fill="green", outline="white", width=0.1)

    # Function to update the loading bar
    def update_progress(step):
        bar_width = 400 * step // num_tasks  # Calculate width of the bar based on current step
        canvas.coords(loading_bar, 0, 0, bar_width, 20)  # Update bar size
        loading_screen.update_idletasks()  # Refresh the loading screen
        if step < num_tasks:
            # Schedule the next update
            root.after(17, lambda: update_progress(step + 1))
        else:
            # Close the loading screen and show the main window once loading is complete
            loading_screen.destroy()
            root.deiconify()

    # Start the progress updates
    update_progress(0)

    # Center the loading screen
    loading_screen.update_idletasks()  # Ensure that the window dimensions are calculated
    screen_width = loading_screen.winfo_screenwidth()
    screen_height = loading_screen.winfo_screenheight()
    x = (screen_width - loading_screen.winfo_width()) // 2
    y = (screen_height - loading_screen.winfo_height()) // 2
    loading_screen.geometry(f"+{x}+{y}")

    return loading_screen



def main():
    global toggle_button, gps_vars, mpu_vars
    global should_update

    should_update = True
    center_window(root, 1280, 740)

    gps_vars = {var: tk.StringVar() for var in ['lat_var', 'lng_var', 'alt_var', 'sat_var']}
    mpu_vars = {var: tk.StringVar() for var in ['accel_x_var', 'accel_y_var', 'accel_z_var', 'gyro_x_var', 'gyro_y_var', 'gyro_z_var']}

    style = ttk.Style()
    style.theme_use('clam')  # or 'alt', 'default', 'classic', 'vista'


    # Define colors
    bg_color = '#333333'  # Dark gray
    text_color = '#FFFFFF'  # White
    button_color = '#e4e4e4'  # Cadet Blue
    frame_bg_color = '#444444'  # A slightly lighter shade of dark gray
    root.configure(bg=bg_color)  # Set the background color for the root window


    # Define fonts
    title_font = tkFont.Font(size=32, family="Myriad Pro")
    subTitle_font = tkFont.Font(size=16, family="Myriad Pro")
    data_font = tkFont.Font(size=14, family="Myriad Pro")
    label_font = tkFont.Font(size=12, family="helvetica")

    # Title
    title_label = tk.Label(root, text="Celestial Object Tracker", font=title_font, fg=text_color, bg=bg_color, anchor="w")
    title_label.pack(pady=10, padx=10, anchor="nw")


    # Frame for Message
    # Frame for Message
    message_frame = tk.Frame(root, bg=frame_bg_color)
    message_frame.place(relx=1, rely=0, anchor='ne', x=-10, y=30)

    message_text = "Welcome to the Celestial Object Tracking Software! Please select a celestial object to track to\nfind out more about it!"
    message_label = tk.Label(message_frame, text=message_text, font=label_font, fg=text_color, bg=frame_bg_color, justify="left")
    message_label.pack(pady=10, padx=10, anchor="ne")




    # Left Frame for Data
    data_frame = tk.Frame(root, bg=frame_bg_color)
    data_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=30)

    data_label = tk.Label(data_frame, text="Data", font=subTitle_font, fg=text_color, bg=frame_bg_color)
    data_label.pack(pady=10)


    # GPS Frame
    gps_frame = tk.LabelFrame(data_frame, text="GPS Data", font=data_font, fg=text_color, bg=frame_bg_color)
    gps_frame.pack(fill="both", expand="yes", padx=10, pady=10)


    # MPU Frame
    mpu_frame = tk.LabelFrame(data_frame, text="MPU Data", font=data_font, fg=text_color, bg=frame_bg_color)
    mpu_frame.pack(fill="both", expand="yes", padx=10, pady=10)


    # Camera frame
    camera_frame = tk.Frame(root, bg=frame_bg_color)
    camera_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=10, pady=30)


    # Label for the camera frame
    camera_label = tk.Label(camera_frame, text="Camera Frame", font=label_font, fg=text_color, bg=frame_bg_color)
    camera_label.pack(pady=10)


    # Right Frame for Buttons
    buttons_frame = tk.Frame(root, bg=frame_bg_color)
    buttons_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=False, padx=10, pady=30)

    data_label = tk.Label(buttons_frame, text="Celestial Objects", font=subTitle_font, fg=text_color, bg=frame_bg_color)
    data_label.pack(pady=10, padx=10)


    # Button style
    button_style = ttk.Style()
    button_style.configure('TButton', padding=6, relief="flat", background=button_color)

    # Populate data_frame with GPS and MPU data labels
    tk.Label(data_frame, text="").pack()
    for var in ['lat_var', 'lng_var', 'alt_var', 'sat_var']:
        tk.Label(gps_frame, textvariable=gps_vars[var], anchor="w", font=label_font, bg=frame_bg_color, fg=text_color).pack(fill="x", padx="10")
    
    tk.Label(data_frame, text="").pack()
    for var in ['accel_x_var', 'accel_y_var', 'accel_z_var', 'gyro_x_var', 'gyro_y_var', 'gyro_z_var']:
        tk.Label(mpu_frame, textvariable=mpu_vars[var], anchor="w", font=label_font, bg=frame_bg_color, fg=text_color).pack(fill="x", padx="10")
        
    for name, body in celestial_objects.items():
        button = ttk.Button(buttons_frame, text=name, command=lambda body=body, name=name: button_command(body, name))
        button.pack(padx=5, pady=5, fill=tk.X)
    
    capture_button = ttk.Button(buttons_frame, text="Capture", command=lambda: capture_and_highlight_object("Object"))
    capture_button.pack(padx=5, pady=10, fill=tk.X)

    return_home_button = ttk.Button(buttons_frame, text="Return to Home", command=return_to_home)
    return_home_button.pack(padx=5, pady=10, fill=tk.X)

    toggle_button = ttk.Button(buttons_frame, text="Placeholder Data", command=toggle_gps_data)
    toggle_button.pack(padx=5, pady=10, fill=tk.X)

    show_loading_screen(root)  # Display the loading screen before the main application
    root.after(1000, update_data)
    root.mainloop()
    should_update = False  # Stop updates when the main loop exits

if __name__ == "__main__":
    root.protocol("WM_DELETE_WINDOW", on_closing)  # Ensure proper closure handling
    main()
