# Celestial Object Tracker

This repository contains the code for a Celestial Object Tracker application that uses GPS and MPU (Inertial Measurement Unit) data to track celestial objects in real-time.

## Description

The Celestial Object Tracker is a Python-based application designed to interface with hardware sensors, motors, and stepper drivers to provide real-time celestial tracking capabilities. It parses GPS coordinates and uses orientation data from an MPU, alongside motor control, to offer users a dynamic method for tracking celestial bodies across the sky.

## Features

- Real-time GPS data parsing.
- MPU data integration for accurate orientation.
- Control of motors and stepper drivers for precise positioning.
- Tkinter-based graphical user interface.
- Automated celestial object tracking based on live sensor data.

## Getting Started

### Dependencies

- Python 3.6 or higher.
- Tkinter (usually comes with Python).
- PySerial, for serial communication.
- Skyfield, for astronomical computations.
- Compatible with NEMA stepper motors and TMC2208 stepper motor drivers.

### Hardware

- GPS Module for real-time location data.
- MPU6050 sensor for orientation data.
- Stepper motors (NEMA 17) for mechanical movement.
- Stepper motor drivers (TMC2208v1.2) for motor control.

### Installing

Clone the repository:

```bash
git clone https://github.com/AngusBlomley/FMP.git
```

Navigate to the project directory:

```bash
cd project_env
```
Activate Virtual Enviroment:

```bash
activate
```

Install the required Python packages:

```bash
pip install -r requirements.txt
```

### Executing Program

1. Connect your GPS, MPU hardware, motors, and stepper drivers to your computer or microcontroller.
2. Ensure the hardware is correctly set up and configured as per the project documentation.
3. Run the main program:

```bash
python main.py
```

4. Use the GUI to interact with the application and start tracking celestial objects.

## Help

If you encounter any serial port issues, ensure that the hardware is correctly connected and the COM port in `main.py` matches your system's configuration. For motor or stepper driver issues, verify the wiring and settings as per the hardware documentation.

## Authors

Angus Blomley  
https://github.com/AngusBlomley

## Version History

- 0.1
  - Initial Release

## License

This project is licensed under the MIT License - see the LICENSE.txt file for details.

## Acknowledgments

- Skyfield API for astronomical calculations.
- The Arduino and Raspberry Pi communities for inspiration and guidance on hardware interfacing.
