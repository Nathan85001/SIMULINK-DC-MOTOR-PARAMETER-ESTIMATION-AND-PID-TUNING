# DC Motor Parameter Estimation and PID Tuning

This repository contains the project files for estimating parameters of a DC motor and tuning a PID controller using an Arduino Mega and a geared DC motor with a rotary magnetic encoder. The project involves data logging using a serial monitor application, parameter estimation in Simulink, and simple PID tuning focusing on proportional (P) and integral (I) control.

## Project Overview

### Hardware Components
- **Arduino Mega**: Used for controlling the motor and collecting data.
- **Geared DC Motor**: The motor for which parameters are estimated.
- **Rotary Magnetic Encoder**: Used for measuring the motor speed (RPM).

### Software Components
- **Arduino IDE**: For writing and uploading the code to the Arduino Mega.
- **Serial Monitor Application**: For logging data from the Arduino and saving it as a CSV file.
- **Simulink**: For performing parameter estimation and PID tuning.
- **Proteus**: For virtual simulation of the hardware setup.
- **Virtual Serial Link Application**: For simulating the serial communication in the virtual environment.

## Getting Started

### Prerequisites

1. **Arduino IDE**: Install the Arduino IDE from [Arduino's official website](https://www.arduino.cc/en/software).
2. **MATLAB and Simulink**: Install MATLAB and Simulink from [MathWorks](https://www.mathworks.com/).
3. **Serial Monitor Application**: You can use the serial monitor integrated into the Arduino IDE or any other serial monitor application of your choice.
4. **Proteus**: Install Proteus for virtual simulation.
5. **Virtual Serial Link Application**: Install a virtual serial link application for simulating serial communication.

### Hardware Setup

1. Connect the geared DC motor to a suitable motor driver connected to the Arduino Mega.
2. Attach the rotary magnetic encoder to the motor shaft.
3. Connect the encoder output to the appropriate pins on the Arduino Mega.
4. Ensure the power supply to the motor is appropriate for its specifications.

### Arduino Code

Upload the `dc_motor_control.ino` sketch to the Arduino Mega. This sketch controls the motor speed via PWM and reads the encoder data to measure the motor speed.

### Data Logging

1. Open the serial monitor application and start logging data.
2. The logged data will include time (t), PWM signal (pwm), and motor speed (rpm).
3. Save the logged data as a CSV file for further analysis.

### Parameter Estimation in Simulink

1. Import the logged data (CSV file) into Simulink.
2. Use the data to perform parameter estimation of the DC motor model.
3. Identify key parameters such as motor resistance, inductance, back EMF constant, and torque constant.

### PID Tuning

1. Use the estimated motor parameters to design a simple PID controller.
2. Focus on tuning the proportional (P) and integral (I) gains for the controller.
3. Validate the performance of the PID controller in Simulink.

### Virtual Simulation

A virtual version of this project is available using Proteus and a virtual serial link application. This setup allows you to simulate the hardware components and the serial communication in a virtual environment, providing a convenient way to test and validate the system without physical hardware.

## Repository Structure

├── Arduino

│ └── dc_motor_control.ino # Arduino sketch for motor control and data logging

├── Data

│ └── motor_data.csv # Sample logged data

├── Simulink

│ └── parameter_estimation.slx # Simulink model for parameter estimation

│ └── pid_tuning.slx # Simulink model for PID tuning

├── Virtual

│ └── proteus_simulation.pdsprj # Proteus project for virtual simulation

│ └── virtual_serial_link.pdf # Documentation for setting up the virtual serial link

├── README.md # Project documentation

## Usage

1. Upload the Arduino code to the Arduino Mega.
2. Log data using the serial monitor application and save it as a CSV file.
3. Import the CSV file into Simulink and perform parameter estimation.
4. Use the estimated parameters to design and tune the PID controller.
5. Validate the controller performance in Simulink.
6. Optionally, use the Proteus simulation and virtual serial link application for virtual testing.

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request for any improvements or additional features.

## License

This project is licensed under the MIT License - see the [LICENSE](./LICENSE) file for details.
