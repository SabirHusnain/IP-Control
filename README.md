# Inverted Pendulum Control Project

The Inverted Pendulum Control Project focuses on developing control strategies for stabilizing an inverted pendulum system using Robust PID and LQR controllers. The project includes the mathematical modeling of the inverted pendulum, parameter estimation of the motor, simulations of the control strategies, and the implementation of the controllers on an embedded system. The hardware used for this project includes encoder motors for measuring the linear distance x and an MPU9250 sensor for measuring the angle of the pendulum theta.

## Directory Structure

- Documentation
- Embedded
  - Gyro_Sensor
    - ...
  - Inverted_Pendulum
    - ...
  - LQR_Control
    - ...
  - LQR_Control_With_Class
    - ...
  - Motor Speed Test
    - ...
  - motor_test
    - ...
  - mpu6050_without_acceleration
    - ...
- Modeling and Simulation
  - Inverted_Pendulum_Model.m
- Parameter Estimation
  - README.md
  - Motor_Parameter_Estimation.m
- Robust PID Control Simulation
  - Robust_PID_Control_Simulation.m
- LQR Control Simulation
  - LQR_Control_Simulation.m
- PID Hardware Control
  - README.md
  - Control Code
- LQR Hardware Control
  - README.md
  - Control Code

## Modelling and Simulation of Inverted Pendulum

The first phase of the project involves deriving the mathematical model of the inverted pendulum system. This includes formulating the equations of motion and characterizing the dynamics of the system. Simulations are then performed to validate the model and gain insights into the behavior of the inverted pendulum.

### How to Use:

1. Navigate to the "Modeling and Simulation" directory.
2. Open the MATLAB file named "Inverted_Pendulum_Model.m".
3. Follow the instructions provided within the file to run the simulations and analyze the results.

## Parameter Estimation of Motors

To accurately control the inverted pendulum system, it is crucial to estimate the parameters of the motor. In this project, parameter estimation techniques are employed to determine the relevant motor parameters using data collected from the motor using Python and the hardware setup.

### How to Use:

1. Go to the "Parameter Estimation" directory.
2. Follow the instructions provided in the README file within the directory to collect data using the Python script and the hardware motor setup.
3. Run the MATLAB file "Motor_Parameter_Estimation.m" and input the collected data.
4. The file will perform parameter estimation and display the estimated motor parameters.

## Robust PID Control Simulation

Robust PID control is implemented to stabilize the inverted pendulum system in the simulation environment. This control strategy accounts for uncertainties and disturbances in the system and aims to maintain the pendulum in an upright position.

### How to Use:

1. Open the "Robust PID Control Simulation" directory.
2. Run the MATLAB file named "Robust_PID_Control_Simulation.m".
3. Follow the instructions provided within the file to adjust the control parameters and observe the simulation results.

## LQR Control Simulation

Linear Quadratic Regulator (LQR) control is another approach used to stabilize the inverted pendulum system. The LQR controller optimizes a cost function to compute the control inputs that minimize the deviation from the desired pendulum position.

### How to Use:

1. Navigate to the "LQR Control Simulation" directory.
2. Open the MATLAB file named "LQR_Control_Simulation.m".
3. Adjust the LQR control parameters as needed and run the file.
4. Follow the instructions provided within the file to analyze the simulation results.

## PID Hardware Control

To validate the control strategies in a real-world setup, the Robust PID controller is implemented on the embedded system using the encoder motors. This allows for the physical control of the inverted pendulum and real-time feedback.

### How to Use:

1. Go to the "PID Hardware Control" directory.
2. Follow the instructions provided in the README file within the directory to set up the hardware and upload the control code to the embedded system.
3. Once the hardware is set up, run the embedded system code to control the inverted pendulum using the Robust PID controller.

## LQR Hardware Control

Similarly, the LQR controller is implemented on the embedded system to control the inverted pendulum in real-time. This provides an alternative control strategy for stabilizing the pendulum using the hardware setup.

### How to Use:

1. Open the "LQR Hardware Control" directory.
2. Refer to the README file within the directory for instructions on hardware setup and code uploading.
3. Once the hardware is ready, run the embedded system code to observe the control performance using the LQR controller.

## Applications

The Inverted Pendulum Control Project has various applications, including:

- Robotics and Automation: The control strategies developed in this project can be applied to stabilize other systems involving balancing or control of unstable dynamics.
- Education and Research: The project provides a valuable resource for students and researchers interested in control systems, robotics, and dynamic modeling.
- Control System Development: The project serves as a foundation for developing more advanced control algorithms and optimizing the performance of inverted pendulum systems.

For detailed project documentation, implementation instructions, and further customization options, please refer to the repository provided.
