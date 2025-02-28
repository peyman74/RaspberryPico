**Introduction:**

This project focuses on designing and implementing a digital Proportional-Integral-Derivative (PID) controller to achieve precise position control for a two-axis robotic arm. The system utilises a Raspberry Pi Pico microcontroller to drive two DC motors equipped with quadrature encoders, enabling closed-loop feedback for accurate motion adjustment. The goal is to ensure the robotic arm reaches and maintains desired positions dynamically by combining real-time sensor data (quadrant encoder) with control algorithms. The inputs or set points of the system either attain in the program by keyboard trough serial ports connected to microcontroller or by potentiometers in manual interpolation.
Applications

•	Educational demonstrations of closed-loop control systems.

•	Precision automation tasks (e.g., pick-and-place, tracking movement or a heat source by installing camera or temperature sensor respectively at tip of the arm).

•	Prototyping for industrial robotics with hybrid input capabilities (Manual/ Auto mode).
Advantages of Dual Input Modes

•	Flexibility: Switch between precise programmatic control and intuitive manual adjustment according to an operator needs.

•	Redundancy: Potentiometers serve as a backup input method if serial communication fails.

![image](https://github.com/user-attachments/assets/e73874cc-0a92-466a-86c7-dd1de6ff2610)
![image](https://github.com/user-attachments/assets/d3c09fb0-2621-4753-a5ef-4c763b40d659)

Picture by Profesor Professor Samuel E. de Lucena

Note: The Joystick in this figure could not be provided hence it was substituted with two potentiometers. 

**Known Limitations & Improvements (hardware and software):**

**Emergency Stop:** Add a normally close emergency switch on the path of main power supply and define soft key as a halt command.

**Emergency axis switch:** each axis needs two switch at each end side to prevent from any probable collisions.

**Zero switch:** add one switch as a zero switch at one side of rotation for each axis and call (perform) zero interpolation method every time the system is powering off/on or resetting. This lead to define zero reference for each axis.

**Current limiting:** Implement current limiting to protect your motors and driver circuitry

**Better Filtering:** Improve noise reduction for smoother especially in manual mode using potentiometers. Adding capacitors parallel to potentiometers and Motors recommended. 

**Multicore:** Using the second core for timely setting/ resetting an LED or a lamp and/or a buzzer during movement of an axis.

**PID function development** – including discard or subtract overflowing


