# KUKA simulator for medical robotics teaching

## About
This project contains the necessary files to run a MATLAB-based simulation of a KUKA LBR iiwa integrated with a surgical navigation camera.
This project was developed by the BIROMED Lab to teach Medical Robotics (Department of Biomedical Engineering, University of Basel). 
To do so, the simulation provided in this repository is paired with additional software to deploy onto the physical hardware.

The Simulink models and MATLAB files are arranged so that students can implement their own solutions at different levels of the control pipeline. 
The proposed task is to use the information from the surgical navigation camera to compensate the physiological movements of the patient while performing a minimally-invasive intervention through a trocar.

## Installation

To install and run the project, simply clone the repository and run the **macroRobot_simulator.slx** file. The existing version of the project was developed in MATLAB R2023b and the following packages are required:

- Simulink
- Simscape
- Simscape Multibody
- Control System Toolbox
- Signal Processing Toolbox

## Developing and using the code

To use the code for personal use or for education, modifications in the **macroRobot_simulator.slx** model need to be performed in order to integrate the camera information into the robot control.

If desired, parts of the underlying MATLAB source code can be removed or edited for the students to complete. Similarly, additional objects can be added to the project using STEP files and the corresponding Simscape tools.

## Contributors

- Rubén Martín-Rodríguez [ruben.martinrodriguez@unibas.ch](mailto:ruben.martinrodriguez@unibas.ch)
- Nicolas Gerig [nicolas.gerig@unibas.ch](mailto:nicolas.gerig@unibas.ch)
- Murali Karnam [murali.karnam@unibas.ch](mailto:murali.karnam@unibas.ch)

