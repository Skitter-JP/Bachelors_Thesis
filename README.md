# Bachelors Thesis

<img src="https://github.com/Skitter-JP/Bachelors_Thesis/blob/main/Images/electrical_engeneering.svg" align="right"
     alt="CVUT Logo" width=40% height=40%>

Topic: Sensorless Field Oriented Control of a Brushless DC Motor

This thesis was written at the Czech Technical University in the department of Circuit Theory.

The Goals of this thesis were

- Select a suitable combination of a microcontroller and power electronics.
- Design the controlling electronics and a printed circuit board.
- Design and implement the controlling algorithm for the estimation of the rotor angle and the field oriented control.

In this repositary the following has been attached

- **Code:** The Code was written in Microchip Studio using C. The USB library has not been included since its not my property.
- **Schematics:** The Schematics and PCBs were designed in Altium. All footprints, symbols and 3D models were manually inserted into my own component library.
- **Boards views:** The 3D views should be opened in Adobe. Github/Chrome does not support 3D PDF models
- **3D Models:** The Motor Mounts, Magnetic Sensor Mounts and Magnet Mounts can be found here.
- **Images:** The manufactured PCB and Mounts can be viewd here.
- **Document:** The thesis document which was submitted

## ESC PCB Board Views

<img src="/Images/PCB_Board_View_Top.JPG" width=55%>
<img src="/Images/PCB_Board_View_Bottom.JPG" width=55%>
<img src="/Images/PCB_3D_View_Side.JPG" width=55%>
<img src="/Images/PCB_3D_View_Top.JPG" width=55%>

## Completed ESC PCB

<img src="/Images/Assembled_PCB.JPG" width=55%>

## Magnetic Encoder Board Views

<img src="/Images/Magnetic_PCB_Board_View_Top.JPG" width=55%>
<img src="/Images/Magnetic_PCB_Board_View_Bottom.JPG" width=55%>
<img src="/Images/Magnetic_PCB_3D_Side.JPG" width=55%>
<img src="/Images/Magnetic_PCB_3D_Bottom.JPG" width=55%>


## Completed Magnetic Encoder PCB

<img src="/Images/Magnetic_Encoder_Assembled_PCB.JPG" width=55%>

## 3D Printed Mounts

<img src="/Images/Magnet_Motor.JPG" width=55%>

- **A:** A small mount for a diametrically magnetized magnet which was mounted to the rotor.
- **B:** A BLDC Motor

<img src="/Images/Magnetic_Sensor_Mount.JPG" width=55%>

- **A:** The magnetic encoder which was mounted just below the rotor.
- **B:** A 3D printed Motor/Magnetic encoder Mount

## Constant Velocity Mode

The following figure was captured from the ESC while operating in a constant velocity. In the figure the
velocity was set to 5 [rad/sec]. The measured velocity was fluctuating between 20 and -10 rad/sec. The
mean value was centered near 5 rad/sec.

A professor in my defense mentioned that these large fluctuations in the measured velocity were most likely caused from the BLDC motor.
A BLDC motor is designed for trapezoidal wave forms, if a BLAC motor would of been used these large fluctuations in the measured velocity would of not been apparent.

<img src="/Images/5rad_sec_noise.JPG" width=55%>

- **Vt:** Target Velocity
- **Vm:** Measured Velocity

## Current Sensing Results

The following figures were captured from the ESC while operating in a constant velocity mode at 5
rad/sec. In the first figure the motor was running with no mechanical load, the wave form appears noisy, but a sinusoidal signal is visible.
In the second figure a mechanical load was placed on the motor, this increased the consumed current. The wave form is clear with minimal noise.

Current capture **with no** mechancal load on the motor

<img src="/Images/Current_sensing_three_phase_no_load.JPG" width=55%>

Current capture **with** a mechancal load on the motor

<img src="/Images/Current_sensing_three_phase_with_load.JPG" width=55%>


## Maintain Set Angle Results

The following figures demonstrate the angle closed loop functionality. In the figure below the set angle is 5 rad.
The motor shaft was mechanically displaced in both directions, the shaft returned its set point with slight over and undershoots.

<img src="/Images/phyiscal_displacement_Large - Labels.JPG" width=55%>

- **A:** The shaft is displaced 5 rad in a negative direction
- **B:** The shaft is released, an overshoot occurs
- **C:** The shaft is displaced 15 rad in the positive direction
- **D:** The shaft is released, an undershoot occurs


## Angle Estimation Results

A state observer was implemented to estimate the angle of the motor shaft using
voltages/currents in the rotating reference frame. Estimating the shaft of the angle was possible but
was not stable. The state observer would saturate due to noise in the current measurements, therefor
to keep the estimated angle stable a load needed to be applied to the motor to push the currents out
of the noise floor.

<img src="/Images/Estimated_angle_with_load.png" width=55%>

- **Ae:** Estimated Angle
- **Am:** Measrued Angle 


## Notes
1. I wanted to split the code up into seperate libraries but this obvisouly did not happen, so please excuse the mess.
2. My supervisor encouraged me to tightly layout the PCB and use small footprints. The smallest being 0402 packages, this was also a challange to solder but worth the effort.
3. The graphs and data seen in the document were created in MATLAB
4. The motor and magnetic mount were created in SketchUp 3D

Lastly, credit must be given to the [ArduinoFOC](https://github.com/simplefoc/Arduino-FOC) project. I spent weeks reading their code and porting ideas over to my project.




