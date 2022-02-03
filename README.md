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

<img src="/Images/Magnet_Motor.jpg" width=55%>
<img src="/Images/Magnetic_Sensor_Mount.jpg" width=55%>

## Notes
1. I wanted to split the code up into seperate libraries but this obvisouly did not happen, so please excuse the mess.
2. My supervisor encouraged me to tightly layout the PCB and use small footprints. The smallest being 0402 packages, this was also a challange to solder but worth the effort.
3. The graphs and data seen in the document were created in MATLAB
4. The motor and magnetic mount were created in SketchUp 3D

Lastly, credit must be given to the [ArduinoFOC](https://github.com/simplefoc/Arduino-FOC) project. I spent weeks reading their code and porting ideas over to my project.




