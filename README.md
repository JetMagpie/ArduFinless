# ArduFinless
An arduino based finless flying wing project, includes sideslip angle detection and drag rudder control law design.

![flight](./flight.png)

**All engineering source files (.dxf, .step, .stp, .stl, .cpp, .h, .ino) of this project are licensed under the CERN-OHL-S-2.0 license, unless otherwise stated within the specific file.**

# Setup
The default code is written for arduino mega 2560 with a AS5047p as sideslip angle sensor and a mpu6500 as gyro. The sideslip angle sensor needs 4 m2.5x4 screws, a m3*10 screw and two 5x8x2 bearings to assemble. 

![sensor](./sensor.jpg)

The default airframe is made of 2mm foam board (vector board, depron, etc.) and 2mm carbon fiber rods. A detailed STP file and a laser engraver friendly dxf file are provided. Recommended electronic devices are KST ds113mg servos, xing2205 motor with 5040 propeller (20a ESC recommended) and 1000mAh3S LiPo battery. 

![airframe](./airframe.png)

The wiring is shown in this figure:

![wiring](./wiring.png)

# Parameters
Parameters can be changed via serial (baud rate 115200), send `help` to serial to see instructions.

It is recommended to change `right_lim_inf`, `right_lim_sup`, `left_lim_inf`, `left_lim_sup` to fit your servo installation, change `encoder_calibration` and `gyro_calibration` to fit your sensor installation, and change `Cn_beta` (coefficient of sideslip angle) and `Cn_damper` (coefficient of yaw rate) to fit your aircraft design.
