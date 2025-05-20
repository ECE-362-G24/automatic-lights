# Automatic-Lights
Automating Light and Temperature Regulation using the STM32

The project is designed to automate lighting and temperature regulation. It will adjust the RGB LED based on the ambient light and will also control the speed of a fan based on room temperature 

Based on this, the four objectives for this project are:
Automatically adjust RGB LED lighting based on ambient light conditions.
Activate/deactivate fan based on room temperature- Turns on when temperature crosses 75 F.      
Enable user input using a keypad, with the chosen input displayed on seven-segment displays
Manually override RGB LED color based on keypad input, use keypad to reset to automatic color control. 

The main features are:
Automatic Lighting Control: Adjust RGB LED based on ambient brightness
Temperature Monitoring and Fan Control: Adjust fan speed to regulate temperature.
Interactive Color Selection via Keypad: Select different lighting colors by inputting letters/numbers on keypad. 
Seven-Segment Display Feedback: Two seven-segment displays provide real-time feedback by showing the buttons pressed on keypad. 
OLED display:  Shows current ambient brightness and current room temperature. 
	
The hardware that we plan to use is:
RGB LED
OLED Display
Fan
Ambient Light Sensor (TEMT6000 )

Temperature Sensor (LM35)


Keypad
Seven-Segment Displays
	…along with the STM32 microcontroller, breadboard, and a laptop.

What the code will do:
Enable the pins to control the fans and the RGB
Toggle control modes to manually control the RGB and the Fan speed
Display output text on OLED
Display input text on the seven-segment
Read the inputs from the sensor
Automatically control the RGB and fan speed based on ambient temperature and lighting

Uses: 
ADC- To read analog data from the ambient light and temperature sensors
Timers: To generate PWM signals and manage timing for various control tasks
 GPIO-  To read inputs from the keypad to select color for RGB LED, and to control the fan. 
 PWM- To adjust LED brightness/color. Also to regulate fan speed based on temperature readings.

Provide an outline of your planned timeline for completing the various steps of your project. Please give specific deliverables (e.g. complete UART initialization code by 10/18/2024) and not vague ones (e.g. complete integration of UART by October):
November 1st
Code Completed test and verify what is possible
Keypad
Seven segment
PWMs and Clocks etc. 
November 6th
Temperature sensor integrated
November 10th 
Ambient Light sensor integrated
November 13th
Project integrated, start testing and optimizing
November 15th
Have final testing completed

Similar Projects- 
DIY Ambient Light Sensor and LED Control- https://www.instructables.com/Ambient-Light-Controlled-LED
Smart Home Temperature Control- https://projecthub.arduino.cc/kirby-b/automatic-fan-to-cool-your-room-be8164
Smart RGB LED Projects-
https://www.hackster.io/thingstudio/pwm-control-of-rgb-led-with-esp8266-and-blynk-2c40d7

Our project will combine parts of what is shown in the links here: ambient lighting, temperature control, and interactive user input, into one to make a large automated system to both help with lighting, and regulate the temperature automatically. 
Additionally, the microcontroller will allow for very responsive control over the lighting and fan speed. This also allows it to support real-time updates, which will be useful for the seven-segment displays and inputs.
