# Rocket_Tracking

The scope of this project is to make a closed control loop system in order to identify and locate a Rocket prior to the landing stage and guide it to the proper location.

## Implementation

The project makes use of the object detention algorithms developed in the _open cv_ and _ARUCO_ python libraries to locate the „Rocket“ through marker detection. The comunication between the different components of the system is given by ROS where the controls to be applied are displayed in a LCD display driven by an Arduino UNO board.

### Hardware description

The „Rocket“ prototype as it was described is Arduino UNO board that drives a LCD using the electrical scheme and programming described in:
[Liquid Crystal Displays (LCD) with Arduino](https://docs.arduino.cc/learn/electronics/lcd-displays)

![Scheme](https://docs.arduino.cc/static/7d7b6e99f40c7e55f2e9c6175c6db5b5/260cd/LCD_Base_bb_Fritz.png "Rocket Prototype")
