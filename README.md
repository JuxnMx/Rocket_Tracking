# Rocket_Tracking

The scope of this project is to make a closed control loop system in order to identify and locate a Rocket prior to the landing stage and guide it to the proper location.

## Implementation

The project makes use of the object detention algorithms developed in the _open cv_ and _ARUCO_ python libraries to locate the „Rocket“ through marker detection. The comunication between the different components of the system is given by ROS where the controls to be applied are displayed in a LCD display driven by an Arduino UNO board.

### Hardware description

The prototype as it was described is

![Alt text](https://assets.digitalocean.com/articles/alligator/boo.svg "a title")
