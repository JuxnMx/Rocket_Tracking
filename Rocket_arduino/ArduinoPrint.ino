/*
* Adapted form rosserial Subscriber Example
* Blinks an LED on callback 
* URL: http://wiki.ros.org/rosserial_arduino/Tutorials/Blink
*
* Copyright @JuxnMx 2023
*/

#include <LiquidCrystal.h>
#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;

// Initialize the library by associating any needed LCD interface pin with the arduino pin number
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
 
//Define the subscriber function 'lcd_print' to print the string of the message in the LCD
void lcd_print(const std_msgs::String& msg){
    lcd.setCursor(0, 1);//Placing the cursor in the second row of the LCD
    lcd.print(msg.data);//Printing the control signal
    digitalWrite(13, HIGH-digitalRead(13));// Blinking the LED as a reference of receiving data
}
ros::Subscriber<std_msgs::String> sub("/ard_msg", &lcd_print);

//
void setup() {
  pinMode(13,OUTPUT);//Initialize the PIN for LED indicator
  // Set the ros node
  nh.getHardware()->setBaud(9600);  nh.initNode();
  // Set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Printing the titles of the message to the first row of the LCD.
  lcd.setCursor(0, 0); lcd.print("  Vx=  |  Vy=   ");
  // Printing the controls into the LCD.
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(10);
}
