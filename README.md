# README: arduino-kids

I recently started volunteering to teach kids about STEM.  I created this project for an Arduino Uno CPU with the GROVE starter kit made by Seeed.  The goal is to be interactive and fun while learning about arduino technology.

## Premise

Use the LCD display with the rotary sensor to choose from a list of activities, and a push button control to "select" the activity.  The first line of the display is the "activity" and the second line will display a result or "response" to the activity.  An LED will be used to highlight the activity doing something; like reading the temperature off of a sensor.

## Activities

1. "Light Detector"
    * Check the light sensor and adjust the LCD Display brightness accordingly.
        * When the light level increases, decrease the LCD brightness.
        * When the light level decreases, increase the LCD brightness.
        * Display the light level on the display.
1. "Touch Sensor"
    * Check the touch sensor for touch state frequently.
        * Blink the LED when the touch sensor reads "active touch".
        * Count the touches in a 10 second time frame.
        * Display the number of touches sonce last time it was updated.
1. "Rotary Position"
    * Check the position value of the rotary and light level every 5 seconds.
        * Update the LCD color, based on the rotary position.
        * Update the LCD brightness or hue, based on the light level value.
        * Change the color and brightness of the LCD 

## What is Needed?

You will need the following to run this code:
1. Arduino Uno CPU (URL: https://www.arduino.cc/en/Guide/ArduinoUno)
1. Grove Starter Kit from Seeed (http://wiki.seeedstudio.com/Grove_Starter_Kit_v3/)
   1. Grove Light Sensor: http://wiki.seeedstudio.com/Grove-Light_Sensor/
   1. Grove LED Socket Kit: http://wiki.seeedstudio.com/Grove-LED_Socket_Kit/
   1. Grove Rotary Angle Sensor: http://wiki.seeedstudio.com/Grove-Rotary_Angle_Sensor/
   1. Grove Touch Sensor: http://wiki.seeedstudio.com/Grove-Touch_Sensor/
   1. Grove LCD RGB Backlight: http://wiki.seeedstudio.com/Grove-LCD_RGB_Backlight/
1. 9V Battery
1. USB 2.0 Printer Cable

## System Diagram

![System Diagram](arduino-kids-overview.png?raw=true "Arduino Kids System Overview")

Device Connections:
* D2: LED Socket Kit
* D3: Touch Sensor
* A0: Rotary Angle Sensor
* A1: Light Sensor
* I2C: LCD RGB Backlight Display

## Teaching Aids

Sensors are a really cool capability of small embedded systems like these Arduino kits.  These sensors were chosen for very simple interaction with the computer system but also to demonstrate key capabilities that someone can do with sensors that perhaps a child hasn't thought about.

The diagram below can be printed and placed on a poster board to aid in answering questions that kids may have about them and how they work.

![Sensor Diagram](arduino-sensors.png?raw=true "Arduino - Grove Sensors")

Color used to be an expensive option for user interaction.  With this demonstration we can show how different colors are made from the 3 primary colors, Red, Green and Blue.  Purple is an obvious choice for anyone, including kids, but how does one derive yellow, or white when these colors don't seem obvious from the 3 primary colors.

The diagram below represents a small set of colors possible with the Grove LCD RBG Backlight display.  There are 3 LED lights of the primary colors inside the device. The arrangement of these LED lights allows for 256 different shades of each LED light color.  By doing the math on each color 256 x 256 x 256 = 16,777,216 color possibilities.  We chose to keep this simple and focus on color values that are between red, green and blue for a total of 8 colors where the color 000000 (normally black) means all colors off.

![Colors Diagram](arduino-colors.png?raw=true "8-bit RGB Color")
