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
1. Arduino Uno CPU (URL: ???)
1. Grove Starter Kit from Seeed (URL: ???)
1. 9V Battery
1. USB 2.0 Printer Cable

## Connections

(INSERT PHOTO OR DRAWING HERE)
