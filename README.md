# README: arduino-kids

I recently started volunteering to teach kids about STEM.  I created this project for an Arduino Uno CPU with the ??? starter kit.  The goal is to be interactive and fun.

Premise:

Use the LCD display with the rotary sensor to choose from a list of activities, and a push button control to "select" the activity.  The first line of the display is the "activity" and the second line will display a result or "response" to the activity.  An LED will be used to highlight the activity doing something; like reading the temperature off of a sensor.

Activities:

1. "Check the Temperature"
    * Once selected with the push button, the arduino will read the temperature sensor and update the display with the current reading.  
    * Every 30s the temperature will be checked for 5s, while reading the temperature the LED will "breath" slowly.
    * If the temperature increases, the LED light will "blink" rapidly
    * If the temperature decreases, the LED light will "blink" slowly
    * If the temperature doesn't change, the LED light will go off.
1. "Noise Detector"
    * Once selected with the push button, the arduino will listen for loud sounds, like clapping, and update the display with the number of times it hears a loud sound.
    * Every 30s the sound sensor will be checked for sound and listen for 10s.
        * While listening the LED will "blink" each time a loud sound is detected.
    * Once listening is complete, the display will be updated with the number of loud sounds that are counted.
        * While NOT listening the LED will "breath" slowly
