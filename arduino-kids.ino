/*
  arduino-kids.ino

  Author:  Dave Edwards
  Data:    4/1/2019
  GitHub:  https://github.com/dedwards-tech/arduino-kids
  License: BSD 3-Clause License

  Copyright (c) 2019, Dave Edwards
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

  * Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <Wire.h>
#include <rgb_lcd.h>


// Device to Board Connections and Settings
#define CFG_SERIAL_SPEED        115200  // Serial Port Speed
#define CFG_PIN_ROTARY_SENSOR       A0  // Grove Rotary A0
#define CFG_PIN_LIGHT_SENSOR        A1  // Grove Light  A1
#define CFG_PIN_LED                  2  // Grove LED    D2
#define CFG_PIN_TOUCH_SENSOR         3  // Grove Touch  D3
#define CFG_INIT_WAIT_TIME         100  // Amount of time to wait after initializing a sensor.

// Light Sensor "INPUT" Parameters - Light levels
#define LIGHT_THRESHOLD_HIGH       500
#define LIGHT_THRESHOLD_MED        250
#define LIGHT_THRESHOLD_LOW        100
#define LIGHT_SENSOR_LEVELS          3

// LED Light "OUTPUT" Parameters - LED brightness
#define LED_ON_HIGH                254
#define LED_ON_MED                 128
#define LED_ON_LOW                  75
#define LED_OFF                      0

// Touch Sensor "INPUT" Parameters - On / Off
#define TOUCH_ACTIVE                 1
#define TOUCH_INACTIVE               0
#define TOUCH_TICKLE_COUNT          20

// Color LCD Display Parameters
#define LCD_COLOR_VALUES             9
#define LCD_NUM_LINES                2
#define LCD_LINE_CHARACTERS         16

// Rotary Sensor "INPUT / OUTPUT" Lookup Table
unsigned long color_lookup[LCD_COLOR_VALUES] =
{
   0xFFFFFF,
   0x0000FF,
   0x00FFFF,
   0x00FF00,
   0xFF00FF,
   0xFFFF00,
   0xFFFF00,
   0xFF0000,
   0x000000
};

// Rotary "INPUT" Parameters
#define ROTARY_MAX_VALUE          1023
#define ROTARY_POSITION_RANGE       10
#define ROTARY_FULL_LEFT          (ROTARY_MAX_VALUE - ROTARY_POSITION_RANGE)
#define ROTARY_FULL_RIGHT          ROTARY_POSITION_RANGE
#define ROTARY_MIDDLE             (ROTARY_MAX_VALUE / 2) - (ROTARY_POSITION_RANGE / 2)

// Current System State Variables - these are updated regularly
unsigned long     total_run_time;
unsigned long     update_uart_time;
unsigned long     update_count_time;
unsigned long     update_text_time;
unsigned int      touch_count;
unsigned int      touch_value;
unsigned int      light_level;
unsigned int      rotary_position;
unsigned long     lcd_color;

// RGB Color LCD Display Device
rgb_lcd           lcd_hw;
byte              color_RED;
byte              color_GREEN;
byte              color_BLUE;
bool              rotary_changed;
bool              light_changed;
bool              touch_changed;

// Allocate memory for LCD display text
char              lcd_text[LCD_NUM_LINES][LCD_LINE_CHARACTERS + 1]; 

// Allocate memory for serial port printing using sprintf
char              text_buffer[80];                   

void setup() 
{    
    // Arduino Serial Port - for debug messages
    Serial.begin(CFG_SERIAL_SPEED);
    Serial.println("Setting up hardware devices...");
    delay(500);

    // Grove LED - Digital (turn on LED at init)
    Serial.println(" + setting up LED...");
    pinMode(CFG_PIN_LED, OUTPUT);
    delay(CFG_INIT_WAIT_TIME);
    digitalWrite(CFG_PIN_LED, LED_ON_HIGH);

    // Grove RGB Backlit LCD
    Serial.println(" + setting up the LCD display...");
    lcd_hw.begin(16, 2);
    delay(CFG_INIT_WAIT_TIME);
    lcd_hw.clear();

    // Put something on the display
    snprintf(lcd_text[0], LCD_LINE_CHARACTERS + 1, "!!ARDI ARDUINO!!");
    snprintf(lcd_text[1], LCD_LINE_CHARACTERS + 1, "**INITIALIZING**");
    lcd_update_display();
 
    // Grove Rotary Angle Sensor - Analog
    Serial.println(" + setting up the Rotary Angle Sensor...");
    pinMode(CFG_PIN_ROTARY_SENSOR, INPUT);
    delay(CFG_INIT_WAIT_TIME);
    rotary_position = analogRead(CFG_PIN_ROTARY_SENSOR);
    
    // Grove LED Light Sensor - Analog
    Serial.println(" + setting up the Light Sensor...");
    pinMode(CFG_PIN_LIGHT_SENSOR, INPUT);
    delay(CFG_INIT_WAIT_TIME);
    light_level = analogRead(CFG_PIN_LIGHT_SENSOR);

    // Grove Touch Sensor - Digital
    Serial.println(" + setting up the Touch Sensor...");
    pinMode(CFG_PIN_TOUCH_SENSOR, INPUT);
    delay(CFG_INIT_WAIT_TIME);
    touch_count = 0;

    // Actual init complete time
    snprintf(text_buffer, 80, " + init time: %d(ms)", millis());
    Serial.println(text_buffer);
    digitalWrite(CFG_PIN_LED, LED_OFF);

    // Delay so there's some indication of initialization doing something
    delay(5000);

    // Initialize timed variables
    update_uart_time  = 0;
    update_count_time = 0;
    total_run_time    = 0;  
    update_text_time  = 10000UL;
    // end of setup

    Serial.println(" * hardware setup complete");
    digitalWrite(CFG_PIN_LED, LED_OFF);
}

#define NUM_SENSOR_SAMPLES    3
#define DELAY_BETWEEN_SAMPLES 1
void read_sensors()
{
    int          ii;
    unsigned int rotary_curr = 0;
    unsigned int light_curr  = 0;
    unsigned int touch_curr  = 0;

    // take a sample of values 3 times every 1 milli-second
    for (ii = 0; ii < NUM_SENSOR_SAMPLES; ii++)
    {
        // take a single sample of each sensor
        rotary_curr += analogRead(CFG_PIN_ROTARY_SENSOR);
        light_curr  += analogRead(CFG_PIN_LIGHT_SENSOR);
        touch_curr  += digitalRead(CFG_PIN_TOUCH_SENSOR);
        delay(DELAY_BETWEEN_SAMPLES);
    }

    // average the samples to avoid bouncing in the changes
    rotary_curr = (rotary_position + rotary_curr) / (NUM_SENSOR_SAMPLES + 1);
    light_curr  = (light_level + light_curr) / (NUM_SENSOR_SAMPLES + 1);
    touch_curr /= NUM_SENSOR_SAMPLES;
    
    // Check sensors for a change from the current values
    if (rotary_curr != rotary_position)
    {
        rotary_position = rotary_curr;
        rotary_changed  = true;
    }

    // Check if the light level changed
    if (light_curr != light_level)
    {
        light_level   = light_curr;
        light_changed = true;
    }

    // Check if there was a change to the touch sensor
    if (touch_curr != touch_value)
    {
        if ((touch_value == TOUCH_INACTIVE) && (touch_curr == TOUCH_ACTIVE))
        {
           // This is a new touch from the previous and we are now "active"
           // so increase the touch count.
           touch_count += 1;
        }
        
        touch_value   = touch_curr;
        touch_changed = true;
    }
}

void lcd_update_display()
{  
    // send the first line of text to the LCD display
    lcd_hw.setCursor(0, 0);
    lcd_hw.print(lcd_text[0]);
    //delay(5);
    
    // send the second line of text to the LCD display
    lcd_hw.setCursor(0, 1);
    lcd_hw.print(lcd_text[1]);
    //delay(5);
}

void lcd_send_color(byte red, byte green, byte blue)
{
    byte light_adjust;

    // based on the light level, adjust the hue of the LCD display
    // color values, which "should" give the appearance of brightness
    // adjustment
    if (light_level > LIGHT_THRESHOLD_HIGH) 
    { 
        light_adjust = 0xBB;
    } else if (light_level > LIGHT_THRESHOLD_MED) 
    { 
       light_adjust = 0x55;
    } else 
    { 
        light_adjust = 0x00;
    }
    
    if (red != 0)   red   -= light_adjust;
    if (green != 0) green -= light_adjust;
    if (blue != 0)  blue  -= light_adjust;

    // program the LCD Display with the current color and adjust
    // for brightness by the light sensor adjustment value.
    lcd_hw.setRGB(red, green, blue);
}

void update_outputs()
{    
    if (touch_changed)
    {                
        // based on the touch sensor value blink the LED on or off and
        // count the number of times it transitions in 10 seconds.
        if (touch_value == 0)
        {
            digitalWrite(CFG_PIN_LED, LED_OFF);

        } else
        { 
            digitalWrite(CFG_PIN_LED, LED_ON_HIGH);
        }
    }

    if (rotary_changed || light_changed)
    {
        int color_index;

        // based on the rotary position, determine which color value in
        // the color lookup table we will use for the LCD display color.
        color_index = rotary_position / (ROTARY_MAX_VALUE / LCD_COLOR_VALUES);
        lcd_color   = color_lookup[color_index];

        // convert the color lookup value to individul red, green and blue
        color_RED   = (lcd_color >> 16) & 0xFF;
        color_GREEN = (lcd_color >> 8) & 0xFF;
        color_BLUE  = lcd_color & 0xFF;
        lcd_send_color((byte)color_RED, (byte)color_GREEN, (byte)color_BLUE);
    }

//    if (touch_changed || rotary_changed)
//    {
//        snprintf(text_buffer, sizeof(text_buffer)-1, " + Rotary: %d, Light: %d, Touch: %d, R: %02X G: %02X B: %02X", 
//                                                 rotary_position, light_level, touch_value, 
//                                                 color_RED, color_GREEN, color_BLUE);
//        Serial.println(text_buffer);        
//    }

    // clear our sensor detection flags.
    rotary_changed = false;
    light_changed  = false;
    touch_changed  = false;
}

void loop() 
{
    unsigned long loop_time;
    unsigned long start_time  = millis();
    
    read_sensors();
    update_outputs();
    delay(10);

    // maintain time in milli-seconds that we've been running using
    // 32 bit integer and send a message to the serial port for debugging
    // and proof we are still running.
    loop_time         = millis() - start_time;
    total_run_time   += loop_time;
    update_uart_time += loop_time;
    update_text_time += loop_time;

    // update the LCD text display periodically; 1000 milli-seconds == 1 second
    if (update_text_time > 8000UL)
    {
        if (touch_count > TOUCH_TICKLE_COUNT)
        {
            snprintf(lcd_text[0], LCD_LINE_CHARACTERS + 1, "...that tickles!");
        } else
        {
            snprintf(lcd_text[0], LCD_LINE_CHARACTERS + 1, "Hi, Ardi here!  ");
        }
        snprintf(lcd_text[1], LCD_LINE_CHARACTERS + 1, "Touch: %03u      ", touch_count);
        lcd_update_display();
        update_text_time = 0;
        touch_count      = 0;
    }

    // update the serial port with status every 4s
    if (update_uart_time >= 4000UL)
    {
        snprintf(text_buffer, sizeof(text_buffer)-1, " + Rotary: %d, Light: %d, Touch: %d, R: %02X G: %02X B: %02X", 
                                                 rotary_position, light_level, touch_value, 
                                                 color_RED, color_GREEN, color_BLUE);
        Serial.println(text_buffer);        
        update_uart_time = 0;
    }
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
