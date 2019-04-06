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
#define CFG_PIN_LED                 D3  // Grove LED    D3 (PWM reqd)
#define CFG_PIN_TOUCH_SENSOR        D4  // Grove Touch  D4
#define CFG_INIT_WAIT_TIME         100  // Amount of time to wait after initializing a sensor.

// Light Sensor "INPUT" Parameters - Light levels
#define LIGHT_THRESHOLD_HIGH       500
#define LIGHT_THRESHOLD_MED        300
#define LIGHT_THRESHOLD_LOW        150
#define LIGHT_SENSOR_LEVELS          3

// LED Light "OUTPUT" Parameters - LED brightness
#define LED_ON_MAX                 254
#define LED_ON_HALF                128
#define LED_OFF                      0

// Rotary and Color LCD Display Parameters
#define LCD_COLOR_VALUES             9
#define LCD_NUM_LINES                2
#define LCD_LINE_CHARACTERS         16
#define ROTARY_MAX_VALUE           700

// Rotary Sensor "INPUT / OUTPUT" Lookup Table
unsigned short color_lookup[LCD_COLOR_VALUES] =
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

// Library Variables (global)
unsigned long int  lcd_color;
unsigned short     touch_count;
unsigned short     touch_value;
unsigned short     light_level;
unsigned short     rotary_position;
bool               inputs_changed;
long int           loop_count;

// RGB Color LCD Display Device
rgb_lcd            lcd_hw;

// Allocate memory for LCD display text
char               lcd_text[LCD_NUM_LINES][LCD_LINE_CHARACTERS + 1]; 

// Allocate memory for using sprintf functions to send messages to
// the serial port for viewing in the Arduino IDE
char               uart_buff[80]; 
                   

void setup() 
{
    // Arduino Serial Port - for debug messages
    Serial.begin(CFG_SERIAL_SPEED);
    Serial.println("Setting up hardware devices...");
    delay(500);

    // Grove LED - Digital (turn on LED at init)
    Serial.println(" -> setting up LED...");
    pinMode(CFG_PIN_LED, OUTPUT);
    delay(CFG_INIT_WAIT_TIME);
    analogWrite(CFG_PIN_LED, LED_ON_HALF);

    // Grove RGB Backlit LCD
    Serial.println(" -> setting up the LCD display...");
    lcd_hw.begin(16, 2);
    delay(CFG_INIT_WAIT_TIME);
    lcd_hw.clear();
    lcd_hw.setCursor(0, 0);
 
    // Grove Rotary Angle Sensor - Analog
    Serial.println(" -> setting up the Rotary Angle Sensor...");
    pinMode(CFG_PIN_ROTARY_SENSOR, INPUT);
    delay(CFG_INIT_WAIT_TIME);
    rotary_position = analogRead(CFG_PIN_ROTARY_SENSOR);
    
    // Grove LED Light Sensor - Analog
    Serial.println(" -> setting up the Light Sensor...");
    pinMode(CFG_PIN_LIGHT_SENSOR, INPUT);
    delay(CFG_INIT_WAIT_TIME);
    light_level = analogRead(CFG_PIN_LIGHT_SENSOR);

    // Grove Touch Sensor - Digital
    Serial.println(" -> setting up the Touch Sensor...");
    pinMode(CFG_PIN_TOUCN_SENSOR, INPUT);
    delay(CFG_INIT_WAIT_TIME);
    touch_count = 0;

    // end of setup
    Serial.println("-> hardware setup complete.");
    analogWrite(CFG_PIN_LED, LED_OFF);
    delay(500);

    // tell the main loop to send current sensor values to
    // the display.
    inputs_changed = true;
    loop_count     = 0;
}

void read_sensors()
{
    int ii;
    int rotary_curr = 0;
    int light_curr  = 0;
    int touch_curr  = 0;

    if (loop_count % 100 == 99)
    {
        // every 99 times we read sensors, send a message to the serial port
        // so we know the computer chip is still running.
        snprintf(uart_buff, sizeof(uart_buff)-1, "sampling input sensors... (loop %d)", loop_count);
        Serial.println(uart_buff);
    }

    // take a sample of values 3 times every 1 milli-second
    for (ii = 0; ii < 3; ii++)
    {
        // take a single sample of each sensor
        rotary_curr  += analogRead(CFG_PIN_ROTARY_SENSOR);
        light_curr   += analogRead(CFG_PIN_LIGHT_SENSOR);
        touch_curr   += digitalRead(CFG_PIN_TOUCH_SENSOR);
        delay(1);
    }

    // average the 3 samples
    rotary_curr  /= 3;
    light_curr   /= light_curr  / 3;
    touch_curr   /= touch_curr  / 3;
    
    // Check sensors for a change from the current values
    inputs_changed  = false;
    if (rotary_curr != rotary_position)
    {
        rotary_position = rotary_curr;
        inputs_changed  = true;
    }

    if (light_curr != light_level)
    {
        light_level    = light_curr;
        inputs_changed = true;
    }

    if (touch_curr != touch_value)
    {
        touch_value    = touch_curr;
        inputs_changed = true;
    }
}

void update_display()
{
    if (inputs_changed)
    {
        // based on the rotary position, determine which color value in
        // the color lookup table we will use for the LCD display color.
        color_index = rotary_value / (ROTARY_MAX_VALUE / LCD_COLOR_VALUES;
        color       = color_lookup[color_index]
        color_RED   = (color >> 16) & 0xFF;
        color_GREEN = (color >> 8) & 0xFF;
        color_BLUE  = color & 0xFF;

        // based on the light level, adjust the hue of the LCD display
        // color values, which "should" give the appearance of brightness
        // adjustment
        if (light_level > LIGHT_THRESHOLD_HIGH) 
        { 
            light_adjust = 0;
        } else if (light_level > 300) 
        { 
            light_adjust = 0x22;
        } else 
        { 
            light_adjust = 0x44;
        }

        // program the LCD Display with the current color and adjust
        // for brightness by the light sensor adjustment value.
        lcd_hw.setRGB(color_RED - light_adjust, color_GREEN - light_adjust, color_BLUE - light_adjust);

        // Update the text messages we will display on the LCD
        lcd.print("testing 1...2...3...");
        inputs_changed = false;
    }
}

void loop() 
{
    read_sensors();
    
    update_display();

    delay(50);
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
