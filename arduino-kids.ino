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

// Light Sensor "INPUT" Parameters - Light levels
#define LIGHT_THRESHOLD_HIGH       500
#define LIGHT_THRESHOLD_MED        300
#define LIGHT_THRESHOLD_LOW        150
#define LIGHT_SENSOR_LEVELS          3

// LED Light "OUTPUT" Parameters - LED brightness
#define LED_ON_MAX                 254
#define LED_ON_HALF                128
#define LED_OFF                      0

// Touch Sensor "INPUT" Parameters - On / Off state
#define TOUCH_

// Rotary and Color LCD Display Parameters
#define LCD_COLOR_VALUES             9
#define LCD_NUM_LINES                2
#define LCD_LINE_CHARACTERS         16
#define ROTARY_MAX_VALUE           700

// Rotary Sensor "INPUT / OUTPUT" Lookup Table
unsigned short color_lookup[ROTARY_LOOKUP_STATES] =
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
rgb_lcd            lcd;
unsigned long int  lcd_color;
unsigned short     touch_count;
unsigned short     touch_value;
unsigned short     light_level;
unsigned short     rotary_position;

// Allocate memory for LCD display text
char               lcd_text[LCD_NUM_LINES][LCD_LINE_CHARACTERS + 1]; 

// Allocate memory for using sprintf functions to send messages to
// the serial port for viewing in the Arduino IDE
char               uart_buff[80]; 
                   

void setup() 
{
    // Arduino Serial Port - for debug messages
    Serial.begin(CFG_SERIAL_SPEED);
    Serial.println("Initializing...");
    delay(500);

    // Grove LED - Digital (turn on LED at init)
    Serial.println(" -> setting up LED...");
    pinMode(CFG_PIN_LED, OUTPUT);
    delay(200);
    analogWrite(CFG_PIN_LED, LED_ON_HALF);

    // Grove RGB Backlit LCD
    Serial.println(" -> setting up the LCD display...");
    lcd.begin(16, 2);
    delay(200);
    lcd.clear();
    lcd.setCursor(0, 0);
    delay(200);

    // Grove Rotary Angle Sensor - Analog
    Serial.println(" -> setting up the Rotary Angle Sensor...");
    pinMode(CFG_PIN_ROTARY_SENSOR, INPUT);
    delay(200);
    rotary_position = analogRead(CFG_PIN_ROTARY_SENSOR);
    
    // Grove LED Light Sensor - Analog
    Serial.println(" -> setting up the Light Sensor...");
    pinMode(CFG_PIN_LIGHT_SENSOR, INPUT);
    delay(200);
    light_level = analogRead(CFG_PIN_LIGHT_SENSOR);

    // Grove Touch Sensor - Digital
    Serial.println(" -> setting up the Touch Sensor...");
    pinMode(CFG_PIN_TOUCN_SENSOR, INPUT);
    delay(200);
    touch_count = 0;

    // end of setup
    Serial.println("-> init done");
    analogWrite(CFG_PIN_LED, LED_OFF);
    delay(500);
}

void read_sensors()
{
int ii;
int rotary_last_position = rotary_position;
int light_last_level     = light_level;
int touch_last_value     = touch_value;

    for (ii = 0; ii < 3; ii++)
    {
        // take a single sample of each sensor
        rotary_last_value += analogRead(CFG_PIN_ROTARY_SENSOR);
        light_last_value  += analogRead(CFG_PIN_LIGHT_SENSOR);
        touch_last_value  += digitalRead(CFG_PIN_TOUCH_SENSOR);
        
        delay(1);
    }

    // average the 4 samples (1 from the last reading)
    rotary_last_value >>= 2;
    light_last_value  >>= 2;
    touch_last_value  >>= 2;
    
}

void update_display()
{
}

void loop() 
{
    int   rotary_value, light_value, sound_value, button_value;
    float curr_temp;

    // Take a sample of our input devices
    rotary_value = analogRead(CFG_PIN_ROTARY_SENSOR);
    light_value  = analogRead(CFG_PIN_LIGHT_SENSOR);
    sound_value  = analogRead(CFG_PIN_SOUND_SENSOR);
    button_value = digitalRead(CFG_PIN_BUTTON);

    curr_temp    = read_temperature();

    // Dump values to serial port then delay
    snprintf(uart_buff, sizeof(uart_buff)-1, "Temp: %d(F)", (int)curr_temp);
    Serial.println(uart_buff);

    snprintf(uart_buff, sizeof(uart_buff)-1, "Rotary: %d",  rotary_value);
    Serial.println(uart_buff);
    lcd.print(uart_buff);

    snprintf(uart_buff, sizeof(uart_buff)-1, "Sound: %d",   sound_value);
    Serial.println(uart_buff);

    snprintf(uart_buff, sizeof(uart_buff)-1, "Light: %d",   light_value);
    Serial.println(uart_buff);

    snprintf(uart_buff, sizeof(uart_buff)-1, "Button: %d",  button_value);
    Serial.println(uart_buff);

    // change to green after we enter loop()
    //lcd.setRGB(0, 255, 0);
    //delay(50);
    
    // Print a message to the LCD.
    //lcd.print("hello, world!");

    delay(10000);
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
