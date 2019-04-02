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


/*** System Configuration Definitions ***/
#define CFG_SERIAL_SPEED        115200
#define CFG_PIN_ROTARY_SENSOR       A0  // Grove Rotary A0
#define CFG_PIN_TEMP_SENSOR         A1  // Grove Temp   A2
#define CFG_PIN_SOUND_SENSOR        A2  // Grove Sound  A3
#define CFG_PIN_LIGHT_SENSOR        A3  // Grove Light  A1
#define CFG_PIN_LED                  3  // Grove LED    D3
#define CFG_PIN_BUTTON               7  // Grove Button D7


/*** LED Light "OUTPUT" Values ***/

// Various integer values represenging the brightness level of the LED
#define LED_ON_MAX                 254
#define LED_ON_HALF                128
#define LED_OFF                      0

// Number of breath increments at each "WAIT STEP"
#define LED_BREATH_STEP              5

// Time (in milliseconds) to wait before updating LED breath value
#define LED_BREATH_WAIT_STEP         5

/*** Light Sensor "INPUT" Values ***/
#define LIGHT_THRESHOLD_HIGH  1200  // Question: what are the range of values possible?
#define LIGHT_THRESHOLD_MED   800
#define LIGHT_THRESHOLD_LOW   400

/*** Sound Sensor "INPUT" Values ***/
#define SOUND_THRESHOLD_HIGH  100  // Question: what are the range of values possible?
#define SOUND_THRESHOLD_MED   75
#define SOUND_THRESHOLD_LOW   50

// Library Variables (global)
rgb_lcd     lcd;
char        lcd_buff[17]; 
char        uart_buff[80]; 

void setup() 
{
    // Arduino Serial Port - for debug messages
    Serial.begin(CFG_SERIAL_SPEED);
    Serial.println("Initializing...");
    delay(500);

    // Grove LED - Digital (turn on LED at init)
    Serial.println("LED init...");
    pinMode(CFG_PIN_LED, OUTPUT);
    delay(200);
    analogWrite(CFG_PIN_LED, LED_ON_HALF);

    // Grove RGB Backlit LCD
    Serial.println("LCD init...");
    lcd.begin(16, 2);
    delay(200);
    lcd.clear();
    lcd.setCursor(0, 0);
    delay(200);

    // Grove Rotary Angle Sensor - Analog
    Serial.println("Rotary init...");
    pinMode(CFG_PIN_ROTARY_SENSOR, INPUT);
    delay(200);
    
    // Grove LED Light Sensor - Analog
    Serial.println("Light sensor init...");
    pinMode(CFG_PIN_LIGHT_SENSOR, INPUT);
    delay(200);

    // Grove Sound Sensor - Analog
    Serial.println("Sound sensor init...");
    pinMode(CFG_PIN_SOUND_SENSOR, INPUT);
    delay(200);

    // Grove Button - Digital
    Serial.println("Button init...");
    pinMode(CFG_PIN_BUTTON, INPUT);
    delay(200);

    // end of setup
    Serial.println("-> init done");
    analogWrite(CFG_PIN_LED, LED_OFF);
    delay(500);
}

void led_breath()
{
    // step up the brightness of the LED
    for (int ii = 0; ii < 256; ii += LED_BREATH_STEP)
    {
        analogWrite(CFG_PIN_LED, ii);
        delay(LED_BREATH_WAIT_STEP);
    }

    // wait just a bit before stepping down the brightness
    delay(100 / LED_BREATH_WAIT_STEP);
    
    for (int ii = 254; ii >= 0; ii -= LED_BREATH_STEP)
    {
        analogWrite(CFG_PIN_LED, ii);
        delay(LED_BREATH_WAIT_STEP);
    }
}

// Question: where does this "thermistor" value come from?
#define HW_THERMISTOR_BASIS  3975.0

float read_temperature()
{
    float temperature, resistance;
    int   sensor_value;

    sensor_value = analogRead(CFG_PIN_TEMP_SENSOR);
    resistance   = (float)((1023 - sensor_value) * 10000) / (float)(sensor_value);

    // Question: is this temperature in degrees Farenhiet or Celcius?
    temperature  = 1.0 / (log(resistance / 10000.0) / HW_THERMISTOR_BASIS + 1.0 / 298.15) - 273.15;

    return(temperature);
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
