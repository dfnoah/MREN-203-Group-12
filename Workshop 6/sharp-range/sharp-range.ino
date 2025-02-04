/**
 * @file sharp-range.ino
 * @author Joshua Marshall (joshua.marshall@queensu.ca), Thomas Sears (thomas.sears@queensu.ca)
 * @brief Arduino program to read proximity data from a Sharp GP2Y0A21YK.
 * @version 2.1
 * @date 2022-12-21
 *
 * @copyright Copyright (c) 2022
 *
 */

// Arduino analog input pin to which the Sharp sensor is connected
const byte SHARP_PIN = A5;

// Variables to store the proximity measurement
int sharp_val = 0; // integer read from analog pin
float sharp_range; // range measurement [cm]

void setup()
{
    // Open the serial port at 115200 bps
    Serial.begin(115200);
}

void loop()
{
    // Read the sensor output (0-1023, which is 10 bits and fits inside an Arduino int-type)
    sharp_val = analogRead(SHARP_PIN);
    sharp_range = 5050 * pow(sharp_val,-.9575) - 3.2;

    // Print all values
    Serial.print(sharp_range);
    Serial.print(" CM\n");

    // Delay for a bit before reading the sensor again
    delay(500);
}
