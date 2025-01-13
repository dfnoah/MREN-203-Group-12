/**
 * @file PWM-motor-control.ino
 * @author Joshua Marshall (joshua.marshall@queensu.ca)
 * @brief Arduino program to drive one wheel motor through a motor driver.
 * @version 2.0
 * @date 2022-12-05
 *
 * @copyright Copyright (c) 2021-2022
 *
 */

int EA = 6; // Wheel PWM pin (must be a PWM pin)
int I1 = 7; // Wheel direction digital pin 1
int I2 = 4; // Wheel direction digital pin 2

int EB = 11; // Wheel PWM pin (must be a PWM pin)
int I3 = 10; // Wheel direction digital pin 1
int I4 = 9; // Wheel direction digital pin 2


void setup()
{
    // Configure digital pins for output
    pinMode(EA, OUTPUT);
    pinMode(I1, OUTPUT); 
    pinMode(I2, OUTPUT);

    pinMode(EB, OUTPUT);
    pinMode(I3, OUTPUT); 
    pinMode(I4, OUTPUT);
}

void loop()
{
    int u; // A variable for the motor PWM command [0-255]

    // Play with this code to write open loop commands to a wheel motor
    for (u = 0; u <= 255; u += 1)
    {
        // Select a direction 
        digitalWrite(I1, LOW);  
        digitalWrite(I2, HIGH); 

        digitalWrite(I3, LOW);  
        digitalWrite(I4, HIGH); 

        // PWM command to the motor driver
        analogWrite(EA, u);
        analogWrite(EB, u);

        // Brief delay (perhaps not necessary)
        delay(10);
    }
}
