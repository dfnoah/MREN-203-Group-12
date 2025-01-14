/**
   @file PWM-motor-control.ino
   @author Joshua Marshall (joshua.marshall@queensu.ca)
   @brief Arduino program to drive one wheel motor through a motor driver.
   @version 2.0
   @date 2022-12-05

   @copyright Copyright (c) 2021-2022

*/

int EA = 3; // Wheel PWM pin (must be a PWM pin)
int I1 = 2; // Wheel direction digital pin 1
int I2 = 5; // Wheel direction digital pin 2

int EB = 11; // Wheel PWM pin (must be a PWM pin)
int I3 = 10; // Wheel direction digital pin 1
int I4 = 9; // Wheel direction digital pin 2

int u; // A variable for the motor PWM command [0-255]



void setup()
{
  // Configure digital pins for output
  pinMode(EA, OUTPUT);
  pinMode(I1, OUTPUT);
  pinMode(I2, OUTPUT);

  pinMode(EB, OUTPUT);
  pinMode(I3, OUTPUT);
  pinMode(I4, OUTPUT);

  Serial.begin(9600);

}

void forward(int del)

{
  int timer = 0;
  
  while (timer <= del/10) {
    for (u = 100; u <= 150; u += 15)
    {
      // Select a direction
      digitalWrite(I1, HIGH);
      digitalWrite(I2, LOW);

      digitalWrite(I3, LOW);
      digitalWrite(I4, HIGH);
      // PWM command to the motor driver

      analogWrite(EA, u);
      analogWrite(EB, u);

      // Brief delay (perhaps not necessary)
      delay(10);
      timer = timer + 1;
      Serial.println(timer);

    }
  }
  Serial.println("stopped");

  digitalWrite(I1, HIGH);
  digitalWrite(I2, HIGH);

  digitalWrite(I3, HIGH);
  digitalWrite(I4, HIGH);

}

void backward(int del)

{
  int timer = 0;
  while (timer <= del/10) {
    for (u = 100; u <= 150; u += 15)
    {
      // Select a direction
      digitalWrite(I1, LOW);
      digitalWrite(I2, HIGH);

      digitalWrite(I3, HIGH);
      digitalWrite(I4, LOW);
      // PWM command to the motor driver

      analogWrite(EA, u);
      analogWrite(EB, u);

      // Brief delay (perhaps not necessary)
      delay(10);
      timer = timer + 1;
      Serial.println(timer);

    }
  }
  Serial.println("stopped");

  digitalWrite(I1, HIGH);
  digitalWrite(I2, HIGH);

  digitalWrite(I3, HIGH);
  digitalWrite(I4, HIGH);

}
void left()
{
  // Select a direction
  digitalWrite(I1, HIGH);
  digitalWrite(I2, LOW);

  digitalWrite(I3, HIGH);
  digitalWrite(I4, LOW);
}

void right()
{
  // Select a direction
  digitalWrite(I1, LOW);
  digitalWrite(I2, HIGH);

  digitalWrite(I3, LOW);
  digitalWrite(I4, HIGH);
}



void loop()
{

  forward(1000);
  delay(100);
  backward(1000);


  while (true) {
    delay(100);
  }
}
