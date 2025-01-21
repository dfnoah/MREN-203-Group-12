/**
   @file motor-angular-rate.ino
   @author Joshua Marshall (joshua.marshall@queensu.ca)
   @brief Arduino program to estimate motor speed from encoder.
   @version 2.1
   @date 2022-12-09

   @copyright Copyright (c) 2021-2022

*/

// Wheel PWM pin (must be a PWM pin)
int EA = 3;

int EB = 11;

// Wheel direction digital pins
int I1 = 2;
int I2 = 5;

int I3 = 10;
int I4 = 9;

// Motor PWM command variable [0-255]
byte R_u = 0;
byte L_u = 0;

// Left wheel encoder digital pins
const byte R_SIGNAL_A = 13;
const byte R_SIGNAL_B = 12;

const byte L_SIGNAL_A = 7;
const byte L_SIGNAL_B = 6;


// Encoder ticks per (motor) revolution (TPR)
const int TPR = 3000;

// Wheel radius [m]
const double RHO = 0.0625;

// Counter to keep track of encoder ticks [integer]
volatile long R_encoder_ticks = 0;
volatile long L_encoder_ticks = 0;

// Variable to store estimated angular rate of left wheel [rad/s]
double omega_R = 0.0;
double omega_L = 0.0;

// Variable to store estimated translational rate of left wheel [m/s]

double v_R = 0.0;
double v_L = 0.0;

// Sampling interval for measurements in milliseconds
const int T = 1000;

// Counters for milliseconds during interval
long t_now = 0;
long t_last = 0;

// This function is called when SIGNAL_A goes HIGH
void R_decodeEncoderTicks()
{
  if (digitalRead(R_SIGNAL_B) == LOW)
  {
    // SIGNAL_A leads SIGNAL_B, so count one way
    R_encoder_ticks--;
  }
  else
  {
    // SIGNAL_B leads SIGNAL_A, so count the other way
    R_encoder_ticks++;
  }
}

void L_decodeEncoderTicks()
{
  if (digitalRead(L_SIGNAL_B) == LOW)
  {
    // SIGNAL_A leads SIGNAL_B, so count one way
    L_encoder_ticks--;
  }
  else
  {
    // SIGNAL_B leads SIGNAL_A, so count the other way
    L_encoder_ticks++;
  }
}

void setup()
{
  // Open the serial port at 9600 bps
  Serial.begin(9600);

  // Set the pin modes for the motor driver
  pinMode(EA, OUTPUT);
  pinMode(I1, OUTPUT);
  pinMode(I2, OUTPUT);

  pinMode(EB, OUTPUT);
  pinMode(I2, OUTPUT);
  pinMode(I3, OUTPUT);

  // Set the pin modes for the encoders
  pinMode(R_SIGNAL_A, INPUT);
  pinMode(R_SIGNAL_B, INPUT);

  pinMode(L_SIGNAL_A, INPUT);
  pinMode(L_SIGNAL_B, INPUT);

  // Every time the pin goes high, this is a pulse
  attachInterrupt(digitalPinToInterrupt(R_SIGNAL_A), R_decodeEncoderTicks, RISING);
  attachInterrupt(digitalPinToInterrupt(L_SIGNAL_A), L_decodeEncoderTicks, RISING);

  // Print a message
  Serial.print("Program initialized.");
  Serial.print("\n");
}

void loop()
{
  // Get the elapsed time [ms]
  t_now = millis();

  if (t_now - t_last >= T)
  {
    // Estimate the rotational speed [rad/s]
    omega_R = -2.0 * PI * ((double)R_encoder_ticks / (double)TPR) * 1000.0 / (double)(t_now - t_last);
    omega_L = 2.0 * PI * ((double)L_encoder_ticks / (double)TPR) * 1000.0 / (double)(t_now - t_last);

    v_R = omega_R * 0.0625;
    v_L = omega_L * 0.0625;

    // Print some stuff to the serial monitor
//    Serial.print("Right Encoder ticks: ");
//    Serial.print(R_encoder_ticks);
//    Serial.print("\t");
//    Serial.print("Left Encoder ticks: ");
//    Serial.print(L_encoder_ticks);
//    Serial.print("\t");

    Serial.print("Right angular speed: ");
    Serial.print(omega_R);
    Serial.print(" rad/s");
    Serial.print("\t");
    Serial.print("Right translational speed: ");
    Serial.print(v_R);
    Serial.print(" m/s");
    Serial.print("\t");
    Serial.print("Left angular speed: ");
    Serial.print(omega_L);
    Serial.print(" rad/s");
    Serial.print("\t");
    Serial.print("Left translational speed: ");
    Serial.print(v_L);
    Serial.print(" m/s");
    Serial.print("\t");
    Serial.print("\n");

    // Record the current time [ms]
    t_last = t_now;

    // Reset the encoder ticks counter
    R_encoder_ticks = 0;
    L_encoder_ticks = 0;
  }

  // Set the wheel motor PWM command [0-255]

  L_u = 128;
  R_u = 255;

  // Select a direction
  digitalWrite(I1, HIGH);
  digitalWrite(I2, LOW);


  digitalWrite(I3, LOW);
  digitalWrite(I4, HIGH);

  // PWM command to the motor driver
  analogWrite(EA, R_u);
  analogWrite(EB, L_u);
}
