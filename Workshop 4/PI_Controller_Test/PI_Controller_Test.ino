// Wheel PWM pin (must be a PWM pin)
int EA = 3;

int EB = 5;

// Wheel direction digital pins
int I1 = 2;
int I2 = 4;

int I3 = 10;
int I4 = 9;

// Motor PWM command variable [0-255]
int R_u = 0;
int L_u = 0;

// Left wheel encoder digital pins
const byte R_SIGNAL_A = 13;
const byte R_SIGNAL_B = 12;

const byte L_SIGNAL_A = 6;
const byte L_SIGNAL_B = 7;


// Encoder ticks per (motor) revolution (TPR)
const int TPR = 3000;

// Wheel radius [m]
const double RHO = 0.0625;
double ell = 0.2775;

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
const int T = 100;

// Counters for milliseconds during interval
long t_now = 0;
long t_last = 0;

double v_desired = .35;
double KP = 400;

double KI = 25;
double L_int_prev = 0;
double R_int_prev = 0;

double KO = 10;
double t_desired = 20;
double t_v_desired = 1;
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

double compute_vehicle_rate(double v_L, double v_R)
{
  double omega;
  omega = 1.0 / ell * (v_L - v_R);
  return omega;
}

double e_now(double v, double v_desired)
{
  double error = v_desired - v;
  return error;
}

double e_int(double v_desired, double v, double prev)
{
  double integral = 0;
  integral = e_now(v, v_desired);
  integral += prev;
  return integral;
}

float PI_controller(double e_now, double k_P, double e_int, double k_I)
{
  float u;
  u = (float)(k_P * e_now + k_I * e_int);
  if (u == 255 || u == (-255))
  {
    u = (k_P * e_now);
  }
  if (u > 255)
  {
    u = 255;
  }
  else if (u < -255)
  {
    u = -255;
  }

  return u;
}

double e_turn(double omega, double omega_desired)
{
  double error = omega_desired - omega;
  return error;
}

float P_turn(double e_turn, double K_turn)
{
  float u;
  u = u = (float)(K_turn * e_turn);
  if (u > 255)
  {
    u = 255;
  }
  else if (u < -255)
  {
    u = -255;
  }

  return u;
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


    // Record the current time [ms]
    t_last = t_now;

    // Reset the encoder ticks counter
    R_encoder_ticks = 0;
    L_encoder_ticks = 0;


  }



  // Set the wheel motor PWM command [0-255]

  if (t_desired != 0)
  {
    L_u = (int)(P_turn(e_turn(compute_vehicle_rate(v_L, v_R), t_desired), KO));
    R_u = (int)(-1 * P_turn(e_turn(compute_vehicle_rate(v_L, v_R), t_desired), KO));
  }
  else {
    L_u = (int)(PI_controller(e_now(v_L, v_desired), KP, e_int(v_desired, v_L, L_int_prev), KI) + P_turn(e_turn(compute_vehicle_rate(v_L, v_R), t_desired), KO));
    R_u = (int)(PI_controller(e_now(v_R, v_desired), KP, e_int(v_desired, v_R, R_int_prev), KI) - P_turn(e_turn(compute_vehicle_rate(v_L, v_R), t_desired), KO));
  }

  L_int_prev = e_int(v_desired, v_L, L_int_prev);
  R_int_prev = e_int(v_desired, v_R, R_int_prev);

  // Select a direction

  if (R_u < 0)
  {
    R_u = R_u * -1;
    digitalWrite(I1, LOW);
    digitalWrite(I2, HIGH);
  }
  else
  {
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);
  }
  if (L_u < 0)
  {
    L_u = L_u * -1;
    digitalWrite(I3, HIGH);
    digitalWrite(I4, LOW);
  }
  else
  {
    digitalWrite(I3, LOW);
    digitalWrite(I4, HIGH);
  }


  Serial.print("Vehicle Rate: ");
  Serial.print(compute_vehicle_rate(v_L, v_R));
  Serial.print(" rad/s");
  Serial.print("\t");
  Serial.print("Rate Adj: ");
  Serial.print(P_turn(e_turn(compute_vehicle_rate(v_L, v_R), t_desired), KO));
  Serial.print("\n");


  // PWM command to the motor driver
  analogWrite(EA, R_u);
  analogWrite(EB, L_u);
}
