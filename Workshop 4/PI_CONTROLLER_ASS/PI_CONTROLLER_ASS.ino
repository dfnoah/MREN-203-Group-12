// Wheel PWM pin (must be a PWM pin)
int EA = 3;

int EB = 5;

// Wheel direction digital pins
int I1 = 2;
int I2 = 4;

int I3 = 10;
int I4 = 9;

// Motor PWM command variable [0-255]
byte R_u = 0;
byte L_u = 0;

// Left wheel encoder digital pins
const byte R_SIGNAL_A = 13;
const byte R_SIGNAL_B = 12;

const byte L_SIGNAL_A = 6;
const byte L_SIGNAL_B = 7;


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

double v_desired = .5;
double KP = 255 / v_desired;

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

double compute_vehicle_speed(double v_L, double v_R)
{
  double v;
  v = 0.5 * (v_L + v_R);
  return v;
}


double L_e_now(double v_L, double v_desired)
{
  double error = v_desired - v_L;
  return error;
}

float L_PI_controller(double e_now, double k_P)
{
  float u;
  u = (float)(k_P * e_now);
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

  Serial.print("Left translational speed: ");
  Serial.print(v_L);
  Serial.print(" m/s");
  Serial.print("\t");
  Serial.print("Left u val: ");
  Serial.print(L_u);
  Serial.print("\t");
  Serial.print("error: ");
  Serial.print(L_e_now(v_L, v_desired));
  Serial.print("\n");



  // Set the wheel motor PWM command [0-255]

  L_u = (int)L_PI_controller(L_e_now(v_L, v_desired), KP);
  R_u = 0;

  // Select a direction
  digitalWrite(I1, HIGH);
  digitalWrite(I2, LOW);


  digitalWrite(I3, LOW);
  digitalWrite(I4, HIGH);

  // PWM command to the motor driver
  analogWrite(EA, R_u);
  analogWrite(EB, L_u);
}
