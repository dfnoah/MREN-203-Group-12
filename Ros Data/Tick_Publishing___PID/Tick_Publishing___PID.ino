// Wheel PWM pin (must be a PWM pin)
int EA = 3;
int EB = 5;

// Wheel direction digital pins
int I1 = 2, I2 = 4, I3 = 10, I4 = 9;

// Motor PWM command variable [0-255]
int R_u = 0, L_u = 0;

// Encoder digital pins (Program 2 only)
const byte SIGNAL_A = 13;  // Right encoder A
const byte SIGNAL_B = 12;  // Right encoder B
const byte SIGNAL_C = 8;   // Left encoder A
const byte SIGNAL_D = 7;   // Left encoder B

// Encoder ticks per (motor) revolution (TPR)
const int TPR = 3000;

// Wheel radius [m]
const double RHO = 0.0625;
double ell = 0.2775;

// Encoder tick counters (Program 2 style)
volatile long R_encoder_ticks = 0;
volatile long L_encoder_ticks = 0;
long R_encoder_ticks_last = 0;
long L_encoder_ticks_last = 0;

// Estimated wheel speeds [m/s]
double v_R = 0.0, v_L = 0.0;

// Sampling interval for measurements [ms]
const int T = 100;
long t_now = 0, t_last = 0;

// PID controller parameters
double KP = 400, KI = 25;
double L_int_prev = 0, R_int_prev = 0;

// Default desired speeds
float t_desired = 0; // Default turning rate [rad/s]
float v_desired = 0; // Default linear speed [m/s]

int v_input = 0;
int t_input = 0;

// Program 2 encoder handling functions
void R_decodeEncoderTicks() {
    if (digitalRead(SIGNAL_B) == LOW) {
        R_encoder_ticks--;
    } else {
        R_encoder_ticks++;
    }
}

void L_decodeEncoderTicks() {
    if (digitalRead(SIGNAL_C) == LOW) {
        L_encoder_ticks--;
    } else {
        L_encoder_ticks++;
    }
}

double compute_vehicle_rate(double v_L, double v_R) {
    return (v_L - v_R) / ell;
}

double compute_desired_speed(double omega, double v_in, char dir) {
    if (omega == 0) return v_in;
    return (dir == 'L') ? v_in + omega * ell : v_in - omega * ell;
}

double e_now(double v, double v_desired) {
    return v_desired - v;
}

double e_int(double v_desired, double v, double prev) {
    return e_now(v, v_desired) + prev;
}

float PI_controller(double e_now, double k_P, double e_int, double k_I) {
    float u = k_P * e_now + k_I * e_int;
    if (u > 255) return 255;
    if (u < -255) return -255;
    return u;
}

void setup() {
    Serial.begin(9600);
    
    // Motor control pins
    pinMode(EA, OUTPUT); pinMode(I1, OUTPUT); pinMode(I2, OUTPUT);
    pinMode(EB, OUTPUT); pinMode(I3, OUTPUT); pinMode(I4, OUTPUT);
    
    // Encoder pins (Program 2 only)
    pinMode(SIGNAL_A, INPUT);
    pinMode(SIGNAL_B, INPUT);
    pinMode(SIGNAL_C, INPUT);
    pinMode(SIGNAL_D, INPUT);
    
    // Attach interrupts (Program 2 style)
    attachInterrupt(digitalPinToInterrupt(SIGNAL_A), R_decodeEncoderTicks, RISING);
    attachInterrupt(digitalPinToInterrupt(SIGNAL_D), L_decodeEncoderTicks, RISING);
}

void loop() {
    // Handle serial input for desired speeds (Program 1)
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        sscanf(input.c_str(), "%d %d", &v_input, &t_input);

        v_desired = (float)v_input/100;
        t_desired = (float)t_input/100;
    }

    // Program 1 functionality - speed calculation and motor control
    t_now = millis();
    if (t_now - t_last >= T) {
        double omega_R = 2.0 * PI * ((double)(R_encoder_ticks - R_encoder_ticks_last) / (double)TPR) * 1000.0 / (double)(t_now - t_last);
        double omega_L = 2.0 * PI * ((double)(L_encoder_ticks - L_encoder_ticks_last) / (double)TPR) * 1000.0 / (double)(t_now - t_last);
        v_R = omega_R * RHO;
        v_L = omega_L * RHO;

        t_last = t_now;
        R_encoder_ticks_last = R_encoder_ticks;
        L_encoder_ticks_last = L_encoder_ticks;
    }

    // Motor control calculations (Program 1)
    L_u = (int)(PI_controller(e_now(v_L, compute_desired_speed(t_desired, v_desired, 'L')), KP, e_int(compute_desired_speed(t_desired, v_desired, 'L'), v_L, L_int_prev), KI));
    R_u = (int)(PI_controller(e_now(v_R, compute_desired_speed(t_desired, v_desired, 'R')), KP, e_int(compute_desired_speed(t_desired, v_desired, 'R'), v_R, R_int_prev), KI));

    if (t_desired == 0 && v_desired == 0){
      L_int_prev = 0;
      R_int_prev = 0;
    }
    else{
      L_int_prev = e_int(v_desired, v_L, L_int_prev);
      R_int_prev = e_int(v_desired, v_R, R_int_prev);
    }
             

    


    // Set motor directions
    if (R_u < 0) {
        R_u = -R_u;
        digitalWrite(I1, LOW); digitalWrite(I2, HIGH);
    } else {
        digitalWrite(I1, HIGH); digitalWrite(I2, LOW);
    }
    
    if (L_u < 0) {
        L_u = -L_u;
        digitalWrite(I3, HIGH); digitalWrite(I4, LOW);
    } else {
        digitalWrite(I3, LOW); digitalWrite(I4, HIGH);
    }

    // Apply motor commands
    analogWrite(EA, R_u);
    analogWrite(EB, L_u);

    // Program 2 functionality - encoder tick reporting (exact original format)
    if ((R_encoder_ticks != R_encoder_ticks_last) || (L_encoder_ticks != L_encoder_ticks_last)) {
        Serial.print(L_encoder_ticks);
        Serial.print(",");
        Serial.print(R_encoder_ticks);
        Serial.println();
    }

    // Small delay to prevent serial flooding
    delay(10);
}
