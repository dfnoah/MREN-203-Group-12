// Encoder digital pins
const byte SIGNAL_A = 13;
const byte SIGNAL_B = 12;

const byte SIGNAL_C = 8;
const byte SIGNAL_D = 7;
// Counter to keep track of encoder ticks [integer]
volatile long R_encoder_ticks = 0;
volatile long L_encoder_ticks = 0;

// Counter to keep track of the last number of ticks [integer]
long R_encoder_ticks_last = 0;
long L_encoder_ticks_last = 0;


// This function is called when SIGNAL_A goes HIGH
void R_decodeEncoderTicks()
{
    if (digitalRead(SIGNAL_B) == LOW)
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
    if (digitalRead(SIGNAL_C) == LOW)
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

    // Set the pin modes for the encoders
    pinMode(SIGNAL_A, INPUT);
    pinMode(SIGNAL_B, INPUT);

    pinMode(SIGNAL_C, INPUT);
    pinMode(SIGNAL_D, INPUT);

    // Every time SIGNAL_A goes HIGH, this is a pulse
    attachInterrupt(digitalPinToInterrupt(SIGNAL_A), R_decodeEncoderTicks, RISING);
    attachInterrupt(digitalPinToInterrupt(SIGNAL_D), L_decodeEncoderTicks, RISING);
}

void loop()
{
    // Do this if the encoder has moved
    if ((R_encoder_ticks != R_encoder_ticks_last)||(L_encoder_ticks != L_encoder_ticks_last))
    {
        // Print some stuff to the serial monitor
        Serial.print(L_encoder_ticks);
        Serial.print(",");
        Serial.print(R_encoder_ticks);
        Serial.print("\n");

        // Record the current number of encoder ticks
        R_encoder_ticks_last = R_encoder_ticks;
        L_encoder_ticks_last = L_encoder_ticks;
    }

    // Short delay [ms]
    delay(100);
}
