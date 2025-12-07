/*
 * Arduino Motor Control via Serial Command + Speed Feedback
 * Command Format (Serial Monitor): "CMD <rpm>"
 * Example: "CMD 60"   (forward)
 *          "CMD -30"  (reverse)
 *          "CMD 0"    (stop)
 */

// --- PIN DEFINITIONS ---
// Motor driver pins (L298N for one motor)
const int L298N_enA = 9;   // PWM Pin
const int L298N_in1 = 12;
const int L298N_in2 = 13;

// Encoder pin (Channel A)
const int ENCODER_PIN_A = 2;   // Must be interrupt pin on UNO (2 or 3)

// --- CONSTANTS ---
const float MAX_MOTOR_RPM = 60.0;     // Your motor's approx max RPM
const int   MAX_PWM       = 255;      // Arduino PWM limit
const float ENCODER_TICKS_PER_REV = 2172.0;  // Your measured value

// --- SERIAL INPUT VARIABLES ---
String inputString = "";

// --- ENCODER / SPEED VARIABLES ---
volatile long encoderCount = 0;  // updated in ISR

unsigned long lastSpeedTimeMs = 0;
long lastEncoderCount = 0;
float actualRPM = 0.0;
float commandedRPM = 0.0;       // last commanded RPM (for sign)

// --- INTERRUPT SERVICE ROUTINE ---
void encoderISR() {
  // Count every edge on channel A
  encoderCount++;
}

void setup() {
  // Motor pins
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);

  // Start with motor stopped
  digitalWrite(L298N_in1, LOW);
  digitalWrite(L298N_in2, LOW);
  analogWrite(L298N_enA, 0);

  // Encoder pin
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), encoderISR, CHANGE);

  // Serial
  Serial.begin(115200);
  Serial.println("System Ready.");
  Serial.println("Enter command format: CMD <rpm>");
  

  lastSpeedTimeMs = millis();
  lastEncoderCount = encoderCount;
}





void loop() {
  // -------- 1. READ SERIAL COMMANDS --------
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();  // remove spaces/newlines at ends

    if (input.startsWith("CMD ")) {
      String valueStr = input.substring(4); // part after "CMD "
      float targetRPM = valueStr.toFloat();
      setMotorSpeed(targetRPM);
    } else {
      Serial.println("Error: Unknown command. Use 'CMD <rpm>'");
    }
  }

  // -------- 2. COMPUTE ACTUAL SPEED PERIODICALLY --------
  unsigned long nowMs = millis();
  unsigned long dtMs = nowMs - lastSpeedTimeMs;

  // e.g. update speed estimate every 100 ms
  if (dtMs >= 100) {  // 0.1 seconds

    long currentCount = encoderCount;  // copy volatile once
    long deltaTicks   = currentCount - lastEncoderCount;
    float dtSec       = dtMs / 1000.0;

    // ticks per second
    float ticksPerSec = deltaTicks / dtSec;

    // rev per second
    float revPerSec = ticksPerSec / ENCODER_TICKS_PER_REV;

    // RPM (always positive from encoder, sign added from command)
    float rpmUnsigned = revPerSec * 60.0;

    // Give the speed a sign depending on commanded direction
    if (commandedRPM < 0) {
      actualRPM = -rpmUnsigned;
    } else {
      actualRPM = rpmUnsigned;
    }

    // Print feedback
    Serial.print("CMD RPM: ");
    Serial.print(commandedRPM, 2);
    Serial.print(" | ACTUAL RPM: ");
    Serial.println(actualRPM, 2);

    // Update history
    lastEncoderCount = currentCount;
    lastSpeedTimeMs  = nowMs;
  }
}

// -------- MOTOR CONTROL FUNCTION --------
void setMotorSpeed(float rpm) {
  // Safety clamp
  if (rpm >  MAX_MOTOR_RPM) rpm =  MAX_MOTOR_RPM;
  if (rpm < -MAX_MOTOR_RPM) rpm = -MAX_MOTOR_RPM;

  commandedRPM = rpm;  // store last command

  // 1. Set direction
  if (rpm > 0) {
    // Forward
    digitalWrite(L298N_in1, HIGH);
    digitalWrite(L298N_in2, LOW);
    Serial.print("Direction: FORWARD | ");
  } 
  else if (rpm < 0) {
    // Reverse
    digitalWrite(L298N_in1, LOW);
    digitalWrite(L298N_in2, HIGH);
    Serial.print("Direction: REVERSE | ");
  } 
  else {
    // Stop
    digitalWrite(L298N_in1, LOW);
    digitalWrite(L298N_in2, LOW);
    analogWrite(L298N_enA, 0);
    Serial.println("Motor STOPPED");
    return;
  }

  // 2. Calculate PWM from RPM (linear mapping)
  float rpmAbs = fabs(rpm);
  int pwmValue = (int)((rpmAbs / MAX_MOTOR_RPM) * MAX_PWM);

  if (pwmValue > MAX_PWM) pwmValue = MAX_PWM;

  // 3. Apply PWM
  analogWrite(L298N_enA, pwmValue);

  // Debug
  Serial.print("Target RPM: ");
  Serial.print(rpm);
  Serial.print(" | PWM Sent: ");
  Serial.println(pwmValue);
}
