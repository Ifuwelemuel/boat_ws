
// --- PIN DEFINITIONS ---
const int L298N_enA = 9;   // PWM Pin
const int L298N_in1 = 12;
const int L298N_in2 = 13;

// Encoder pin (Channel A)
const int ENCODER_PIN_A = 2;

// --- CONSTANTS ---
// UPDATED: Set to 100 as requested.
const float MAX_MOTOR_RPM = 100.0; 
const float ENCODER_TICKS_PER_REV = 2172.0;

// --- PID CONSTANTS (Tunable) ---
float Kp = 2.0;   // Proportional Gain
float Ki = 5.0;   // Integral Gain
float Kd = 0.0;   // Derivative Gain

// --- VARIABLES ---
volatile long encoderCount = 0;
unsigned long lastSpeedTimeMs = 0;
long lastEncoderCount = 0;

float actualRPM = 0.0;
float commandedRPM = 0.0;
float integralSum = 0.0;
float lastError = 0.0;

// --- INTERRUPT SERVICE ROUTINE ---
void encoderISR() {
  encoderCount++;
}

void setup() {
  // Motor pins
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);

  // Encoder pin
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), encoderISR, CHANGE);

  // Serial
  Serial.begin(115200);
  Serial.println("System Ready: Max RPM 100, Forward Only.");
  
  lastSpeedTimeMs = millis();
}

void loop() {
  // -------- 1. READ SERIAL COMMANDS --------
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.startsWith("CMD ")) {
      float newTarget = input.substring(4).toFloat();
      
      // SAFETY CHECK: Clamp input between 0 and 100
      if (newTarget < 0) {
        newTarget = 0;
        Serial.println("Warning: Reverse disabled. Target set to 0.");
      } 
      else if (newTarget > MAX_MOTOR_RPM) {
        newTarget = MAX_MOTOR_RPM;
        Serial.println("Warning: Request exceeds limit. Target set to 100.");
      }
      
      commandedRPM = newTarget;
    }
  }

  // -------- 2. PID LOOP (Runs every 100ms) --------
  unsigned long nowMs = millis();
  unsigned long dtMs = nowMs - lastSpeedTimeMs;

  if (dtMs >= 100) { 
    float dtSec = dtMs / 1000.0;

    // A. CALCULATE ACTUAL SPEED
    long currentCount = encoderCount;
    long deltaTicks = currentCount - lastEncoderCount;
    float ticksPerSec = deltaTicks / dtSec;
    
    // Reverse disabled, so speed is always positive
    actualRPM = (ticksPerSec / ENCODER_TICKS_PER_REV) * 60.0;

    // B. PID CALCULATION
    float error = commandedRPM - actualRPM;
    
    // Proportional
    float P = Kp * error;

    // Integral
    integralSum += (error * dtSec);
    // Anti-windup
    if (integralSum >  100) integralSum =  100; 
    if (integralSum < -100) integralSum = -100;
    float I = Ki * integralSum;

    // Derivative
    float D = Kd * ((error - lastError) / dtSec);
    lastError = error;

    // Output
    float output = P + I + D;

    // C. APPLY TO MOTOR
    runMotorDriver(output);

    // D. DEBUG
    Serial.print("Tgt: "); Serial.print(commandedRPM);
    Serial.print(" | Act: "); Serial.print(actualRPM);
    Serial.print(" | PWM: "); Serial.println(output);

    lastEncoderCount = currentCount;
    lastSpeedTimeMs = nowMs;
  }
}

// -------- MOTOR DRIVER HELPER --------
void runMotorDriver(float pwmVal) {
  // Output Safety: If PID outputs negative (braking), just stop.
  if (pwmVal < 0) {
    pwmVal = 0; 
  }
  
  // Cap at 255
  if (pwmVal > 255) pwmVal = 255;

  if (pwmVal > 0) {
    // Forward Configuration
    digitalWrite(L298N_in1, HIGH);
    digitalWrite(L298N_in2, LOW);
    analogWrite(L298N_enA, (int)pwmVal);
  } else {
    // Stop / Coast
    digitalWrite(L298N_in1, LOW);
    digitalWrite(L298N_in2, LOW);
    analogWrite(L298N_enA, 0);
  }
}
