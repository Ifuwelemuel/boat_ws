// --- PIN DEFINITIONS ---
// LEFT MOTOR (Motor A)
const int PIN_EN_L  = 9;    // PWM
const int PIN_IN1_L = 12;
const int PIN_IN2_L = 13;
const int PIN_ENC_L = 2;    // Interrupt Pin (Uno: 2 or 3)

// RIGHT MOTOR (Motor B) - NEW
const int PIN_EN_R  = 10;   // PWM
const int PIN_IN3_R = 7;
const int PIN_IN4_R = 8;
const int PIN_ENC_R = 3;    // Interrupt Pin (Uno: 2 or 3)

// --- CONSTANTS ---
const float MAX_MOTOR_RPM = 100.0; 
const int   MAX_PWM       = 255; 
const float ENCODER_TICKS_PER_REV = 2172.0;

// --- PID CONSTANTS (Shared for both motors) ---
float Kp = 2.0; 
float Ki = 5.0; 
float Kd = 0.0; 

// --- VARIABLES: LEFT ---
volatile long encoderCount_L = 0;
long lastEncoderCount_L = 0;
float actualRPM_L = 0.0;
//PID PARAM
float commandedRPM_L = 0.0;
float integralSum_L = 0.0;
float lastError_L = 0.0;

// --- VARIABLES: RIGHT ---
volatile long encoderCount_R = 0;
long lastEncoderCount_R = 0;
float actualRPM_R = 0.0;
//PID PARAM
float commandedRPM_R = 0.0;
float integralSum_R = 0.0;
float lastError_R = 0.0;

// Shared Timing
unsigned long lastSpeedTimeMs = 0;

// --- INTERRUPT SERVICE ROUTINES ---
void encoderISR_L() { encoderCount_L++; }
void encoderISR_R() { encoderCount_R++; }

void setup() {
  // --- SETUP LEFT MOTOR ---
  pinMode(PIN_EN_L, OUTPUT);
  pinMode(PIN_IN1_L, OUTPUT);
  pinMode(PIN_IN2_L, OUTPUT);
  pinMode(PIN_ENC_L, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L), encoderISR_L, CHANGE);

  // --- SETUP RIGHT MOTOR ---
  pinMode(PIN_EN_R, OUTPUT);
  pinMode(PIN_IN3_R, OUTPUT);
  pinMode(PIN_IN4_R, OUTPUT);
  pinMode(PIN_ENC_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R), encoderISR_R, CHANGE);

  Serial.begin(115200);
  Serial.println("System Ready: Dual Motor Control.");
  Serial.println("Send Command: CMD <LeftRPM> <RightRPM>");
  
  lastSpeedTimeMs = millis();
}

void loop() {
  // 1. READ SERIAL COMMANDS
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    // Format expected: "CMD 50 50"
    if (input.startsWith("CMD ")) {
      // Remove "CMD " prefix
      String params = input.substring(4);
      
      // Find space separator
      int spaceIndex = params.indexOf(' ');
      
      if (spaceIndex > 0) {
        // Parse Left
        String leftStr = params.substring(0, spaceIndex);
        float targetL = leftStr.toFloat();
        
        // Parse Right
        String rightStr = params.substring(spaceIndex + 1);
        float targetR = rightStr.toFloat();
          //Serial.print("commandedRPM_L: ");Serial.print(targetL);
          //Serial.print(" commandedRPM_R: ");Serial.print(targetR);

        // Update Global Targets
        commandedRPM_L = constrainTarget(targetL);
        commandedRPM_R = constrainTarget(targetR);
      }
      else {
        Serial.println("Error: Invalid Format. Use 'CMD <Left> <Right>'");
      }
    }
  }

    ///----------------------------------- PID LOOP (Runs every 100ms)---------------------
    unsigned long nowMs = millis();
    unsigned long dtMs = nowMs - lastSpeedTimeMs;

    if (dtMs >= 100) { 
      float dtSec = dtMs / 1000.0;

      // --- CONTROL LEFT MOTOR ---
      float output_L = calculatePID(
        encoderCount_L, lastEncoderCount_L, commandedRPM_L, 
        integralSum_L, lastError_L, actualRPM_L, dtSec
      );

      runMotorDriver(PIN_EN_L, PIN_IN1_L, PIN_IN2_L, output_L);

      // --- CONTROL RIGHT MOTOR ---
      float output_R = calculatePID(
        encoderCount_R, lastEncoderCount_R, commandedRPM_R, 
        integralSum_R, lastError_R, actualRPM_R, dtSec
      );
      runMotorDriver(PIN_EN_R, PIN_IN3_R, PIN_IN4_R, output_R);

      // --- DEBUG PRINT ---
      Serial.print("L_Tgt: "); Serial.print(commandedRPM_L);
      Serial.print(" L_Act: "); Serial.print(actualRPM_L);
      Serial.print(" | R_Tgt: "); Serial.print(commandedRPM_R);
      Serial.print(" R_Act: "); Serial.println(actualRPM_R);

      // Update history for next loop
      lastEncoderCount_L = encoderCount_L;
      lastEncoderCount_R = encoderCount_R;
      lastSpeedTimeMs = nowMs;
    }
  }

  // --- HELPER: PID CALCULATION ---
  // Passed by reference (&) to update state variables (integral, error, actualRPM)
  float calculatePID(long currentTicks, long lastTicks, float targetRPM, 
                      float &integralTerm, float &lastErrorTerm, float &measuredRPM, float dt) {
    // Calculate Speed
    long deltaTicks = currentTicks - lastTicks;
    float ticksPerSec = deltaTicks / dt;
    measuredRPM = (ticksPerSec / ENCODER_TICKS_PER_REV) * 60.0;

    // PID
    float error = targetRPM - measuredRPM;
    
    // P
    float P = Kp * error;

    // I
    integralTerm += (error * dt);
    if (integralTerm >  100) integralTerm =  100; // Anti-windup
    if (integralTerm < -100) integralTerm = -100;
    float I = Ki * integralTerm;

    // D
    float D = Kd * ((error - lastErrorTerm) / dt);
    lastErrorTerm = error;

    return P + I + D;
  }

  // --- HELPER: MOTOR DRIVER ---
  void runMotorDriver(int pinPWM, int pinIn1, int pinIn2, float pidOutput) {
    
    // Convert PID output (RPM-ish) to PWM
    int pwmValue = (int)((pidOutput / MAX_MOTOR_RPM) * MAX_PWM);

    // Safety Clamps
    if (pwmValue < 0) pwmValue = 0; 
    if (pwmValue > MAX_PWM) pwmValue = MAX_PWM;

    if (pwmValue > 0) {
      digitalWrite(pinIn1, HIGH);
      digitalWrite(pinIn2, LOW);
      analogWrite(pinPWM, pwmValue);
    } else {
      digitalWrite(pinIn1, LOW);
      digitalWrite(pinIn2, LOW);
      analogWrite(pinPWM, 0);
    }
  }

  //--- HELPER: INPUT SAFETY ---
  float constrainTarget(float input) {
    if (input < 0) return 0;
    if (input > MAX_MOTOR_RPM) return MAX_MOTOR_RPM;
    return input;
  }

