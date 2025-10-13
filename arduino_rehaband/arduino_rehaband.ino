#include <ArduinoBLE.h>
#include <Arduino_BMI270_BMM150.h>

// BLE Service and Characteristics
BLEService rehabandService("180D"); // Heart Rate service UUID (repurposed)
BLEIntCharacteristic romChar("2A37", BLERead | BLENotify);        // ROM angle
BLEFloatCharacteristic speedChar("2A38", BLERead | BLENotify);    // Speed (angular velocity)
BLEFloatCharacteristic jerkinessChar("2A39", BLERead | BLENotify);  // RMS Jerkiness value
BLEIntCharacteristic repCountChar("2A3A", BLERead | BLENotify);   // Current rep number
BLEIntCharacteristic sessionControlChar("2A3B", BLEWrite);        // Session control (start/reset)
BLEIntCharacteristic calibrationChar("2A3C", BLEWrite | BLERead | BLENotify); // Calibration control
BLEFloatCharacteristic targetROMChar("2A3D", BLEWrite);          // Target ROM from app
BLEFloatCharacteristic targetSpeedChar("2A3E", BLEWrite);        // Target speed from app

// IMU raw data
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float prevGyroY = 0;

// Advanced angle calculation variables
float pitch = 0;
float gyroAngle = 0;
float accelAngle = 0;
float filteredAngle = 0;
float baselineAngle = 0;
float kneeFlexionAngle = 0;
const float FILTER_ALPHA = 0.98; // Complementary filter coefficient
const float DT = 0.05; // 50ms sampling period (20Hz)

// Exercise tracking variables
int currentRep = 0;
unsigned long repStartTime = 0;
float repDuration = 0;

// Speed tracking variables
float currentAngularSpeed = 0;
float peakSpeed = 0;
float avgSpeed = 0;
float speedSum = 0;
int speedCount = 0;
const int SPEED_WINDOW = 5;
float speedBuffer[SPEED_WINDOW] = {0};
int speedIndex = 0;

// Jerkiness tracking variables
float rmsJerk = 0;
float jerkSum = 0;
int jerkCount = 0;
float smoothnessScore = 0;
const int JERK_WINDOW = 10;
float jerkBuffer[JERK_WINDOW] = {0};
int jerkIndex = 0;

// Calibration variables
bool isCalibrating = false;
int calibrationRep = 0;
float calibrationAngles[5] = {0};
float calibrationSpeeds[5] = {0};
float calibrationJerks[5] = {0};
float targetROM = 90.0; // Default target
float targetSpeed = 30.0; // Default angular speed target (deg/s)
float targetJerk = 100.0; // Default jerk threshold

// Squat state machine variables
int squatState = 0;         // 0: STRAIGHT, 1: SQUATTING_DOWN, 2: BOTTOM_POSITION, 3: RISING_UP
float straightAngle = 0;    // Reference angle when leg is straight
float peakSquatAngle = 0;   // Deepest squat angle achieved
float squatStartAngle = 0;  // Angle at start of squat movement
unsigned long stateStartTime = 0; // Time when current state started
unsigned long lastRepTime = 0; // Time of the last completed rep

// Squat detection thresholds (optimized for thigh-mounted IMU)
const float MIN_SQUAT_ANGLE = 30.0;    // Minimum knee flexion to count as squat
const float MOVEMENT_THRESHOLD = 15.0;  // Degrees/s to detect start of movement
const float STABLE_THRESHOLD = 5.0;     // Degrees/s to detect stable position
const int MIN_SQUAT_TIME = 800;         // Minimum squat duration (ms)
const int MAX_SQUAT_TIME = 10000;       // Maximum squat duration (ms)
const int REP_COOLDOWN = 1000;          // 1s cooldown between reps
const int STABLE_TIME = 500;            // Time to be stable before state change

// --- FUNCTION PROTOTYPES ---
void readIMU();
void calculateAdvancedAngle();
void calculateSpeed();
void calculateJerkiness();
void trackSquatMovement();
void completeSquat(unsigned long endTime);
void resetSession();
void handleCalibration();
void processCalibrationData();
float movingAverage(float buffer[], float newValue, int windowSize, int &index);
// -------------------------

void setup() {
  Serial.begin(9600);
  while (!Serial); // Wait for Serial Monitor to open

  Serial.println("Initializing IMU...");
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.println("IMU initialized.");

  Serial.println("Initializing BLE...");
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }
  Serial.println("BLE initialized.");
  
  // Set BLE device name and advertised service
  BLE.setLocalName("REHABAND");
  BLE.setAdvertisedService(rehabandService);
  
  // Add characteristics to service
  rehabandService.addCharacteristic(romChar);
  rehabandService.addCharacteristic(speedChar);
  rehabandService.addCharacteristic(jerkinessChar);
  rehabandService.addCharacteristic(repCountChar);
  rehabandService.addCharacteristic(sessionControlChar);
  rehabandService.addCharacteristic(calibrationChar);
  rehabandService.addCharacteristic(targetROMChar);
  rehabandService.addCharacteristic(targetSpeedChar);
  
  // Add service
  BLE.addService(rehabandService);
  
  // Set initial characteristic values
  romChar.writeValue(0);
  speedChar.writeValue(0.0);
  jerkinessChar.writeValue(0.0);
  repCountChar.writeValue(0);
  sessionControlChar.writeValue(0);
  calibrationChar.writeValue(0);
  targetROMChar.writeValue(90.0);
  targetSpeedChar.writeValue(30.0);
  
  // Start advertising
  BLE.advertise();
  
  Serial.println("REHABAND BLE device active, waiting for connections...");
}

void loop() {
  // Listen for BLE connections
  BLEDevice central = BLE.central();
  
  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    
    // Set initial baseline angle when connection starts
    delay(1000); // Allow IMU to stabilize
    readIMU();
    calculateAdvancedAngle();
    baselineAngle = filteredAngle;
    straightAngle = baselineAngle;
    Serial.print("Initial baseline angle set to: ");
    Serial.println(baselineAngle);
    
    // While connected, continuously read IMU and track reps
    while (central.connected()) {
      // Check for session control commands
      if (sessionControlChar.written()) {
        int command = sessionControlChar.value();
        if (command == 1) {
          // Start session command
          readIMU();
          calculateAdvancedAngle();
          baselineAngle = filteredAngle;
          straightAngle = baselineAngle;
          resetSession();
          Serial.print("Session started - new baseline angle: ");
          Serial.println(baselineAngle);
        } else if (command == 0) {
          // Reset session command
          resetSession();
          Serial.println("Session reset by user");
        }
      }
      
      // Check for calibration commands
      if (calibrationChar.written()) {
        int command = calibrationChar.value();
        if (command == 1 && !isCalibrating) {
          // Start calibration
          isCalibrating = true;
          calibrationRep = 0;
          resetSession();
          Serial.println("Calibration started - perform 5 squats");
        }
      }
      
      // Check for target value updates from app
      if (targetROMChar.written()) {
        targetROM = targetROMChar.value();
        Serial.print("Target ROM updated to: ");
        Serial.println(targetROM);
      }
      
      if (targetSpeedChar.written()) {
        targetSpeed = targetSpeedChar.value();
        Serial.print("Target Speed updated to: ");
        Serial.println(targetSpeed);
      }
      
      // Main processing loop
      readIMU();
      calculateAdvancedAngle();
      calculateSpeed();
      calculateJerkiness();
      
      if (isCalibrating) {
        handleCalibration();
      } else {
        trackSquatMovement();
      }
      
      delay(50); // 20Hz sampling rate
    }
    
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
    
    // Reset session when disconnected
    resetSession();
  }
}

void readIMU() {
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accelX, accelY, accelZ);
  }
  
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gyroX, gyroY, gyroZ);
  }
}

void calculateAdvancedAngle() {
  // Calculate pitch from accelerometer (instant measurement)
  // For thigh-mounted IMU: pitch represents knee flexion angle
  accelAngle = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;
  
  // Integrate gyroscope for smooth angle tracking
  gyroAngle += gyroY * DT; // gyroY is pitch rate
  
  // Complementary filter: combines gyro stability with accel accuracy
  filteredAngle = FILTER_ALPHA * gyroAngle + (1 - FILTER_ALPHA) * accelAngle;
  
  // Calculate knee flexion angle relative to baseline (straight leg position)
  kneeFlexionAngle = abs(filteredAngle - baselineAngle);
  
  // Ensure reasonable range (0-180 degrees)
  kneeFlexionAngle = constrain(kneeFlexionAngle, 0, 180);
  
  // Update gyro angle to prevent drift
  gyroAngle = filteredAngle;
}

void calculateSpeed() {
  // Primary method: Angular velocity from gyroscope (degrees/second)
  currentAngularSpeed = abs(gyroY);
  
  // Track peak speed during movement
  if (currentAngularSpeed > peakSpeed) {
    peakSpeed = currentAngularSpeed;
  }
  
  // Moving average smoothing for speed
  float smoothedSpeed = movingAverage(speedBuffer, currentAngularSpeed, SPEED_WINDOW, speedIndex);
  currentAngularSpeed = smoothedSpeed;
  
  // Accumulate for average calculation
  speedSum += currentAngularSpeed;
  speedCount++;
  
  // Calculate average speed
  if (speedCount > 0) {
    avgSpeed = speedSum / speedCount;
  }
}

float movingAverage(float buffer[], float newValue, int windowSize, int &index) {
  // Add new value to buffer
  buffer[index] = newValue;
  index = (index + 1) % windowSize;
  
  // Calculate average
  float sum = 0;
  for (int i = 0; i < windowSize; i++) {
    sum += buffer[i];
  }
  return sum / windowSize;
}

void calculateJerkiness() {
  // Calculate angular jerk (change in angular velocity)
  float gyroJerk = abs(gyroY - prevGyroY) / DT; // deg/s²
  
  // Store previous gyro value for next calculation
  prevGyroY = gyroY;
  
  // Accumulate jerk for RMS calculation
  jerkSum += gyroJerk * gyroJerk;
  jerkCount++;
  
  // Calculate RMS jerk (Root Mean Square)
  if (jerkCount > 0) {
    rmsJerk = sqrt(jerkSum / jerkCount);
  }
  
  // Calculate smoothness score (0-1 scale, higher = smoother)
  smoothnessScore = 1.0 / (1.0 + rmsJerk * 0.01);
  
  // Optional: Apply moving average to jerk for smoother readings
  float smoothedJerk = movingAverage(jerkBuffer, gyroJerk, JERK_WINDOW, jerkIndex);
}

void trackSquatMovement() {
  unsigned long currentTime = millis();
  
  // Use new filtered angle and speed measurements
  float currentKneeAngle = kneeFlexionAngle;
  float currentSpeed = currentAngularSpeed;
  
  switch (squatState) {
    case 0: // STRAIGHT
      // Update straight reference angle when stable
      if (currentSpeed < STABLE_THRESHOLD) {
        straightAngle = currentKneeAngle;
        // Reset peak values for new squat
        peakSquatAngle = 0;
        peakSpeed = 0;
        // Reset metrics for new squat
        speedSum = 0;
        speedCount = 0;
        jerkSum = 0;
        jerkCount = 0;
      }
      
      // Detect start of squat movement
      if (currentSpeed > MOVEMENT_THRESHOLD && 
          currentKneeAngle > (straightAngle + 10) && 
          (currentTime - lastRepTime) > REP_COOLDOWN) {
        
        squatState = 1; // SQUATTING_DOWN
        repStartTime = currentTime;
        stateStartTime = currentTime;
        squatStartAngle = currentKneeAngle;
        
        Serial.print("Squat started - Straight angle: ");
        Serial.print(straightAngle);
        Serial.print("°, Current angle: ");
        Serial.println(currentKneeAngle);
      }
      break;

    case 1: // SQUATTING_DOWN
      // Track deepest squat angle
      if (currentKneeAngle > peakSquatAngle) {
        peakSquatAngle = currentKneeAngle;
      }
      
      // Detect when reaching bottom position (movement slows down)
      // Use dynamic threshold during calibration
      float minAngleForBottom = isCalibrating ? (targetROM * 0.6) : MIN_SQUAT_ANGLE;
      
      if (currentSpeed < STABLE_THRESHOLD && 
          (currentTime - stateStartTime) > STABLE_TIME &&
          peakSquatAngle > minAngleForBottom) {
        
        squatState = 2; // BOTTOM_POSITION
        stateStartTime = currentTime;
        
        Serial.print("Bottom position reached - Peak angle: ");
        Serial.print(peakSquatAngle);
        Serial.print("°, ROM: ");
        Serial.println(peakSquatAngle - straightAngle);
      }
      
      // Timeout protection
      if ((currentTime - repStartTime) > MAX_SQUAT_TIME) {
        Serial.println("Squat timeout during down phase");
        squatState = 0; // Reset to STRAIGHT
      }
      break;

    case 2: // BOTTOM_POSITION
      // Wait for upward movement to start
      if (currentSpeed > MOVEMENT_THRESHOLD && 
          currentKneeAngle < peakSquatAngle &&
          (currentTime - stateStartTime) > STABLE_TIME) {
        
        squatState = 3; // RISING_UP
        stateStartTime = currentTime;
        
        Serial.println("Rising phase started");
      }
      
      // Timeout protection
      if ((currentTime - stateStartTime) > (STABLE_TIME * 4)) {
        Serial.println("Bottom position timeout");
        squatState = 3; // Force to rising
      }
      break;

    case 3: // RISING_UP
      // Detect return to straight position
      if (currentSpeed < STABLE_THRESHOLD && 
          currentKneeAngle <= (straightAngle + 15) &&
          (currentTime - stateStartTime) > STABLE_TIME) {
        
        // Check if squat meets criteria (dynamic thresholds during calibration)
        float minAngleRequired = isCalibrating ? (targetROM * 0.8) : MIN_SQUAT_ANGLE;
        
        if (peakSquatAngle >= minAngleRequired && 
            (currentTime - repStartTime) >= MIN_SQUAT_TIME) {
          
          completeSquat(currentTime);
          squatState = 0; // Back to STRAIGHT
        } else {
          if (isCalibrating) {
            Serial.print("Calibration squat rejected - ROM ");
            Serial.print(peakSquatAngle);
            Serial.print("° < required ");
            Serial.println(minAngleRequired);
          } else {
            Serial.println("Insufficient squat - not counting");
          }
          squatState = 0;
        }
      }
      
      // Timeout protection
      if ((currentTime - repStartTime) > MAX_SQUAT_TIME) {
        Serial.println("Squat timeout during rising phase");
        squatState = 0;
      }
      break;
  }
  
  // Debug output every 1 second
  static unsigned long lastDebug = 0;
  if (currentTime - lastDebug > 1000) {
    Serial.print("State: ");
    Serial.print(squatState);
    Serial.print(", Knee Angle: ");
    Serial.print(currentKneeAngle);
    Serial.print("°, Speed: ");
    Serial.print(currentSpeed);
    Serial.print("°/s, RMS Jerk: ");
    Serial.println(rmsJerk);
    lastDebug = currentTime;
  }
}

void completeSquat(unsigned long endTime) {
  currentRep++;
  lastRepTime = endTime;
  
  // Calculate squat metrics with advanced measurements
  repDuration = (endTime - repStartTime) / 1000.0; // Convert to seconds
  int romAngle = (int)(peakSquatAngle - straightAngle); // ROM from straight to peak squat
  
  // Calculate average speed during the squat
  float avgSquatSpeed = avgSpeed;
  if (speedCount > 0) {
    avgSquatSpeed = speedSum / speedCount;
  }
  
  // Use RMS jerk value calculated during movement
  float squatJerkiness = rmsJerk;
  
  // Handle calibration mode with validation
  if (isCalibrating && calibrationRep < 5) {
    // Validate calibration squat against user's settings
    bool validROM = romAngle >= (targetROM * 0.8); // At least 80% of target ROM
    bool validSpeed = avgSquatSpeed <= (targetSpeed * 1.5); // Within 150% of target speed
    
    if (validROM && validSpeed) {
      // Accept this calibration squat
      calibrationAngles[calibrationRep] = romAngle;
      calibrationSpeeds[calibrationRep] = avgSquatSpeed;
      calibrationJerks[calibrationRep] = squatJerkiness;
      calibrationRep++;
      
      Serial.print("✓ Calibration squat ");
      Serial.print(calibrationRep);
      Serial.print("/5 ACCEPTED - ROM: ");
      Serial.print(romAngle);
      Serial.print("° (req: ");
      Serial.print(targetROM * 0.8, 1);
      Serial.print("°), Speed: ");
      Serial.print(avgSquatSpeed);
      Serial.print("°/s (max: ");
      Serial.print(targetSpeed * 1.5, 1);
      Serial.print("°/s), Jerk: ");
      Serial.println(squatJerkiness);
    } else {
      // Reject this calibration squat
      Serial.print("✗ Calibration squat REJECTED - ");
      if (!validROM) {
        Serial.print("ROM too low: ");
        Serial.print(romAngle);
        Serial.print("° < ");
        Serial.print(targetROM * 0.8, 1);
        Serial.print("°");
      }
      if (!validSpeed) {
        if (!validROM) Serial.print(", ");
        Serial.print("Speed too fast: ");
        Serial.print(avgSquatSpeed);
        Serial.print("°/s > ");
        Serial.print(targetSpeed * 1.5, 1);
        Serial.print("°/s");
      }
      Serial.println(" - Try again with better form!");
      return; // Don't send BLE data for rejected squats
    }
    
    // Send calibration progress to app via BLE
    romChar.writeValue(romAngle);
    speedChar.writeValue(avgSquatSpeed);
    jerkinessChar.writeValue(squatJerkiness);
    repCountChar.writeValue(calibrationRep); // Send current calibration count
    
    // Send calibration progress signal (1 = in progress, rep count in repCountChar)
    calibrationChar.writeValue(1);
    
    if (calibrationRep >= 5) {
      processCalibrationData();
    }
    return; // Exit after sending calibration data
  }
  
  // Send data via BLE (using new advanced metrics)
  romChar.writeValue(romAngle);
  speedChar.writeValue(avgSquatSpeed); // Send average angular speed instead of duration
  jerkinessChar.writeValue(squatJerkiness); // Send RMS jerk value
  repCountChar.writeValue(currentRep);
  
  // Debug output
  Serial.print("Squat ");
  Serial.print(currentRep);
  Serial.print(" completed - ROM: ");
  Serial.print(romAngle);
  Serial.print("° (straight: ");
  Serial.print(straightAngle);
  Serial.print("°, peak: ");
  Serial.print(peakSquatAngle);
  Serial.print("°), Avg Speed: ");
  Serial.print(avgSquatSpeed);
  Serial.print("°/s, RMS Jerk: ");
  Serial.print(squatJerkiness);
  Serial.print(", Smoothness: ");
  Serial.println(smoothnessScore);
  
  // Stop after 10 reps (optional - can be removed if not desired)
  if (currentRep >= 10) {
    Serial.println("Session complete - 10 squats finished");
    delay(5000); // Wait 5 seconds before allowing new session
    resetSession();
  }
}

void resetSession() {
  currentRep = 0;
  squatState = 0; // Back to STRAIGHT
  lastRepTime = 0;
  
  // Reset speed tracking
  speedSum = 0;
  speedCount = 0;
  avgSpeed = 0;
  peakSpeed = 0;
  
  // Reset jerkiness tracking
  jerkSum = 0;
  jerkCount = 0;
  rmsJerk = 0;
  smoothnessScore = 1.0;
  
  // Reset squat-specific variables
  peakSquatAngle = 0;
  squatStartAngle = 0;
  
  // Reset BLE characteristics
  romChar.writeValue(0);
  speedChar.writeValue(0.0);
  jerkinessChar.writeValue(0.0);
  repCountChar.writeValue(0);
  
  Serial.println("Session reset");
}

void handleCalibration() {
  // During calibration, track squats normally but don't send BLE data
  // The completeSquat function handles calibration data collection
  trackSquatMovement();
}

void processCalibrationData() {
  // Calculate averages from 5 calibration squats
  float avgCalibrationROM = 0;
  float avgCalibrationSpeed = 0;
  float avgCalibrationJerk = 0;
  
  for (int i = 0; i < 5; i++) {
    avgCalibrationROM += calibrationAngles[i];
    avgCalibrationSpeed += calibrationSpeeds[i];
    avgCalibrationJerk += calibrationJerks[i];
  }
  
  avgCalibrationROM /= 5.0;
  avgCalibrationSpeed /= 5.0;
  avgCalibrationJerk /= 5.0;
  
  // Update target values based on calibration
  targetROM = avgCalibrationROM * 0.9; // Set target to 90% of calibrated average
  targetSpeed = avgCalibrationSpeed * 1.1; // Allow 10% slower than average
  targetJerk = avgCalibrationJerk * 1.2; // Allow 20% more jerk than average
  
  // Send calibration results back to app via BLE
  // Use a special calibration complete signal (value = 2)
  calibrationChar.writeValue(2);
  
  // Also update the target characteristics
  targetROMChar.writeValue(targetROM);
  targetSpeedChar.writeValue(targetSpeed);
  
  // Reset calibration state
  isCalibrating = false;
  calibrationRep = 0;
  
  Serial.println("=== Calibration Complete ===");
  Serial.print("Average ROM: ");
  Serial.print(avgCalibrationROM);
  Serial.print("° → Target ROM: ");
  Serial.println(targetROM);
  Serial.print("Average Speed: ");
  Serial.print(avgCalibrationSpeed);
  Serial.print("°/s → Target Speed: ");
  Serial.println(targetSpeed);
  Serial.print("Average Jerk: ");
  Serial.print(avgCalibrationJerk);
  Serial.print(" → Target Jerk: ");
  Serial.println(targetJerk);
  Serial.println("=============================");
  
  // Reset session for normal use
  resetSession();
}