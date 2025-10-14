#include <ArduinoBLE.h>
#include <Arduino_BMI270_BMM150.h>

// BLE Service and Characteristics
BLEService rehabandService("180D"); // Heart Rate service UUID (repurposed)
BLEIntCharacteristic romChar("2A37", BLERead | BLENotify);      // ROM angle
BLEFloatCharacteristic speedChar("2A38", BLERead | BLENotify);    // Speed (seconds per rep)
BLEIntCharacteristic jerkinessChar("2A39", BLERead | BLENotify);  // Jerkiness level (0-2)
BLEIntCharacteristic repCountChar("2A3A", BLERead | BLENotify);   // Current rep number
BLEIntCharacteristic sessionControlChar("2A3B", BLEWrite);        // Session control
BLEIntCharacteristic calibrationChar("2A3C", BLEWrite | BLERead | BLENotify); // Calibration control
BLEFloatCharacteristic targetROMChar("2A3D", BLEWrite);           // Target ROM setting
BLEFloatCharacteristic targetSpeedChar("2A3E", BLEWrite);         // Target speed setting
BLEIntCharacteristic targetRepsChar("2A3F", BLEWrite);            // Target reps setting

// IMU and exercise tracking variables
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float pitch, roll;
int currentRep = 0;
unsigned long repStartTime = 0;
float jerkinessSum = 0;
int jerkinessCount = 0;
float alpha = 0.98; // Filter coefficient (higher value trusts the gyroscope more)
unsigned long lastTime = 0;
// ---------------------------------------------------------


// State machine and angle tracking variables
int deviceState = 0;        // 0: IDLE (connected but not active), 1: READY (session started), 2: CALIBRATING
int exerciseState = 0;      // 0: WAITING, 1: BENDING, 2: STRAIGHTENING
float startBendAngle = 0;   // Angle at the start of the bend
float peakBendAngle = 0;    // Lowest angle achieved during the bend
unsigned long bendEndTime = 0;  // NEW: Track when bending phase ends

// New variables for robust rep counting
float lastStableAngle = 0;   // Stores the angle when the leg is at rest
unsigned long lastRepTime = 0; // Time of the last completed rep
const int REP_COOLDOWN = 500;  // 500ms cooldown between reps to prevent double counting

// Calibration variables
bool isCalibrating = false;
int calibrationRep = 0;
float calibrationAngles[5] = {0};
float calibrationSpeeds[5] = {0};
float targetROM = 90.0;   // Default - will be updated from app settings
float targetSpeed = 30.0; // Default - will be updated from app settings
int targetReps = 10;      // Default - will be updated from app settings

// Calibration and thresholds - made more sensitive
const float ANGLE_THRESHOLD = 25.0;  // Minimum angle change to count as rep (reduced from 30)
const float GYRO_THRESHOLD = 20.0;   // More sensitive movement detection (reduced from 30)
const int MIN_REP_TIME = 1000;       // Minimum rep time in milliseconds
const int MAX_REP_TIME = 10000;      // Increased - allow longer reps

// --- FUNCTION PROTOTYPES ---
void readIMU();
void trackExercise();
void completeRep(unsigned long endTime);
void resetSession();
// -------------------------

void setup() {
  Serial.begin(9600);
  while (!Serial); // Wait for Serial Monitor to open

  Serial.println("====================================");
  Serial.println("   REHABAND Smart Squat Tracker");
  Serial.println("====================================");
  Serial.println("Initializing IMU sensor...");
  if (!IMU.begin()) {
    Serial.println("[ERROR] FAILED to initialize IMU!");
    Serial.println("Please check connections and try again.");
    while (1);
  }
  Serial.println("[OK] IMU sensor initialized successfully");

  Serial.println("Initializing Bluetooth...");
  if (!BLE.begin()) {
    Serial.println("[ERROR] FAILED to start Bluetooth!");
    Serial.println("Please reset Arduino and try again.");
    while (1);
  }
  Serial.println("[OK] Bluetooth initialized successfully");
  
  BLE.setLocalName("REHABAND");
  BLE.setAdvertisedService(rehabandService);
  
  rehabandService.addCharacteristic(romChar);
  rehabandService.addCharacteristic(speedChar);
  rehabandService.addCharacteristic(jerkinessChar);
  rehabandService.addCharacteristic(repCountChar);
  rehabandService.addCharacteristic(sessionControlChar);
  rehabandService.addCharacteristic(calibrationChar);
  rehabandService.addCharacteristic(targetROMChar);
  rehabandService.addCharacteristic(targetSpeedChar);
  rehabandService.addCharacteristic(targetRepsChar);
  
  BLE.addService(rehabandService);
  
  romChar.writeValue(0);
  speedChar.writeValue(0.0);
  jerkinessChar.writeValue(0);
  repCountChar.writeValue(0);
  sessionControlChar.writeValue(0);
  calibrationChar.writeValue(0);
  targetROMChar.writeValue(90.0);   // Match HTML default
  targetSpeedChar.writeValue(30.0);
  targetRepsChar.writeValue(10);
  
  BLE.advertise();
  
  Serial.println("[BLE] REHABAND now advertising as 'REHABAND'");
  Serial.println("[INFO] Open your web app and click 'Connect to REHABAND'");
  Serial.println("[INFO] Ensure device is mounted 3cm above kneecap on thigh");
  Serial.println("------------------------------------");
  Serial.println("USAGE TIPS:");
  Serial.println("   1. Always calibrate first (5 perfect squats)");
  Serial.println("   2. Start session to begin tracking reps");
  Serial.println("   3. Monitor this window for real-time feedback");
  Serial.println("====================================");
  Serial.println("Waiting for connection...");
}

void loop() {
  BLEDevice central = BLE.central();
  
  if (central) {
    Serial.println("[CONNECT] CONNECTED to web app!");
    Serial.print("[INFO] Device MAC: ");
    Serial.println(central.address());
    Serial.println("[STATUS] Device IDLE - Click 'Calibrate' or 'Start Session' to begin");
    
    while (central.connected()) {
      // Handle session control
      if (sessionControlChar.written()) {
        int command = sessionControlChar.value();
        if (command == 1) {
          // Start session - reset everything and set device to READY
          deviceState = 1; // READY for session
          resetSession();
          Serial.println("[SESSION] SESSION STARTED!");
          Serial.println("[INFO] Current angle set as reference (0 degrees)");
          Serial.println("[INFO] Begin squatting - reps will be tracked automatically");
          Serial.println("------------------------------------");
        } else if (command == 0) {
          // Reset session - go back to IDLE
          deviceState = 0; // IDLE
          resetSession();
          Serial.println("[SESSION] SESSION RESET - Device now IDLE, ready for calibration or session");
        }
      }
      
      // Handle calibration
      if (calibrationChar.written()) {
        int command = calibrationChar.value();
        if (command == 1 && !isCalibrating) {
          deviceState = 2; // CALIBRATING
          isCalibrating = true;
          calibrationRep = 0;
          resetSession();
          Serial.println("[CALIBRATION] === CALIBRATION MODE ACTIVATED ===");
          Serial.println("[INSTRUCTIONS]:");
          Serial.println("   - Perform 5 squats with PERFECT form");
          Serial.println("   - Use your maximum comfortable depth");
          Serial.println("   - Maintain steady, controlled pace");
          Serial.println("   - Keep device position consistent");
          Serial.println("[START] Begin your first calibration squat now!");
          Serial.println("=====================================");
        } else if (command == 0 && isCalibrating) {
          // Stop calibration - go back to IDLE
          deviceState = 0; // IDLE
          isCalibrating = false;
          calibrationRep = 0;
          resetSession();
          Serial.println("[STOP] CALIBRATION STOPPED manually");
          Serial.println("[STATUS] Device now IDLE, ready for new calibration or session");
          Serial.println("=====================================");
        }
      }
      
      // Handle target ROM updates
      if (targetROMChar.written()) {
        targetROM = targetROMChar.value();
        Serial.print("[SETTING] Target ROM updated: ");
        Serial.print(targetROM);
        Serial.println(" degrees (knee flexion depth)");
      }
      
      // Handle target speed updates  
      if (targetSpeedChar.written()) {
        targetSpeed = targetSpeedChar.value();
        Serial.print("[SETTING] Target Time updated: ");
        Serial.print(targetSpeed);
        Serial.println("s (per rep duration)");
      }
      
      // Handle target reps updates
      if (targetRepsChar.written()) {
        targetReps = targetRepsChar.value();
        Serial.print("[SETTING] Target Reps updated: ");
        Serial.print(targetReps);
        Serial.println(" (per session)");
      }
      
      readIMU();
      
      // Only track exercise when device is active (not IDLE)
      if (deviceState > 0) {
        trackExercise();
      }
      
      delay(50);
    }
    
    Serial.println("[DISCONNECT] DISCONNECTED from web app");
    Serial.print("[INFO] Device was: ");
    Serial.println(central.address());
    Serial.println("[RESET] Session reset - ready for reconnection");
    Serial.println("[BLE] Advertising as 'REHABAND' again...");
    
    // Reset session and device state when device disconnects
    deviceState = 0; // Back to IDLE
    resetSession();
  }
}

void readIMU() {
  // Get the current time and calculate the time difference (dt)
  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0; // dt in seconds
  lastTime = currentTime;

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    // Read the latest sensor data
    IMU.readAcceleration(accelX, accelY, accelZ);
    IMU.readGyroscope(gyroX, gyroY, gyroZ);

    // Calculate the pitch angle from the accelerometer (long-term stability)
    float accelPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;

    // Integrate the gyroscope data to get the change in angle (short-term accuracy)
    // We use gyroY for pitch, as it measures rotation around the Y-axis.
    pitch = alpha * (pitch + gyroY * dt) + (1 - alpha) * (accelPitch);
  }
  
  // You no longer need to calculate roll unless you want to use it
  // You also don't need abs(pitch) anymore unless your logic requires it
  
  // The auto-calibration part remains the same
  if (lastStableAngle == 0 && pitch != 0) {
    lastStableAngle = pitch;
  }
}


void trackExercise() {
  float currentAngle = pitch;
  unsigned long currentTime = millis();
  float totalGyro = sqrt(gyroX * gyroX + gyroY * gyroY);

  switch (exerciseState) {
    case 0: // IDLE
      if (totalGyro < GYRO_THRESHOLD / 2) {
        lastStableAngle = currentAngle;
      }
      
      if (totalGyro > GYRO_THRESHOLD && (currentTime - lastRepTime) > REP_COOLDOWN) {
        exerciseState = 1;
        repStartTime = currentTime;
        startBendAngle = lastStableAngle;
        peakBendAngle = lastStableAngle;
        jerkinessSum = totalGyro;
        jerkinessCount = 1;
        if (isCalibrating) {
          Serial.print("[CAL] Rep ");
          Serial.print(calibrationRep + 1);
          Serial.print("/5 - Starting squat from ");
          Serial.print(startBendAngle);
          Serial.println(" degrees");
        } else {
          Serial.print("[REP] Rep ");
          Serial.print(currentRep + 1);
          Serial.print("/");
          Serial.print(targetReps);
          Serial.print(" - Starting squat from ");
          Serial.print(startBendAngle);
          Serial.println(" degrees");
        }
      }
      break;

    case 1: // BENDING
      jerkinessSum += totalGyro;
      jerkinessCount++;

      if (abs(currentAngle - startBendAngle) > abs(peakBendAngle - startBendAngle)) {
        peakBendAngle = currentAngle;
      }
      
      // Detect end of bending phase - more sensitive detection
      if (totalGyro < GYRO_THRESHOLD / 2 && (currentTime - repStartTime) > 600) {
        if (abs(peakBendAngle - startBendAngle) > ANGLE_THRESHOLD) {
          exerciseState = 2;
          bendEndTime = currentTime;  // Record when bending ends
          Serial.print("[SQUAT] Peak bend: ");
          Serial.print(abs(peakBendAngle - startBendAngle));
          Serial.println(" degrees - Now straightening back up...");
        }
      }

      if ((currentTime - repStartTime) > MAX_REP_TIME) {
        Serial.println("Rep timeout during bend - resetting");
        resetSession();
      }
      break;

    case 2: // STRAIGHTENING
      jerkinessSum += totalGyro;
      jerkinessCount++;

      // More sensitive straightening detection
      // Check if we're close to starting position (within 20 degrees) AND movement has slowed down
      float angleDiff = abs(currentAngle - startBendAngle);
      if (totalGyro < GYRO_THRESHOLD && angleDiff < 20) {
        Serial.print("[COMPLETE] Rep completed - back to ");
        Serial.print(currentAngle);
        Serial.print(" degrees (diff: ");
        Serial.print(angleDiff);
        Serial.println(")");
        completeRep(currentTime);
        exerciseState = 0;
      }

      if ((currentTime - repStartTime) > MAX_REP_TIME) {
        Serial.println("Rep timeout during straighten - resetting");
        resetSession();
      }
      break;
  }
}

void completeRep(unsigned long endTime) {
  currentRep++;
  lastRepTime = endTime;
  
  float repDuration = (endTime - repStartTime) / 1000.0;  // Full rep time from start to finish
  int romAngle = (int)abs(peakBendAngle - startBendAngle);
  
  float avgJerkiness = jerkinessSum / jerkinessCount;
  int jerkinessLevel;
  if (avgJerkiness < 100) jerkinessLevel = 0;
  else if (avgJerkiness < 200) jerkinessLevel = 1;
  else jerkinessLevel = 2;
  
  // Handle calibration logic
  if (isCalibrating && calibrationRep < 5) {
    bool validROM = romAngle >= (targetROM * 0.8);  // 80% of target ROM
    bool validTime = abs(repDuration - targetSpeed) <= 1.5;  // Within 1.5 seconds of target time
    
    if (validROM) {  // For now, just check ROM - time validation can be improved
      calibrationAngles[calibrationRep] = romAngle;
      calibrationSpeeds[calibrationRep] = repDuration;
      calibrationRep++;
      
      Serial.print("[ACCEPT] Calibration ");
      Serial.print(calibrationRep);
      Serial.print("/5 ACCEPTED:");
      Serial.print(" ROM: ");
      Serial.print(romAngle);
      Serial.print(" deg, Time: ");
      Serial.print(repDuration, 1);
      Serial.print("s");
      if (calibrationRep < 5) {
        Serial.println(" - Continue with next squat");
      } else {
        Serial.println(" - Final calibration squat!");
      }
      
      calibrationChar.writeValue(1); // Progress signal
    } else {
      Serial.print("[REJECT] Calibration REJECTED:");
      Serial.print(" ROM: ");
      Serial.print(romAngle);
      Serial.print(" deg (need >=");
      Serial.print(targetROM * 0.8, 1);
      Serial.println(" deg) - Try deeper squat");
    }
    
    if (calibrationRep >= 5) {
      // Calculate averages
      float avgROM = 0, avgSpeed = 0;
      for (int i = 0; i < 5; i++) {
        avgROM += calibrationAngles[i];
        avgSpeed += calibrationSpeeds[i];
      }
      avgROM /= 5.0;
      avgSpeed /= 5.0;
      
      Serial.println("[COMPLETE] === CALIBRATION COMPLETE! ===");
      Serial.print("[TARGETS] Your Personal Targets:");
      Serial.print(" ROM: ");
      Serial.print(avgROM, 1);
      Serial.print(" deg, Time: ");
      Serial.print(avgSpeed, 1);
      Serial.println("s");
      Serial.println("[INFO] These are now your 'Good' standards!");
      Serial.println("[READY] Ready to start a session - reps will be compared to these targets");
      Serial.println("=====================================");
      
      calibrationChar.writeValue(2); // Complete signal
      isCalibrating = false;
      calibrationRep = 0;
      deviceState = 0; // Back to IDLE after calibration
      
      // Send the calibration values to the app via target characteristics
      targetROMChar.writeValue(avgROM);
      targetSpeedChar.writeValue(avgSpeed);
      
      resetSession();
      return;
    }
  }
  
  romChar.writeValue(romAngle);
  speedChar.writeValue(repDuration);
  jerkinessChar.writeValue(jerkinessLevel);
  repCountChar.writeValue(currentRep);
  
  // Determine rep quality for user feedback
  String quality = "Good";
  if (romAngle < targetROM * 0.8) quality = "Shallow";
  else if (abs(repDuration - targetSpeed) > 1.0) quality = (repDuration > targetSpeed) ? "Slow" : "Fast";
  else if (jerkinessLevel > 1) quality = "Jerky";
  
  Serial.print("[DONE] Rep ");
  Serial.print(currentRep);
  Serial.print("/");
  Serial.print(targetReps);
  Serial.print(" COMPLETED - ");
  Serial.print(quality);
  Serial.print(" (ROM: ");
  Serial.print(romAngle);
  Serial.print(" deg, Time: ");
  Serial.print(repDuration, 1);
  Serial.print("s)");
  
  // Check if target reps reached
  if (currentRep >= targetReps) {
    Serial.println("");
    Serial.println("[SUCCESS] === SESSION COMPLETE! ===");
    Serial.print("[ACHIEVED] Target achieved: ");
    Serial.print(targetReps);
    Serial.println(" reps completed");
    Serial.println("[INFO] Check your web app for detailed results");
    Serial.println("[READY] Ready for new session or calibration");
    Serial.println("===============================");
  } else {
    Serial.print(" - ");
    Serial.print(targetReps - currentRep);
    Serial.println(" more to go!");
  }
}

void resetSession() {
  currentRep = 0;
  jerkinessSum = 0;
  jerkinessCount = 0;
  
  lastStableAngle = 0;
  lastRepTime = 0;
  exerciseState = 0;
  
  romChar.writeValue(0);
  speedChar.writeValue(0.0);
  jerkinessChar.writeValue(0);
  repCountChar.writeValue(0);
  
  Serial.println("[CLEAR] Session data cleared - ready for new session");
}