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
int exerciseState = 0;      // 0: IDLE, 1: BENDING, 2: STRAIGHTENING
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

// Calibration and thresholds
const float ANGLE_THRESHOLD = 30.0;  // Minimum angle change to count as rep
const float GYRO_THRESHOLD = 30.0;   // Reduced - more sensitive movement detection
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
  
  BLE.addService(rehabandService);
  
  romChar.writeValue(0);
  speedChar.writeValue(0.0);
  jerkinessChar.writeValue(0);
  repCountChar.writeValue(0);
  sessionControlChar.writeValue(0);
  calibrationChar.writeValue(0);
  targetROMChar.writeValue(90.0);   // Match HTML default
  targetSpeedChar.writeValue(30.0);
  
  BLE.advertise();
  
  Serial.println("REHABAND BLE device active, waiting for connections...");
}

void loop() {
  BLEDevice central = BLE.central();
  
  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    
    while (central.connected()) {
      // Handle session control
      if (sessionControlChar.written()) {
        int command = sessionControlChar.value();
        if (command == 1) {
          // Start session - reset everything
          resetSession();
          Serial.println("✓ Session started");
        } else if (command == 0) {
          resetSession();
          Serial.println("✓ Session reset");
        }
      }
      
      // Handle calibration
      if (calibrationChar.written()) {
        int command = calibrationChar.value();
        if (command == 1 && !isCalibrating) {
          isCalibrating = true;
          calibrationRep = 0;
          resetSession();
          Serial.println("=== CALIBRATION STARTED ===");
          Serial.println("Perform 5 controlled squats");
        }
      }
      
      // Handle target ROM updates
      if (targetROMChar.written()) {
        targetROM = targetROMChar.value();
        Serial.print("Target ROM updated: ");
        Serial.println(targetROM);
      }
      
      // Handle target speed updates  
      if (targetSpeedChar.written()) {
        targetSpeed = targetSpeedChar.value();
        Serial.print("Target Speed updated: ");
        Serial.println(targetSpeed);
      }
      
      readIMU();
      trackExercise();
      delay(50);
    }
    
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
    
    // The session is now reset only when the central device disconnects.
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
        Serial.println("Starting Bend...");
      }
      break;

    case 1: // BENDING
      jerkinessSum += totalGyro;
      jerkinessCount++;

      if (abs(currentAngle - startBendAngle) > abs(peakBendAngle - startBendAngle)) {
        peakBendAngle = currentAngle;
      }
      
      // More conservative bend ending - require lower movement AND minimum time
      if (totalGyro < GYRO_THRESHOLD / 3 && (currentTime - repStartTime) > 800) {
        if (abs(peakBendAngle - startBendAngle) > ANGLE_THRESHOLD) {
          exerciseState = 2;
          bendEndTime = currentTime;  // NEW: Record when bending ends
          Serial.print("Bend complete after ");
          Serial.print((bendEndTime - repStartTime) / 1000.0, 1);
          Serial.println("s. Now straightening...");
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

      if (totalGyro < GYRO_THRESHOLD / 2 && abs(currentAngle - startBendAngle) < 15) {
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
  
  float bendDuration = (bendEndTime - repStartTime) / 1000.0;  // Only bending phase time
  int romAngle = (int)abs(peakBendAngle - startBendAngle);
  
  float avgJerkiness = jerkinessSum / jerkinessCount;
  int jerkinessLevel;
  if (avgJerkiness < 100) jerkinessLevel = 0;
  else if (avgJerkiness < 200) jerkinessLevel = 1;
  else jerkinessLevel = 2;
  
  // Handle calibration logic
  if (isCalibrating && calibrationRep < 5) {
    bool validROM = romAngle >= (targetROM * 0.8);  // 80% of target ROM
    bool validSpeed = bendDuration <= (targetSpeed / 10.0);  // Convert deg/s to rough time estimate
    
    if (validROM) {  // For now, just check ROM - speed validation needs work
      calibrationAngles[calibrationRep] = romAngle;
      calibrationSpeeds[calibrationRep] = bendDuration;
      calibrationRep++;
      
      Serial.print("✓ Calibration ");
      Serial.print(calibrationRep);
      Serial.print("/5 ACCEPTED - ROM:");
      Serial.print(romAngle);
      Serial.print("° Speed:");
      Serial.print(bendDuration, 1);
      Serial.println("s");
      
      calibrationChar.writeValue(1); // Progress signal
    } else {
      Serial.print("✗ Calibration REJECTED - ROM:");
      Serial.print(romAngle);
      Serial.print("° < required ");
      Serial.println(targetROM * 0.8, 1);
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
      
      Serial.println("=== CALIBRATION COMPLETE ===");
      Serial.print("Average ROM: ");
      Serial.print(avgROM, 1);
      Serial.print("°, Average Speed: ");
      Serial.print(avgSpeed, 1);
      Serial.println("s");
      
      calibrationChar.writeValue(2); // Complete signal
      isCalibrating = false;
      calibrationRep = 0;
      resetSession();
      return;
    }
  }
  
  romChar.writeValue(romAngle);
  speedChar.writeValue(bendDuration);
  jerkinessChar.writeValue(jerkinessLevel);
  repCountChar.writeValue(currentRep);
  
  Serial.print("Rep ");
  Serial.print(currentRep);
  Serial.print(" completed - Bend ROM: ");
  Serial.print(romAngle);
  Serial.print("°, Speed: ");
  Serial.print(bendDuration);
  Serial.print("s, Jerkiness: ");
  Serial.println(jerkinessLevel);
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
  
  Serial.println("Session reset");
}