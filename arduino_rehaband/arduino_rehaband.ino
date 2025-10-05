#include <ArduinoBLE.h>
#include <Arduino_BMI270_BMM150.h>

// BLE Service and Characteristics
BLEService rehabandService("180D"); // Heart Rate service UUID (repurposed)
BLEIntCharacteristic romChar("2A37", BLERead | BLENotify);        // ROM angle
BLEFloatCharacteristic speedChar("2A38", BLERead | BLENotify);    // Speed (seconds per rep)
BLEIntCharacteristic jerkinessChar("2A39", BLERead | BLENotify);  // Jerkiness level (0-2)
BLEIntCharacteristic repCountChar("2A3A", BLERead | BLENotify);   // Current rep number
BLEIntCharacteristic sessionControlChar("2A3B", BLEWrite);        // Session control (start/reset)

// IMU and exercise tracking variables
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float pitch, roll;
int currentRep = 0;
unsigned long repStartTime = 0;
float jerkinessSum = 0;
int jerkinessCount = 0;

// State machine and angle tracking variables
int exerciseState = 0;      // 0: IDLE, 1: BENDING, 2: STRAIGHTENING
float startBendAngle = 0;   // Angle at the start of the bend
float peakBendAngle = 0;    // Lowest angle achieved during the bend

// New variables for robust rep counting
float lastStableAngle = 0;   // Stores the angle when the leg is at rest
unsigned long lastRepTime = 0; // Time of the last completed rep
const int REP_COOLDOWN = 500;  // 500ms cooldown between reps to prevent double counting

// Calibration and thresholds
const float ANGLE_THRESHOLD = 30.0;  // Minimum angle change to count as rep
const float GYRO_THRESHOLD = 50.0;   // Gyroscope threshold for movement detection
const int MIN_REP_TIME = 1000;       // Minimum rep time in milliseconds
const int MAX_REP_TIME = 8000;       // Maximum rep time in milliseconds

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
  
  // Set BLE device name and advertised service
  BLE.setLocalName("REHABAND");
  BLE.setAdvertisedService(rehabandService);
  
  // Add characteristics to service
  rehabandService.addCharacteristic(romChar);
  rehabandService.addCharacteristic(speedChar);
  rehabandService.addCharacteristic(jerkinessChar);
  rehabandService.addCharacteristic(repCountChar);
  rehabandService.addCharacteristic(sessionControlChar);
  
  // Add service
  BLE.addService(rehabandService);
  
  // Set initial characteristic values
  romChar.writeValue(0);
  speedChar.writeValue(0.0);
  jerkinessChar.writeValue(0);
  repCountChar.writeValue(0);
  sessionControlChar.writeValue(0);
  
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
    
    // Set initial stable angle when connection starts
    delay(1000); // Allow IMU to stabilize
    readIMU();
    lastStableAngle = pitch;
    Serial.print("Initial stable angle set to: ");
    Serial.println(lastStableAngle);
    
    // While connected, continuously read IMU and track reps
    while (central.connected()) {
      // Check for session control commands
      if (sessionControlChar.written()) {
        int command = sessionControlChar.value();
        if (command == 1) {
          // Start session command
          readIMU();
          lastStableAngle = pitch; // Set current angle as stable reference
          resetSession();
          Serial.print("Session started - new stable angle: ");
          Serial.println(lastStableAngle);
        } else if (command == 0) {
          // Reset session command
          resetSession();
          Serial.println("Session reset by user");
        }
      }
      
      readIMU();
      trackExercise();
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
  
  // Calculate pitch and roll from accelerometer
  // Assuming device is oriented with Y-axis along the arm
  pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;
  roll = atan2(accelY, accelZ) * 180.0 / PI;
  
  // Use pitch as the primary ROM measurement (flexion/extension)
  // Normalize to 0-180 degrees range
  pitch = abs(pitch);

  // Initialize stable angle if not set
  if (lastStableAngle == 0 && pitch > 0) {
    lastStableAngle = pitch;
  }
}

void trackExercise() {
  float currentAngle = pitch;
  unsigned long currentTime = millis();
  float totalGyro = sqrt(gyroX * gyroX + gyroY * gyroY + gyroZ * gyroZ);

  // Update jerkiness calculation
  jerkinessSum += totalGyro;
  jerkinessCount++;

  switch (exerciseState) {
    case 0: // IDLE
      // Update stable angle when not moving
      if (totalGyro < GYRO_THRESHOLD / 2) {
        lastStableAngle = currentAngle;
      }
      
      // Detect start of bending motion
      if (totalGyro > GYRO_THRESHOLD && (currentTime - lastRepTime) > REP_COOLDOWN) {
        exerciseState = 1; // BENDING
        repStartTime = currentTime;
        startBendAngle = lastStableAngle;
        peakBendAngle = lastStableAngle;
        jerkinessSum = totalGyro;
        jerkinessCount = 1;
        Serial.print("Starting Bend from angle: ");
        Serial.println(startBendAngle);
      }
      break;

    case 1: // BENDING
      // Track the peak bend angle (most bent position)
      if (abs(currentAngle - startBendAngle) > abs(peakBendAngle - startBendAngle)) {
        peakBendAngle = currentAngle;
        Serial.print("Peak bend updated to: ");
        Serial.println(peakBendAngle);
      }
      
      // Detect end of bending phase (movement stops)
      if (totalGyro < GYRO_THRESHOLD / 2) {
        // Check if we achieved sufficient bend
        if (abs(peakBendAngle - startBendAngle) > ANGLE_THRESHOLD) {
          exerciseState = 2; // STRAIGHTENING
          Serial.print("Bend complete. Peak: ");
          Serial.print(peakBendAngle);
          Serial.print("°, ROM: ");
          Serial.print(abs(peakBendAngle - startBendAngle));
          Serial.println("°. Now straightening...");
        } else {
          // Insufficient bend, reset to idle
          Serial.println("Insufficient bend detected, resetting to idle");
          exerciseState = 0;
        }
      }

      // Timeout protection
      if ((currentTime - repStartTime) > MAX_REP_TIME) {
        Serial.println("Rep timeout during bend - resetting to idle");
        exerciseState = 0;
      }
      break;

    case 2: // STRAIGHTENING
      // Detect return to starting position
      if (totalGyro < GYRO_THRESHOLD / 2 && abs(currentAngle - startBendAngle) < 15) {
        // Check minimum rep time
        if ((currentTime - repStartTime) > MIN_REP_TIME) {
          completeRep(currentTime);
          exerciseState = 0; // Back to IDLE
        } else {
          Serial.println("Rep too fast, not counting");
          exerciseState = 0;
        }
      }

      // Timeout protection
      if ((currentTime - repStartTime) > MAX_REP_TIME) {
        Serial.println("Rep timeout during straighten - resetting to idle");
        exerciseState = 0;
      }
      break;
  }
}

void completeRep(unsigned long endTime) {
  currentRep++;
  lastRepTime = endTime;
  
  // Calculate rep metrics
  float repDuration = (endTime - repStartTime) / 1000.0; // Convert to seconds
  int romAngle = (int)abs(peakBendAngle - startBendAngle); // ROM from start to peak bend
  
  // Calculate jerkiness level (0 = smooth, 1 = some jerk, 2 = very jerky)
  float avgJerkiness = jerkinessSum / jerkinessCount;
  int jerkinessLevel;
  if (avgJerkiness < 100) jerkinessLevel = 0;      // Smooth
  else if (avgJerkiness < 200) jerkinessLevel = 1; // Some jerk
  else jerkinessLevel = 2;                         // Very jerky
  
  // Send data via BLE
  romChar.writeValue(romAngle);
  speedChar.writeValue(repDuration);
  jerkinessChar.writeValue(jerkinessLevel);
  repCountChar.writeValue(currentRep);
  
  // Debug output
  Serial.print("Rep ");
  Serial.print(currentRep);
  Serial.print(" completed - ROM: ");
  Serial.print(romAngle);
  Serial.print("° (start: ");
  Serial.print(startBendAngle);
  Serial.print("°, peak: ");
  Serial.print(peakBendAngle);
  Serial.print("°), Speed: ");
  Serial.print(repDuration);
  Serial.print("s, Jerkiness: ");
  Serial.println(jerkinessLevel);
  
  // Reset rep-specific variables
  jerkinessSum = 0;
  jerkinessCount = 0;
  
  // Stop after 10 reps (optional - can be removed if not desired)
  if (currentRep >= 10) {
    Serial.println("Session complete - 10 reps finished");
    delay(5000); // Wait 5 seconds before allowing new session
    resetSession();
  }
}

void resetSession() {
  currentRep = 0;
  exerciseState = 0; // Back to IDLE
  jerkinessSum = 0;
  jerkinessCount = 0;
  
  // Keep lastStableAngle but reset timing
  lastRepTime = 0;
  
  // Reset BLE characteristics
  romChar.writeValue(0);
  speedChar.writeValue(0.0);
  jerkinessChar.writeValue(0);
  repCountChar.writeValue(0);
  
  Serial.println("Session reset");
}