#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

// BLE Service and Characteristics
BLEService rehabandService("180D"); // Heart Rate service UUID (repurposed)
BLEIntCharacteristic romChar("2A37", BLERead | BLENotify);        // ROM angle
BLEFloatCharacteristic speedChar("2A38", BLERead | BLENotify);    // Speed (seconds per rep)
BLEIntCharacteristic jerkinessChar("2A39", BLERead | BLENotify);  // Jerkiness level (0-2)
BLEIntCharacteristic repCountChar("2A3A", BLERead | BLENotify);   // Current rep number

// IMU and exercise tracking variables
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float pitch, roll;
int currentRep = 0;
bool inRep = false;
unsigned long repStartTime = 0;
float maxAngle = 0;
float minAngle = 180;
float jerkinessSum = 0;
int jerkinessCount = 0;

// Calibration and thresholds
const float ANGLE_THRESHOLD = 30.0;  // Minimum angle change to count as rep
const float GYRO_THRESHOLD = 50.0;   // Gyroscope threshold for movement detection
const int MIN_REP_TIME = 1000;       // Minimum rep time in milliseconds
const int MAX_REP_TIME = 8000;       // Maximum rep time in milliseconds

void setup() {
  Serial.begin(9600);
  
  // Initialize IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  
  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }
  
  // Set BLE device name and advertised service
  BLE.setLocalName("REHABAND");
  BLE.setAdvertisedService(rehabandService);
  
  // Add characteristics to service
  rehabandService.addCharacteristic(romChar);
  rehabandService.addCharacteristic(speedChar);
  rehabandService.addCharacteristic(jerkinessChar);
  rehabandService.addCharacteristic(repCountChar);
  
  // Add service
  BLE.addService(rehabandService);
  
  // Set initial characteristic values
  romChar.writeValue(0);
  speedChar.writeValue(0.0);
  jerkinessChar.writeValue(0);
  repCountChar.writeValue(0);
  
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
    
    // While connected, continuously read IMU and track reps
    while (central.connected()) {
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
}

void trackExercise() {
  float currentAngle = pitch;
  unsigned long currentTime = millis();
  
  // Calculate jerkiness (variance in gyroscope readings)
  float totalGyro = sqrt(gyroX*gyroX + gyroY*gyroY + gyroZ*gyroZ);
  jerkinessSum += totalGyro;
  jerkinessCount++;
  
  // Detect start of rep (significant movement)
  if (!inRep && totalGyro > GYRO_THRESHOLD) {
    inRep = true;
    repStartTime = currentTime;
    maxAngle = currentAngle;
    minAngle = currentAngle;
    jerkinessSum = totalGyro;
    jerkinessCount = 1;
    
    Serial.println("Rep started");
  }
  
  // Track angle extremes during rep
  if (inRep) {
    if (currentAngle > maxAngle) maxAngle = currentAngle;
    if (currentAngle < minAngle) minAngle = currentAngle;
    
    // Detect end of rep (return to starting position with low movement)
    if (totalGyro < GYRO_THRESHOLD/2 && 
        (currentTime - repStartTime) > MIN_REP_TIME &&
        (maxAngle - minAngle) > ANGLE_THRESHOLD) {
      
      completeRep(currentTime);
    }
    
    // Timeout protection
    if ((currentTime - repStartTime) > MAX_REP_TIME) {
      Serial.println("Rep timeout - resetting");
      inRep = false;
    }
  }
}

void completeRep(unsigned long endTime) {
  currentRep++;
  inRep = false;
  
  // Calculate rep metrics
  float repDuration = (endTime - repStartTime) / 1000.0; // Convert to seconds
  int romAngle = (int)(maxAngle - minAngle);
  
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
  Serial.print("°, Speed: ");
  Serial.print(repDuration);
  Serial.print("s, Jerkiness: ");
  Serial.println(jerkinessLevel);
  
  // Reset for next rep
  maxAngle = 0;
  minAngle = 180;
  jerkinessSum = 0;
  jerkinessCount = 0;
  
  // Stop after 10 reps
  if (currentRep >= 10) {
    Serial.println("Session complete - 10 reps finished");
    delay(5000); // Wait 5 seconds before allowing new session
    resetSession();
  }
}

void resetSession() {
  currentRep = 0;
  inRep = false;
  maxAngle = 0;
  minAngle = 180;
  jerkinessSum = 0;
  jerkinessCount = 0;
  
  // Reset BLE characteristics
  romChar.writeValue(0);
  speedChar.writeValue(0.0);
  jerkinessChar.writeValue(0);
  repCountChar.writeValue(0);
  
  Serial.println("Session reset");
}