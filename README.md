# REHABAND - Smart Rehabilitation Device

A real-time rehabilitation squat monitoring system that uses Arduino Nano 33 BLE and Web Bluetooth API to track knee flexion form and provide instant feedback. Optimized for thigh-mounted IMU placement with realistic biomechanical ranges (0°-150° knee flexion).

![REHABAND Interface](https://img.shields.io/badge/Status-Active-green) ![Arduino](https://img.shields.io/badge/Arduino-Nano%2033%20BLE-blue) ![Web Bluetooth](https://img.shields.io/badge/Web%20Bluetooth-Supported-purple)

## 🎯 What is REHABAND?

REHABAND is a smart squat rehabilitation device that:
- **Tracks squat form** using thigh-mounted IMU sensors with complementary filtering
- **Provides real-time feedback** on knee flexion angle (30°-150°), angular velocity, and movement smoothness
- **Color-codes performance** (Good/Caution/Poor) with personalized calibrated targets
- **Works wirelessly** via Bluetooth Low Energy
- **Displays live data** in a beautiful web interface with realistic knee biomechanics

## 📱 Features

### Core Features
- **Real-time Squat Tracking**: 4-state machine (STRAIGHT → SQUATTING → BOTTOM → RISING) with robust detection
- **Advanced Performance Metrics**: Knee flexion angle, angular velocity (°/s), and RMS jerkiness analysis
- **Realistic Settings**: ROM range 30°-150° (quarter to deep squat), Speed 20-50°/s for controlled rehabilitation
- **Smart Sound Alerts**: Audio feedback for poor form (silent for good performance)
- **Live Dashboard**: Session summary with stats and rep history
- **Enhanced Demo Mode**: Realistic squat examples with proper knee flexion angles
- **Persistent Settings**: Your preferences save automatically using localStorage
- **Mobile Optimized**: Responsive design with improved spacing and layout
- **Cross-platform**: Works on any device with Chrome/Edge browser

### Advanced Session Control Features
- **Start/Reset Button**: Control squat sessions with reference angle setting (0° straight leg baseline)
- **5-Rep Calibration**: Personalized target calculation based on user's best form (90% ROM, 110% speed tolerance)
- **Standing Position Calibration**: True 0° knee flexion baseline for accurate measurements
- **Two-Way BLE Communication**: Sends targets to Arduino, receives calibrated values

### Advanced Biomechanical Tracking
- **4-State Squat Machine**: STRAIGHT → SQUATTING_DOWN → BOTTOM_POSITION → RISING_UP with timeout protection
- **Complementary Filtering**: Combines accelerometer + gyroscope for drift-free pitch measurement
- **Realistic Knee Flexion**: 0° (standing) to 150° (deep squat) with 30° minimum for valid reps
- **RMS Jerkiness Analysis**: Root Mean Square calculation for movement smoothness assessment
- **Angular Velocity Tracking**: Real-time speed measurement in °/s with moving average smoothing
- **Peak Squat Detection**: Accurately captures maximum knee flexion during squat

## 🛠 Hardware Requirements

- **Arduino Nano 33 BLE** (with built-in BMI270_BMM150 or LSM9DS1 IMU sensor)
- **Mounting Solution**: Strap or band to secure device 3cm above kneecap on thigh (parallel to thigh)
- USB cable for programming
- Computer with Chrome or Edge browser (Web Bluetooth support required)

### IMU Sensor Compatibility
- **BMI270_BMM150**: Default for newer Arduino Nano 33 BLE boards
- **LSM9DS1**: Legacy sensor for older Arduino Nano 33 BLE boards
- The code automatically detects which sensor your board uses

## 📋 Quick Start

### 0. Clone the Repository First
```bash
git clone https://github.com/AfshalG/rehaband-project.git
cd rehaband-project
```

### 1. Try Demo Mode First
1. Open `rehaband-app-ble.html` in Chrome or Edge (from the cloned folder)
2. **For best demo results**: Set Target ROM to **90°** and Max Speed to **30°/s** in settings
3. Click **"Try Demo Mode"** (green button turns red while running)  
4. Watch as 5 simulated squats appear with realistic knee flexion angles and real-time feedback
5. Observe how Rep 1 (90°) and Rep 3 (95°) show as "Good" with controlled speed
6. Adjust settings with +/- controls to see how thresholds affect evaluation

### 2. Set Up Arduino (for real data)
1. Install required libraries in Arduino IDE:
   ```
   - ArduinoBLE (by Arduino)
   - Arduino_BMI270_BMM150 (by Arduino) - for newer boards
   - Arduino_LSM9DS1 (by Arduino) - for older boards
   ```
2. Open `arduino_rehaband/arduino_rehaband.ino` in Arduino IDE
3. The code uses `Arduino_BMI270_BMM150.h` by default - change to `Arduino_LSM9DS1.h` if you have an older board
4. Select **Tools > Board > Arduino Nano 33 BLE**
5. Upload the sketch to your Arduino
6. Arduino will start advertising as "REHABAND"

### 3. Connect Real Device
1. **Mount Arduino**: Secure device 3cm above kneecap on thigh (parallel to thigh)
2. Make sure Arduino is powered and running the sketch
3. Open `rehaband-app-ble.html` in Chrome/Edge (from the cloned folder)
4. Click **"Connect to REHABAND"** (blue button)
5. Select "REHABAND" from Bluetooth device list
6. **Session Controls** will appear with two buttons:
   - **"Start Session"** - Sets 0° reference (standing position) and begins tracking
   - **"Calibrate (5 Reps)"** - Personalizes targets based on your 5 best squats
7. **IMPORTANT**: Always calibrate first - perform 5 squats with perfect form
8. Start squatting - system detects 30°-150° knee flexion automatically!

## 📁 File Structure

```
rehaband-project/
├── README.md                           # This file with updated features
├── rehaband-app-ble.html              # Main BLE-enabled web app with new features
├── rehaband-app.html                  # Static wireframe (original)
└── arduino_rehaband/
    └── arduino_rehaband.ino           # Arduino sketch with enhanced tracking
```

## 🎮 How to Use

### Demo Mode
- **Purpose**: Test the interface without Arduino hardware
- **Duration**: Shows 5 simulated reps over ~15 seconds
- **Visual Feedback**: Button turns red while running, disclaimer appears
- **Resets**: Automatically resets after completion
- **Perfect for**: Understanding the interface before using real hardware

### Real Device Mode
- **Connection**: Bluetooth Low Energy (BLE)
- **Range**: ~10 meters from Arduino
- **Data**: Real IMU sensor readings
- **Session Controls Section**: Prominent buttons for Start/Reset and Calibration
- **Calibration**: 5-rep calibration to personalize targets
- **Reps**: Up to 10 reps per session
- **Auto-reset**: Session resets after 10 reps
- **Status Feedback**: Clear indicators for connection and session state

### Session Control
1. **Connect** to your Arduino device
2. **Start Session** - Sets current angle as reference position
3. **Begin Exercising** - Rep detection automatically starts
4. **Reset Session** - Clears data and sets new reference angle

### Calibration Process
1. **Connect** to your Arduino device
2. **Click "Calibrate (5 Reps)"** in the Session Controls section
3. **Perform 5 PERFECT form repetitions** - use your absolute best technique
4. **System automatically calculates** your personal ROM and speed targets
5. **Settings update** immediately with your calibrated values
6. **All future reps** are evaluated against YOUR personalized standards

## 🎯 Calibration Guide

### What Calibration Does
Calibration creates **YOUR personalized targets** based on your 5 best repetitions instead of using generic defaults (95° ROM, 3.2s speed). This makes feedback much more accurate since everyone's body and exercise form are different.

### Why Calibration is Important
- **Generic targets may not suit you**: Default 95° ROM might be too high/low for your body
- **Personalized feedback**: Get accurate "Good/Caution/Poor" ratings based on YOUR capabilities
- **Better motivation**: Targets that match your actual best performance are more achievable
- **Improved accuracy**: No more "Poor" ratings when you're actually doing well for your body type

### When to Calibrate
- **First time setup**: Always calibrate when you first use the device
- **Position changes**: If you move the device to a different location on your leg
- **Inaccurate feedback**: If ratings seem off (too easy or too hard)
- **After long breaks**: If you haven't used the device in weeks/months

### Calibration Process (Real-Time Progress with Dynamic Validation)
1. **Set your standards** - Adjust Target ROM and Max Speed in settings first
2. **Connect to Arduino** and click "Calibrate (5 Reps)" button
3. **Watch live progress** - button updates: "Calibrating... (1/5)", "Calibrating... (2/5)", etc.
4. **Perform 5 PERFECT squats** - only squats meeting YOUR settings count:
   - **ROM requirement**: ≥80% of your Target ROM setting
   - **Speed requirement**: ≤150% of your Max Speed setting
   - **Rejected squats**: Clear feedback shows why (too shallow, too fast)
5. **Automatic completion** - system calculates and applies your personal targets
6. **Ready to exercise** - all future reps evaluated against YOUR standards

### Calibration Best Practices
1. **Perfect form only**: Use your absolute BEST technique for all 5 reps
2. **Consistent positioning**: Keep device in exact same position throughout calibration
3. **Full range of motion**: Use your maximum comfortable ROM
4. **Steady pace**: Use your ideal, controlled speed
5. **Focus and concentration**: Don't rush - quality over speed
6. **Wait for updates** - each squat completion shows progress on the app

### Understanding Results
After calibration, check your new targets in the settings:
- **ROM Target**: Your average maximum bend angle (updated automatically)
- **Speed Target**: Your average angular velocity (updated automatically)
- **These become your new "Good" standards** for all future exercises
- **Real-time sync** - Arduino and app communicate seamlessly during calibration

## 🔊 Sound Alerts

### Audio Feedback System
- **Good Form**: 🔇 **Silent** (no beep - good form shouldn't interrupt)
- **Caution Form**: ⚠️ **Medium beep** (600Hz, 0.15s) - gentle warning
- **Poor Form**: ❌ **Lower, longer beep** (300Hz, 0.4s) - needs attention

### Sound Controls
- **Toggle**: ON/OFF button in Adjust Settings
- **Test Sound**: Plays when you turn sound alerts ON
- **Persistent**: Setting saves automatically across sessions
- **Works**: Both demo mode and real Arduino data

## ⚙️ Interactive Settings

### Adjustable Thresholds (Realistic Biomechanical Ranges)
- **Target ROM**: Set your ideal knee flexion (30°-150° range) - 90° = parallel squat standard
- **Max Speed for "Controlled" Rating**: Set angular velocity threshold (20°/s-50°/s) - lower = more controlled
- **+/- Controls**: Click buttons to adjust values in real-time
- **Auto-Save**: Settings persist across browser sessions
- **Live Updates**: Changes affect evaluation immediately

### Dynamic Color-Coding (Based on Realistic Knee Biomechanics)
- **ROM Tolerance**: Target ±10° = Good, Target ±25° = Caution, beyond = Poor
- **Speed Evaluation**: ≤Target = Controlled (Good), Target to 1.5x = Moderate (Caution), >1.5x = Too Fast (Poor)
- **Smoothness Analysis**: RMS Jerk <50 = Smooth, 50-150 = Moderate, >150 = Jerky
- **Instant Feedback**: Both demo and real data use your custom thresholds

## 📊 Performance Metrics (Biomechanically Accurate)

### Default Thresholds (Personalized via Calibration)
- **ROM Target**: 90° knee flexion (Good: 80°-100°, Caution: 65°-115°, Poor: <65° or >115°)
- **Speed Target**: 30°/s angular velocity (Good: ≤30°/s, Caution: 30-45°/s, Poor: >45°/s)
- **Smoothness**: RMS Jerk analysis (Good: <50, Caution: 50-150, Poor: >150)
- **Knee Flexion Range**: 0° (standing) to 150° (deep squat) with 30° minimum for valid reps

### Realistic Squat Biomechanics

**Knee Flexion Ranges:**
- **0°**: Standing (full leg extension)
- **30°**: Quarter squat (minimum for valid rep)
- **90°**: Parallel squat (standard rehabilitation target)  
- **130-150°**: Deep squat (advanced range)

**Angular Velocity Guidelines:**
- **20°/s**: Slow, controlled pace (~4.5 seconds per squat)
- **30°/s**: Standard controlled pace (~3 seconds per squat)
- **40°/s**: Moderate pace (~2.25 seconds per squat)
- **50°/s**: Upper safe limit (~1.8 seconds per squat)

## 🔧 Troubleshooting

### Web App Issues
- **"Web Bluetooth not supported"**: Use Chrome or Edge browser
- **Can't connect**: Make sure Arduino is advertising as "REHABAND"
- **Connection drops**: Stay within 10m range of Arduino

### Arduino Issues
- **Upload fails**: Check board selection (Arduino Nano 33 BLE)
- **Libraries missing**: Install ArduinoBLE and Arduino_BMI270_BMM150 (or Arduino_LSM9DS1 for older boards)
- **Not advertising**: Check serial monitor for "BLE device active" message
- **Wrong IMU library**: Change to `Arduino_LSM9DS1.h` if you have an older board

### BLE Connection Issues
- **Device not found**: Reset Arduino and try again
- **Connection timeout**: Make sure Arduino sketch is running
- **No data**: Check that Arduino is detecting movement

### Session Controls Issues
- **Buttons disabled/grayed**: Make sure you're connected to Arduino device first
- **Start Session not working**: Check connection status and try reconnecting
- **Calibration stuck**: Disconnect and reconnect, then try calibration again

### Calibration Issues
- **Weird results after calibration**: Your 5 reps may have been inconsistent - try again with more consistent form
- **Targets too high/low**: Reset to defaults by manually adjusting ROM/Speed in settings
- **"Poor" ratings after calibration**: Device position may have shifted - try recalibrating
- **How to reset calibration**: Manually adjust ROM and Speed targets back to 95° and 3.2s

## 🌐 Browser Compatibility

| Browser | Web Bluetooth | Status |
|---------|---------------|--------|
| Chrome | ✅ Supported | ✅ Works |
| Edge | ✅ Supported | ✅ Works |
| Firefox | ❌ Not supported | ❌ No BLE |
| Safari | ❌ Not supported | ❌ No BLE |

**Note**: HTTPS or localhost required for Web Bluetooth security

## 🔄 Technical Details

### Enhanced BLE Service Structure
- **Service UUID**: `180D` (Heart Rate Service - repurposed)
- **ROM Characteristic**: `2A37` (knee flexion angle data in degrees)
- **Speed Characteristic**: `2A38` (angular velocity data in °/s)  
- **Jerkiness Characteristic**: `2A39` (RMS jerk smoothness analysis)
- **Rep Count Characteristic**: `2A3A` (current rep number)
- **Session Control Characteristic**: `2A3B` (start/reset commands)
- **Calibration Characteristic**: `2A3C` (calibration control and results)
- **Target ROM Characteristic**: `2A3D` (target ROM from app to Arduino)
- **Target Speed Characteristic**: `2A3E` (target speed from app to Arduino)

### Advanced Arduino IMU Processing
- **Sensor**: BMI270_BMM150 or LSM9DS1 (accelerometer + gyroscope)
- **Sampling Rate**: 20Hz (50ms intervals)
- **Advanced Filtering**: Complementary filter combining accelerometer + gyroscope for drift-free pitch
- **4-State Machine**: STRAIGHT → SQUATTING_DOWN → BOTTOM_POSITION → RISING_UP
- **Biomechanical Angle Calculation**: True knee flexion from 0° (standing) baseline
- **RMS Jerkiness Analysis**: Root Mean Square calculation for movement smoothness
- **Angular Velocity Tracking**: Real-time speed measurement with moving average smoothing
- **Robust Squat Detection**: 30° minimum knee flexion with timeout protection
- **Two-Way Calibration**: Receives targets from app, calculates personalized thresholds
- **Thigh-Mounted Optimization**: Designed for 3cm above kneecap placement

## 🤝 Contributing

1. Fork the repository
2. Create your feature branch: `git checkout -b feature/amazing-feature`
3. Commit your changes: `git commit -m 'Add amazing feature'`
4. Push to the branch: `git push origin feature/amazing-feature`
5. Open a Pull Request

## 📝 License

This project is open source and available under the [MIT License](LICENSE).

## 👥 Team

Created by **Nusurvivors** for smart rehabilitation monitoring.

---

## 🔬 **Recent Major Improvements (v2.1)**

### **Enhanced BLE Communication & Smart Calibration**
- **Real-time Calibration Progress** - App shows live updates "Calibrating... (1/5)" as you complete each squat
- **Dynamic Calibration Validation** - Only accepts squats meeting YOUR Target ROM (≥80%) and Speed (≤150%) settings
- **Intelligent Squat Rejection** - Clear feedback: "ROM too low: 65° < 72°, Speed too fast: 35°/s > 30°/s"
- **Robust BLE Retry Logic** - Eliminates "GATT operation not permitted" errors with automatic retries
- **Seamless Arduino-App Sync** - Perfect communication during calibration with progress tracking
- **Improved Error Handling** - Clear error messages and graceful recovery from connection issues

### **Previous Major Improvements (v2.0)**

### **Biomechanically Accurate Tracking**
- **Realistic Knee Flexion**: 0° (standing) to 150° (deep squat) with proper biomechanical ranges
- **Complementary Filtering**: Advanced sensor fusion for drift-free angle measurement  
- **4-State Squat Machine**: Robust detection system optimized for rehabilitation

### **Personalized Calibration System**
- **5-Rep Calibration**: Creates user-specific targets based on individual capabilities
- **Two-Way BLE Communication**: App sends commands, receives personalized thresholds
- **Standing Position Baseline**: True 0° knee flexion reference for accurate measurement

### **Clinical-Grade Metrics**
- **RMS Jerkiness Analysis**: Objective movement smoothness assessment
- **Angular Velocity Tracking**: Real-time speed measurement in °/s with filtering
- **Realistic Speed Ranges**: 20-50°/s for controlled rehabilitation (1.8-4.5 seconds per squat)

### **Enhanced User Experience**
- **Intuitive Settings**: "Max Speed for Controlled Rating" instead of technical jargon
- **Realistic Demo Mode**: Examples with proper knee flexion angles and clear guidance
- **Comprehensive Instructions**: Step-by-step setup from hardware mounting to calibration

---

**Questions?** Open an issue or contact the team!

**Want to contribute?** We welcome pull requests and suggestions!