# REHABAND - Smart Rehabilitation Device

A real-time rehabilitation monitoring system that uses Arduino Nano 33 BLE and Web Bluetooth API to track exercise form and provide instant feedback.

![REHABAND Interface](https://img.shields.io/badge/Status-Active-green) ![Arduino](https://img.shields.io/badge/Arduino-Nano%2033%20BLE-blue) ![Web Bluetooth](https://img.shields.io/badge/Web%20Bluetooth-Supported-purple)

## 🎯 What is REHABAND?

REHABAND is a smart rehabilitation device that:
- **Tracks exercise form** using IMU sensors
- **Provides real-time feedback** on range of motion, speed, and smoothness
- **Color-codes performance** (Good/Caution/Poor)
- **Works wirelessly** via Bluetooth Low Energy
- **Displays live data** in a beautiful web interface

## 📱 Features

### Core Features
- **Real-time Rep Tracking**: Automatically detects and counts exercise repetitions
- **Performance Metrics**: ROM angle, speed analysis, and jerkiness detection
- **Interactive Settings**: Adjust ROM and Speed targets with +/- controls
- **Smart Sound Alerts**: Audio feedback for poor form (silent for good performance)
- **Live Dashboard**: Session summary with stats and rep history
- **Enhanced Demo Mode**: Test interface with visual feedback and calibration notes
- **Persistent Settings**: Your preferences save automatically using localStorage
- **Mobile Optimized**: Responsive design with improved spacing and layout
- **Cross-platform**: Works on any device with Chrome/Edge browser

### New Session Control Features
- **Start/Reset Button**: Control exercise sessions with reference angle setting
- **5-Rep Calibration**: Automatic calibration system to personalize targets
- **Reference Angle Setting**: Each session starts with current position as baseline

### Enhanced Angle Tracking
- **State Machine Algorithm**: Robust IDLE → BENDING → STRAIGHTENING progression
- **Stable Angle Tracking**: Continuous baseline calibration during rest periods
- **Rep Cooldown System**: 500ms cooldown prevents double-counting of reps
- **Peak Bend Detection**: Accurately captures maximum bend angle achieved
- **Better ROM Calculation**: Based on stable start position to peak bend angle

## 🛠 Hardware Requirements

- **Arduino Nano 33 BLE** (with built-in BMI270_BMM150 or LSM9DS1 IMU sensor)
- USB cable for programming
- Computer with Chrome or Edge browser

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
2. Click **"Try Demo Mode"** (green button turns red while running)  
3. Watch as 5 simulated reps appear with real-time feedback and sound alerts
4. Adjust settings with +/- controls to see how thresholds affect evaluation
5. This shows you exactly how the interface works!

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
1. Make sure Arduino is powered and running the sketch
2. Open `rehaband-app-ble.html` in Chrome/Edge (from the cloned folder)
3. Click **"Connect to REHABAND"** (blue button)
4. Select "REHABAND" from Bluetooth device list
5. **Session Controls** will appear with two buttons:
   - **"Start Session"** - Sets reference angle and begins tracking
   - **"Calibrate (5 Reps)"** - Personalizes targets based on your best form
6. **Recommended**: Calibrate first for personalized feedback
7. Start exercising - reps will appear automatically!

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

### Calibration Best Practices
1. **Perfect form only**: Use your absolute BEST technique for all 5 reps
2. **Consistent positioning**: Keep device in exact same position throughout calibration
3. **Full range of motion**: Use your maximum comfortable ROM
4. **Steady pace**: Use your ideal, controlled speed
5. **Focus and concentration**: Don't rush - quality over speed

### Understanding Results
After calibration, check your new targets in the settings:
- **ROM Target**: Your average maximum bend angle
- **Speed Target**: Your average rep duration
- **These become your new "Good" standards** for all future exercises

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

### Adjustable Thresholds
- **Target ROM**: Set your ideal range of motion (50° - 150° range)
- **Speed Target**: Set your preferred rep timing (1.0s - 6.0s range)
- **+/- Controls**: Click buttons to adjust values in real-time
- **Auto-Save**: Settings persist across browser sessions
- **Live Updates**: Changes affect evaluation immediately

### Dynamic Color-Coding
- **ROM Tolerance**: Target ±10° = Good, Target ±25° = Caution, beyond = Poor
- **Speed Tolerance**: Target ±0.5s = Good, beyond = Poor
- **Instant Feedback**: Both demo and real data use your custom thresholds

## 📊 Performance Metrics

### Default Thresholds (Customizable)
- **ROM Target**: 95° (Good: 85°-105°, Caution: 70°-120°, Poor: beyond)
- **Speed Target**: 3.2s (Good: 2.7s-3.7s, Poor: beyond)
- **Jerkiness**: Based on sensor data (Good: smooth, Caution: some, Poor: jerky)

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

### BLE Service Structure
- **Service UUID**: `180D` (Heart Rate Service - repurposed)
- **ROM Characteristic**: `2A37` (angle data)
- **Speed Characteristic**: `2A38` (timing data)  
- **Jerkiness Characteristic**: `2A39` (smoothness level)
- **Rep Count Characteristic**: `2A3A` (current rep number)
- **Session Control Characteristic**: `2A3B` (start/reset commands)

### Arduino IMU Processing
- **Sensor**: BMI270_BMM150 or LSM9DS1 (accelerometer + gyroscope)
- **Sampling Rate**: 20Hz
- **Algorithm**: State machine with IDLE → BENDING → STRAIGHTENING states
- **Angle Calculation**: Pitch from accelerometer data with stable angle tracking
- **Rep Detection**: Peak bend detection with cooldown system
- **Baseline System**: Continuous stable angle calibration during rest periods
- **Session Control**: BLE commands for start/reset with angle reference setting

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

**Questions?** Open an issue or contact the team!

**Want to contribute?** We welcome pull requests and suggestions!