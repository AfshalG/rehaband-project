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

- **Real-time Rep Tracking**: Automatically detects and counts exercise repetitions
- **Performance Metrics**: ROM angle, speed analysis, and jerkiness detection
- **Live Dashboard**: Session summary with stats and rep history
- **Demo Mode**: Test the interface without hardware
- **Cross-platform**: Works on any device with Chrome/Edge browser

## 🛠 Hardware Requirements

- **Arduino Nano 33 BLE** (with built-in LSM9DS1 IMU sensor)
- USB cable for programming
- Computer with Chrome or Edge browser

## 📋 Quick Start

### 0. Clone the Repository First
```bash
git clone https://github.com/AfshalG/rehaband-project.git
cd rehaband-project
```

### 1. Try Demo Mode First
1. Open `rehaband-app-ble.html` in Chrome or Edge (from the cloned folder)
2. Click **"Try Demo Mode"** (green button)  
3. Watch as 5 simulated reps appear with real-time feedback
4. This shows you exactly how the interface works!

### 2. Set Up Arduino (for real data)
1. Install required libraries in Arduino IDE:
   ```
   - ArduinoBLE (by Arduino)
   - Arduino_LSM9DS1 (by Arduino)
   ```
2. Open `arduino_rehaband.ino` in Arduino IDE
3. Select **Tools > Board > Arduino Nano 33 BLE**
4. Upload the sketch to your Arduino
5. Arduino will start advertising as "REHABAND"

### 3. Connect Real Device
1. Make sure Arduino is powered and running the sketch
2. Open `rehaband-app-ble.html` in Chrome/Edge (from the cloned folder)
3. Click **"Connect to REHABAND"** (blue button)
4. Select "REHABAND" from Bluetooth device list
5. Start exercising - reps will appear automatically!

## 📁 File Structure

```
rehaband-project/
├── README.md                    # This file
├── rehaband-app-ble.html       # Main BLE-enabled web app
├── rehaband-app.html           # Static wireframe (original)
└── arduino_rehaband.ino        # Arduino sketch for Nano 33 BLE
```

## 🎮 How to Use

### Demo Mode
- **Purpose**: Test the interface without Arduino hardware
- **Duration**: Shows 5 simulated reps over ~15 seconds
- **Resets**: Automatically resets after completion
- **Perfect for**: Understanding the interface before using real hardware

### Real Device Mode
- **Connection**: Bluetooth Low Energy (BLE)
- **Range**: ~10 meters from Arduino
- **Data**: Real IMU sensor readings
- **Reps**: Up to 10 reps per session
- **Auto-reset**: Session resets after 10 reps

## 📊 Performance Metrics

### ROM (Range of Motion)
- **Good**: 85° - 105°
- **Caution**: 70° - 84°  
- **Poor**: Below 70°

### Speed
- **Good**: 2.5 - 4.0 seconds per rep
- **Poor**: Too fast (<2.5s) or too slow (>4.0s)

### Jerkiness
- **Good**: Smooth movement (no jerk)
- **Caution**: Some jerkiness detected
- **Poor**: Very jerky movement

## 🔧 Troubleshooting

### Web App Issues
- **"Web Bluetooth not supported"**: Use Chrome or Edge browser
- **Can't connect**: Make sure Arduino is advertising as "REHABAND"
- **Connection drops**: Stay within 10m range of Arduino

### Arduino Issues
- **Upload fails**: Check board selection (Arduino Nano 33 BLE)
- **Libraries missing**: Install ArduinoBLE and Arduino_LSM9DS1
- **Not advertising**: Check serial monitor for "BLE device active" message

### BLE Connection Issues
- **Device not found**: Reset Arduino and try again
- **Connection timeout**: Make sure Arduino sketch is running
- **No data**: Check that Arduino is detecting movement

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

### Arduino IMU Processing
- **Sensor**: LSM9DS1 (accelerometer + gyroscope)
- **Sampling Rate**: 20Hz
- **Movement Detection**: Gyroscope threshold-based
- **Angle Calculation**: Pitch from accelerometer data

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