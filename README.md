Here is your generated `README.md` file content:

---

````markdown
# 🤖 The Robot Car

## 📖 Overview

The Robot Car is an autonomous line-following robot with obstacle avoidance capabilities built on the ESP32 platform. This project demonstrates advanced robotics concepts using affordable hardware components and leverages the ESP-IDF framework for reliable performance.

|![4](https://github.com/user-attachments/assets/bb859a50-6aed-43e2-97e6-ca3f20f0be3d).|
![3](https://github.com/user-attachments/assets/f55ed96a-3160-4357-b160-d2310869e66b)
![2](https://github.com/user-attachments/assets/e83f064c-286e-4e30-a1d1-f58c0c2f86dc)
![1](https://github.com/user-attachments/assets/12988612-b737-4998-b2b0-fca4b504326a)


## ✨ Features

- **Line following** using PD (Proportional-Derivative) control  
- **Obstacle detection and avoidance** with intelligent path planning  
- **Three IR sensor** setup for precise line tracking  
- **MPU6050 integration** for motion tracking and balance  
- **Real-time debug information** via serial logging  
- **Customizable parameters** for different environments  

## 🔧 Components

### Hardware Requirements

- ESP32 38-pin Development Board  
- TB6612FNG Motor Driver  
- 3× IR Line Sensors  
- HC-SR04 Ultrasonic Distance Sensor  
- SG90 Servo Motor  
- MPU6050 Accelerometer/Gyroscope Module  
- 2× DC Motors with wheels  
- Chassis  
- Power supply (LiPo battery recommended)  
- Jumper wires  

### Software Requirements

- ESP-IDF framework  
- PlatformIO IDE (recommended)  
- Git (for version control)  

## 📋 Pin Connections

| Component     | Pin     | ESP32 GPIO    |
|---------------|---------|---------------|
| Motor Driver  | STBY    | GPIO33        |
|               | AIN1    | GPIO25        |
|               | AIN2    | GPIO26        |
|               | PWMA    | GPIO27        |
|               | BIN1    | GPIO14        |
|               | BIN2    | GPIO13        |
|               | PWMB    | GPIO12        |
| IR Sensors    | Left    | GPIO36 (ADC0) |
|               | Middle  | GPIO39 (ADC3) |
|               | Right   | GPIO34 (ADC6) |
| Ultrasonic    | Trigger | GPIO23        |
|               | Echo    | GPIO19        |
| Servo         | Signal  | GPIO18        |
| MPU6050       | SDA     | GPIO16        |
|               | SCL     | GPIO17        |

## ⚙️ Installation & Setup

### Using PlatformIO (Recommended)

1. Install [Visual Studio Code](https://code.visualstudio.com/) and the [PlatformIO Extension](https://platformio.org/install/ide?install=vscode)  
2. Clone this repository:
   ```bash
   git clone https://github.com/Silvadnd/Robot-Car-.git
````

3. Open the project in PlatformIO
4. Configure `platformio.ini` if necessary
5. Build and upload the code to your ESP32

### Using ESP-IDF Directly

1. Install [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/)
2. Clone this repository
3. Navigate to the project directory
4. Build and flash:

   ```bash
   idf.py build
   idf.py -p (PORT) flash
   ```

## 🚀 Usage

1. Assemble the robot with all components connected according to the pin diagram
2. Upload the code to the ESP32
3. Place the robot on a line track (black line on white background recommended)
4. Power on the robot

The robot will:

* Follow the line using its IR sensors
* Detect obstacles using the ultrasonic sensor
* Navigate around obstacles and return to the line

## 🛠️ Customization

The following parameters can be adjusted in the code to fine-tune performance:

```c
// Line following parameters
#define MOTOR_BASE_SPEED 55   // Default motor speed (0-255)
#define IR_THRESHOLD 800      // Threshold for line detection

// PD control parameters
float Kp = 15.0;  // Proportional gain
float Kd = 7.0;   // Derivative gain

// Obstacle avoidance parameters
#define OBSTACLE_THRESHOLD 20  // Distance in cm to detect obstacles
#define TURN_DELAY_MS 700      // Turning duration
#define FORWARD_DELAY_MS 900   // Forward movement duration
#define MANEUVER_SPEED 60      // Speed during obstacle avoidance
```

## 🔍 Troubleshooting

### Robot doesn't follow the line properly

* Adjust the `IR_THRESHOLD` value
* Tune the PD parameters (`Kp` and `Kd`)
* Check the IR sensor positions and height

### Obstacle avoidance is not reliable

* Adjust `OBSTACLE_THRESHOLD`
* Increase `TURN_DELAY_MS` or `FORWARD_DELAY_MS`
* Check ultrasonic sensor mounting angle

### Motors not responding

* Verify the `STBY_PIN` is HIGH
* Check motor driver connections
* Ensure sufficient battery power

## 🔮 Future Improvements

* Web interface for real-time parameter adjustments
* Bluetooth/WiFi control options
* Data logging and performance analysis
* Dynamic speed adjustment based on line complexity
* Machine learning for improved navigation

## 📃 License

This project is licensed under the MIT License - see the LICENSE file for details.

## 🙏 Acknowledgements

* ESP-IDF team for the excellent development framework
* ESP32 community for their support and resources
* All open source libraries used in this project

---

Created by **@dineth nawanjana**
*Last Updated: 2025-05-11*

```

---

Let me know if you'd like this saved as a downloadable file or converted to PDF.
```
