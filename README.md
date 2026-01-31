# 🐭 MazeMaster 2026 - Team MazeRiders

![Competition Status](https://img.shields.io/badge/Status-Top%205%20Finalist-gold)
![Microcontroller](https://img.shields.io/badge/MCU-Arduino%20Nano-blue)
![Algorithm](https://img.shields.io/badge/Algorithm-Flood%20Fill-green)

The official code repository for **Team MazeRiders**, a Top 5 Finalist at the **MazeMaster 2026** Micromouse competition organized by IEEE RAS Uva Wellassa University.

This project marks our transition from basic obstacle avoidance to an autonomous, intelligent maze-solving robot using the **Flood Fill Algorithm**.

---

## 🚀 Key Features

* **⚡ Autonomous Navigation:** Implements a full Flood Fill Algorithm to map and solve a 16x16 maze dynamically.
* **🧠 Two-Way Search Logic:** Features a robust "Start $\to$ Center $\to$ Start" verification run before committing the map to memory.
* **💾 Dual-Mode Operation:**
    * **Search Mode:** Explores the maze, updates wall data, and saves the verified map to EEPROM.
    * **Fast Run:** Loads the saved map and executes a high-speed run using a "Strict Visited Mask" for safety.
* **⚙️ Advanced Control System:**
    * **PID Wall Following:** Precision control using tuned $K_p$ and $K_d$ constants.
    * **Dual-Encoder Odometry:** Uses Hardware Interrupts and PCINT for precise distance tracking.
    * **Virtual Gyro:** Uses encoder differentials to maintain a straight heading in open spaces.

---

## 🛠️ Hardware Stack

| Component | Specification |
| :--- | :--- |
| **Microcontroller** | Arduino Nano (ATmega328P) |
| **Sensors** | 3x VL53L0X Time-of-Flight (ToF) Laser Sensors |
| **Motors** | N20 Gear Motors with Magnetic Encoders |
| **Driver** | TB6612FNG Dual Channel Motor Driver |
| **Power** | 2S LiPo (7.4V - 8.4V) |
| **Chassis** | Custom PCB & 3D Printed Structural Components |

---

## 🏎️ How It Works

### 1. The Algorithm
The mouse treats the maze as a grid of values representing the distance to the center. We use the Flood Fill equation to update the "cost" of each cell:

$$Distance(cell) = 1 + \min(Neighbors)$$



### 2. Competition Strategy
1.  **Search Run:** The robot starts at (0,0), finds the center, and immediately drives back to the start to verify the path. The verified map is saved to non-volatile memory (**EEPROM**).
2.  **Fast Run:** The user flips a hardware switch. The robot loads the map and races to the center without scanning, strictly avoiding unvisited cells to prevent collisions.

---

## 📂 Project Structure

```bash
├── 📄 Master_Code_Final.ino    # The final competition code (Search + Fast Run)
├── 📂 Tuning_Tools             # Calibration scripts
│   ├── Tune_Distance.ino       # Tune TICKS_PER_CELL
│   └── Tune_Rotation.ino       # Tune TICKS_FOR_90
└── 📜 README.md                # Project documentation