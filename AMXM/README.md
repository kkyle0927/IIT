## Quick Use Guide

1. Power on the **Angel SUIT H10**.
2. Connect the computer to the **C‑port** of the XM10.
3. Run the GUI:

   ```bash
   python data_saving_RT_monitoring_GUI.py
   ```

   (For first‑time users, ensure that all required Python packages are installed.)
4. Select the correct **serial port** in the GUI.
   If the port does not appear, click **Refresh**.
5. Choose the **output folder** and **output file name**.
6. **(Order is important)** Click **Connect & Start Logging** in the GUI.
7. **(Order is important)** Press the **second button on the XM10** controller to start data transmission.
   The LED will begin blinking, indicating active transmission.
8. **(Order is important)** Press the **second button on the XM10** again to stop data transmission.
   The LED will turn off.
9. **(Order is important)** Click **Disconnect & Stop** in the GUI to finish logging.
10. If you want to monitor a saved csv file, Run the GUI:

    ```bash
    python plot_total_data.py
    ```

This application provides real-time monitoring of MCU data via USB CDC, visualizes key variables in six live plots, and automatically logs the data into CSV files. A separate CSV Review Viewer allows users to browse and inspect complete recorded datasets immediately after logging.

---

## 1. Overview

**data_saving_RT_monitoring_GUI.py** is a PyQt-based GUI program designed for real-time data acquisition from an external MCU. The application:

* Connects to the MCU through a USB CDC serial interface.
* Reads structured data packets at high frequency.
* Validates packets using CRC16 (Modbus-style).
* Displays real-time graphs for selected variables.
* Stores all data into CSV files with efficient buffered writing.
* Automatically opens a CSV Review Viewer after logging ends.

The tool is useful for debugging, performance evaluation, data collection for research, and real-time monitoring of wearable robot systems.

---

## 2. Features

### Real-Time Monitoring

* Six plots arranged in a 2×3 grid.
* Groups include hip angles, torques, IMU accelerations/gyros, trunk IMU data, and FSR values.
* Updated at configurable intervals (`UPDATE_INTERVAL_MS`).

### Automatic Data Logging

* Data is saved as CSV with a fixed header format.
* Buffered writing (default flush every 500 lines) reduces disk I/O overhead.
* Output folder is automatically created if it does not exist.
* Default output path is `./data`.

### CRC-Protected Packet Handling

* Validates SOF, packet length, payload size, and CRC.
* Reports CRC mismatches and framing errors.
* Displays cumulative packet error count.

### CSV Review Viewer

* Opens automatically after logging stops.
* Displays eight larger plot groups.
* Checkboxes control visibility.
* Layout adjusts dynamically depending on selected plots.

### Modern GUI Theme

* Light theme with unified color palette.
* Rounded UI elements and improved readability.

---

## 3. Requirements

### Python Version

* Python **3.8 or later** recommended.

### Required Packages

Install dependencies using pip:

```bash
pip install pyqt5 pyqtgraph pyserial numpy
```

---

## 4. Running the Application

From the project directory:

```bash
python data_saving_RT_monitoring_GUI.py
```

After launching, follow these steps:

1. Select a serial port from the dropdown.
2. Set output folder (default: `./data`).
3. Set output CSV file name.
4. Press **Connect & Start Logging**.
5. Observe real-time plots.
6. Press **Disconnect & Stop** when done.
7. A CSV viewer will automatically open to inspect the recorded data.

---

## 5. GUI Components

### 5.1 Top Control Bar

* **Serial Port**: List of available serial devices.
* **Refresh**: Reload serial port list.
* **Connect & Start Logging**: Opens the port, sends start command, begins reception.
* **Disconnect & Stop**: Stops data reception, closes file, opens CSV review viewer.
* **Output folder**: Target directory for CSV storage.
* **Output file**: CSV file name.
* **Folder… / File…** buttons: Browse dialogs.

### 5.2 Status Indicators

* Current mode values: `H10Mode`, `H10AssistLevel`, `SmartAssist`.
* Packet error counter.
* Status bar messages for connection, CRC errors, and latest received data.

### 5.3 Real-Time Plot Groups (6)

Each plot displays the last `WINDOW_SIZE_SAMPLES` values (default: 1000 samples).

#### Group List

1. Hip Angles & Torques
2. Trunk IMU Local Acc (X/Y/Z)
3. Trunk IMU Local Gyro (X/Y/Z)
4. Trunk IMU Quaternion (W/X/Y/Z)
5. Left FSR (14 channels)
6. Right FSR (14 channels)

---

## 6. Packet Format and Decoding

Packet format matches the MCU structure:

```c
typedef struct __attribute__((packed)) {
    uint16_t sof;      // 0xAA55
    uint16_t len;      // Total packet size
    uint32_t loopCnt;
    uint8_t  h10Mode;
    uint8_t  h10AssistLevel;
    uint8_t  SmartAssist;
    float    ... (58 floats)
    uint16_t crc;      // CRC16 (0xFFFF, poly 0xA001)
} SavingData_t;
```

### Validation Steps

1. Check SOF (`0xAA55`).
2. Verify packet length.
3. Wait until full payload is received.
4. Compute CRC16 over `sof` to last payload byte.
5. Compare with received CRC.
6. Unpack payload using Python format:

```python
STRUCT_FMT = '<IBBB58f'
```

### Error Handling

* All packet errors are counted.
* Only valid packets emit `data_received` to the GUI.

---

## 7. CSV Logging

### Header

A fixed header is written at the top of each log file. It includes:

* Counters and mode fields.
* Hip kinematics and torques.
* Hip IMU (acc/gyro).
* Trunk IMU (acc/gyro/quaternion).
* 14×2 FSR channels.

### Logging Mechanics

* Each packet row is formatted and appended to a pending buffer.
* Every `FLUSH_EVERY` lines, data is flushed to disk.
* When disconnected or closing the app, all remaining lines are flushed.

### Output Path Handling

* Output folder defaults to: `./data`.
* Folders are automatically created.
* Example output:

```
data/
  cdc_monitoring_data_log_20250101_153022.csv
```

---

## 8. CSV Review Viewer

Launched automatically after logging stops.

### Features

* Eight plot groups (more detailed than the real-time view).
* Checkboxes control visibility.
* Layout adapts:

  * 1–4 plots → single column
  * 5–6 plots → 2×3 grid
  * 7–8 plots → 2×4 grid

### Plot Groups

1. Hip angles/torques
2. Hip IMU Acc (left/right)
3. Hip IMU Gyro (left/right)
4. Trunk Acc
5. Trunk Gyro
6. Trunk Quaternion
7. Left FSR (14)
8. Right FSR (14)

---

## 9. Git Usage (Optional)

To prevent CSV logs from entering the repository, add this to `.gitignore`:

```gitignore
*.csv
```

Or only ignore logs inside `data/`:

```gitignore
data/*.csv
```

If CSVs were already committed, untrack them:

```bash
git rm --cached data/*.csv
git commit -m "Stop tracking CSV logs"
```

---

## 10. Typical Workflow Summary

1. Start the GUI.
2. Select serial port.
3. Set output folder and file name.
4. Click **Connect & Start Logging**.
5. Observe real-time plots.
6. Click **Disconnect & Stop** to finish logging.
7. Review complete CSV using the built-in viewer.
8. Use the CSV for post-processing or machine learning tasks.

---

If you want, I can also generate a shorter "Quick Start" README version or create a version formatted for GitHub Pages or mkdocs.
