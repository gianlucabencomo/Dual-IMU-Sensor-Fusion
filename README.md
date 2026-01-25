# Dual-IMU Sensor Fusion

This project connects two Inertial Measurement Units (IMUs) to a single Raspberry Pi I2C bus for high-precision motion tracking and sensor fusion.

## Hardware Wiring

| Raspberry Pi Pin | Function | IMU 1 Pin | IMU 2 Pin |
| :--- | :--- | :--- | :--- |
| **Pin 1 (3.3V)** | Power | VCC | VCC |
| **Pin 3 (GPIO 2)** | SDA (Data) | SDA | SDA |
| **Pin 5 (GPIO 3)** | SCL (Clock) | SCL | SCL |
| **Pin 6 (GND)** | Ground | GND | GND |
| **-** | **Address Select** | (Leave Floating/GND) | **AD0 to VCC** |



---

## Pin Definitions

* **VCC:** Power input (3.3V).
* **GND:** Ground reference.
* **SCL:** Serial Clock (The metronome for I2C data).
* **SDA:** Serial Data (The actual movement/rotation numbers).
* **AD0:** **Address Select.**
    * IMU 1 (Default): Address 0x68.
    * IMU 2 (AD0 tied to VCC): Address 0x69.
    * This allows both sensors to share one party line without a data collision.
* **INT:** **Interrupt.** The trigger pin. It tells the Pi exactly when new data is ready, saving CPU power and improving timing (Optional).
* **XCL / XDA:** **Auxiliary I2C.** Used for connecting a third sensor (like a compass) directly to the IMU (Unused).

---

## Setup & Verification

1.  **Enable I2C on the Pi:**
    ```bash
    sudo raspi-config
    # Navigate to Interface Options -> I2C -> Enable -> Yes
    ```

2.  **Install I2C Tools:**
    ```bash
    sudo apt-get update
    sudo apt-get install -y i2c-tools
    ```

3.  **Run the Hardware Roll Call:**
    Scan the bus to ensure both sensors are detected:
    ```bash
    i2cdetect -y 1
    ```

### Expected Output
If wired correctly, you will see both hex addresses appearing in the table:
```text
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
60: -- -- -- -- -- -- -- -- 68 69 -- -- -- -- -- --