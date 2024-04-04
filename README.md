# Borda Academy Embedded Software Internship Submission
**Environmental Sensor for Borda Academy Embedded Software Development Qualification Exam**

This code implements an environmental sensor for temperature, humidity, and CO2 on an ESP32 platform. It utilizes Bluetooth Low Energy (BLE) to transmit filtered sensor data to a connected device.

**Features:**

* Reads simulated sensor data for temperature, humidity, and CO2.
* Applies a median filter to remove noise from sensor readings.
* Calculates and transmits statistics (max, min, median, standard deviation) for each sensor in a packet every 30 seconds.
* Utilizes FreeRTOS for multi-core processing to improve efficiency.
* Handles connection/disconnection events and restarts advertising automatically.

**Hardware Requirements:**

* ESP32 development board

**Software Requirements:**

* Arduino IDE
* ESP32 IDF (optional, for advanced development)

**Code Structure:**

* `E7.ino`: Main program code
* Header files included:
    * `<stdint.h>`: Standard integer types
    * `<BLEDevice.h>`: BLE library header
    * `<BLEServer.h>`: BLE server header
    * `<BLEUtils.h>`: BLE utility functions
    * `<BLE2902.h>`: BLE characteristic descriptor header
    * `<freertos/FreeRTOS.h>`: FreeRTOS kernel header
    * `<freertos/task.h>`: FreeRTOS task management header
    * `<freertos/semphr.h>`: FreeRTOS semaphore header
    * `<stdlib.h>`: Standard library functions
    * `<pthread.h>`: POSIX threads header (for mutex)

**Key Functions:**

* `setup()`: Initializes the device, configures BLE, creates tasks, and starts advertising.
* `getTemperature()`, `getHumidity()`, `getCO2()`: Simulated sensor reading functions (replace with actual sensor code).
* `find_median(float data[])`: Calculates the median of a data array (used for filtering).
* `calculate_std(float data[])`: Calculates the standard deviation of a data array.
* `Calculate_and_Pack(circular_buffer_t* buffer, int type)`: Calculates statistics for a specific sensor and packs them into a string.
* `producer_thread()`: Continuously reads sensor data, applies filtering, fills circular buffers, and handles data ready signals.
* `consumer_thread()`: Waits for data in buffers, applies calculations, prepares data packets, transmits data, and handles empty buffer signals.

**Notes:**

* This code uses POSIX mutexes (`pthread_mutex_t`) for thread synchronization.
* The code includes comments and Serial Monitor prints for debugging purposes.
* Modify the simulated sensor reading functions (`getTemperature()`, etc.) to integrate with your actual sensors.

**Getting Started:**

1. Install the Arduino IDE and ESP32 development environment.
2. Copy the code into a new Arduino sketch.
3. Connect your ESP32 board and upload the code.
4. Open the Arduino Serial Monitor at a baud rate of 115200.
5. The code will print messages and wait for a connection.
6. Use a BLE scanner app on your smartphone or another device to connect to the sensor ("Environmental Sensor").
7. After connection, the sensor will transmit environmental data packets every 30 seconds.

**License:**

This code is provided for educational purposes only. You are free to modify and use it as needed, but please respect the original authorship.

