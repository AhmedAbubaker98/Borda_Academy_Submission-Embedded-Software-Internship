// ___________________________________________________________
// |Author: Ahmed Elageib, Ahmedkata@gmail.com
// |For: Borda Academy program 
// |As part of: Embedded Software Department Qualification Exam
// |
// |This code implements an environmental sensor for 
// |[Temperature, Humidity, CO2] on an ESP32 platform. 
// |It utilizes Bluetooth Low Energy (BLE) to transmit 
// |filtered sensor data to a connected device.
// |____________________________________________________________
#include <stdint.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <freertos/FreeRTOS.h> 
#include <freertos/task.h> 
#include <freertos/semphr.h> 
#include <stdlib.h>
#include <pthread.h>

#define LED_PIN 2
#define BUFFER_SIZE 10
#define WINDOW_SIZE 5
#define SERVICE_UUID        (BLEUUID((uint16_t)0x181A)) // Bluetooth standards assigned value for the title "Environmental Sensing"
#define CHARACTERISTIC_UUID (BLEUUID((uint16_t)0x2AF4)) // Bluetooth standards assigned value for the title "Event Statistics"
// Circular buffer structure
typedef struct {
  float buffer[BUFFER_SIZE];
  uint8_t head;
  uint8_t tail;
  } circular_buffer_t;
//Initialization of essential Variables
  BLEServer* pServer = NULL;
  BLECharacteristic* pCharacteristic = NULL;
  bool deviceConnected = false;
  bool oldDeviceConnected = false;
  String Packet = "";
  int Timer = 0;
  const int SamplingTime = 1000; //in milliSeconds
  const int AdvertisingTime = 30; //in Seconds
  int discardedValues = 0; // Counter for discarded values due to lagging
  //Circular Buffers
  circular_buffer_t temperature_buffer = {0};
  circular_buffer_t humidity_buffer = {0};
  circular_buffer_t CO2_buffer = {0};
  //Temporary arrays to store sensor data before filtering
  float temperature_array[BUFFER_SIZE];
  float humidity_array[BUFFER_SIZE];
  float CO2_array[BUFFER_SIZE];
  //Floats to store the filtered variable from each array
  float filtered_temperature;
  float filtered_humidity;
  float filtered_CO2;
  //FreeRTOS variables
  TaskHandle_t Task1;
  TaskHandle_t Task2;
  pthread_mutex_t xMutex;  // POSIX mutex
  bool isEmpty = true;
  pthread_cond_t notEmpty, notFull;
//connection status srever call back
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

void setup() {//Here we setup the device by doing the following: 
              //1- Getting simulated sensor readings and fill the Buffers firstly (otherwise first calculations will include zeros and give incorrect statistics)
              //2- Setup the Bluetooth Low-Energy device 3- Setup the Real-Time Operating System tasks for each CPU
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  // Blink LED three times on startup
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
    Serial.println(i + 1);
  }
  // variables necessary for lag detection preventing race condition between tasks
  pthread_mutex_init(&xMutex, NULL);
  pthread_cond_init(&notEmpty, NULL);
  pthread_cond_init(&notFull, NULL);
  //// Simulated sensor readings and fill the Buffers firstly (otherwise first calculations will include zeros and give incorrect statistics)
  for(int j = 0 ; j < BUFFER_SIZE ; j++){
    for (int i = 0 ; i < BUFFER_SIZE ; i++) {
    temperature_array[i] = getTemperature();
    humidity_array[i] = getHumidity();
    CO2_array[i] = getCO2();
    }
    filtered_temperature = find_median(temperature_array);
    filtered_humidity = find_median(humidity_array);
    filtered_CO2 = find_median(CO2_array);

    // Add filtered data to circular buffers and discard oldest data on overflow
    temperature_buffer.buffer[temperature_buffer.head] = filtered_temperature;
    temperature_buffer.head = (temperature_buffer.head + 1) % BUFFER_SIZE;
    if (temperature_buffer.head == temperature_buffer.tail) {// Only advance the tail if the buffer is full, not directly after adding a value
      temperature_buffer.tail = (temperature_buffer.tail + 1) % BUFFER_SIZE;
    }

    humidity_buffer.buffer[humidity_buffer.head] = filtered_humidity;
    humidity_buffer.head = (humidity_buffer.head + 1) % BUFFER_SIZE;
    if (humidity_buffer.head == humidity_buffer.tail) {// Only advance the tail if the buffer is full, not directly after adding a value
      humidity_buffer.tail = (humidity_buffer.tail + 1) % BUFFER_SIZE;
    }

    CO2_buffer.buffer[CO2_buffer.head] = filtered_CO2;
    CO2_buffer.head = (CO2_buffer.head + 1) % BUFFER_SIZE;
    if (CO2_buffer.head == CO2_buffer.tail) {// Only advance the tail if the buffer is full, not directly after adding a value
      CO2_buffer.tail = (CO2_buffer.tail + 1) % BUFFER_SIZE;
    }     
  }

  //// Create the BLE Device
  BLEDevice::init("Environmental Sensor");
  BLEDevice::setMTU(517); //The current Packet to be sent only requires 190
                          //but it is set as 517 in case client (real person not BLE client) needs to add more content at any point
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  // Create a BLE Descriptor
  pCharacteristic->addDescriptor(new BLE2902());
  
  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  
  //Wait for connection to start sending data (notify)
  Serial.println("Setup Done, Waiting for connection...");
  while(!deviceConnected){
      //Blink rapidly while not connected
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
  }
  digitalWrite(LED_PIN, HIGH);  // Keep LED on while threads are running
  //Each task is assigned to diferent core to decrease load and make efficient use of resources
  xTaskCreatePinnedToCore(producer_thread,"Task1",10000,NULL,1,&Task1,1); //Core 1 in ESP32                      
  delay(500); 
  xTaskCreatePinnedToCore(consumer_thread,"Task2",10000,NULL,1,&Task2,0); //Core 0 in ESP32 
  delay(500);
}


// Function prototypes for simulated sensor readings (to be replaced with actual sensor functions)
float getTemperature() { return random(20, 30);}// Simulate temperature between 20-30 degrees
float getHumidity() { return random(30, 60);}// Simulate humidity between 30-60%
float getCO2() { return random(400, 500);} // Simulate CO2 between 400-500 ppm

float calculate_std(float data[]) { // To calculate standard deviation
  float sum = 0.0, mean, variance = 0.0;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    sum += data[i];
  }
  mean = sum / BUFFER_SIZE;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    variance += pow(data[i] - mean, 2);
  }
  return sqrt(variance / BUFFER_SIZE);
  }
float find_median(float data[]) { // To find moving median based on a window size
  float window[WINDOW_SIZE];

  for (int i = 0; i < WINDOW_SIZE; i++) {
      // Copy data into the window, handling potential overflow at the end
      int window_index = i % WINDOW_SIZE;
      window[window_index] = data[i];
  }
  // Sort the data
  for (int i = 0; i < WINDOW_SIZE - 1; i++) {
    for (int j = 0; j < WINDOW_SIZE - i - 1; j++) {
      if (window[j] > window[j + 1]) {
        float temp = window[j];
        window[j] = window[j + 1];
        window[j + 1] = temp;
      }
    }
  }
  // If even number of elements, return average of middle two
  if (WINDOW_SIZE % 2 == 0) {
    return (window[WINDOW_SIZE / 2 - 1] + window[WINDOW_SIZE / 2]) / 2.0;
  } else {
    // If odd number of elements, return middle element
    return window[WINDOW_SIZE / 2];
  }
  }
  
void print_circular_buffer(const char* label, circular_buffer_t* buffer) { //Print the values of a Circular Buffer(for debugging)

  // Calculate number of elements
  int elements = (buffer->head - buffer->tail + BUFFER_SIZE) % BUFFER_SIZE;

  // Print label
  Serial.println(label);

  // Iterate through the buffer, handling wrap-around
  int index = buffer->tail;
  for (int i = 0; i < elements; i++) {
    Serial.print("-> ");
    Serial.print(buffer->buffer[index]);
    index = (index + 1) % BUFFER_SIZE;
  }
  Serial.println();
  }

String Calculate_and_Pack(circular_buffer_t* buffer,int type) { //Calculate specific statistics from a Buffer
    int elements = (buffer->head - buffer->tail + BUFFER_SIZE) % BUFFER_SIZE;

    if (elements == 0) {
      Serial.println("Buffer is empty. No statistics available.");
    }

    float max_value = buffer->buffer[buffer->tail];
    float min_value = buffer->buffer[buffer->tail];
    float sum = 0.0;

    // Iterate through the buffer, handling wrap-around
    int index = buffer->tail;
    for (int i = 0; i < elements; i++) {
      if (buffer->buffer[index] > max_value) {
        max_value = buffer->buffer[index];
      }
      if (buffer->buffer[index] < min_value) {
        min_value = buffer->buffer[index];
      }
      sum += buffer->buffer[index];
      index = (index + 1) % BUFFER_SIZE;
    }

    float std_dev = calculate_std(buffer->buffer + buffer->tail); // Access data starting from tail
    float median = find_median(buffer->buffer + buffer->tail); // Access data starting from tail
  String str = "";
  switch (type) {
    case 1:
    Serial.println("Temperature Sensor statistics:"); 
    Serial.print(" | Max: ");
    Serial.print(max_value);
    Serial.print("°");
    //
    str = str + " T_max: ";
    str = str + max_value;

    Serial.print(" | Min: ");
    Serial.print(min_value);
    Serial.print("°");
    //
    str = str + " T_min: ";
    str = str + min_value;

    Serial.print(" | Median: ");
    Serial.print(median);
    Serial.print("°");
    //
    str = str + " T_med: ";
    str = str + median;

    Serial.print(" | Std. Dev: ");
    Serial.print(std_dev);
    Serial.print("°"); Serial.println("");
    //
    str = str + " T_std.dev: ";
    str = str + std_dev;
    str = str + " | ";
    break;
    case 2:
    Serial.println("Humidity Sensor statistics:"); 
    Serial.print(" | Max: ");
    Serial.print(max_value);
    Serial.print("%");
    //
    str = str + " H_max: ";
    str = str + max_value;

    Serial.print(" | Min: ");
    Serial.print(min_value);
    Serial.print("%");
    //
    str = str + " H_min: ";
    str = str + min_value;

    Serial.print(" | Median: ");
    Serial.print(median);
    Serial.print("%");
    //
    str = str + " H_med: ";
    str = str + median;

    Serial.print(" | Std. Dev: ");
    Serial.print(std_dev);
    Serial.print("%"); Serial.println("");
    //
    str = str + " H_std.dev: ";
    str = str + std_dev;
    str = str + " | ";
    break;
    case 3:
    Serial.println("CO2 Sensor statistics:"); 
    Serial.print(" | Max: ");
    Serial.print(max_value);
    Serial.print(" ppm");
    //
    str = str + " C_max: ";
    str = str + max_value;

    Serial.print(" | Min: ");
    Serial.print(min_value);
    Serial.print(" ppm");
    //
    str = str + " C_min: ";
    str = str + min_value;

    Serial.print(" | Median: ");
    Serial.print(median);
    Serial.print(" ppm");
    //
    str = str + " C_med: ";
    str = str + median;
    
    Serial.print(" | Std. Dev: ");
    Serial.print(std_dev);
    Serial.print(" ppm"); Serial.println("");
    //
    str = str + " C_std.dev: ";
    str = str + std_dev;
    str = str + " | ";
    break;
  }

    return str;
}
void producer_thread(void *pvParameters) {
  while (1) {
    if (deviceConnected){
      // Wait for empty space in the buffer
      pthread_mutex_lock(&xMutex);
        while (isEmpty) {
          pthread_cond_wait(&notFull, &xMutex);
        }  // Take mutex to protect critical section
      Serial.print("Producer is running on core ");
      Serial.println(xPortGetCoreID());

      // Get simulated sensor readings to re-fill the arrays with new values so they then get median filtered
      for (int i = 0 ; i < BUFFER_SIZE ; i++) {
      temperature_array[i] = getTemperature();
      humidity_array[i] = getHumidity();
      CO2_array[i] = getCO2();
      }
      //apply median filter to each sensor data array
      filtered_temperature = find_median(temperature_array);
      filtered_humidity = find_median(humidity_array);
      filtered_CO2 = find_median(CO2_array);

      // Add filtered data to circular buffers and discard oldest data on overflow
      temperature_buffer.buffer[temperature_buffer.head] = filtered_temperature;
      temperature_buffer.head = (temperature_buffer.head + 1) % BUFFER_SIZE;
      if (temperature_buffer.head == temperature_buffer.tail) {  // Advance the tail if the buffer is full
        temperature_buffer.tail = (temperature_buffer.tail + 1) % BUFFER_SIZE;
      }
      
      humidity_buffer.buffer[humidity_buffer.head] = filtered_humidity;
      humidity_buffer.head = (humidity_buffer.head + 1) % BUFFER_SIZE;
      if (humidity_buffer.head == humidity_buffer.tail) {  // Advance the tail if the buffer is full
        humidity_buffer.tail = (humidity_buffer.tail + 1) % BUFFER_SIZE;
      }

      CO2_buffer.buffer[CO2_buffer.head] = filtered_CO2;
      CO2_buffer.head = (CO2_buffer.head + 1) % BUFFER_SIZE;
      if (CO2_buffer.head == CO2_buffer.tail) {  // Advance the tail if the buffer is full
        CO2_buffer.tail = (CO2_buffer.tail + 1) % BUFFER_SIZE;
      }
      //// Uncomment the next 3 lines to Print Circular Buffers contents (for debugging) ////
      // print_circular_buffer("Temperature Buffer:", &temperature_buffer);
      // print_circular_buffer("Humidity Buffer:", &humidity_buffer);
      // print_circular_buffer("CO2 Buffer:", &CO2_buffer);
      Serial.println("Sampling Done!");

      // Release mutex
      isEmpty = true;  // Set flag to indicate buffer is no longer empty
      pthread_cond_signal(&notEmpty);  // Signal data available
    
      pthread_mutex_unlock(&xMutex);
      vTaskDelay( (SamplingTime / 2) / portTICK_PERIOD_MS ); //Sampling time is divided in two delays between consumer and producer 
                                                            //because the delay period is effectively used for mutex exchange to happen      

    }    
  
    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
      delay(500); // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising(); // restart advertising 

      oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
      oldDeviceConnected = deviceConnected;
    }  
    //Blink Rapidly while Diconnected
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}
void consumer_thread(void *pvParameters) {
  TickType_t xLastUpdateTime = xTaskGetTickCount();
  while (1) {    
    if (deviceConnected){
      TickType_t xCurrentTime = xTaskGetTickCount();
      if (xCurrentTime - xLastUpdateTime > (SamplingTime * BUFFER_SIZE)) { //function to account for lag by consumer which would cause a race condition if unhandled
        Serial.println("WARNING: Consumer lagging behind producer - potential data loss!");
        Serial.print("Discarding old data..."); //fate of oldest data is to be discarded
      }      
      xLastUpdateTime = xCurrentTime;
      
      // Wait for data in the buffer
      pthread_mutex_lock(&xMutex);
      while (!isEmpty) {
        pthread_cond_wait(&notEmpty, &xMutex);
      }    // Discard last 5 values if lagging was detected
      if (discardedValues > 0) {
        int elements = (temperature_buffer.head - temperature_buffer.tail + BUFFER_SIZE) % BUFFER_SIZE;
        temperature_buffer.tail = (temperature_buffer.tail + elements - discardedValues) % BUFFER_SIZE;

        elements = (humidity_buffer.head - humidity_buffer.tail + BUFFER_SIZE) % BUFFER_SIZE;
        humidity_buffer.tail = (humidity_buffer.tail + elements - discardedValues) % BUFFER_SIZE;

        elements = (CO2_buffer.head - CO2_buffer.tail + BUFFER_SIZE) % BUFFER_SIZE;
        CO2_buffer.tail = (CO2_buffer.tail + elements - discardedValues) % BUFFER_SIZE;
        discardedValues = 0; // Reset discard counter
      }

      Serial.print("Consumer is running on core ");
      Serial.println(xPortGetCoreID());

      // Apply filtering and calculations 
      float filtered_temperature = find_median(temperature_array);
      float filtered_humidity = find_median(humidity_array);
      float filtered_co2 = find_median(CO2_array);

      //data transmission logic 
      Timer++;
      Serial.print("Timer: ");
      Serial.println(Timer);

      if (Timer % AdvertisingTime == 0){
        Packet = Calculate_and_Pack(&temperature_buffer, 1);  // Pass buffer pointers to the function along with a case identifier
        Packet = Packet + Calculate_and_Pack(&humidity_buffer, 2);
        Packet = Packet + Calculate_and_Pack(&CO2_buffer, 3);
        pCharacteristic->setValue(Packet.c_str());
        //Packet = "";
        pCharacteristic->notify();
        Timer = 0;
      }
      
      // Release mutex
      isEmpty = false;  // Set flag to indicate empty space available
      pthread_cond_signal(&notFull);  // Signal empty slot available

      pthread_mutex_unlock(&xMutex);
      vTaskDelay( (SamplingTime / 2) / portTICK_PERIOD_MS ); //Sampling time is dividedin two delays between consumer and producer 
                                                            //because the delay period is effectively used for mutex exchange to happen
    } 
    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("Disconnected, Waiting for Connection...");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }  
    //Blink rapidly while Disconnected
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}


void loop() {
//Redundant
}
