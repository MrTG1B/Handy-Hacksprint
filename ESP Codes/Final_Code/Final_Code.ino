#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "HardwareSerial.h"

// Create a second hardware serial object for HM-10
HardwareSerial HM10(2); // Use UART2, which is GPIO16 (RX2) and GPIO17 (TX2)

// Initialize the PCA9685 driver
// We are using the default I2C address (0x40)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ESP32 I2C Pins (default)
#define SDA_PIN 21
#define SCL_PIN 22

// Servo pulse length settings. These may need tuning
// for your specific servos.
#define SERVOMIN 150  // Min pulse length (0 degrees)
#define SERVOMAX 600  // Max pulse length (180 degrees)
#define SERVO_FREQ 50 // PWM frequency for analog servos (50Hz)


// Number of servos
const int NUM_SERVOS = 5;

// FSR and Motor Pins
const int numFSRs = 4;
const int fsrPins[numFSRs] = {32, 35, 34, 36};     // FSR pins array
const int motorPins[numFSRs] = {4, 5, 18, 19};     // Motor pins array

// EMG sensor pin and threshold
const int emgPin = 39;

// Thresholds
const int fsrThreshold = 500;  // Adjust based on your FSR sensitivity

// Bluetooth setup (using Serial2 for communication)
#define RXD2 16
#define TXD2 17

// bool commandExecuted = false; // REMOVED - Replaced by new state flags

String commandData="";

// --- New EMG Averaging and State Logic ---
// --- Fast Average (What the sensor is doing NOW) ---
const int fast_window_size = 5;
int fast_values[fast_window_size];
int fast_value_index = 0;
long fast_sum = 0;

// --- Long Average (The "center" or "baseline" signal) ---
const int long_window_size = 100; // Larger window to track the slow drift
int long_values[long_window_size];
int long_value_index = 0;
long long_sum = 0;

// --- Threshold ---
// This is the trigger for the 'deviation'. 
int threshold = 200; // You may need to tune this value

// --- State Tracking ---
bool flex_detected_flag = false;
bool switch_state = false; // false = Hand Open, true = Hand Gripped

typedef void (*CommandFunction)();

void setup() {
  // Initialize motor pins as outputs
  for (int i = 0; i < numFSRs; i++) {
    pinMode(motorPins[i], OUTPUT);
    digitalWrite(motorPins[i], LOW);  // Start with motors off
  }

  // Initialize EMG pin as input
  pinMode(emgPin, INPUT);

  // Initialize serial communication for HM-10 and debugging
  Serial.begin(115200);                        // Debugging on Serial Monitor
  
  Serial.println("PCA9685 Servo Test for ESP32");

  // Initialize I2C communication on the ESP32's pins
  Wire.begin(SDA_PIN, SCL_PIN);

  // Initialize the PCA9685 driver
  pwm.begin();
  
  // Set the PWM frequency
  pwm.setPWMFreq(SERVO_FREQ);

  Serial.println("Setup complete. Waiting for Bluetooth data...");
  
  HM10.begin(9600, SERIAL_8N1, RXD2, TXD2); // HM-10 on Serial2 (Pins 16 and 17)
  
  // --- EMG CALIBRATION ---
  Serial.println("Calibrating EMG... Please relax for a few seconds.");
  // Initialize both value arrays
  for (int i = 0; i < fast_window_size; i++) {
    fast_values[i] = 0;
  }
  for (int i = 0; i < long_window_size; i++) {
    long_values[i] = 0;
  }

  // Prime the averages with initial readings to avoid a long startup
  for (int i = 0; i < long_window_size; i++) {
    int initial_reading = analogRead(emgPin);
    long_values[i] = initial_reading;
    long_sum += initial_reading;
    
    // Also fill the fast average
    if (i < fast_window_size) {
      fast_values[i] = initial_reading;
      fast_sum += initial_reading;
    }
    delay(15); // Short delay to get good samples
  }
  Serial.println("Calibration complete. Ready to flex.");
  
  Serial.println("Waiting for Bluetooth data...");
}

void loop() {
  // Check for available data from Bluetooth
  if (HM10.available()) {
    commandData = HM10.readString(); // Read incoming character
    Serial.println("Stored command: " + commandData); 
  }

  // Handle FSR readings and control vibration motors
  for (int i = 0; i < numFSRs; i++) {
    int fsrValue = analogRead(fsrPins[i]);

    // Map FSR values to motor intensity (0-255 for PWM control)
    int motorIntensity = map(fsrValue, fsrThreshold, 4095, 0, 255); // Adjust 4095 if using a 10-bit ADC
    motorIntensity = constrain(motorIntensity, 0, 255); // Constrain to valid PWM range

    analogWrite(motorPins[i], motorIntensity); // Set motor intensity based on FSR value
  }
  
  // Check EMG data
  updateEmgState();
  delay(50); // Delay from new EMG logic
}

/**
 * @brief Reads EMG, updates averages, and toggles hand state.
 */
void updateEmgState() {
  // --- Read the new value ---
  int new_value = analogRead(emgPin);

  // --- 1. Calculate the Fast Average (your "current" signal) ---
  fast_sum = fast_sum - fast_values[fast_value_index];
  fast_values[fast_value_index] = new_value;
  fast_sum = fast_sum + new_value;
  fast_value_index = (fast_value_index + 1) % fast_window_size;
  float fast_avg = (float)fast_sum / fast_window_size;

  // --- 2. Calculate the Long Average (your "baseline" or "center") ---
  long_sum = long_sum - long_values[long_value_index];
  long_values[long_value_index] = new_value;
  long_sum = long_sum + new_value;
  long_value_index = (long_value_index + 1) % long_window_size;
  float long_avg = (float)long_sum / long_window_size;

  // --- 3. Calculate Deviation ---
  // This is now the difference between the fast and slow signals
  float deviation = abs(fast_avg - long_avg); 

  // --- Toggle Logic ---
  // Triggers if deviation goes ABOVE your threshold (e.g., 200)
  if (!flex_detected_flag && deviation > threshold) {
    switch_state = !switch_state; // Toggle the state
    flex_detected_flag = true;     // Mark that we just flexed
    
    if (switch_state) {
      Serial.println("EMG State: ON - Executing Command");
      executeCommand(commandData); // Perform the grip
    } else {
      Serial.println("EMG State: OFF - Opening Hand");
      openAllServos(); // Release the grip
    }
  }
  
  // --- Reset Logic ---
  // Resets when deviation drops BELOW 50% of threshold (e.g., 100)
  else if (flex_detected_flag && deviation < threshold * 0.5) { 
    flex_detected_flag = false; // Ready for next toggle
  }
}


void openAllServos() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    setServoAngle(i, 0);
  }
  Serial.println("All five servos opened to 0 degrees.");
}

/**
 * @brief Sets the angle of a specific servo.
 * @param servoNum The servo number (0-15) to control.
 * @param angle The desired angle (0-180 degrees).
 */
void setServoAngle(uint8_t servoNum, int angle) {
  // Map the 0-180 degree angle to the min/max pulse length
  int pulseLength = map(angle, 0, 180, SERVOMIN, SERVOMAX);

  // Set the PWM for the specified servo
  // The '0' means the pulse starts at tick 0
  pwm.setPWM(servoNum, 0, pulseLength);
}


// Command functions
void func1() {
  for (int i = 0; i < 5; i++) { 
    setServoAngle(i, 90);
  }
}

void func2() {
  setServoAngle(0, 0);
  for (int i = 1; i < 5; i++) { 
    setServoAngle(i, 180);
  }
}

void func3() {
  setServoAngle(0, 180);
  setServoAngle(1, 45);
  setServoAngle(2, 90);
  setServoAngle(3, 135);
  setServoAngle(4, 180);
}

void func4() {
  setServoAngle(0, 180);
  for (int i = 1; i < 5; i++) { 
    setServoAngle(i, 30);
  }
}

void func5() {
  for (int i = 0; i < 5; i++) { 
    setServoAngle(i, 180);
  }
}

// Optimized: func6, 7, and 8 just call func5 to avoid duplicate code
void func6() { func5(); }

void func7() { func5(); }

void func8() { func5(); }

// Removed func9, func10, func11, func12 as they were for wrist control

void func13(){
  setServoAngle(0, 180);
  setServoAngle(2, 180);
  setServoAngle(3, 180);
  setServoAngle(1, 0);
  setServoAngle(4, 0);
}

void func14(){
  for(int i=0;i<5;i++)
  {
    setServoAngle(i, 180);
  }
  delay(100);
  setServoAngle(2, 0);
}



// Array of function pointers
CommandFunction functions[] = {
  func1, func2, func3, func4, func5, func6, func7,
  func8, func13, func14 // Removed func9, 10, 11, 12
};


void executeCommand(String command) {
  int index = command.toInt();  // Convert string to integer
  // Note: In Arduino, use sizeof(functions)/sizeof(functions[0]) instead of .length()
  if (index >= 1 && index <= sizeof(functions)/sizeof(functions[0])) {
    functions[index - 1]();  // Arrays are 0-based, so subtract 1
  } else {
    Serial.println("Unknown command received.");
  }
}

