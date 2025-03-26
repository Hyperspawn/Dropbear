#include "HX711.h"
#include "SPIFFS.h"

// Shared clock pin for all HX711 modules
#define SHARED_HX711_SCK 4

// Define individual data pins for each HX711 load cell
// (These pins are chosen to avoid conflicts with the already occupied pins.)
#define LOADCELL_DOUT_PIN0 32
#define LOADCELL_DOUT_PIN1 12
#define LOADCELL_DOUT_PIN2 13
#define LOADCELL_DOUT_PIN3 15

// Create one HX711 instance per load cell
HX711 scale0;
HX711 scale1;
HX711 scale2;
HX711 scale3;

// Calibration factor for each load cell (default values; updated via calibration)
float calibrationFactor0 = 2280.0;
float calibrationFactor1 = 2280.0;
float calibrationFactor2 = 2280.0;
float calibrationFactor3 = 2280.0;

// Threshold for detecting a “pressed” load cell (adjust as needed)
#define PRESS_THRESHOLD 0.5

// Function prototypes
void calibrate();
float getStableReading(int cell, const char* phase);
float getWeightInput(String prompt);
void saveCalibrationToSPIFFS();
void loadCalibrationFromSPIFFS();

void setup() {
  Serial.begin(115200);
  while (!Serial);  // Wait for serial connection

  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed");
    return;
  }
  
  // Load saved calibration factors if available
  loadCalibrationFromSPIFFS();

  // Initialize each HX711 module with its dedicated data pin and shared clock pin
  scale0.begin(LOADCELL_DOUT_PIN0, SHARED_HX711_SCK);
  scale1.begin(LOADCELL_DOUT_PIN1, SHARED_HX711_SCK);
  scale2.begin(LOADCELL_DOUT_PIN2, SHARED_HX711_SCK);
  scale3.begin(LOADCELL_DOUT_PIN3, SHARED_HX711_SCK);
  
  // Set gain for each HX711 module (typically 128 for channel A)
  scale0.set_gain(128);
  scale1.set_gain(128);
  scale2.set_gain(128);
  scale3.set_gain(128);
  
  // Tare each scale to zero out the baseline (no load condition)
  scale0.tare();
  scale1.tare();
  scale2.tare();
  scale3.tare();
  
  // Set the calibration factors (from calibration or defaults)
  scale0.set_scale(calibrationFactor0);
  scale1.set_scale(calibrationFactor1);
  scale2.set_scale(calibrationFactor2);
  scale3.set_scale(calibrationFactor3);
  
  // Print header for the Serial Plotter and instructions
  Serial.println("Scale0,Scale1,Scale2,Scale3");
  Serial.println("Type /calibrate to enter calibration mode.");
}

void loop() {
  // Check if the user types a command into Serial
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.equalsIgnoreCase("/calibrate")) {
      calibrate();
    }
  }
  
  // Normal operation: get and print calibrated weight readings
  float weight0 = scale0.get_units();
  float weight1 = scale1.get_units();
  float weight2 = scale2.get_units();
  float weight3 = scale3.get_units();
  
  // Print the readings as CSV for the Serial Plotter
  Serial.print(weight0, 2);
  Serial.print(",");
  Serial.print(weight1, 2);
  Serial.print(",");
  Serial.print(weight2, 2);
  Serial.print(",");
  Serial.println(weight3, 2);
  
  delay(15);
}

//
// Calibration routine: Allows selection of a load cell by applying pressure,
// then guides the user through baseline and two known-force measurements.
//
void calibrate() {
  Serial.println("Entering calibration mode.");
  Serial.println("Apply pressure on one load cell to select it for calibration.");
  
  int selectedCell = -1;
  // Wait until one load cell shows a reading above the threshold
  while (selectedCell < 0) {
    float w0 = scale0.get_units();
    float w1 = scale1.get_units();
    float w2 = scale2.get_units();
    float w3 = scale3.get_units();
    
    if (w0 > PRESS_THRESHOLD) {
      selectedCell = 0;
    } else if (w1 > PRESS_THRESHOLD) {
      selectedCell = 1;
    } else if (w2 > PRESS_THRESHOLD) {
      selectedCell = 2;
    } else if (w3 > PRESS_THRESHOLD) {
      selectedCell = 3;
    }
    delay(100);
  }
  
  Serial.print("Load cell ");
  Serial.print(selectedCell);
  Serial.println(" detected. Do you want to calibrate this cell? (Y/N)");
  
  // Wait for user confirmation
  while (true) {
    while (!Serial.available()) { delay(100); }
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.equalsIgnoreCase("Y") || input.equalsIgnoreCase("YES")) {
      break;
    } else {
      Serial.println("Calibration cancelled.");
      return;
    }
  }
  
  Serial.println("Starting calibration for selected load cell.");
  
  // Step 1: Baseline measurement (no load)
  Serial.println("Step 1: Baseline measurement (no load). Remove all force from the load cell.");
  float baseline = getStableReading(selectedCell, "baseline");
  
  // Step 2: First calibration measurement with known force
  Serial.println("Step 2: First calibration measurement.");
  Serial.println("Apply a known force to the load cell and press Enter when ready.");
  float reading1 = getStableReading(selectedCell, "first calibration");
  float weight1 = getWeightInput("Enter the known weight (in kg) for the first calibration measurement:");
  
  // Step 3: Second calibration measurement with a different known force
  Serial.println("Step 3: Second calibration measurement.");
  Serial.println("Apply a different known force to the load cell and press Enter when ready.");
  float reading2 = getStableReading(selectedCell, "second calibration");
  float weight2 = getWeightInput("Enter the known weight (in kg) for the second calibration measurement:");
  
  // Calculate calibration factors from each measurement (assuming a linear response)
  float calFactor1 = (reading1 - baseline) / weight1;
  float calFactor2 = (reading2 - baseline) / weight2;
  float newCalFactor = (calFactor1 + calFactor2) / 2.0;
  
  // Update the calibration factor for the selected load cell and set the new scale factor
  switch (selectedCell) {
    case 0:
      calibrationFactor0 = newCalFactor;
      scale0.set_scale(newCalFactor);
      break;
    case 1:
      calibrationFactor1 = newCalFactor;
      scale1.set_scale(newCalFactor);
      break;
    case 2:
      calibrationFactor2 = newCalFactor;
      scale2.set_scale(newCalFactor);
      break;
    case 3:
      calibrationFactor3 = newCalFactor;
      scale3.set_scale(newCalFactor);
      break;
  }
  
  Serial.print("New calibration factor for load cell ");
  Serial.print(selectedCell);
  Serial.print(": ");
  Serial.println(newCalFactor);
  
  // Save updated calibration factors to SPIFFS
  saveCalibrationToSPIFFS();
  Serial.println("Calibration data saved to SPIFFS.");
  Serial.println("Exiting calibration mode.");
}

//
// Helper function: getStableReading
// Prompts the user to press Enter, then takes 50 samples from the chosen load cell.
// If the variation among the samples exceeds 5 units, it asks to try again.
//
float getStableReading(int cell, const char* phase) {
  while (true) {
    Serial.print("Press Enter to record ");
    Serial.print(phase);
    Serial.println(" measurement.");
    // Wait for user to press Enter
    while (!Serial.available()) { delay(100); }
    String dummy = Serial.readStringUntil('\n');
    
    float sum = 0;
    float minVal = 1e9;
    float maxVal = -1e9;
    for (int i = 0; i < 50; i++) {
      float reading = 0;
      switch (cell) {
        case 0: reading = scale0.get_units(); break;
        case 1: reading = scale1.get_units(); break;
        case 2: reading = scale2.get_units(); break;
        case 3: reading = scale3.get_units(); break;
      }
      sum += reading;
      if (reading < minVal) minVal = reading;
      if (reading > maxVal) maxVal = reading;
      delay(15);
    }
    if ((maxVal - minVal) > 5) {
      Serial.println("Unstable readings detected (variation > 5 units). Please try again.");
      continue;
    }
    float average = sum / 50.0;
    Serial.print("Recorded average reading: ");
    Serial.println(average);
    return average;
  }
}

//
// Helper function: getWeightInput
// Prompts the user to enter a weight (in kilograms) and returns the value.
//
float getWeightInput(String prompt) {
  Serial.println(prompt);
  while (true) {
    while (!Serial.available()) { delay(100); }
    String input = Serial.readStringUntil('\n');
    input.trim();
    float value = input.toFloat();
    // If conversion fails (and weight is not "0"), ask again
    if (value == 0 && input != "0" && input != "0.0") {
      Serial.println("Invalid input. Please enter a valid number.");
    } else {
      return value;
    }
  }
}

//
// Helper function: saveCalibrationToSPIFFS
// Saves the calibration factors for all load cells to a file in SPIFFS.
//
void saveCalibrationToSPIFFS() {
  File file = SPIFFS.open("/calibration.txt", FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open calibration file for writing.");
    return;
  }
  file.printf("%f,%f,%f,%f\n", calibrationFactor0, calibrationFactor1, calibrationFactor2, calibrationFactor3);
  file.close();
}

//
// Helper function: loadCalibrationFromSPIFFS
// Loads calibration factors from SPIFFS if the file exists; otherwise, default values remain.
//
void loadCalibrationFromSPIFFS() {
  if (!SPIFFS.exists("/calibration.txt")) {
    Serial.println("No calibration file found. Using default calibration factors.");
    return;
  }
  File file = SPIFFS.open("/calibration.txt", FILE_READ);
  if (!file) {
    Serial.println("Failed to open calibration file for reading.");
    return;
  }
  String data = file.readStringUntil('\n');
  file.close();
  
  int firstComma = data.indexOf(',');
  int secondComma = data.indexOf(',', firstComma + 1);
  int thirdComma = data.indexOf(',', secondComma + 1);
  if (firstComma == -1 || secondComma == -1 || thirdComma == -1) {
    Serial.println("Calibration file format error. Using default calibration factors.");
    return;
  }
  calibrationFactor0 = data.substring(0, firstComma).toFloat();
  calibrationFactor1 = data.substring(firstComma + 1, secondComma).toFloat();
  calibrationFactor2 = data.substring(secondComma + 1, thirdComma).toFloat();
  calibrationFactor3 = data.substring(thirdComma + 1).toFloat();
  Serial.println("Calibration factors loaded from SPIFFS.");
}
