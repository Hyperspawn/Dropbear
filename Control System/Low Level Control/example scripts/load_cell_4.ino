#include "HX711.h"

// Define GPIO pins for each HX711 module on the ESP32 DevKit V1
#define LOADCELL_DOUT_PIN0 32
#define LOADCELL_SCK_PIN0 33

#define LOADCELL_DOUT_PIN1 25
#define LOADCELL_SCK_PIN1 26

#define LOADCELL_DOUT_PIN2 27
#define LOADCELL_SCK_PIN2 14

#define LOADCELL_DOUT_PIN3 12
#define LOADCELL_SCK_PIN3 13

// Create one HX711 instance per load cell
HX711 scale0;
HX711 scale1;
HX711 scale2;
HX711 scale3;

// Calibration factor for each load cell (adjust based on your calibration)
float calibrationFactor0 = 2280.0;
float calibrationFactor1 = 2280.0;
float calibrationFactor2 = 2280.0;
float calibrationFactor3 = 2280.0;

void setup() {
  Serial.begin(115200);
  while (!Serial);  // Wait for serial connection

  // Initialize each HX711 module with its corresponding pins
  scale0.begin(LOADCELL_DOUT_PIN0, LOADCELL_SCK_PIN0);
  scale1.begin(LOADCELL_DOUT_PIN1, LOADCELL_SCK_PIN1);
  scale2.begin(LOADCELL_DOUT_PIN2, LOADCELL_SCK_PIN2);
  scale3.begin(LOADCELL_DOUT_PIN3, LOADCELL_SCK_PIN3);
  
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
  
  // Set calibration factors (update these based on your calibration)
  scale0.set_scale(calibrationFactor0);
  scale1.set_scale(calibrationFactor1);
  scale2.set_scale(calibrationFactor2);
  scale3.set_scale(calibrationFactor3);
  
  // Optionally, print a header for the Serial Plotter
  Serial.println("Scale0,Scale1,Scale2,Scale3");
}

void loop() {
  // Get calibrated weight readings from each load cell
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
