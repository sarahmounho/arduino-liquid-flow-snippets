/*
 * Copyright (c) 2018, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*******************************************************************************
 * Purpose:  Modified code from example Sensiron data to get the total volume
 ******************************************************************************/

#include <Wire.h> // Arduino library for I2C

const int ADDRESS = 0x08; // Standard address for LD20 Liquid Flow Sensors
const float SCALE_FACTOR_FLOW = 10.0;
const float SCALE_FACTOR_TEMP = 200.0;
const char *UNIT_FLOW = " ul/min";
const char *UNIT_TEMP = " deg C";

int ret;
int16_t signed_flow_value;
float sensor_reading;
double totalizer_volume = 0.0;
int16_t counter = -1;
double delta_t;
double this_micros;
double last_micros;

// -----------------------------------------------------------------------------
// Measurement routine
// -----------------------------------------------------------------------------
void measure_flow(){
  Serial.println("Begin Measure Flow");
  delay(10);
  Wire.requestFrom(ADDRESS, 3);
  if (Wire.available() < 3) {
    Serial.println("Error while reading flow measurement");
  }

  signed_flow_value  = Wire.read() << 8; // read the MSB from the sensor
  signed_flow_value |= Wire.read();      // read the LSB from the sensor

  sensor_reading = ((float) signed_flow_value) / SCALE_FACTOR_FLOW;
}
// -----------------------------------------------------------------------------
// Arduino setup routine, just runs once:
// -----------------------------------------------------------------------------
void setup() {

  Serial.begin(9600); // initialize serial communication
  Wire.begin();       // join i2c bus (address optional for master)

  do {
    // Soft reset the sensor
    Wire.beginTransmission(0x00);
    Wire.write(0x06);
    ret = Wire.endTransmission();
    if (ret != 0) {
      Serial.println("Error while sending soft reset command, retrying...");
      delay(500); // wait long enough for chip reset to complete
    }
  } while (ret != 0);
  delay(50); // wait long enough for chip reset to complete

  // Begin measurement
  Wire.beginTransmission(ADDRESS);
  Wire.write(0x36);
  Wire.write(0x08);
  ret = Wire.endTransmission();
  if (ret != 0) {
    Serial.println("Error while sending start measurement command, retrying...");
  }
}


// -----------------------------------------------------------------------------
// The Arduino loop routine runs over and over again forever:
// -----------------------------------------------------------------------------
void loop(){
  
  

  // init
  this_micros = micros();
  totalizer_volume = 0.0;

  while (true) {
    counter++;
    measure_flow();
    last_micros = this_micros;
    this_micros = micros(); // get microseconds since board has been running program
    delta_t = ((this_micros - last_micros)) / 1000000.0 / 60.0 ; // ul/min
    totalizer_volume = totalizer_volume + sensor_reading * delta_t ; // seems to be off by 0.1 ul/min
    Serial.print("Reading (ul): ");
    Serial.print(sensor_reading);
    Serial.print("Delta_t (min passed): ");
    Serial.print(delta_t);
    Serial.print("Totalizer (ul): "); 
    Serial.println(totalizer_volume);
    delay(20); // 
    

  }
}
