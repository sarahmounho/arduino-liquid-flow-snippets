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
const float SCALE_FACTOR_FLOW = 500.0;
const float SCALE_FACTOR_TEMP = 200.0;
const char *UNIT_FLOW = " ml/min";
const char *UNIT_TEMP = " deg C";

float sensor_reading;
double totalizer_volume = 0.0;
uint16_t counter = 0;
float tail_correction = 0.0;
double delta_t;
  
double this_micros;
double last_micros;

int ret;
uint16_t aux_value;
uint16_t sensor_flow_value;
uint16_t sensor_temp_value;
int16_t signed_flow_value;
int16_t signed_temp_value;
float scaled_flow_value;
float scaled_temp_value;
// -----------------------------------------------------------------------------
// Arduino setup routine, just runs once:
// -----------------------------------------------------------------------------
void setup() {
  int ret;

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
}

void measure_flow(){
  Wire.requestFrom(ADDRESS, 3);
  if (Wire.available() < 3) {
    Serial.println("Error while reading flow measurement");
  }

  signed_flow_value  = Wire.read() << 8; // read the MSB from the sensor
  signed_flow_value |= Wire.read();      // read the LSB from the sensor

  sensor_reading = ((float) signed_flow_value) / SCALE_FACTOR_FLOW;
}
// -----------------------------------------------------------------------------
// The Arduino loop routine runs over and over again forever:
// -----------------------------------------------------------------------------
void loop() {
  

  
  byte aux_crc;
  byte sensor_flow_crc;
  byte sensor_temp_crc;

  // To perform a measurement, first send 0x3608 to switch to continuous
  // measurement mode, then read 3x (2 bytes + 1 CRC byte) from the sensor.
  Wire.beginTransmission(ADDRESS);
  Wire.write(0x36);
  Wire.write(0x08);
  ret = Wire.endTransmission();
  if (ret != 0) {
    Serial.println("Error during write measurement mode command");

  } else {
    delay(1000);

    for(int i = 0; i < 10; ++i) {
      delay(100);
      Wire.requestFrom(ADDRESS, 9);
      if (Wire.available() < 9) {
        Serial.println("Error while reading flow measurement");
        continue;
      }
	  counter++;

      sensor_flow_value  = Wire.read() << 8; // read the MSB from the sensor
      sensor_flow_value |= Wire.read();      // read the LSB from the sensor
      sensor_flow_crc    = Wire.read();
      sensor_temp_value  = Wire.read() << 8; // read the MSB from the sensor
      sensor_temp_value |= Wire.read();      // read the LSB from the sensor
      sensor_temp_crc    = Wire.read();
      aux_value          = Wire.read() << 8; // read the MSB from the sensor
      aux_value         |= Wire.read();      // read the LSB from the sensor
      aux_crc            = Wire.read();

      Serial.print("Flow value from Sensor: ");
      Serial.print(sensor_flow_value);

      signed_flow_value = (int16_t) sensor_flow_value;
      Serial.print(", signed value: ");
      Serial.print(signed_flow_value);

      scaled_flow_value = ((float) signed_flow_value) / SCALE_FACTOR_FLOW;
      Serial.print(", scaled value: ");
      Serial.print(scaled_flow_value);
      Serial.print(UNIT_FLOW);

      Serial.print(", Temp value from Sensor: ");
      Serial.print(sensor_temp_value);

      signed_temp_value = (int16_t) sensor_temp_value;
      Serial.print(", signed value: ");
      Serial.print(signed_temp_value);

      scaled_temp_value = ((float) signed_temp_value) / SCALE_FACTOR_TEMP;
      Serial.print(", scaled value: ");
      Serial.print(scaled_temp_value);
      Serial.print(UNIT_TEMP);
      
      // measure the flow and check the volume, approximatelly every 10 ms
      if (counter % 1 == 0) {
        measure_flow();
        last_micros = this_micros;
        this_micros = micros();
        delta_t = ((this_micros - last_micros)) / 1000000.0 / 60.0; // since the unit of flow is ml/min
        totalizer_volume = totalizer_volume + sensor_reading * delta_t;


        // Print the status every 100 loops
        if (counter % 100 == 0) {
          Serial.print("Sensor reading: ");   Serial.print(sensor_reading);
          Serial.print(" dalta t: ");         Serial.print(delta_t);
          Serial.print(" Tail correction: "); Serial.print(tail_correction);
          Serial.print(" Totalizer: ");       Serial.println(totalizer_volume);
        }
      }

      Serial.println("");
    }
    // To stop the continuous measurement, first send 0x3FF9.
    Wire.beginTransmission(ADDRESS);
    Wire.write(0x3F);
    Wire.write(0xF9);
    ret = Wire.endTransmission();
    if (ret != 0) {
      Serial.println("Error during write measurement mode command");
    }
  }

  delay(1000); // milliseconds delay between reads (for demo purposes)
}
