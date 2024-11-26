#include "HMC5983.h"

HMC5983 mag(&Wire);

void setup() {
  Serial.begin(115200);
  while(!Serial && millis() < 5000) {}

  if(!mag.Begin()) {
    Serial.println("Mag failed to start!!");
    //while(1) {}
  };

  mag.setRange(HMC5983_RANGE_8_1GA);
  mag.setMeasurementMode(HMC5983_CONTINOUS);
  mag.setSampleAverages(HMC5983_SAMPLEAVERAGE_8);
  mag.setDataRate(HMC5983_DATARATE_75HZ);

}

void loop() {
  float mag_val[3];
  int16_t mag_val1[3];
  mag.getMagRaw(mag_val1);
    Serial.print(mag_val1[0]);
    Serial.print("   ");   
    Serial.print(mag_val1[1]);
    Serial.print("   ");   
    Serial.print(mag_val1[2]);
    Serial.print("   ");   

  mag.getMagScaled(mag_val);
    Serial.print(mag_val[0]);
    Serial.print("   ");   
    Serial.print(mag_val[1]);
    Serial.print("   ");   
    Serial.print(mag_val[2]);

  Serial.print("   "); 
  Serial.println(mag.readHeading());
  delay(50);
}
