
#include <Arduino.h>
#include <avr/wdt.h>
#include <Wire.h>
#include <SoftwareSerial.h>

#define LIG_SENSOR_PORT     A12

void light_sensor_init(void)
{
  pinMode(LIG_SENSOR_PORT, INPUT);
}

int16_t light_senor_aRead(void)
{
  int16_t val;
  val = analogRead(LIG_SENSOR_PORT);
  return (val);
}

void setup() {
  Serial.begin(115200);
  Serial.println("This is an example for Light Sensor ");
  light_sensor_init();
}

void loop() {
  Serial.print("Current Sensor data: ");
  Serial.println(light_senor_aRead());
  delay(500);
}
