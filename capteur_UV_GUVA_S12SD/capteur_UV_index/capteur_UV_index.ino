void setup() 
{
  Serial.begin(9600);
}
 
void loop() 
{
  float sensorVoltage; 
  float sensorValue;
 
  sensorValue = analogRead(A0);
  sensorVoltage = sensorValue/1024*5;
     //float voltage = sensorValue / 1024.0;
  int uv_index = sensorVoltage / 0.1;

  Serial.print("sensor reading = ");
  Serial.print(sensorValue);
  Serial.println("");
  Serial.print("sensor voltage = ");
  Serial.print(sensorVoltage);
  Serial.println(" V");
   Serial.print("uv_index");
  Serial.println( uv_index);
  delay(1000);
}
