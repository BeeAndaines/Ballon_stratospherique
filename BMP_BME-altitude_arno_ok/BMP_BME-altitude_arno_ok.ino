#include "SparkFunBME280.h"
//Library allows either I2C or SPI, so include both.
#include "Wire.h"
#include "SPI.h"

BME280 capteur;

float p;
float valintermed;
float altitude;
float valinter;
float puissance;
//pmer=1013,25; // p0 est la pression au niveau de la mer en hPa

void setup() {

   Serial.begin(9600);
  while (!Serial) {
    // Attente de l'ouverture du port sÃ©rie pour Arduino LEONARDO
  }
  //configuration du capteur
  capteur.settings.commInterface = I2C_MODE; 
  capteur.settings.I2CAddress = 0x76;
  capteur.settings.runMode = 3; 
  capteur.settings.tStandby = 0;
  capteur.settings.filter = 0;
  capteur.settings.tempOverSample = 1 ;
  capteur.settings.pressOverSample = 1;
  capteur.settings.humidOverSample = 1;

  Serial.println("Starting BME280... ");
  delay(10);  // attente de la mise en route du capteur. 2 ms minimum
  // chargement de la configuration du capteur
  capteur.begin();
}

void loop() {
  Serial.print("TempÃ©rature: ");
  Serial.print(capteur.readTempC(), 2);
  Serial.print(" Â°C");
  Serial.print("\t Pression: ");
  Serial.print(capteur.readFloatPressure(), 2);
  Serial.print(" Pa");
  Serial.print("\t humiditÃ© relative : ");
  Serial.print(capteur.readFloatHumidity(), 2);
  Serial.println(" %");
  p=capteur.readFloatPressure();
//altitude=44330*[1-(p/101325)^(1/5,255)];
valintermed=p/101325;
Serial.print (valintermed); Serial.println ( "valintermed " );
puissance=1/5.255;
Serial.print (puissance); Serial.println ( "puissance " );
valinter=1-pow(valintermed,puissance);
Serial.print (valinter); Serial.println ( "valinter " );
altitude=44330*valinter;
Serial.print (altitude); Serial.println ( "m " );
// Ajouter un délai de 2 secondes.
  delay(1000);
}
