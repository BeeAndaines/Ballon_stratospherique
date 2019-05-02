//zone SD horloge 

/** 
 *  Nous utilisons la librairie OneWire
 *  Elle doit être présente dans le répertoire libraries
 *  situé dans le répertoire des croquis/sketchs
 *  voir dans le menu Préférences
 *  cf: https://github.com/PaulStoffregen/OneWire/archive/master.zip
 */
#include <OneWire.h>

#include <Wire.h>

// zone BME
#include "cactus_io_BME280_I2C.h"
// Créer un objet BME280
//BME280_I2C bme; // I2C en utilisant l'adresse 0x77
BME280_I2C bme (0x76); // I2C en utilisant l'adresse 0x76
float p;
float valintermed;
float altitude1;
float valinter1;
float puissance1;


float P0;// p0 est la pression au moment de la mise en route elle permet de connaitre l'altitude par rapport au lacher
//pmer=1013,25; // pmer est la pression au niveau de la mer en hPa

/**
 * nous utilisons Tiny RTC module horloge
  * DS1307 pour bus I2C
 * avec batterie au lithium CR1225
 * Le port I2C de l'Arduino est situé
 * sur les pin A4 et A5
 *
 * Analog pin A5 <-> SCL
 * Analog pin A4 <-> SDA
 */
// nous utilisons la librairie RTClib
//pour lire les données de l'horloge
//cf: https://github.com/adafruit/RTClib
 #include <Wire.h>
#include "RTClib.h"

RTC_DS1307 RTC;

/**
 * Écriture sur une carte SD
 *
 * SD card reliée au bus SPI :
 * MOSI       - pin 11
 * MISO       - pin 12
 * CLK ou SCK - pin 13
 * CS         - pin 4
 *
 * SPI pour Serial Peripheral Interface
 *
 * created  24 Nov 2010
 * modified 9 Apr 2012
 * by Tom Igoe
 * cf: https://www.arduino.cc/en/Tutorial/Datalogger
 */
#include <SPI.h>
#include <SD.h>

// Arduino Uno pin 4
// cf: https://www.arduino.cc/en/Reference/SPI
const int chipSelect = 4;


/* zoneozone
 */
#include "MQ131.h"

// Init the sensor
// - Heater control on pin 2
// - Sensor analog read on pin A1
// - Model LOW_CONCENTRATION
// - Load resistance RL of 10KOhms (10000 Ohms)
MQ131 sensor(2,A1, LOW_CONCENTRATION, 10000);

//zone methane 
// - Sensor analog read on pin A0

/*******************Demo for MQ-2 Gas Sensor Module V1.0*****************************
Support:  Tiequan Shao: support[at]sandboxelectronics.com
 
Lisence: Attribution-NonCommercial-ShareAlike 3.0 Unported (CC BY-NC-SA 3.0)
 
Note:    This piece of source code is supposed to be used as a demostration ONLY. More
         sophisticated calibration is required for industrial field application. 
 
                                                    Sandbox Electronics    2011-04-25
************************************************************************************/
 
/************************Hardware Related Macros************************************/
#define         MQ_PIN                       (0)     //define which analog input channel you are going to use
#define         RL_VALUE                     (5)     //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR          (9.83)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
                                                     //which is derived from the chart in datasheet
 
/***********************Software Related Macros************************************/
#define         CALIBARAION_SAMPLE_TIMES     (50)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interal(in milisecond) between each samples in the
                                                     //cablibration phase
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interal(in milisecond) between each samples in 
                                                     //normal operation
 
/**********************Application Related Macros**********************************/
#define         GAS_CH4                      (0)
 
/*****************************Globals***********************************************/
float           CH4Curve[3]  =  {2.3,0.21,-0.41};   //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent"
                                                    //to the original curve. 
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.25), point2: (lg10000, -0.46) 
                                                   
float           Ro           =  10;                 //Ro is initialized to 10 kilo ohms
 
void setup()
{
  Serial.begin(9600);    
// zone SD horloge
 

   Wire.begin();
      RTC.begin();
      if (! RTC.isrunning())
      {
        Serial.println("RTC is NOT running!");
      }
      // Décommenter cette ligne pour mettre à jour l'heure dans RTC
     RTC.adjust(DateTime(__DATE__, __TIME__));


//zone SD
  while (!Serial)
    {
    ; // wait for serial port to connect. Needed for native USB port only
     }
  
  Serial.println("Initialisation de la carte SD ...");
 
  // on verifie que la carte est présente et peut être initialisée
  if (!SD.begin(chipSelect))
    {
    Serial.println("Carte Sd inaccesible ou absente");
    // ne fait rien d'autre
    return;
    }
  Serial.println("Carte OK");

//bme.begin;
if (!bme.begin()) {
Serial.println ( " Impossible de trouver un capteur BME280 valide, vérifiez le câblage!" );
while (1);
}
Serial.println("BME initialise avec succes!");
bme.setTempCal (-1); // Temp était en lecture haute pour soustraire 1 degré

Serial.println ("Pression Pa \ Humidite \ Temperature \ altitude 1 ");
bme.readSensor ();
P0 = bme.getPressure_HP ();
Serial.print("P0="); Serial.println(P0);
// zone methane
  
                             //UART setup, baudrate = 9600bps
  Serial.print("Calibrating...\n");                
  Ro = MQCalibration(MQ_PIN);                       //Calibrating the sensor. Please make sure the sensor is in clean air 
                                                    //when you perform the calibration                    
  Serial.print("Calibration is done...\n"); 
  Serial.print("Ro=");
  Serial.print(Ro);
  Serial.print("kohm");
  Serial.print("\n");

  /* zoneozone
 */
 Serial.println("Calibration in progress...");
  
  sensor.calibrate();
  
  Serial.println("Calibration done!");
  Serial.print("R0 = ");
  Serial.print(sensor.getR0());
  Serial.println(" Ohms");
  Serial.print("Time to heat = ");
  Serial.print(sensor.getTimeToRead());
  Serial.println(" s");
}
 
void loop()
{
 // nous lisons la date et l'heure sur l'horloge RTC
  DateTime now = RTC.now();

  
/*  
 *   zoneozone
 */
  Serial.println("Sampling...");
  sensor.begin();
  Serial.print("Concentration O3 : ");
  Serial.print(sensor.getO3(PPM));
  Serial.println(" ppm");
  Serial.print("Concentration O3 : ");
  Serial.print(sensor.getO3(PPB));
  Serial.println(" ppb");
  Serial.print("Concentration O3 : ");
  Serial.print(sensor.getO3(MG_M3));
  Serial.println(" mg/m3");
  Serial.print("Concentration O3 : ");
  Serial.print(sensor.getO3(UG_M3));
  Serial.println(" ug/m3");

/*  
zonemethane
*/
   Serial.print("CH4:"); 
   Serial.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CH4) );
   Serial.print( "ppm" );
   Serial.println("\n");
//zone bme
bme.readSensor ();
Serial.print (bme.getPressure_HP ()); Serial.print (" Pa " );
Serial.print (bme.getHumidity ()); Serial.print ( "% " );
Serial.print (bme.getTemperature_C ()); Serial.print ( "C " );
p=bme.getPressure_HP ();
valintermed=p/P0;
Serial.print ( "     P/P0=" ); Serial.println (valintermed);
//altitude1=44330*[1-(p/P0)^(1/5,255)];
puissance1=1/5.255;
//Serial.print (puissance1); Serial.println ( "puiss1 " );
valinter1=1-pow(valintermed,puissance1);
//Serial.print (valinter1); Serial.println ( "valinter1 " );
altitude1=44330*valinter1;
Serial.print ("altitude1: ");Serial.print (altitude1); Serial.println ( "m " );

// on enregistre sur SD année; mois; jour; heure; minute; seconde; ozone ; methane
//Zone SD
  // nous créons une chaîne de caractères pour
  // concaténer les données à écrire :
  String dataString = "";

  // nous convertissons la valeur 
  // avec l'objet String() afin de pouvoir 
  // l'écrire sur la carte
  dataString += String(now.year(), DEC);
  dataString += ";";
  dataString += String(now.month(), DEC);
  dataString += ";";
  dataString += String(now.day(), DEC);
  dataString += ";";
  dataString += String(now.hour(), DEC);
  dataString += ";";
  dataString += String(now.minute(), DEC);
  dataString += ";";
  dataString += String(now.second(), DEC);
  dataString += ";";
  dataString += String(sensor.getO3(PPM));
  dataString +=  ";";
  dataString += String(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CH4));
  dataString += ";";
  dataString += String(bme.getPressure_HP ());
  dataString += ";";
  dataString += String(bme.getHumidity ());
  dataString += ";";
  dataString += String(bme.getTemperature_C ());
  dataString += ";";
dataString += String(altitude1);
  dataString += ";";
  /**
   * nous ouvrons le fichier
   * Nb: un seul fichier peut être ouvert à la fois
   * le fichier se nomme : journal.csv
   */
  File dataFile = SD.open("journal.csv", FILE_WRITE);

  // si le fichier est disponible, nous écrivons dedans :
  if (dataFile) {
    Serial.print("journal ouvert");
    dataFile.println(dataString);
    dataFile.close();

    // nous affichons aussi notre chaîne sur le port série :
    Serial.println(dataString);
  }
 
  // Si le fichier n'est pas ouvert nous affichons
  // un message d'erreur
  else {
    Serial.println("nous ne pouvons ouvrir journal.csv");
  }


   delay(10000);
   
}
 
/****************** MQResistanceCalculation ****************************************
Input:   raw_adc - raw value read from adc, which represents the voltage
Output:  the calculated sensor resistance
Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
         across the load resistor and its resistance, the resistance of the sensor
         could be derived.
************************************************************************************/ 
float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}
 
/***************************** MQCalibration ****************************************
Input:   mq_pin - analog channel
Output:  Ro of the sensor
Remarks: This function assumes that the sensor is in clean air. It use  
         MQResistanceCalculation to calculates the sensor resistance in clean air 
         and then divides it with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about 
         10, which differs slightly between different sensors.
************************************************************************************/ 
float MQCalibration(int mq_pin)
{
  int i;
  float val=0;
 
  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value
 
  val = val/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro 
                                                        //according to the chart in the datasheet 
 
  return val; 
}
/*****************************  MQRead *********************************************
Input:   mq_pin - analog channel
Output:  Rs of the sensor
Remarks: This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
         The Rs changes as the sensor is in the different consentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
************************************************************************************/ 
float MQRead(int mq_pin)
{
  int i;
  float rs=0;
 
  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }
 
  rs = rs/READ_SAMPLE_TIMES;
 
  return rs;  
}
 
/*****************************  MQGetGasPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         gas_id      - target gas type
Output:  ppm of the target gas
Remarks: This function passes different curves to the MQGetPercentage function which 
         calculates the ppm (parts per million) of the target gas.
************************************************************************************/ 
int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == GAS_CH4 ) {
     return MQGetPercentage(rs_ro_ratio,CH4Curve);
  }    
 
  return 0;
}
 
/*****************************  MQGetPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         pcurve      - pointer to the curve of the target gas
Output:  ppm of the target gas
Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm) 
         of the line could be derived if y(rs_ro_ratio) is provided. As it is a 
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic 
         value.
************************************************************************************/ 
int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}
