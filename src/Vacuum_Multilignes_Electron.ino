/*
* Project Vacuum_Multilignes_Electron
* Description: Code running on an Electron on the "Capteur de vide multilignes"
* Author: Pierre Leboeuf
* Date: 26 octobre 2017
*/

/*
*** Notes about sleep ***
Sleep mode: Stop mode + SLEEP_NETWORK_STANDBY, use SETUP BUTTON (20) to wake
Publish: NO_ACK
Delay loop: 500 ms for publish and print
Sleep duration: See #define SLEEPTIMEinMINUTES
*/

#include "Particle.h"
#include "math.h"

SYSTEM_MODE(SEMI_AUTOMATIC);
STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));

// General definitions
#define FirmwareVersion "0.0.3"               // Version of this firmware.
String F_Date  = __DATE__;
String F_Time = __TIME__;
String FirmwareDate = F_Date + " " + F_Time;  //compilation date and time (UTC)
#define SLEEPTIMEinMINUTES 2                  // *** Duration of sleep for test ***
#define ONE_DAY_MILLIS (24 * 60 * 60 * 1000)
#define myEventName "Dev2_Test_Multilignes"   // Name of the event to be sent to the cloud
#define TimeBoundaryOffset 0                  // Wake at time boundary plus some seconds
#define NUMSAMPLES 5                          // Number of readings to average to reduce the noise
#define resetCycleCountPin  B0                // Connect to Vcc to reset the cycle counter

// Thermistor parameters and variable definitions
#define TEMPERATURENOMINAL 25                 // Ref temperature for thermistor
#define BCOEFFICIENT 3270                     // The beta coefficient at 0 degrees of the thermistor (nominal is 3435 (25/85))
#define SERIESRESISTOR 10000UL                // the value of the resistor in serie with the thermistor
#define THERMISTORNOMINAL 10000UL             // thermistor resistance at 25 degrees C
int thermistorPowerPin = D1;
int thermistorInputPin = A4;
float tempDegreeC = 0;

// Light sensor parameters and variable definitions
#define LOADRESISTOR 51000UL                  // Resistor used to convert current to voltage on the light sensor
int lightSensorPowerPin = D0;
int lightSensorInputPin = A5;
float lightIntensityLux = 0;

// Vacuum transducer parameters and variable definitions
#define R1 16900UL                            // pressure xducer scaling resistor 1
#define R2 32400UL                            // Pressure xducer scaling resistor 2
#define Vref 3.3                              // Analog input reference voltage
#define K_fact 0.007652                       // Vacuum xducer K factor
#define Vs 5.0                                // Vacuum xducer supply voltage
#define Vcc 3.347                             //Analog system reference voltage
#define ResistorScaling (R1 + R2) / R2        // To scale output of 5.0V xducer to 3.3V Electron input
int vacuum5VoltsEnablePin = D5;               // Vacuums sensors operate at 5V
int VacuumPins[]      = {A0, A1, A2, A3};
float VacuumRawData[] = {0, 0, 0, 0};         // Vacuum raw data array
float VacuumInHg[]    = {0, 0, 0, 0};         // Vacuum scaled data arra

// Cellular signal and data variables definitions
int signalRSSI;
int signalQuality;
int      txPrec   = 0;                        // Previous tx data count
int      rxPrec   = 0;                        // Previous rx data count
int      deltaTx  = 0;                        // Difference tx data count
int      deltaRx  = 0;                        // Difference rx data count
uint32_t start    = 0;
uint32_t w_time   = 0;                        // Wakeup time in ms
uint32_t s_time   = 0;                        // Go to sleep time in ms
uint32_t aw_time  = 0;                        // Awake time in sec
retained int lastDay     = 0;
retained int cycleNumber = 0;

unsigned long lastSync = millis();
char publishStr[45];

/* Define a log handler on Serial1 for log messages */
Serial1LogHandler logHandler(115200, LOG_LEVEL_WARN, {  // Logging level for non-application messages
   { "app", LOG_LEVEL_TRACE }                            // Logging level for application messages
});




void setup() {
   //Serial1.printlnf("System version: %s", System.version().c_str());
   Time.zone(-4);
   pinMode(vacuum5VoltsEnablePin, OUTPUT);              //Put all control pins in output mode
   pinMode(lightSensorPowerPin, OUTPUT);
   pinMode(thermistorPowerPin, OUTPUT);
   pinMode(resetCycleCountPin, INPUT_PULLDOWN);
   w_time = millis();
   /*EEPROM.get(addr, cycleNumber); // Get the cycleNumber from non-volatile storage*/
   Particle.connect(); // This imply Cellular.on() and Cellular.connect()
   waitUntil(Particle.connected);
   readCellularData("Initial data \t", TRUE);
   Log.trace("Setup Completed");
}


void loop() {
   if (Time.day() != lastDay || Time.year() < 2000){   // a new day calls for a sync
      //  Particle.connect();
       //Serial1.println("Sync time");
       Log.trace("Sync time");
       if(waitFor(Particle.connected, 1*60000)){
         Particle.syncTime();
         start = millis();
         while (millis() - start < 1000UL) {
                 Particle.process(); // Wait a second to received the time.
         }
         lastDay = Time.day();
         Log.trace("Sync time completed");
       }
   }
   waitUntil(Particle.connected);
   Log.trace("Cloud connedted!");
   tempDegreeC = readThermistor();
   lightIntensityLux = readLightIntensitySensor();
   readVacuums();

   readCellularData("Before publish \t", TRUE);
   checkSignal(); // Print signal RSSI and quality
   s_time = millis(); //Sleep time is now
   aw_time = s_time - w_time; //Time the system is awake
   publishData(); // Publish a message indicating battery status
   Log.trace("Going to sleep at: %d\n", s_time);

// sleeps duration corrected to next time boundary +30 seconds
   uint32_t dt = (SLEEPTIMEinMINUTES - Time.minute() % SLEEPTIMEinMINUTES) * 60 - Time.second() + TimeBoundaryOffset; // wake at next time boundary +30 seconds
   System.sleep(BTN, FALLING, dt, SLEEP_NETWORK_STANDBY);                 // use SETUP BUTTON to awake
   w_time = millis();
   Log.trace("Wake up at: %d", w_time);                                                     // Time of awakening
}

void publishData() {
   FuelGauge fuel;
   // Publish voltage and SOC, RSSI, QUALITY. plus Tx & Rx count
  //  sprintf(publishStr, "%d, %02.4f, %03.3f, %d, %d, %d, %d, %03.3f",
  //          cycleNumber - 1, fuel.getVCell(), fuel.getSoC(), signalRSSI, signalQuality, deltaTx, deltaRx, aw_time);
   // Publish temperature, light intensity , 4 channels of vacuum, battery voltage and % charge, RSSI, QUALITY. plus Tx & Rx count
  sprintf(publishStr, "%.1f, %.0f, %.1f, %.1f, %.1f, %.1f, %02.4f, %03.3f, %d, %d, %d, %d, %d",
              tempDegreeC, lightIntensityLux, VacuumInHg[0], VacuumInHg[1], VacuumInHg[2], VacuumInHg[3],
              fuel.getVCell(), fuel.getSoC(), signalRSSI, signalQuality,  deltaTx, deltaRx, aw_time);
   Particle.publish(myEventName, publishStr, PRIVATE, NO_ACK);
   start = millis();
   while (millis() - start < 500UL) {
           Particle.process(); // Wait a second to received the time.
   }

}

void readCellularData(String s, bool prtFlag) {
   CellularData data;
   if (!Cellular.getDataUsage(data)) {
           Serial1.print("Error! Not able to get data.");
   }
   else {
         deltaTx = data.tx_session - txPrec;
         deltaRx = data.rx_session - rxPrec;
         if (prtFlag) {
         }
         txPrec = data.tx_session;
         rxPrec = data.rx_session;
   }
}

void checkSignal() {
   CellularSignal sig = Cellular.RSSI();
   signalRSSI = sig.rssi;
   signalQuality = sig.qual;
   String s = "RSSI.QUALITY: \t" + String(signalRSSI) + "\t" + String(signalQuality) + "\t";
   //Serial1.print(s);
}

/* Read and log the temperature */
float readThermistor(){
float thermistorRawValue;
float temp;
digitalWrite(thermistorPowerPin, true);                                  // Turn ON thermistor Power
thermistorRawValue = AverageReadings(thermistorInputPin, NUMSAMPLES, 0); // Average multiple raw readings to reduce noise
Log.trace("Thermistor raw = %.0f", thermistorRawValue);                   // Log raw data at trace level for debugging
temp = rawTemp2DegreesC(thermistorRawValue);                             // Convert to degrees C
Log.info("Temperature = %.1f°C", temp);                                  // Log final value at info level
digitalWrite(thermistorPowerPin, false);                                 // Turn OFF thermistor
return temp;
}

/* Read and log the ambieant light intensity */
float readLightIntensitySensor(){
float lightRawValue;
float lightIntensity;
digitalWrite(lightSensorPowerPin, true);                                  // Turn ON the light sensor Power
lightRawValue = AverageReadings(lightSensorInputPin, NUMSAMPLES, 0);      // Average multiple raw readings to reduce noise
Log.trace("lightRawValue = %.0f", lightRawValue);                         // Log raw data at trace level for debugging
lightIntensity = LightRaw2Lux(lightRawValue);                             // Convert to Lux
Log.info("Light int. = %.0f Lux", lightIntensity);                        // Log final value at info level
digitalWrite(lightSensorPowerPin, false);                                 // Turn OFF the light sensor
return lightIntensity;
}

void readVacuums(){
digitalWrite(vacuum5VoltsEnablePin, true);                                // Turn ON the vacuum trasnducer
delay(25UL);                                                              // Wait 25 ms for the vacuum sensors to stabilize
/* Read and log the vacuum values for the four (4) sensors */
for (int i = 0 ; i < 4; i++){
  VacuumRawData[i] = AverageReadings(VacuumPins[i], NUMSAMPLES, 0);       // Average multiple raw readings to reduce noise
  VacuumInHg[i] = VacRaw2inHg(VacuumRawData[i]);                          // Convert to inHg
  Log.trace("Vacuum_%d_raw = %.0f", i, VacuumRawData[i] );                // Log raw data at trace level for debugging
  Log.info("Vacuum_%d = %.1f inHg", i, VacuumInHg[i]);                    // Log final value at info level
}
digitalWrite(vacuum5VoltsEnablePin, false); // Turn OFF the pressure transducers

}

/* Convert ADC raw value to Kpa or inHg
* From MPXV6115V6U datasheet
* Vout = Vs * (K_fact * Vac + 0.92)
* where K_fact = 0.007652
* thus : Vac_kpa = (Vout / Vs * K_fact) - (0,92 / K_fact)
* and Vout = (Vref*Vraw/4095UL)*(r1+r2)/r2
* To convert kpa to Hg (inch of mercury) multiply by 0.295301
*/
double VacRaw2inHg(float raw) {
double Vout = (Vref * raw / 4095.0f) * (R1 + R2) / R2;      // Vout = Vref*Vraw*(r1_+r2_)/(4096*r2_)
double Vac_kpa = (Vout/(K_fact*Vs)) - 0.92/K_fact;  // Vac_kpa = (Vout-(Vs-0,92))/(Vs*k)
double Vac_inHg = Vac_kpa * 0.2953001;
/*Serial.printlnf("Vout= %f, Vac_kpa= %f, Vac_inHg= %f", Vout, Vac_kpa, Vac_inHg);*/
return Vac_inHg;                              // multiplie par 0.295301 pour avoir la valeur en Hg
}

/* Convert thermistor resistance to °C
* Use the simplified Steinhart equation
*/
float rawTemp2DegreesC(float RawADC) {
float resistance;
/* convert the value to resistance */
resistance = SERIESRESISTOR / ((4095.0f / RawADC) - 1);
Log.trace("Rtm = %.0f ohms", resistance);         // Log (trace) intermediate results for debugging
/* Converst to degrees Celcius */
double steinhart;
steinhart = resistance / THERMISTORNOMINAL;       // (R/Ro)
steinhart = log(steinhart);                       // ln(R/Ro)
steinhart /= BCOEFFICIENT;                        // 1/B * ln(R/Ro)
steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
steinhart = 1.0 / steinhart;                      // Invert
steinhart -= 273.15;                              // convert to deg. C;
return steinhart - 0.7;                           // Return the temperature
}

/* Acquire N readings of an analog input and compute the average */
float AverageReadings (int anInputPinNo, int NumberSamples, int interval) {
float accumulator = 0;
for (int i=0; i< NumberSamples; i++){
  delay(interval);                                // In case a delay is required between successives readings
  accumulator += analogRead(anInputPinNo);        // Read data from selected analog input and accumulate the readings
}
return accumulator / NumberSamples;               // Divide the accumulator by the number of readings
}

/* Convert ADC numerical value to light intensity in Lux for the APDS-9007 chip
* The resistor LOADRESISTOR is used to convert (Iout) the output current to a voltage
* that is measured by the analog to digital converter. This chip is logarithmic (base 10).
*/
double LightRaw2Lux (float raw){
double Iout = (Vcc * raw / 4095.0f) / LOADRESISTOR; // Calculate the chip output current from raw data
Log.trace("Iout = %.6f A", Iout);                   // Log (trace) intermediate results for debugging
double Lux = pow( 10.0f, Iout / 0.00001f);          // Compute the value in LUX
return Lux;
}
