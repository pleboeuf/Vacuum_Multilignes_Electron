/*
 * Project Vacuum_Multilignes_Electron
 * Description: Code running on an Electron on the "Capteur de vide multilignes"
 * Author: Pierre Leboeuf
 * Date: 29 mars 2018
 */

/*
 *** Notes about sleep ***
   Sleep mode: Stop mode + SLEEP_NETWORK_STANDBY, use SETUP BUTTON (20) to wake
   Publish: NO_ACK
   Delay loop: 500 ms for publish and print
   Sleep duration: See #define SLEEPTIMEinMINUTES
 */

//Dev name      No de lignes
// VA1-4     4  Lignes A1 à A4
// VA5B1-2   3  Lignes A5, B1 et B2
// VC1-3     3  Lignes C1 à C3
// VC4-6     3  Lignes C4 à C6
// VC7-8     2  Lignes C7 et C8
// VD1A-2B   4  Lignes D1A, D1B, D2A et D2B
// VE1-3     3  Lignes E1 à E3
// VE4-6     3  Lignes E4 à E6
// VE7-9     3  Lignes E7 à E9
// VE10-12   3  Lignes E10 à E12
// VF1-3     3  Lignes F1 à F3
// VF4-6     3  Lignes F4 à F6
// VF7-9     3  Lignes F7 à F9
// VF10-12   3  Lignes F10 à F12
// VF13-16   4  Lignes F13 à F16
// VG1-2-H14 3  Lignes G1, G2 et H14
// VG3-5     3  Lignes G3 à G5
// VG6-8     3  Lignes G6 à G8
// VG9-12    4  Lignes G9 à G12
// VH2-4     3  Lignes H2 à H4
// VH5-7     3  Lignes H5 à H7
// VH8-10    3  Lignes H8 à H10
// VH11-13   3  Lignes H11 à H13

#include "Particle.h"
#include "math.h"
#include "photon-thermistor.h"
// #include "ClosedCube_Si7051.h"

SYSTEM_MODE(SEMI_AUTOMATIC);
STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));

// General definitions
String FirmwareVersion = "0.6.1";             // Version of this firmware.
String thisDevice = "";
String F_Date  = __DATE__;
String F_Time = __TIME__;
String FirmwareDate = F_Date + " " + F_Time;  //compilation date and time (UTC)
String myEventName = "Dev1_Vacuum/Lignes";    // Name of the event to be sent to the cloud

#define SLEEPTIMEinMINUTES 5                  // Wake every SLEEPTIMEinMINUTES and check if there a reason to publish
#define ONE_DAY_MILLIS (24 * 60 * 60 * 1000)
#define TimeBoundaryOffset 0                  // Wake at time boundary plus some seconds
#define NUMSAMPLES 5                          // Number of readings to average to reduce the noise
#define SAMPLEsINTERVAL 10UL                  // Interval of time between samples in ms
#define VacuumPublishLimits 1                 // Minimum vacuum required to permit publish( 1 always publish, -1: publish only if vacuum)
#define VacMinChange 1                        // Minimum changes in vacuum to initiate a publish within SLEEPTIMEinMINUTES
#define BLUE_LED  D7                          // Blue led awake activity indicator
#define minBatteryLevel 25                    // Sleep unless battery is above this level

// wakeupPin definition
#define wakeupPin  D2

// Thermistor parameters and variable definitions
#define TEMPERATURENOMINAL 25                 // Ref temperature for thermistor
#define BCOEFFICIENT 3470                     // The beta coefficient at 0 degrees of the thermistor (nominal is 3435 (25/85))
#define SERIESRESISTOR 10000UL                // the value of the resistor in serie with the thermistor
#define THERMISTORNOMINAL 10000UL             // thermistor resistance at 25 degrees C

Thermistor *thermistor;
int thermistorPowerPin = D1;
int thermistorInputPin = A4;
float tempDegreeC = 0;
int minPublishTemp = -10;                      // Do not publish below -5

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
#define Vcc 3.3                               //Analog system reference voltage
#define ResistorScaling (R1 + R2) / R2        // To scale output of 5.0V xducer to 3.3V Electron input

#define NVac 4                                // Number of vacuum sensors
int vacuum5VoltsEnablePin = D6;               // Vacuums sensors operate at 5V
int VacuumPins[] = {A0, A1, A2, A3};
float minActVac = 100;
static float VacuumInHg[] = {0, 0, 0, 0};         // Vacuum scaled data array
static float PrevVacuumInHg[] = {10, 0, 0, 0};     // Vacuum reading at the preceding iteration

// Cellular signal and data variables definitions
int signalRSSI;
int signalQuality;
int txPrec   = 0;                             // Previous tx data count
int rxPrec   = 0;                             // Previous rx data count
int deltaTx  = 0;                             // Difference tx data count
int deltaRx  = 0;                             // Difference rx data count
uint32_t startTime = 0;
uint32_t start    = 0;
uint32_t w_time   = 0;                        // Wakeup time in ms
uint32_t aw_time  = 0;                        // Awake time in sec
retained int lastDay     = 0;

unsigned long lastSync = millis();
char publishStr[120];
retained uint32_t noSerie;                             // Le numéro de série est généré automatiquement
retained time_t newGenTimestamp = 0;
retained uint32_t restartCount = 0;
int wakeCount = 0;
retained int FailCount = 0;
FuelGauge fuel;
// ClosedCube_Si7051 si7051;

/* Define a log handler on Serial1 for log messages */
Serial1LogHandler log1Handler(115200, LOG_LEVEL_WARN, {   // Logging level for non-application messages
        { "app", LOG_LEVEL_ALL }                      // Logging level for application messages
});

// ***************************************************************
// Setup
// ***************************************************************
void setup() {
  Log.info("(setup) Firmware " + FirmwareVersion);
  Log.info("(setup) Firmware date: " + FirmwareDate);
  Time.zone(-4);
    pinMode(vacuum5VoltsEnablePin, OUTPUT);        // Put all control pins in output mode
    pinMode(lightSensorPowerPin, OUTPUT);
    pinMode(thermistorPowerPin, OUTPUT);
    pinMode(BLUE_LED, OUTPUT);
    pinMode(wakeupPin, INPUT_PULLUP);
    RGB.mirrorTo(B1, B0, B2, true);
    // pinMode(D5, INPUT);   // Only required for old PCB design
    // si7051.begin(0x40); // default I2C address is 0x40
    // PMIC pmic; //Initalize the PMIC class so you can call the Power Management functions below.
    // pmic.setChargeVoltage(4112);                   // Set charge to more than 80%
    // pmic.setInputVoltageLimit(4840); //Set the lowest input voltage to 4.84 volts. This keeps my 5v solar panel from operating below 4.84 volts.
    thermistor = new Thermistor(thermistorInputPin, SERIESRESISTOR, 4095, THERMISTORNOMINAL, TEMPERATURENOMINAL, BCOEFFICIENT, NUMSAMPLES, SAMPLEsINTERVAL);
    delay(2000UL);
    float soc = fuel.getSoC();
    Log.info("(setup) Boot battery level %0.1f" , soc);
    if(soc > minBatteryLevel){
      Particle.connect();
      if(waitFor(Particle.connected, 30000UL)){
        if (Particle.connected){
          Log.info("(setup) Cloud connected!" + Time.timeStr());
          newGenTimestamp = Time.now();
          if (newGenTimestamp == 0 || Time.year() < 2018) {  // Make sure the time is valid
            Particle.syncTime();
            waitUntil(Particle.syncTimeDone);
            Log.info("(setup) syncTimeDone " + Time.timeStr());
          }
          Log.info("(setup) Setup Completed");
        }
      }
    } else {
    // Sleep again for 1 hour to recharge the battery.
    restartCount++;
    Log.warn("(setup) SETUP Restart. Count: %d, battery: %d", restartCount, fuel.getSoC());
    System.sleep(SLEEP_MODE_DEEP, 3600);
  }
}

// ***************************************************************
// Main loop
// ***************************************************************
void loop() {
  bool vacChanged;
  tempDegreeC = readThermistor(NUMSAMPLES, 1);              // First check the temperature
  // Do not publish if its too cold or charge is lower than 20%
  float soc = fuel.getSoC();
  if(soc > minBatteryLevel){
    if (tempDegreeC >= (float)minPublishTemp) {
      vacChanged = readVacuums();                                          // Then VacuumPublishLimits
      if (wakeCount == 3 || vacChanged){
        lightIntensityLux = readLightIntensitySensor();         // Then read light intensity
        Log.trace("(loop) Vin: %.3f", readVin());                      // And Vin value
        if (minActVac <= VacuumPublishLimits) {
          publishData();                         // Publish a message indicating battery status
        } else {
          Log.trace("(loop) No vacuum, nothing to publish!");
        }
      } else {
        Log.trace("(loop) Nothing to publish!");
      }                                                        // Finally red the 4 vacuum transducers
    }
  } else {
    // else DEEP SLEEP the Electron for an hours
    restartCount++;
    Log.warn("(loop) LOOP Restart. Count: %d, battery: %d", restartCount, fuel.getSoC());
    System.sleep(SLEEP_MODE_DEEP, 3600);  //Put the Electron into Deep Sleep for 1 Hour.
  }
  Log.info("(loop) Going to sleep at: %d\n", millis());
  // sleeps duration corrected to next time boundary + TimeBoundaryOffset seconds
  // wake at next time boundary + TimeBoundaryOffset seconds
  uint32_t dt = (SLEEPTIMEinMINUTES - Time.minute() % SLEEPTIMEinMINUTES) * 60 - Time.second() + TimeBoundaryOffset;
  System.sleep(wakeupPin, FALLING, dt, SLEEP_NETWORK_STANDBY); // Press wakup BUTTON to awake
  w_time = millis();
  Particle.process();
  // Log.trace("(loop) " + Time.timeStr());
  wakeCount++;
  if (wakeCount > 3) {wakeCount = 0;}
  Log.info("(loop) Wake up at: %d, WakeCount= %d", w_time, wakeCount);             // Log wakup time
}

// ***************************************************************
// Publish data
// ***************************************************************
bool publishData() {
  // waitUntil(Particle.connected);
  Log.info("(publishData) Connected to Particle. Publishing now...");
  syncCloudTime();                                          // Sync time with cloud as required
  checkSignal();                                        // Read cellular signal strength and quality
  if (newGenTimestamp == 0) {
    newGenTimestamp = Time.now();
  }
  bool pubSuccess = Particle.publish(myEventName,
        makeJSON(noSerie, newGenTimestamp, VacuumInHg[0], VacuumInHg[1], VacuumInHg[2], VacuumInHg[3],
                  tempDegreeC, lightIntensityLux, fuel.getSoC(), fuel.getVCell(), signalRSSI, signalQuality),
        PRIVATE, NO_ACK);
  for(int i=0; i<300; i++) { // Gives 3 seconds to send the data
    Particle.process();
    delay(10);
  }
  if (pubSuccess){
    Log.info("(publishData) Published success!");
    Log.info("(publishData) Incrementing noSerie now");
    noSerie++;
  } else {
    FailCount++;
    Log.warn("(publishData) Published fail! Count: %d :(", FailCount);
    delay(1000);
    System.reset();
  }
  return pubSuccess;
}

// ***************************************************************
// readThermistor: Read and log the temperature
// ***************************************************************
float readThermistor(int NSamples, int interval){
  // float thermistorRawValue;
  digitalWrite(thermistorPowerPin, true); // Turn ON thermistor Power
  delay(5UL);                           // Wait for voltage to stabilize
  float sum = 0;
  for (int i=0; i< NSamples; i++) {
    delay(interval);              // Delay between successives readings
    sum += thermistor->readTempC(); // Read temperature and accumulate the readings
  }
  digitalWrite(thermistorPowerPin, false); // Turn OFF thermistor
  float temp = sum / NSamples;          // Average the readings
  Log.info("(readThermistor) Temperature = %.1f°C", temp); // Log final value at info level
  return temp;
}

// ***************************************************************
// Read and average vacuum readings
// ***************************************************************
bool readVacuums(){
  float VacuumRawData[] = {0, 0, 0, 0};                               // Vacuum raw data array
  digitalWrite(vacuum5VoltsEnablePin, true);                          // Turn ON the vacuum trasnducer
  minActVac = 100;                                                    // Reset min to a high value initially
  delay(25UL);                                                        // Wait 25 ms for the vacuum sensors to stabilize
  /* Read and log the vacuum values for the four (4) sensors */
  for (int i = 0; i < NVac; i++) {
    Particle.process();
    VacuumRawData[i] = AverageReadings(VacuumPins[i], NUMSAMPLES, SAMPLEsINTERVAL); // Average multiple raw readings to reduce noise
    Log.trace("(readVacuums) Vacuum_%d_raw = %.0f", i, VacuumRawData[i]);           // Log raw data at trace level for debugging
    VacuumInHg[i] = VacRaw2inHg(VacuumRawData[i]);
    if (VacuumInHg[i] < -30) {VacuumInHg[i] = 0; };                   // Convert to inHg
    Log.info("(readVacuums) Vacuum_%d = %.1f inHg", i, VacuumInHg[i]);              // Log final value at info level
    minActVac = min(minActVac, VacuumInHg[i]);                        // Find the minimum readings of all sensors
  }
  Log.info("(readVacuums) Min Vacuum readings %.1f inHg", minActVac);               // Log final value at info level
  digitalWrite(vacuum5VoltsEnablePin, false);                         // Turn OFF the pressure transducers
  // Check if there was a significant change in the vacuum readings
  bool vacChanged = false;
  for (int i = 0; i < NVac; i++){
     if (abs(PrevVacuumInHg[i] - VacuumInHg[i]) > VacMinChange ){
       vacChanged = true;
       break;
     }
  }
  for (int i = 0; i < NVac; i++){
    PrevVacuumInHg[i] = VacuumInHg[i];                                // Remember previous readings
  }
  return vacChanged;
}

// ***************************************************************
/* Convert ADC raw value to Kpa or inHg
 * From MPXV6115V6U datasheet
 * Vout = Vs * (K_fact * Vac + 0.92)
 * where K_fact = 0.007652
 * thus : Vac_kpa = (Vout / Vs * K_fact) - (0,92 / K_fact)
 * and Vout = (Vref*Vraw/4095UL)*(r1+r2)/r2
 * To convert kpa to Hg (inch of mercury) multiply by 0.295301
 */
 // ***************************************************************
double VacRaw2inHg(float raw) {
  double Vout = (Vref * raw / 4095.0f) * (R1 + R2) / R2; // Vout = Vref*Vraw*(r1_+r2_)/(4096*r2_)
  double Vac_kpa = (Vout/(K_fact*Vs)) - 0.92/K_fact; // Vac_kpa = (Vout-(Vs-0,92))/(Vs*k)
  double Vac_inHg = Vac_kpa * 0.2953001;
  return Vac_inHg;                        // multiplie par 0.295301 pour avoir la valeur en Hg
}

// ***************************************************************
/* Read and log the ambieant light intensity */
// ***************************************************************
float readLightIntensitySensor(){
  float lightRawValue;
  float lightIntensity;
  digitalWrite(lightSensorPowerPin, true);                            // Turn ON the light sensor Power
  lightRawValue = AverageReadings(lightSensorInputPin, NUMSAMPLES, 10); // Average multiple raw readings to reduce noise
  Log.trace("(readLightIntensitySensor) lightRawValue = %.0f", lightRawValue);                   // Log raw data at trace level for debugging
  lightIntensity = LightRaw2Lux(lightRawValue);                       // Convert to Lux
  Log.info("(readLightIntensitySensor) Light int. = %.0f Lux", lightIntensity);                  // Log final value at info level
  digitalWrite(lightSensorPowerPin, false);                           // Turn OFF the light sensor
  return lightIntensity;
}

// ***************************************************************
// Check supply input voltage
// ***************************************************************
float readVin(){
  float raw = AverageReadings(A6, 5, 1);                              // Average 5 readings at 1 ms interval
  float Sf = 4.414;                                                   // Precise value TDB
  float Vin = Sf * Vcc * raw / 4095.0f;
  return Vin;
}

// ***************************************************************
/* Convert ADC numerical value to light intensity in Lux for the APDS-9007 chip
 * The resistor LOADRESISTOR is used to convert (Iout) the output current to a voltage
 * that is measured by the analog to digital converter. This chip is logarithmic (base 10).
 */
 // ***************************************************************
double LightRaw2Lux (float raw){
  double Iout = (Vcc * raw / 4095.0f) / LOADRESISTOR; // Calculate the chip output current from raw data
  Log.trace("(LightRaw2Lux) Iout = %.6f A", Iout);             // Log (trace) intermediate results for debugging
  double Lux = pow( 10.0f, Iout / 0.00001f);    // Compute the value in LUX
  return Lux;
}

// ***************************************************************
/* Acquire N readings of an analog input and compute the average */
// ***************************************************************
float AverageReadings (int anInputPinNo, int NSamples, int interval) {
  float sum = 0;
  for (int i=0; i< NSamples; i++) {
    delay(interval);                    // In case a delay is required between successives readings
    sum += analogRead(anInputPinNo); // Read data from selected analog input and accumulate the readings
    Particle.process();
  }
  return sum / NSamples;         // Divide the sum by the number of readings
}

// ***************************************************************
// Formattage standard pour les données sous forme JSON
// ***************************************************************
String makeJSON(uint32_t numSerie, uint32_t timeStamp, float va, float vb, float vc, float vd, float temp, float li, float soc, float volt, int RSSI, int signalQual){
  char publishString[200];
  sprintf(publishString,"{\"noSerie\": %lu,\"generation\": %lu,\"va\":%.1f,\"vb\":%.1f,\"vc\":%.1f,\"vd\":%.1f,\"temp\":%.1f,\"li\":%.0f,\"soc\":%.1f,\"volt\":%.3f,\"rssi\":%d,\"qual\":%d}",
          numSerie, newGenTimestamp, VacuumInHg[0], VacuumInHg[1], VacuumInHg[2], VacuumInHg[3], tempDegreeC, lightIntensityLux, soc, volt, RSSI, signalQual);
  Log.info("(makeJSON) : %s", publishString);
  return publishString;
}

// ***************************************************************
// Read cellular data and substract from the previous cycle
// ***************************************************************
void readCellularData(String s, bool prtFlag) {
  CellularData data;
  if (!Cellular.getDataUsage(data)) {
    Log.warn("(readCellularData) Error! Not able to get Cellular data.");
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

// ***************************************************************
// Read cellular signal strength and quality
// ***************************************************************
void checkSignal() {
  CellularSignal sig = Cellular.RSSI();
  signalRSSI = sig.rssi;
  signalQuality = sig.qual;
  String s = "RSSI.QUALITY: \t" + String(signalRSSI) + "\t" + String(signalQuality) + "\t";
}

// ***************************************************************
// Synchronize clock with cloud one a day
// ***************************************************************
void syncCloudTime(){
  if (Time.day() != lastDay || Time.year() < 2018) { // a new day calls for a sync
    Log.info("(syncCloudTime) Sync time");
    if(waitFor(Particle.connected, 1 * 60000UL)) {
      Particle.syncTime();
      start = millis();
      while (millis() - start < 1000UL) {
              delay (10);
              Particle.process(); // Wait a second to received the time.
      }
      lastDay = Time.day();
      Log.info("(syncCloudTime) Sync time completed");
    }
  }
  // RGB.mirrorDisable();
}
