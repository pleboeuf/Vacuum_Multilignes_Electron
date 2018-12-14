/*
* Project Vacuum_Multilignes_Electron
* Description: Code running on an Electron on the "Capteur de vide multilignes"
* Author: Pierre Leboeuf
* Date: 30 oct 2018
*/

/*
*** Notes about sleep ***
Sleep mode: Stop mode + SLEEP_NETWORK_STANDBY, use SETUP BUTTON (20) to wake-up
Publish: NO_ACK
Delay loop: 500 ms for publish and print
Sleep duration: See #define SLEEPTIMEinMINUTES
*/
// *** Commande pour tracer sur le Raspberry pi
//screen -S DEV1 -L dev1_auto_inside_080rc11 /dev/ttyUSB0 115200

//Dev name      No de lignes
// VA1-4     4  Lignes A1 à A4 RSSI = -77, qual 37
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
// VF7-9     3  Lignes F7 à F9 RSSI = -81, qual 37
// VF10-12   3  Lignes F10 à F12
// VF13-16   4  Lignes F13 à F16
// VG1-2-H14 3  Lignes G1, G2 et H14
// VG3-5     3  Lignes G3 à G5
// VG6-8     3  Lignes G6 à G8
// VG9-12    4  Lignes G9 à G12
// VH2-4     3  Lignes H2 à H4
// VH5-7     3  Lignes H5 à H7
// VH8-10    3  Lignes H8 à H10
// VH11-13   3  Lignes H11 à H13 RSSI = -91, qual 19

// Date de changement d'heure au Québec: 2ième dimanche de mars (10 mars 2019) +1h.
// Premier dimanche de novembre (3 novembre 2019) -1h.

#include "Particle.h"
#include "math.h"
#include "photon-thermistor.h"

SYSTEM_MODE(MANUAL);
// SYSTEM_THREAD(ENABLED);
STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));

// General definitions
String FirmwareVersion = "0.8.29";             // Version of this firmware.
String thisDevice = "";
String F_Date  = __DATE__;
String F_Time = __TIME__;
String FirmwareDate = F_Date + " " + F_Time;  //compilation date and time (UTC)
String myEventName = "test1_Vacuum/Lignes";   // Name of the event to be sent to the cloud
String myNameIs = "";

#define SLEEPTIMEinMINUTES 5                  // wake-up every SLEEPTIMEinMINUTES and check if there a reason to publish
#define ONEHOURinSECONDS 3600UL
#define MINUTES 60UL                          //
#define WakeCountToPublish 3                  // Number of wake-up before publishing
#define ONE_DAY_MILLIS (24 * 60 * 60 * 1000)
#define NIGHT_SLEEP_START_HR 21               // Sleep for the night beginning at 19h00
#define NIGHT_SLEEP_START_MIN 00              // Sleep for the night beginning at 19h00
#define NIGHT_SLEEP_LENGTH_HR 10              // Night sleep duration in hours
#define TOO_COLD_SLEEP_IN_SECONDs 10 * 60     //
#define TimeBoundaryOffset - 0                // wake-up at time boundary plus some seconds
#define WatchDogTimeout 480000UL              // Watch Dog Timeaout delay

#define NUMSAMPLES 5                          // Number of readings to average to reduce the noise
#define SAMPLEsINTERVAL 10UL                  // Interval of time between samples in ms
#define VacuumPublishLimits 1                 // Minimum vacuum required to permit publish( 1 always publish, -1: publish only if vacuum)
#define VacMinChange 1                        // Minimum changes in vacuum to initiate a publish within SLEEPTIMEinMINUTES

#define BLUE_LED  D7                          // Blue led awake activity indicator
#define maxConnectTime 900                    // Maximum allowable time for connection to the Cloud

// wakeupPin definition
#define wakeupPin  D2

// Thermistor parameters and variable definitions
#define TEMPERATURENOMINAL 25                 // Ref temperature for thermistor
#define BCOEFFICIENT 3470                     // The beta coefficient at 0 degrees of the thermistor (nominal is 3435 (25/85))
#define SERIESRESISTOR 10000UL                // the value of the resistor in serie with the thermistor
#define THERMISTORNOMINAL 10000UL             // thermistor resistance at 25 degrees C

float minBatteryLevel = 30.0;                    // Sleep unless battery is above this level

Thermistor *ext_thermistor;
Thermistor *battery_thermistor;
int thermistorPowerPin = D1;
int Ext_thermistorInputPin = A4;
int Bat_thermistorInputPin = B5;
int VinPin = A6;

enum sleepTypeDef {SLEEP_NORMAL, SLEEP_LOW_BATTERY, SLEEP_TWO_MINUTES, SLEEP_TOO_COLD, SLEEP_All_NIGHT};
enum chState {off, lowCurrent, highCurrent, unknown};
chState chargerStatus = unknown;

float ExtTemp = 0;
float BatteryTemp = 0;
float minPublishTemp = 5;                     // Do not publish below 5 (pour tes. normalement -3)
String myID;                                  // Device Id

// Light sensor parameters and variable definitions
#define LOADRESISTOR 51000UL                  // Resistor used to convert current to voltage on the light sensor
int lightSensorEnablePin = D0;
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
float minActualVacuum = 100;
static float VacuumInHg[] = {0, 0, 0, 0};         // Vacuum scaled data array
static float PrevVacuumInHg[] = {10, 0, 0, 0};     // Vacuum reading at the preceding iteration

// Cellular signal and data variables definitions
int signalRSSI;
int signalQuality;
int txPrec   = 0;                             // Previous tx data count
int rxPrec   = 0;                             // Previous rx data count
int deltaTx  = 0;                             // Difference tx data count
int deltaRx  = 0;                             // Difference rx data count
int startTime = 0;
int start    = 0;
int wakeup_time   = 0;                             // Wakeup time in ms
retained int lastDay = 0;
// int lastDay = 0;

unsigned long lastSync = millis();
char publishStr[120];
retained int noSerie;                             // Le numéro de série est généré automatiquement
// int noSerie;                             // Le numéro de série est généré automatiquement
retained time_t newGenTimestamp = 0;
// time_t newGenTimestamp = 0;
retained int restartCount = 0;
// int restartCount = 0;
int wakeCount = 0;
retained int FailCount = 0;
// int FailCount = 0;
FuelGauge fuel;
float soc = 0;
float Vbat = 0;

/* Define a log handler on Serial1 for log messages */
Serial1LogHandler log1Handler(115200, LOG_LEVEL_TRACE, {   // Logging level for non-application messages
    { "app", LOG_LEVEL_INFO }                      // Logging level for application messages
});

// ***************************************************************
// Setup
// ***************************************************************
void setup() {
    Log.info("\n\n__________BOOTING_________________");
    Log.info("(setup) Boot on: " + Time.timeStr(Time.now() - 5 * 60 * 60));
    Log.info("(setup) System version: " + System.version());
    Log.info("(setup) Firmware: " + FirmwareVersion);
    Log.info("(setup) Firmware date: " + FirmwareDate);
    Time.zone(-5);
    pinMode(vacuum5VoltsEnablePin, OUTPUT);        // Put all control pins in output mode
    pinMode(lightSensorEnablePin, OUTPUT);
    pinMode(thermistorPowerPin, OUTPUT);
    pinMode(BLUE_LED, OUTPUT);
    pinMode(wakeupPin, INPUT_PULLUP);
    RGB.mirrorTo(B1, B0, B2, true); // Mirror LED on external LED
    // pinMode(D5, INPUT);   // Only required for old PCB design
    ext_thermistor = new Thermistor(Ext_thermistorInputPin, SERIESRESISTOR, 4095, THERMISTORNOMINAL, TEMPERATURENOMINAL, BCOEFFICIENT, NUMSAMPLES, SAMPLEsINTERVAL);
    battery_thermistor = new Thermistor(Bat_thermistorInputPin, SERIESRESISTOR, 4095, THERMISTORNOMINAL, TEMPERATURENOMINAL, BCOEFFICIENT, NUMSAMPLES, SAMPLEsINTERVAL);
    soc = fuel.getSoC();
    Vbat = fuel.getVCell();
    delay(5000UL);
    myID = System.deviceID();
    getDeviceEventName(myID);
    soc = fuel.getSoC();
    Vbat = fuel.getVCell();
    initSI7051();
    ExtTemp = readThermistor(NUMSAMPLES, 1, "Ext");              // First check the temperature
    BatteryTemp = readThermistor(NUMSAMPLES, 1, "Bat");
    Log.info("(setup) Battery boot voltage: %0.3f ,charge level: %0.2f, and temperature: %0.1f", Vbat, soc, BatteryTemp);
    configCharger(true);
    if (soc > minBatteryLevel) {
        Log.info("(setup) Connecting to tower and cloud.");
        Particle.connect();
        if (waitFor(Particle.connected, maxConnectTime * 1000UL)) {
            Particle.process();
            if (Particle.connected()) {
                Log.info("(setup) Cloud connected! - " + Time.timeStr());
                if (not(Time.isValid())) {
                    Log.info("(setup) Syncing time ");
                    Particle.syncTime();
                    waitUntil(Particle.syncTimeDone);
                    Log.info("(setup) syncTimeDone " + Time.timeStr());
                }
                newGenTimestamp = Time.now();
                Log.info("(setup) Setup Completed");
            } else {
                Log.warn("(setup) Failed to connect! Try again in 2 miutes");
                restartCount++;
                Log.warn("(setup) SETUP Restart. Count: %d, SOC: %.1f\%, Vbat: %.3f", restartCount, soc, Vbat);
                goToSleep(SLEEP_TWO_MINUTES);
            }
        }
    } else {
        // Sleep again for 1 hour to recharge the battery.
        restartCount++;
        Log.warn("(setup) SETUP Restart. Count: %d, SOC: %.1f\%, Vbat: %.3f", restartCount, soc, Vbat);
        delay(2000UL);
        goToSleep(SLEEP_LOW_BATTERY);
    }
}

// ***************************************************************
// Main loop
// ***************************************************************
void loop() {
    bool vacChanged = false;

    soc = fuel.getSoC();
    Vbat = fuel.getVCell();
    Log.info("(loop) Vin: %.2f, Battery level %0.1f",readVin() ,soc);
    if (soc < minBatteryLevel) {
        // SLEEP the Electron for an hour to recharge the battery
        restartCount++;
        Log.warn("\n(loop) LOOP Restart. Count: %d, battery: %.1f", restartCount, soc);
        goToSleep(SLEEP_LOW_BATTERY);
    }
    configCharger(true);

    if (Time.hour() >= NIGHT_SLEEP_START_HR && Time.minute() >= NIGHT_SLEEP_START_MIN) {
        goToSleep(SLEEP_All_NIGHT);
    }

    ExtTemp = readThermistor(NUMSAMPLES, 1, "Ext");                      // First check the temperature
    // Do not publish if its too cold or charge is lower than 20%
    if (ExtTemp >= minPublishTemp) {
        vacChanged = readVacuums();                                   // Then VacuumPublishLimits
        lightIntensityLux = readLightIntensitySensor();               // Then read light intensity
        String timeNow = Time.format(Time.now(), TIME_FORMAT_ISO8601_FULL);
        Log.info("(loop) Summary: Time, Li, Vin, Vbat, SOC, Temp°C, RSSI, Qual:\t"+ timeNow +
                 "\t%.0f\t%.3f\t%.3f\t%.2f\t%.1f\t%d\t%d", lightIntensityLux, readVin(), Vbat, soc, ExtTemp, signalRSSI, signalQuality);
        if (wakeCount == WakeCountToPublish || vacChanged) {
            publishData();                                            // Publish a message the data
        }
        goToSleep(SLEEP_NORMAL);
                                                                    // Finally read the 4 vacuum transducers
    } else {
        Log.info("(loop) Too cold to publish");
        goToSleep(SLEEP_TOO_COLD);
    }

    wakeCount++;
    if (wakeCount > WakeCountToPublish) {
        wakeCount = 0;
    }
    Log.info(" ");
    Log.info("(loop) Wake-up on: " + Time.timeStr() + ", WakeCount= %d", wakeCount);             // Log wakup time
}

void initSI7051(){
    Wire1.begin();
    Wire1.beginTransmission(0x40);
    Wire1.write(0xE6);
    Wire1.write(0x0);
    Wire1.endTransmission();
}

// ***************************************************************
// Determine sleep time and mode
// ***************************************************************
void goToSleep(int sleepType) {
    unsigned long  dt = 0;
    digitalWrite(lightSensorEnablePin, true);                           // Turn OFF the light sensor
    RGB.mirrorDisable();                                                // Disable RGB LED mirroring
    Particle.process();
    Log.info("(goToSleep) Sleep type:= %d", sleepType);
    switch (sleepType)
    {
        case SLEEP_NORMAL:
            // Normal sleep i.e. STOP Mode sleep
            // sleeps duration corrected to next time boundary + TimeBoundaryOffset seconds
            dt = (SLEEPTIMEinMINUTES - Time.minute() % SLEEPTIMEinMINUTES) * 60 - Time.second() + TimeBoundaryOffset;
            Log.info("(goToSleep) dt= %lu seconds", dt);
            if (dt > 360UL){
                dt = 300UL;
            }
            Log.info("(loop) Going to STOP Mode sleep for %lu seconds", dt);
            delay(2000UL);
            System.sleep(wakeupPin, FALLING, dt, SLEEP_NETWORK_STANDBY); // Press wakup BUTTON to awake
            break;

        case SLEEP_LOW_BATTERY:
            // Low battery sleep i.e. STOP Mode sleep for 1 hour to gives time to battery to recharge
            Log.info("(goToSleep) Battery below %0.1f percent. Deep sleep for one hour.", minBatteryLevel);
            // Particle.disconnect();
            delay(2000UL);
            System.sleep(SLEEP_MODE_DEEP, ONEHOURinSECONDS); // Check again in 1 hour if publish conditions are OK
            break;

        case SLEEP_TWO_MINUTES:
            // Two minutes retry sleep
            Log.info("(goToSleep) Difficulty connecting to the cloud. Resetting in 2 minutes.");
            Particle.disconnect();
            delay(2000UL);
            System.sleep(SLEEP_MODE_DEEP, 2 * MINUTES);
            break;

        case SLEEP_TOO_COLD:
            // Wait until its warmer
            dt = (SLEEPTIMEinMINUTES - Time.minute() % SLEEPTIMEinMINUTES) * 60 - Time.second() + TimeBoundaryOffset;
            if (dt > 360UL){
                dt = 300UL;
            }
            Log.info("(goToSleep) Too cold to publish. Try again in %d seconds.", dt);
            if (Particle.connected()){
                Particle.disconnect();
                Cellular.disconnect();
                Cellular.off();
            }
            delay(2000UL);
            System.sleep(wakeupPin, FALLING, dt); // Low power mode
            break;

        case SLEEP_All_NIGHT:
            // Night sleep
            Log.info("(goToSleep) Night time! It's %d O'clock. Going to sleep for %d minutes and %d seconds.", Time.hour(), dt / 60, (dt /60)%60);
            delay(2000UL);
            unsigned long  NightSleepTimeInMinutes = NIGHT_SLEEP_LENGTH_HR * 60;
            dt = (NightSleepTimeInMinutes - Time.minute() % NightSleepTimeInMinutes) * 60 - Time.second() + TimeBoundaryOffset;
            configCharger(false);
            Log.info("(goToSleep) Go to sleep at : " + Time.timeStr());
            Log.info("(goToSleep) dt = %d", dt);
            if (dt > NIGHT_SLEEP_LENGTH_HR * 60 * 60 ){
                dt = NIGHT_SLEEP_LENGTH_HR * 60 * 60;
            }
            Log.info("(goToSleep) dt = %d", dt);
            if (Particle.connected()){
                Particle.disconnect();
                Cellular.disconnect();
                Cellular.off();
            }
            delay(2000UL);
            System.sleep(wakeupPin, FALLING, dt); // Low power mode
            break;
    }

    // wake-up time
    wakeup_time = millis();
    digitalWrite(lightSensorEnablePin, false);                          // Turn ON the light sensor
    RGB.mirrorTo(B1, B0, B2, true);                                     // Enable RGB LED mirroring
    for (int i=0; i<30; i++) {                                          // Gives 3 seconds to system
        Particle.process();
        delay(100);
    }
}

// ***************************************************************
// Publish data
// ***************************************************************
bool publishData() {
    bool pubSuccess = false;
    if (not (Particle.connected())) {
        Log.info("(publishData) NOT CONNECTED! Connecting to Particle cloud.");
        Particle.connect();
    }
    if (waitFor(Particle.connected, 60000)) {
        Log.info("(publishData) Connected to Particle cloud!");
        syncCloudTime();
        if (myEventName == "") {
            myID = System.deviceID();
            getDeviceEventName(myID);
        }                                          // Sync time with cloud as required
        checkSignal();                                            // Read cellular signal strength and quality
        if (newGenTimestamp == 0 || Time.year(newGenTimestamp) > 2030) {
            newGenTimestamp = Time.now();
        }
        String msg = makeJSON(noSerie, newGenTimestamp, VacuumInHg[0], VacuumInHg[1], VacuumInHg[2], VacuumInHg[3],
            ExtTemp, lightIntensityLux, readVin(), fuel.getSoC(), fuel.getVCell(), signalRSSI, signalQuality);
        Log.info("(publishData) Publishing now...");
        pubSuccess = Particle.publish(myEventName, msg, PRIVATE, NO_ACK);
        for (int i=0; i<100; i++) { // Gives 10 seconds to send the data
            Particle.process();
            delay(100);
        }
        if (pubSuccess) {
            Log.info("(publishData) Published success!");
            // Log.info("(publishData) Incrementing noSerie now");
            noSerie++;
        } else {
            FailCount++;
            Log.warn("(publishData) Published fail! Count: %d :(", FailCount);
            delay(500);
        }
    } else {
        Log.info("(publishData) Attempt to connect timeout after 60 seconds. Not connected!");
        Log.info("(publishData) Publish cancelled!!!");
    }
    return pubSuccess;
}

// ***************************************************************
// readThermistor: Read and log the temperature
// ***************************************************************
float readThermistor(int NSamples, int interval, String SelectThermistor) {
    // float thermistorRawValue;
    digitalWrite(thermistorPowerPin, true); // Turn ON thermistor Power
    delay(5UL);                           // Wait for voltage to stabilize
    float sum = 0;
    float temp = 0;
    for (int i=0; i< NSamples; i++) {
        delay(interval);              // Delay between successives readings
        if (SelectThermistor == "Ext") {
            sum += ext_thermistor->readTempC(); // Read temperature and accumulate the readings
        } else {
            sum += battery_thermistor->readTempC(); // Read temperature and accumulate the readings
        }
    }
    temp = sum / NSamples;          // Average the readings
    if (SelectThermistor == "Ext") {
        Log.info("(readThermistor) Temperature Ext. = %.1f°C", temp); // Log final value at info level
    } else {
        Log.info("(readThermistor) Temperature Bat. = %.1f°C", temp); // Log final value at info level
    }
    digitalWrite(thermistorPowerPin, false); // Turn OFF thermistor
    return temp;
}

// ***************************************************************
// readSI7051: Read the SI7051 chip and log the temperature
// ***************************************************************
float readSI7051(){
    Wire1.beginTransmission(0x40);
    Wire1.write(0xF3); //calling for Si7051 to make a temperature measurement
    Wire1.endTransmission();

    delay(15); //14 bit temperature conversion needs 10+ms time to complete.

    Wire1.requestFrom(0x40, (uint8_t)2);
    delay(25);
    byte msb = Wire1.read();
    byte lsb = Wire1.read();
    uint16_t val = msb << 8 | lsb;
    float temperature = (175.72*val) / 65536 - 46.85;
    Log.info("(readSI7051) Si7051 Temperature: %.2f", temperature);
    return temperature;
}

// ***************************************************************
// Read and average vacuum readings
// ***************************************************************
bool readVacuums() {
    float VacuumRawData[] = {0, 0, 0, 0};                                     // Vacuum raw data array
    digitalWrite(vacuum5VoltsEnablePin, true);                                // Turn ON the vacuum trasnducer
    minActualVacuum = 100;                                                    // Reset min to a high value initially
    delay(25UL);                                                              // Wait 25 ms for the vacuum sensors to stabilize
    /* Read and log the vacuum values for the four (4) sensors */
    for (int i = 0; i < NVac; i++) {
        Particle.process();
        VacuumRawData[i] = AverageReadings(VacuumPins[i], NUMSAMPLES, SAMPLEsINTERVAL); // Average multiple raw readings to reduce noise
        Log.trace("(readVacuums) Vacuum_%d_raw = %.0f", i, VacuumRawData[i]);           // Log raw data at trace level for debugging
        VacuumInHg[i] = VacRaw2inHg(VacuumRawData[i]);
        if (VacuumInHg[i] < -30) {VacuumInHg[i] = 0; };                                 // Convert to inHg
        Log.trace("(readVacuums) Vacuum_%d = %.1f inHg", i, VacuumInHg[i]);              // Log final value at info level
        minActualVacuum = min(minActualVacuum, VacuumInHg[i]);                          // Find the minimum readings of all sensors
    }
    Log.info("(readVacuums) Min Vacuum readings %.1f inHg", minActualVacuum);         // Log final value at info level
    digitalWrite(vacuum5VoltsEnablePin, false);                                       // Turn OFF the pressure transducers
    // Check if there was a significant change in the vacuum readings
    bool vacChanged = false;
    for (int i = 0; i < NVac; i++){
        if (abs(PrevVacuumInHg[i] - VacuumInHg[i]) > VacMinChange ){
            vacChanged = true;
            break;
        }
    }
    for (int i = 0; i < NVac; i++){
        PrevVacuumInHg[i] = VacuumInHg[i];                                      // Remember previous vacuum readings
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
float readLightIntensitySensor() {
    float lightRawValue;
    float lightIntensity;
    lightRawValue = AverageReadings(lightSensorInputPin, NUMSAMPLES, 10); // Average multiple raw readings to reduce noise
    Log.trace("(readLightIntensitySensor) lightRawValue = %.0f", lightRawValue);                   // Log raw data at trace level for debugging
    lightIntensity = LightRaw2Lux(lightRawValue);                       // Convert to Lux
    Log.info("(readLightIntensitySensor) Light int. = %.0f Lux", lightIntensity);                  // Log final value at info level
    return lightIntensity;
}

// ***************************************************************
// Check supply input voltage
// ***************************************************************
float readVin() {
    float raw = AverageReadings(VinPin, 5, 1);                              // Average 5 readings at 1 ms interval
    float Sf = 4.012;                                                   // Precise value TDB
    float Vin = Sf * Vcc * raw / 4095.0f;
    return Vin;
}

// ***************************************************************
/* Convert ADC numerical value to light intensity in Lux for the APDS-9007 chip
* The resistor LOADRESISTOR is used to convert (Iout) the output current to a voltage
* that is measured by the analog to digital converter. This chip is logarithmic (base 10).
*/
// ***************************************************************
double LightRaw2Lux (float raw) {
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
String makeJSON(int numSerie, int timeStamp, float va, float vb, float vc, float vd, float temp, float li, float Vin, float soc, float volt, int RSSI, int signalQual) {
    char publishString[200];
    sprintf(publishString,"{\"noSerie\": %d,\"generation\": %lu,\"va\":%.1f,\"vb\":%.1f,\"vc\":%.1f,\"vd\":%.1f,\"temp\":%.1f,\"li\":%.0f,\"Vin\":%.3f,\"soc\":%.2f,\"volt\":%.3f,\"rssi\":%d,\"qual\":%d}",
    numSerie, newGenTimestamp, VacuumInHg[0], VacuumInHg[1], VacuumInHg[2], VacuumInHg[3], ExtTemp, lightIntensityLux, Vin, soc, volt, RSSI, signalQual);
    Log.info("(makeJSON) %s", publishString);
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
bool checkSignal() {
    CellularSignal sig = Cellular.RSSI();
    signalRSSI = sig.rssi;
    signalQuality = sig.qual;
    // String s = "RSSI.QUALITY: \t" + String(signalRSSI) + "\t" + String(signalQuality) + "\t";
    if (sig.rssi == 0 || sig.qual == 0){
        Log.info("(checkSignal) NETWORK CONNECTION LOST!!");
        delay(2000UL);
        System.reset();
        // return false;
    } else if (sig.rssi == 1){
        Log.info("(checkSignal) Cellular module or time-out error");
        return false;
    } else if (sig.rssi == 2){
        Log.info("(checkSignal) RSSI value is not known, not detectable or currently not available");
        return false;
    } else {
        Log.info("(checkSignal) Read the cellular signal!");
        return true;
    }
}

// ***************************************************************
// Synchronize clock with cloud one a day
// ***************************************************************
void syncCloudTime() {
    if (Time.day() != lastDay || Time.year() < 2018) { // a new day calls for a sync
        Log.info("(syncCloudTime) Sync time");
        if(waitFor(Particle.connected, maxConnectTime * 1000UL)) {
            if (Particle.connected()){
                Particle.syncTime();
                start = millis();
                while (millis() - start < 1000UL) {
                    delay (20);
                    Particle.process(); // Wait a second to received the time.
                }
                lastDay = Time.day();
                Log.info("(syncCloudTime) Sync time completed");
            } else {
                Log.warn("(syncCloudTime) Failed to connect! Try again in 5 miutes");
                restartCount++;
                Log.warn("(syncCloudTime) SETUP Restart. Count: %d, battery: %.1f", restartCount, fuel.getSoC());
                goToSleep(SLEEP_TWO_MINUTES);
            }
        }
    }
    // RGB.mirrorDisable();
}

void configCharger(bool mode) {
    PMIC pmic; //Initalize the PMIC class so you can call the Power Management functions below.
    BatteryTemp = readThermistor(NUMSAMPLES, 1, "Bat");
    readSI7051();
    // Mode true: normal operation
    if (mode == true) {
        if (BatteryTemp <= 1.0 or BatteryTemp >= 45.0) {
            if (chargerStatus != off) {
                // Disable charger to protect the battery
                pmic.setChargeVoltage(4208);                   // Set charge voltage to standard 100%
                pmic.setChargeCurrent(0,0,0,0,0,0); //Set charging current to 1024mA (512 + 512 offset)
                pmic.disableCharging();
                chargerStatus = off;
                Log.info("(configCharger) Battery temperature: %0.1f - Disable charger", BatteryTemp);
            }
        } else if (BatteryTemp <= 5.0) {
            if (chargerStatus != lowCurrent) {
                // Reduce the charge current
                pmic.setChargeVoltage(4208);                   // Set charge voltage to standard 100%
                pmic.setChargeCurrent(0,0,0,0,0,1); //Set charging current to 1024mA (512 + 512 offset)
                pmic.enableCharging();
                chargerStatus = lowCurrent;
                Log.info("(configCharger) Battery temperature: %0.1f - Set charger to low current", BatteryTemp);
            }
        } else {
            if (chargerStatus != highCurrent) {
                // Charge at full current
                pmic.setChargeVoltage(4208);                   // Set charge voltage to standard 100%
                pmic.setChargeCurrent(0,0,1,0,0,0); //Set charging current to 1024mA (512 + 512 offset)
                chargerStatus = highCurrent;
                pmic.enableCharging();
                Log.info("(configCharger) Battery temperature: %0.1f - Set charger to max current", BatteryTemp);
            }
        }
    } else {
        // Mode false: Disable charger to protect the battery for night sleep
        if (chargerStatus != off) {
            // Disable charger to protect the battery
            pmic.setChargeVoltage(4208);                   // Set charge voltage to standard 100%
            pmic.setChargeCurrent(0,0,0,0,0,0); //Set charging current to 1024mA (512 + 512 offset)
            pmic.disableCharging();
            chargerStatus = off;
            Log.info("(configCharger) Battery temperature: %0.1f - Disable charger", BatteryTemp);
        }
    }
}

//Dev name      No de lignes
// VA1-4     4  Lignes A1 à A4 RSSI = -77, qual 37
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
// VF7-9     3  Lignes F7 à F9 RSSI = -81, qual 37
// VF10-12   3  Lignes F10 à F12
// VF13-16   4  Lignes F13 à F16
// VG1-2-H14 3  Lignes G1, G2 et H14
// VG3-5     3  Lignes G3 à G5
// VG6-8     3  Lignes G6 à G8
// VG9-12    4  Lignes G9 à G12
// VH2-4     3  Lignes H2 à H4
// VH5-7     3  Lignes H5 à H7
// VH8-10    3  Lignes H8 à H10
// VH11-13   3  Lignes H11 à H13 RSSI = -91, qual 19

String getDeviceEventName(String devId){
    // std::unordered_map<std::string, String> deviceMap;
    // deviceMap.insert({"36004f000251353337353037", "EB-Elec-Dev1"});
    // deviceMap.insert({"240051000c51343334363138", "EB-VF7-9"});
    // myNameIs = deviceMap[devId];
    if (devId == "36004f000251353337353037"){
        myNameIs = "EB-Elec-Dev1";
        myEventName = "test1_Vacuum/Lignes";
    } else if (devId == "240051000c51343334363138"){
        myNameIs = "EB-VF7-9";
        myEventName = "test2_Vacuum/Lignes";
    } else {
        myEventName = "Vacuum/Lignes";
    }
    Log.info("Device Name is: " + myNameIs);
    Log.info("Event Name is: " + myEventName);
}
