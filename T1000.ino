#include <math.h>
#include <avr/power.h>
#include <EEPROM.h>

#include <MemoryFree.h>
#include <Vcc.h>
#include <LowPower.h>
#include <RFM69.h>
#include <SparkFunBME280.h>

#include <SystemClock.h>
#include <SleepMode.h>

#include <MovingAverage.h>

#include <MessageStore.h>
#include <MessageItem.h>
#include <MessageTypes.h>

#include "settings.h"
#include "encryption.h"

#define SKETCH_VERSION "0.5a" // max. 4 Bytes !!!
#define BAUD_RATE 9600L
#define F_CPU_STABLE 4000000L

char cmd[25];
int cmdPos = 0;

bool debug = false;
bool onBattery = true;

BME280 bme280;
RFM69 radio;
Vcc vcc(0.99);
settings_V1 settings;

clock_div_t clockDivStable;
clock_div_t clockDivCurrent;

uint16_t loopCounter = 0;

// VCC
const uint8_t vccBufferMax = 5;

int16_t vccBuffer[vccBufferMax];
SimpleRingBuffer<int16_t>vccSimpleRingBuffer = SimpleRingBuffer<int16_t>(vccBuffer, vccBufferMax);
MovingAverage<int16_t> vccAverage = MovingAverage<int16_t>(&vccSimpleRingBuffer);

MessageItem vccMessageItem(50);


// Sketch-Version
MessageItem versionMessageItem(true);

// CPU & Memory
MessageItem cpuMessageItem(1);
MessageItem freeMemoryMessageItem(1);

// Temperature
const uint8_t temperatureBufferMax = 5;

int16_t temperatureBuffer[temperatureBufferMax];
SimpleRingBuffer<int16_t>temperatureSimpleRingBuffer = SimpleRingBuffer<int16_t>(temperatureBuffer, temperatureBufferMax);

MessageItem temperatureMessageItem(15);


// Humidity
const uint8_t humidityBufferMax = 5;

uint16_t humidityBuffer[humidityBufferMax];
SimpleRingBuffer<uint16_t>humiditySimpleRingBuffer = SimpleRingBuffer<uint16_t>(humidityBuffer, humidityBufferMax);

MessageItem humidityMessageItem(25);

// Pressure
const uint8_t pressureBufferMax = 5;

uint16_t pressureBuffer[pressureBufferMax];
SimpleRingBuffer<uint16_t>pressureSimpleRingBuffer = SimpleRingBuffer<uint16_t>(pressureBuffer, pressureBufferMax);

MessageItem pressureMessageItem(5);

// Message Store
const uint8_t msgBufferMax = 12;

message_id_t  kMsgBuffer[msgBufferMax];
MessageItem  *vMsgBuffer[msgBufferMax];
SimpleHashMap<message_id_t, MessageItem *> msgHashMap = SimpleHashMap<message_id_t, MessageItem *>(kMsgBuffer, vMsgBuffer, msgBufferMax);

MessageStore msgStore = MessageStore(&msgHashMap);


void printHelp() {
    Serial.println(F("nodeid\t\t0-255\tGet/set node id"));
    Serial.println(F("networkid\t0-255\tGet/set network id")); 
    Serial.println(F("gatewayid\t1-255\tGet/set gateway id"));  
    Serial.println(F("frequency\tfreq\tGet/set frequency, 31(3)|43(3)|86(8)|91(3)"));   
    Serial.println(F("powerlevel\t0-31\tGet/set powerlevel"));     
    Serial.println(F("highpower\t0|1\tGet/set highpower mode"));
    Serial.println(F("pressure\t0|1\tMeasure pressure"));
    Serial.println(F("wakeups\t\t1-720\tWakeups per hour"));
    Serial.println(F("save\t\t\tSave settings to EEPROM"));
    Serial.println(F("mem\t\t\tPrint free memory"));
    Serial.println(F("cpu\t\t\tPrint cpu speed"));    
    Serial.println(F("debug\t\t\tToggle debug mode"));    
}

void printPrompt() {
    Serial.print(F("\nnode"));
    Serial.print(settings.nodeid);
    Serial.print(F("@"));
    Serial.print(settings.networkid);
    Serial.print(F("> "));  
}

bool executeCommand() {
    char in;
    
    if (Serial.available() == 0) {
        return false; 
    }
    
    in = Serial.read();
        
    if (in != '\n') {
        cmd[cmdPos] = in;
        cmdPos += 1;

        return false;
    }

    cmd[cmdPos] = '\0';
    cmdPos = 0;
    
    Serial.println(cmd);

    if (strncmp(cmd, "nodeid", 6) == 0) {
        uint8_t nodeid;
        if (sscanf(cmd, "%*s %hhu", &nodeid) == 1 && nodeid >= 1 && nodeid <= 255) {
            settings.nodeid = nodeid;
            setupRFM69();
        }
        
        Serial.print(F("Nodeid: "));
        Serial.println(String(settings.nodeid));
    }
    else if (strncmp(cmd, "networkid", 9) == 0) {
        uint8_t networkid;
        if (sscanf(cmd, "%*s %hhu", &networkid) == 1 && networkid >= 0 && networkid <= 255) {
            settings.networkid = networkid;
            setupRFM69();
        }
        
        Serial.print(F("Networkid: "));
        Serial.println(String(settings.networkid));
    }
    else if (strncmp(cmd, "gatewayid", 9) == 0) {
        uint8_t gatewayid;
        if (sscanf(cmd, "%*s %hhu", &gatewayid) == 1 && gatewayid >= 1 && gatewayid <= 255) {
            settings.gatewayid = gatewayid;
            setupRFM69();
        }
                
        Serial.print(F("Gatewayid: "));
        Serial.println(String(settings.gatewayid));
    }
    else if (strncmp(cmd, "frequency", 9) == 0) {
        uint8_t frequency;
        if (sscanf(cmd, "%*s %hhu", &frequency) == 1 && (frequency == 31 || frequency == 43 || frequency == 86 || frequency == 91)) {
            settings.radioFrequency = frequency;
            setupRFM69();
        }
        
        Serial.print(F("Frequency: "));
        Serial.println(settings.radioFrequency);
    }   
    else if (strncmp(cmd, "powerlevel", 10) == 0) {
        uint8_t powerlevel;
        if (sscanf(cmd, "%*s %hhu", &powerlevel) == 1 && powerlevel >= 0 && powerlevel <= 31) {
            settings.radioPowerLevel = powerlevel;
            setupRFM69();
        }
                
        Serial.print(F("Powerlevel: "));
        Serial.println(String(settings.radioPowerLevel));
    } 
    else if (strncmp(cmd, "highpower", 9) == 0) {
        uint8_t highpower;
        if (sscanf(cmd, "%*s %hhu", &highpower) == 1 && highpower >= 0 && highpower <= 1) {
            settings.radioIsHighPower = highpower;
            setupRFM69();
        }
        
        Serial.print(F("Highpower: "));
        Serial.println(String(settings.radioIsHighPower));
        
    }
    else if (strcmp(cmd, "help") == 0) {
        printHelp();    
    }
    else if (strncmp(cmd, "pressure", 8) == 0) {
        uint8_t pressure;
        if (sscanf(cmd, "%*s %hhu", &pressure) == 1 && pressure >= 0 && pressure <= 1) {
            settings.bme280Pressure = pressure;
            setupBME280();
        }
        
        Serial.print(F("Pressure: "));
        Serial.println(settings.bme280Pressure);
    }
    else if (strncmp(cmd, "wakeups", 7) == 0) {
        uint16_t wakeups;
        if (sscanf(cmd, "%*s %hu", &wakeups) == 1 && wakeups > 0 && wakeups <= 720) {
            settings.wakeupsPerHour = wakeups;
        }
        
        Serial.print(F("Wakeups: "));
        Serial.println(settings.wakeupsPerHour);
    }    
    else if (strcmp(cmd, "save") == 0) {
        EEPROM.put(0x01, settings);    
        Serial.println(F("Settings saved."));    
    }
    else if (strcmp(cmd, "debug") == 0) {
        debug = ! debug;    
        Serial.print(F("Debug mode: "));
        Serial.println(String(debug));    
    }
    else if (strcmp(cmd, "mem") == 0) {    
        Serial.print(F("Free memory: "));
        Serial.print(freeMemory());
        Serial.println(F(" Bytes"));    
    }
    else if (strcmp(cmd, "cpu") == 0) {    
        Serial.print(F("CPU speed:\t\t"));
        Serial.print(F_CPU / 1000.);
        Serial.println(F(" KHz"));
        
        Serial.print(F("Current Divisor:\t"));
        Serial.println(SystemClock.getAdjustFactor());
        
        Serial.print(F("Current CPU speed:\t"));
        Serial.print(F_CPU * SystemClock.getAdjustFactor() / 1000);
        Serial.println(F(" KHz"));    
    }    
    else {
        Serial.print(F("Command unknown: "));
        Serial.println(String(cmd));
    }

    return true;
}

bool forceEverySeconds(int16_t seconds) {
    int16_t sleepTimeSec;
    
    sleepTimeSec = 3600 / settings.wakeupsPerHour;
    seconds = seconds / sleepTimeSec * sleepTimeSec;
    if ((sleepTimeSec * (loopCounter + 1)) % seconds == 0) {
        return true;
    }

    return false;
}

void setupRFM69() {
    Serial.print(F("Initializing RFM69HW: "));
    if (! radio.initialize(settings.radioFrequency, settings.nodeid, settings.networkid)) {
        Serial.println(F("failed, please check wiring!"));
        
        delay(SystemClock.adjustDelay(5000));        
        while (true) {
            SleepMode.powerDown(-1);
        }
    }

    Serial.println(F("done"));

    radio.encrypt(ENCRYPTION_KEY);
    radio.setHighPower(settings.radioIsHighPower);
    radio.setPowerLevel(settings.radioPowerLevel);
}


void setupBME280() {
    Serial.print(F("Initializing BME280:  "));

    bme280.settings.commInterface = I2C_MODE;
    bme280.settings.I2CAddress = 0x76;
    
    bme280.settings.runMode = 1;
    bme280.settings.tStandby = 0; //  0, 0.5ms
    bme280.settings.filter = 0;   //  0, filter off
    bme280.settings.tempOverSample = 1;
    bme280.settings.humidOverSample = 1;
    bme280.settings.pressOverSample = settings.bme280Pressure;
        
    if (bme280.begin() != 0x60) {
        Serial.println(F("failed, please check wiring!"));
        
        delay(SystemClock.adjustDelay(5000));     
        while (true) {
            SleepMode.powerDown(-1);
        }
    }

    Serial.println("done");
}


void beforePowerDown() {
    Serial.flush();
    radio.sleep();
}


void measureBME280() {
    uint8_t measureTime, minMeasureTime;
    float temperature, humidity, pressure;
    
    measureTime = bme280.measure();  
    minMeasureTime = max(15, (int8_t)measureTime + 1);
    SleepMode.powerDown(minMeasureTime);
    
    bme280.bulkRead();

    // Temperature
    temperature = bme280.readTempC();

    temperatureMessageItem.priority = 1;


    if (temperatureMessageItem.set((int32_t)(temperature * 100), forceEverySeconds(300))) {
        msgStore.set(message_id_temperature, &temperatureMessageItem);
    }
    else {
        Serial.println((int32_t)(temperature * 100));
    }

    // Humidity
    humidity = bme280.readFloatHumidity();

    humidityMessageItem.priority = 1;
    if (humidityMessageItem.set((uint32_t)(humidity * 100), forceEverySeconds(300))) {
        msgStore.set(message_id_humidity, &humidityMessageItem);
    }

    // Pressure
    if (bme280.settings.pressOverSample > 0) {
        pressure = bme280.readFloatPressure();
        
        pressureMessageItem.priority = 1;    
        if (pressureMessageItem.set(pressure, forceEverySeconds(300))) {
            msgStore.set(message_id_pressure, &pressureMessageItem);
        }

    }
}


void setup() {
    Serial.begin(SystemClock.adjustBaudRate(BAUD_RATE));

    // Read settings from eeprom
    EEPROM.get(0x01, settings);
    SleepMode.before(beforePowerDown);

    if (settings.wakeupsPerHour < 1 || settings.wakeupsPerHour > 720) {
        settings.wakeupsPerHour = 60;
    }
     
    uint8_t clock_divisor = (uint8_t)(F_CPU * SystemClock.getFusesFactor() / F_CPU_STABLE);
    clockDivStable = (clock_div_t)(log(clock_divisor) / log(2));
    clockDivCurrent = (clock_div_t)(log(SystemClock.getFusesFactor()) / log(2));

    setupRFM69();
    setupBME280();  

    printPrompt();
}


void loop() {  
    unsigned long start = millis();
    clock_div_t clockDiv = clockDivCurrent;
    
    int32_t sleepTime = 3600 / (uint32_t)settings.wakeupsPerHour * 1000;
       
    if (loopCounter % 10 == 0) {
        // Stabilize ADC before measuring VCC:
        SleepMode.powerDown(15, ADC_ON);

        vccAverage.push_back(vcc.Read_Volts() * 100);
        uint32_t _vccAverage = vccAverage.simple();
        
        vccMessageItem.priority = 2;
        if (vccMessageItem.set(_vccAverage, forceEverySeconds(1800))) {
            msgStore.set(message_id_vcc, &vccMessageItem);
        }

        // Adjust clock divisor below 2.45 volts:
        if (_vccAverage <= 245) {
            clockDiv = clockDivStable;
        }
        else {
            clockDiv = clock_div_1;
        }

        // Detect battery state
        if (_vccAverage <= 340) {
            onBattery = true;
        }
        else {
            onBattery = false;
        }
        
        cpuMessageItem.priority = 3;
        uint32_t cpuSpeed = (uint32_t)(F_CPU * SystemClock.getAdjustFactor() / 1000);
        if (cpuMessageItem.set(cpuSpeed, forceEverySeconds(3600))) {              
            msgStore.set(message_id_cpu_speed, &cpuMessageItem);
        }

        if (clockDiv != clockDivCurrent) {
            Serial.end();
    
            SystemClock.setDivisor(clockDiv);
            clockDivCurrent = clockDiv;
    
            Serial.begin(SystemClock.adjustBaudRate(BAUD_RATE));
            return;
        }
    }

    measureBME280();

    versionMessageItem.priority = 3;
    if (versionMessageItem.set(SKETCH_VERSION, forceEverySeconds(7200))) {
        msgStore.set(message_id_version, &versionMessageItem);
    }

    freeMemoryMessageItem.priority = 2;
    if (freeMemoryMessageItem.set((uint32_t)freeMemory(), forceEverySeconds(1800))) {
        msgStore.set(message_id_memory_free, &freeMemoryMessageItem);
    }
    
    char   *msgData = msgStore.encode();
    uint8_t msgSize = msgStore.size();    

    if (msgSize > 0) {
        radio.send(settings.gatewayid, msgData, msgSize);
        
        if (debug) {
            Serial.print(F("Message-Length: "));
            Serial.println(String(msgSize));
            Serial.print(F("RAW Data:"));
            
            for (uint8_t i = 0; i < msgSize; i++) {
                Serial.print(" " + String(msgData[i], DEC));
            }
        
            Serial.println();
        }
    }
    else if (debug) {
        Serial.println(F("No data to send"));
    }

    if (onBattery) {
        // We are on Battery, goto sleep until next tick ...:
        SleepMode.powerDown(sleepTime - millis() - start);
    }
    else {    
        // Provide a simple cmdline ...:
        while (true) {
            int16_t runtime = SystemClock.adjustTime(millis() - start);
            uint16_t cmdSleepSteps = 100;
                
            if (runtime >= sleepTime) {
                break;
            }
            
            if (executeCommand()) {
                printPrompt();
            }

            if (sleepTime - runtime < cmdSleepSteps) {
                cmdSleepSteps = sleepTime - runtime;
            }

            delay(SystemClock.adjustDelay(cmdSleepSteps));
        }
    }
      
    loopCounter++;
}
