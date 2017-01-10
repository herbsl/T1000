#include <math.h>
#include <avr/power.h>
#include <EEPROM.h>

#include <MemoryFree.h>
#include <Vcc.h>
#include <LowPower.h>
#include <RFM69.h>
#include <SparkFunBME280.h>

#include <SystemClock.h>
#include <MovingAverage.h>

#include <MessageStore.h>
#include <MessageItem.h>
#include <MessageTypes.h>

#include <settings.h>
#include "encryption.h"

#define DEBUG

#define SKETCH_VERSION "0.1"

#define BAUD_RATE 9600L

#define F_CPU_STABLE 4000000L


BME280 bme280;
RFM69 radio;
Vcc vcc(0.98);
settings_V1 settings;

clock_div_t clockDivStable;
clock_div_t clockDivLast = clock_prescale_get();

uint32_t loopCounter = 0;

// VCC
const uint8_t vccBufferMax = 5;

int16_t vccBuffer[vccBufferMax];
SimpleRingBuffer<int16_t>vccSimpleRingBuffer = SimpleRingBuffer<int16_t>(vccBuffer, vccBufferMax);
MovingAverage<int16_t> vccAverage = MovingAverage<int16_t>(&vccSimpleRingBuffer);

MessageItem vccMessageItem;

int16_t vccLast = 0;


// Sketch-Version
MessageItem versionMessageItem;

// CPU & Memory
MessageItem cpuMessageItem;
MessageItem freeMemoryMessageItem;

uint32_t freeMemoryLast = 0;


// Temperature
const uint8_t temperatureBufferMax = 5;

int16_t temperatureBuffer[temperatureBufferMax];
SimpleRingBuffer<int16_t>temperatureSimpleRingBuffer = SimpleRingBuffer<int16_t>(temperatureBuffer, temperatureBufferMax);
MovingAverage<int16_t> temperatureAverage = MovingAverage<int16_t>(&temperatureSimpleRingBuffer);

MessageItem temperatureMessageItem;


// Humidity
const uint8_t humidityBufferMax = 5;

uint16_t humidityBuffer[humidityBufferMax];
SimpleRingBuffer<uint16_t>humiditySimpleRingBuffer = SimpleRingBuffer<uint16_t>(humidityBuffer, humidityBufferMax);
MovingAverage<uint16_t> humidityAverage = MovingAverage<uint16_t>(&humiditySimpleRingBuffer);

MessageItem humidityMessageItem;

// Pressure
const uint8_t pressureBufferMax = 5;

uint16_t pressureBuffer[pressureBufferMax];
SimpleRingBuffer<uint16_t>pressureSimpleRingBuffer = SimpleRingBuffer<uint16_t>(pressureBuffer, pressureBufferMax);
MovingAverage<uint16_t> pressureAverage = MovingAverage<uint16_t>(&pressureSimpleRingBuffer);

MessageItem pressureMessageItem;


// Message Store
const uint8_t msgBufferMax = 12;

message_id_t  kMsgBuffer[msgBufferMax];
MessageItem  *vMsgBuffer[msgBufferMax];
SimpleHashMap<message_id_t, MessageItem *> msgHashMap = SimpleHashMap<message_id_t, MessageItem *>(kMsgBuffer, vMsgBuffer, msgBufferMax);

MessageStore msgStore = MessageStore(&msgHashMap);


void setupRFM69() {
#ifdef DEBUG
    Serial.println("Nodeid: " + String(settings.nodeid));
    Serial.println("Networkid: " + String(settings.networkid));
    Serial.print("Initializing RFM69HW: ");
#endif
    if (! radio.initialize(settings.radioFrequency, settings.nodeid, settings.networkid)) {
#ifdef DEBUG
        Serial.println("failed, please check wiring!");
#endif
        delay(SystemClock.adjustDelay(5000));
        while (true) {
            powerDown(-1);
        }
    }

#ifdef DEBUG
    Serial.println("done");
#endif

    radio.encrypt(ENCRYPTION_KEY);
    radio.setHighPower(settings.radioIsHighPower);
    radio.setPowerLevel(settings.radioPowerLevel);
}


void setupBME280() {
#ifdef DEBUG
    Serial.print("Initializing BME280:  ");
#endif

    bme280.settings.commInterface = I2C_MODE;
    bme280.settings.I2CAddress = 0x76;
    
    bme280.settings.runMode = 1;
    bme280.settings.tStandby = 0; //  0, 0.5ms
    bme280.settings.filter = 0;   //  0, filter off
    bme280.settings.tempOverSample = 1;
    bme280.settings.humidOverSample = 1;
    bme280.settings.pressOverSample = 0;

    if (bme280.begin() != 0x60) {
#ifdef DEBUG
      Serial.println("failed, please check wiring!");
#endif

      delay(SystemClock.adjustDelay(5000));
      
      while (true) {
        powerDown(-1);
      }
    }

#ifdef DEBUG
    Serial.println("done");
#endif    
}

void loopBME280() {
    uint8_t measureTime, minMeasureTime;
    float temperature, humidity, pressure;
    
    measureTime = bme280.measure();  
    minMeasureTime = max(15, (int8_t)measureTime + 1);
    powerDown(minMeasureTime);
    
    bme280.bulkRead();

    // Temperature
    temperature = bme280.readTempC();
    temperatureAverage.push_back(temperature * 100);

    temperatureMessageItem.priority = 1;
    temperatureMessageItem.set((int32_t)(temperature * 100));
    msgStore.set(message_id_temperature, &temperatureMessageItem);    

    // Humidity
    humidity = bme280.readFloatHumidity();
    humidityAverage.push_back(humidity * 100);

    humidityMessageItem.priority = 1;
    humidityMessageItem.set((uint32_t)(humidity * 100));
    msgStore.set(message_id_humidity, &humidityMessageItem);     

    // Pressure
    if (bme280.settings.pressOverSample > 0) {
        pressure = bme280.readFloatPressure();
        pressureAverage.push_back(pressure * 100);
    
        pressureMessageItem.priority = 1;
        pressureMessageItem.set(pressure * 100);
        msgStore.set(message_id_pressure, &pressureMessageItem);
    }
}

void setup() {
#ifdef DEBUG
    Serial.begin(SystemClock.adjustBaudRate(BAUD_RATE));
#endif
      
    uint8_t clock_divisor = (uint8_t)(F_CPU * SystemClock.getFusesFactor() / F_CPU_STABLE);
    clockDivStable = (clock_div_t)(log(clock_divisor) / log(2));

    EEPROM.get(0x01, settings);

    setupRFM69();
    setupBME280();

    loopCounter = 1;

    versionMessageItem.set(SKETCH_VERSION);
    versionMessageItem.priority = 2;
    msgStore.set(message_id_version, &versionMessageItem);
}

void loop() {
    clock_div_t clockDiv;
    uint32_t start = micros();
      
    vccAverage.push_back(vcc.Read_Volts() * 100);
    uint32_t _vccAverage = vccAverage.simple();

    if (_vccAverage <= 1245) {
        clockDiv = clockDivStable;
    }
    else {
        clockDiv = clock_div_1;
    }

    if (clockDivLast != clockDiv) {
#ifdef DEBUG
        Serial.end();
#endif

        SystemClock.setDivisor(clockDiv);
        clockDivLast = clockDiv;

#ifdef DEBUG
        Serial.begin(SystemClock.adjustBaudRate(BAUD_RATE));
#endif
    }

    if (loopCounter % 10 == 0 || clockDivLast != clockDiv) {
        
        cpuMessageItem.set((uint32_t)(F_CPU * SystemClock.getAdjustFactor() / 1000));
        cpuMessageItem.priority = 3;
        
        msgStore.set(message_id_cpu_speed, &cpuMessageItem);
    }
  
    if (loopCounter % 10 == 0 || vccLast != _vccAverage) {
        vccMessageItem.set(_vccAverage);
        vccMessageItem.priority = 2;
        
        msgStore.set(message_id_vcc, &vccMessageItem);
        vccLast = _vccAverage;
    }

    uint32_t _freeMemory = freeMemory();
    if (loopCounter % 10 == 0 || _freeMemory != freeMemoryLast) {
        freeMemoryMessageItem.set(_freeMemory);
        freeMemoryMessageItem.priority = 2;
        
        msgStore.set(message_id_memory_free, &freeMemoryMessageItem);
        freeMemoryLast = _freeMemory;
    }

    loopBME280();
    
    char   *msgData = msgStore.encode();
    uint8_t msgSize = msgStore.size();

    radio.send(settings.gatewayid, msgData, msgSize);
    
#ifdef DEBUG
    if (msgSize > 0) {
        Serial.println("Message-Size: " + String(msgSize));
        Serial.print("RAW Data:");
        
        for (uint8_t i = 0; i < msgSize; i++) {
            Serial.print(" " + String(msgData[i], DEC));
        }
    
        Serial.println();
    }
#endif

    loopCounter++;

#ifdef DEBUG
    Serial.println("Runtime: " + String(SystemClock.adjustTime(micros() - start)));
#endif
    
    powerDown(5000);
}

void powerDown(int ms) {
    period_t sleep;

    radio.sleep();
    
#ifdef DEBUG
    Serial.flush();
#endif

    while (true) {
        if (ms >= 8000) {
            sleep = SLEEP_8S;
            ms -= 8000;
        }
        else if (ms >= 4000) {
            sleep = SLEEP_4S;
            ms -= 4000;
        }
        else if (ms >= 2000) {
            sleep = SLEEP_2S;
            ms -= 2000;
        }
        else if (ms >= 1000) {
            sleep = SLEEP_1S;
            ms -= 1000;
        }
        else if (ms >= 500) {
            sleep = SLEEP_500MS;
            ms -= 500;
        }
        else if (ms >= 250) {
            sleep = SLEEP_250MS;
            ms -= 250;
        }
        else if (ms >= 120) {
            sleep = SLEEP_120MS;
            ms -= 120;
        }
        else if (ms >= 60) {
            sleep = SLEEP_60MS;
            ms -= 60;
        }
        else if (ms >= 30) {
            sleep = SLEEP_30MS;
            ms -= 30;
        }
        else if (ms >= 15) {
            sleep = SLEEP_15MS;
            ms -= 15;
        }
        else if (ms == 0) {
            return;
        }
        else if (ms == -1) {
            sleep = SLEEP_FOREVER;
            ms = 0;
        }
        else {
            sleep = SLEEP_15MS;
            ms = 0;
        }

        LowPower.powerDown(sleep, ADC_OFF, BOD_OFF);
    }
}
