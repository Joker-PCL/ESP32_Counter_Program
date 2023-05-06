/*
  Modbus-Arduino Example - Lamp (Modbus Serial)
  Copyright by André Sarmento Barbosa
  http://github.com/andresarmento/modbus-arduino
*/
//=============================Include Header Files===========================//
#include <Ticker.h>
#include <Wire.h>
// #include <EEPROM.h>
#include <Modbus.h>
#include <ModbusSerial.h>

//  multitask
TaskHandle_t Task0;
TaskHandle_t Task1;

// Modbus Registers Offsets (0-9999)
// Used Pins
#define ACTIVE_PIN 15
#define DEVICE_CNT 8
const int VALVE_ADDRESS[DEVICE_CNT] = { 600, 601, 602, 603, 604, 605, 606, 608 };    // กำหนด address relay output
const int COUNTER_ADDRESS[DEVICE_CNT] = { 600, 601, 602, 603, 604, 605, 606, 608 };  // กำหนด address counter
const int ON_ADDRESS[DEVICE_CNT] = { 610, 611, 612, 613, 614, 615, 616, 618 };       // กำหนด address on/off counter
const int RESET_ADDRESS[DEVICE_CNT] = { 720, 721, 722, 723, 724, 725, 726, 728 };    // กำหนด address reset counter
const int TOTAL_ADDRESS = 630;                                                       // กำหนด address total
const int SETCOUNT_ADDRESS = 635;                                                    // กำหนด address set จำนวนที่จะนับ
const int SETDELAY_ADDRESS = 640;                                                    // กำหนด address set หน่วงเวลา relay output
const int RESETALL_ADDRESS = 651;                                                    // กำหนด address reset all counter
const int START_ADDRESS = 652;                                                       // กำหนด address start/stop

// ModbusSerial object
ModbusSerial mb;

//=============================Counter Ticker=================================//
int RELAY_TIMER;   // save RELAY_TIMER to EEPROM
int SENSOR_TIMER;  // save SENSOR_TIMER to EEPROM
int COUNT_UNIT;    // save COUNT_UNIT to EEPROM
bool START_STATE;  // save START_STATE to EEPROM

Ticker Counter;     // ticker with function Counter
Ticker TestGPIO;    // ticker with function Counter
Ticker CountReset;  // ticker with function Counter
Ticker Setting;     // ticker with function Counter

long unsigned int tick = 0;
const int SENSOR_PINS[DEVICE_CNT] = { 35, 34, 39, 36, 32, 33, 25, 26 };  // set sensorpins && EEPROM address on,off couter
const int RELAY[DEVICE_CNT] = { 17, 16, 2, 4, 13, 12, 14, 27 };          // set relaypins
bool RELAY_STATE[DEVICE_CNT] = { false };                                // set relay_state
unsigned int ITEM_COUNTER[DEVICE_CNT] = { 0 };

unsigned long currentMillis = millis();
long int previousMillis[DEVICE_CNT] = { 0 };

// Variable for Counter
int previousstate[DEVICE_CNT] = { 0 };
int currentstate[DEVICE_CNT] = { 0 };

// test GPIO
void gpioTest() {
  if (!START_STATE) {
    for (int i = 0; i < DEVICE_CNT; i++) {
      // Attach ledPin to LAMP1_COIL register
      digitalWrite(RELAY[i], mb.Coil(VALVE_ADDRESS[i]));
    }
  }
}

// setting
void setting() {
  COUNT_UNIT = mb.Hreg(SETCOUNT_ADDRESS);
  RELAY_TIMER = mb.Hreg(SETDELAY_ADDRESS);
  START_STATE = mb.Coil(START_ADDRESS);
}

void countReset() {
  for (int i = 0; i < DEVICE_CNT; i++) {
    if (mb.Coil(RESET_ADDRESS[i])) {
      ITEM_COUNTER[i] = 0;
      mb.Hreg(COUNTER_ADDRESS[i], 0);
    } else {
      mb.Coil(RESET_ADDRESS[i], 0);
    }
  }

  if (mb.Coil(RESETALL_ADDRESS)) {
    for (int x = 0; x < DEVICE_CNT; x++) {
      ITEM_COUNTER[x] = 0;
      mb.Hreg(COUNTER_ADDRESS[x], 0);
    }
  }
}

// function Counter
void CountItem() {
  if (START_STATE) {
    for (int i = 0; i < DEVICE_CNT; i++) {
      currentstate[i] = digitalRead(SENSOR_PINS[i]);

      if (currentstate[i] == 0 && previousstate[i] == 1 && mb.Coil(ON_ADDRESS[i])) {
        ITEM_COUNTER[i]++;
        mb.Hreg(COUNTER_ADDRESS[i], ITEM_COUNTER[i]);
      }

      if (ITEM_COUNTER[i] == COUNT_UNIT) {
        RELAY_STATE[i] = true;
        ITEM_COUNTER[i] = 0;
        mb.Hreg(COUNTER_ADDRESS[i], ITEM_COUNTER[i]);
      }

      previousstate[i] = currentstate[i];
    }
  }
}

// Run in Core 0
void mainLoop(void *val) {
  Serial.println("Run in Core: " + String(xPortGetCoreID()));
  TestGPIO.attach_ms(100, gpioTest);      // sample every 100ms, 0.5sec
  CountReset.attach_ms(100, countReset);  // sample every 100ms, 0.5sec
  Setting.attach_ms(100, setting);        // sample every 100ms, 0.5sec
  for (;;) {
    mb.task();
  }
}

// Run in Core 1
void counterLoop(void *val) {
  Serial.println("Run in Core: " + String(xPortGetCoreID()));
  Counter.attach_ms(5, CountItem);  // sample every 100ms, 0.5sec
  for (;;) {
    if (mb.Coil(START_ADDRESS)) {
      //  Capture Timing
      currentMillis = millis();

      // Check Full, relay activate
      for (int i = 0; i < DEVICE_CNT; i++) {
        if (RELAY_STATE[i] == true) {
          digitalWrite(RELAY[i], HIGH);
          mb.Coil(VALVE_ADDRESS[i], 1);
          RELAY_STATE[i] = false;
          previousMillis[i] = currentMillis;
        }
      }

      for (int x = 0; x < DEVICE_CNT; x++) {
        if (currentMillis - previousMillis[x] >= RELAY_TIMER) {
          digitalWrite(RELAY[x], LOW);
          mb.Coil(VALVE_ADDRESS[x], 0);
          previousMillis[x] = currentMillis;
        }
      }
    }
  }
}

void setup() {
  // Config Modbus Serial (port, speed, byte format)
  mb.config(&Serial, 115200, SERIAL_8N1, ACTIVE_PIN);
  // Set the Slave ID (1-247)
  mb.setSlaveId(1);

  // Set ledPin mode
  for (int i = 0; i < DEVICE_CNT; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
    pinMode(RELAY[i], OUTPUT);
    mb.addCoil(VALVE_ADDRESS[i], 0);
    mb.addCoil(ON_ADDRESS[i], 0);
    mb.addCoil(RESET_ADDRESS[i], 0);
    mb.addHreg(COUNTER_ADDRESS[i], 0);
  }

  mb.addHreg(TOTAL_ADDRESS);
  mb.addHreg(SETCOUNT_ADDRESS);
  mb.addHreg(SETDELAY_ADDRESS);
  mb.addCoil(RESETALL_ADDRESS);
  mb.addCoil(START_ADDRESS);

  xTaskCreatePinnedToCore(mainLoop, "Task0", 3000, NULL, 10, &Task0, 0);
  xTaskCreatePinnedToCore(counterLoop, "Task1", 1000, NULL, 9, &Task1, 1);
}

void loop() {
}
