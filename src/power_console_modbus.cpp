#ifndef POWER_CONSOLE_MODBUS_H
#define POWER_CONSOLE_MODBUS_H

#define USE_HOLDING_REGISTERS_ONLY

#include <Arduino.h>
#include <Modbus.h>
#include <Automaton.h>

Atm_led red, green, blue;
Atm_button powerOn, cableConnect;
Atm_bit poweredOn;

extern void modbus_setup();
extern void modbus_loop();
extern void modbus_set(word event, word value);


void cableConnectCallback(int idx, int v, int up);
void powerOnCallback(int idx, int v, int up);

//////////////// registers of POWER_CONSOLE ///////////////////
enum {
    // The first register starts at address 0
    ACTIONS,      // Always present, used for incoming actions

    // Any registered events, denoted by 'triggered_by_register' in rs485_node of Lua script, 1 and up
    CONNECT,
    POWER_ON,

    TOTAL_ERRORS     // leave this one, error counter
};
#endif //POWER_CONSOLE_MODBUS_H

#ifdef USE_SOFTWARE_SERIAL

#include <ModbusSerial.h>

ModbusSerial mb;

#define SSerialGND       10
#define SSerialRX        9  //Serial Receive pin
#define SSerialTX        8  //Serial Transmit pin
#define SSerialVCC       7
#define SSerialTxControl 6   //RS485 Direction control
SoftwareSerial RS485Serial(SSerialRX, SSerialTX); // RX, TX
#endif

#ifdef USE_ALT_SOFT_SERIAL
#include <ModbusSerial.h>
ModbusSerial mb;

#define SSerialTxControl 6   //RS485 Direction control
#define SSerialRX        8  //Serial Receive pin
#define SSerialTX        9  //Serial Transmit pin
AltSoftSerial RS485Serial(111, 222); // RX, TX hardcoded
#endif

#ifdef USE_SERIAL1
#include <ModbusSerial.h>
ModbusSerial mb;

#define SSerialRX        19  //Serial3 Receive pin (just a reference, can't be changed)
#define SSerialTX        18  //Serial3 Transmit pin (just a reference, can't be changed)
#define SSerialTxControl 20   //RS485 Direction control

#define RS485Serial Serial1
#endif

#ifdef USE_ESP8266_TCP
#include <ESP8266WiFi.h>
#include <ModbusIP_ESP8266.h>
ModbusIP mb;
#endif

// Action handler. Add all your actions mapped by action_id in rs485_node of Lua script
void process_actions() {
    if (mb.Hreg(ACTIONS) == 0)
        return;

    switch (mb.Hreg(ACTIONS)) {
        case 1 : // Put here code for Reset
            Serial.println("[Reset] action fired");
            digitalWrite(LED_BUILTIN, HIGH);
            red.off();
            blue.off();
            green.off();
            poweredOn.off();

            mb.Hreg(CONNECT, 0);
            mb.Hreg(POWER_ON, 0);

            break;
        case 2 : // Put here code for Connect
            Serial.println("[Connect] action fired");
            digitalWrite(LED_BUILTIN, LOW);
            cableConnectCallback(0, 0, 0);
            break;
        case 3 : // Put here code for Power_on
            Serial.println("[Power_on] action fired");
            powerOnCallback(0, 0, 0);
            break;
        default:
            break;
    }

    // Signal that action was processed
    mb.Hreg(ACTIONS, 0);
}

/* Holds current button state in register */
void buttonStatus(int reg, uint8_t pin) { // LOOP
    mb.Hreg(reg, !digitalRead(pin));
}

void buttonStatus_setup(int reg, uint8_t pin) { // SETUP
    pinMode(pin, INPUT_PULLUP);
}

/* Outputs register value to pin */
void gpioWrite(int reg, uint8_t pin) {
    digitalWrite(pin, mb.Hreg(reg));
}
/////////////////////////////////////////////////////////////////

void modbus_set(word event, word value) {
    mb.Hreg(event, value);
}

// PIN mapping for both archs
#ifdef USE_ESP8266_TCP
#define CONNECT_PIN D6
#define POWER_ON_PIN D7
#endif

void modbus_setup() {
    Serial.println("ModBus Slave POWER_CONSOLE:6 for lua/Aliens.lua");

#ifdef EMULATE_RS3485_POWER_PINS
    pinMode(SSerialVCC, OUTPUT);
    digitalWrite(SSerialVCC, HIGH);
    pinMode(SSerialGND, OUTPUT);
    digitalWrite(SSerialGND, LOW);
    delay(10);
#endif

#ifndef USE_ESP8266_TCP
    mb.config(&RS485Serial, 31250, SSerialTxControl);
    mb.setSlaveId(6);
#else
    mb.config("Aliens Room", "123123123");
    WiFi.config(IPAddress(6), IPAddress(), IPAddress(), IPAddress(), IPAddress());

    Serial.print("Connecting to Aliens Room ");
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }

    Serial.println(" CONNECTED!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    Serial.print("Netmask: ");
    Serial.println(WiFi.subnetMask());

    Serial.print("Gateway: ");
    Serial.println(WiFi.gatewayIP());
#endif

    mb.addHreg(ACTIONS, 0);
    mb.addHreg(CONNECT, 0);
    mb.addHreg(POWER_ON, 0);
//	buttonStatus_setup(CONNECT, CONNECT_PIN);
//	buttonStatus_setup(POWER_ON, POWER_ON_PIN);


    pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output (D4)
}


void modbus_loop() {
    mb.task();              // not implemented yet: mb.Hreg(TOTAL_ERRORS, mb.task());
    process_actions();

    // Notify main console of local events
    // mb.Hreg(CONNECT, 1);
    // mb.Hreg(POWER_ON, 1);
//	buttonStatus(CONNECT, CONNECT_PIN);
//	buttonStatus(POWER_ON, POWER_ON_PIN);

}

void cableConnectCallback(int idx, int v, int up) {
    red.on();
    mb.Hreg(CONNECT, 1);
    poweredOn.on();
}

void powerOnCallback(int idx, int v, int up) {
    if (poweredOn.state()) {
        red.off();
        green.on();
        mb.Hreg(POWER_ON, 1);
    }
}

void setup() {
    Serial.begin(115200);
    modbus_setup();

    poweredOn.begin()
            .trace(Serial);
    red.begin(A2).off();
    green.begin(A3).off();
    blue.begin(A4).off();

    powerOn.begin(A1)
            .onPress(powerOnCallback);

    cableConnect.begin(A0)
            .onPress(cableConnectCallback);

#ifdef MY_TEST_MODE
    red.blink(500, 500).start();
    green.blink(500, 500).start();
    blue.blink(500, 500).start();

    powerOn.onPress(green, green.EVT_TOGGLE_BLINK);
    cableConnect.onPress(blue, blue.EVT_TOGGLE_BLINK);
#endif
}

void loop() {
    modbus_loop();
    automaton.run();
}
