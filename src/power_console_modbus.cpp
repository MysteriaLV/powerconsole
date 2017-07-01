#ifndef POWER_CONSOLE_MODBUS_H
#define POWER_CONSOLE_MODBUS_H

#define USE_HOLDING_REGISTERS_ONLY
#include <Arduino.h>
#include <Modbus.h>

extern void modbus_setup();
extern void modbus_loop();
extern void modbus_set(word event, word value);

//////////////// registers of POWER_CONSOLE ///////////////////
enum
{
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

    #define SSerialTxControl 6   //RS485 Direction control
    #define SSerialRX        8  //Serial Receive pin
    #define SSerialTX        9  //Serial Transmit pin
    SoftwareSerial RS485Serial(SSerialRX, SSerialTX); // RX, TX
#endif

#ifdef USE_ALT_SOFT_SERIAL
#include <ModbusSerial.h>
    ModbusSerial mb;

    #define SSerialTxControl 6   //RS485 Direction control
    #define SSerialRX        8  //Serial Receive pin
    #define SSerialTX        9  //Serial Transmit pin
    AltSoftSerial RS485Serial(SSerialRX, SSerialTX); // RX, TX
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
          break;
    case 2 : // Put here code for Connect
      Serial.println("[Connect] action fired");
          digitalWrite(LED_BUILTIN, LOW);
          break;
    default:
      break;
  }

  // Signal that action was processed
  mb.Hreg(ACTIONS, 0);
}

/* Holds current button state in register */
void buttonStatus(int reg, int pin) { // LOOP
  mb.Hreg(reg, !digitalRead(pin));
}
void buttonStatus_setup(int reg, int pin) { // SETUP
  pinMode(pin, INPUT_PULLUP);
}

/* Outputs register value to pin */
void gpioWrite(int reg, int pin) {
  digitalWrite(pin, mb.Hreg(reg));
}
/////////////////////////////////////////////////////////////////

void modbus_set(word event, word value) {
  mb.Hreg(event, value);
}

void modbus_setup()
{
  Serial.println("ModBus Slave POWER_CONSOLE:192.168.14.11 for lua/Aliens.lua");

#ifndef USE_ESP8266_TCP
  mb.config(&RS485Serial, 57600, SSerialTxControl);
  mb.setSlaveId(192.168.14.11);
#else
  mb.config("Aliens Room", "123123123");
  WiFi.config(IPAddress(192, 168, 14, 11), IPAddress(), IPAddress(), IPAddress(), IPAddress());

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
  buttonStatus_setup(CONNECT, D6);
  buttonStatus_setup(POWER_ON, D7);


  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output (D4)

  // Emulate ground for buttons above
  pinMode(D0, OUTPUT);
  digitalWrite(D0, 0);
  pinMode(D5, OUTPUT);
  digitalWrite(D5, 0);

}


void modbus_loop()
{
  mb.task();              // not implemented yet: mb.Hreg(TOTAL_ERRORS, mb.task());
  process_actions();

  // Notify main console of local events
  // mb.Hreg(CONNECT, 1);
  // mb.Hreg(POWER_ON, 1);
  buttonStatus(CONNECT, D6);
  buttonStatus(POWER_ON, D7);

}

void setup()
{
  Serial.begin(115200);
  modbus_setup();
}

void loop() {
  modbus_loop();
}