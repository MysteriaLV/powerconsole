#define USE_HOLDING_REGISTERS_ONLY
#include <ESP8266WiFi.h>
#include <Modbus.h>
#include <ModbusIP_ESP8266.h>

//////////////// registers of POWER_CONSOLE ///////////////////
enum
{
  // The first register starts at address 0
  ACTIONS,      // Always present, used for incoming actions

  // Any registered events, denoted by 'triggered_by_register' in rs485_node of Lua script, 1 and up
  CONNECT,
  POWER_ON,
  
  TOTAL_ERRORS,     // leave this one, error counter
  TOTAL_REGS_SIZE   // INTERNAL: total number of registers for function 3 and 16 share the same register array
};

//ModbusIP object
ModbusIP mb;

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
void setup()
{
  Serial.println("TCP ModBus Slave POWER_CONSOLE:192.168.14.11 for lua/Aliens.lua");

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


void loop()
{
  mb.task();              // not implemented yet: mb.Hreg(TOTAL_ERRORS, mb.task());
  process_actions();

  // Notify main console of local events
  // mb.Hreg(CONNECT, 1);
  // mb.Hreg(POWER_ON, 1);
  buttonStatus(CONNECT, D6);
  buttonStatus(POWER_ON, D7);
  
}