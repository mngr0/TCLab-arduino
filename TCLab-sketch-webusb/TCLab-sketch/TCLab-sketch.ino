/*
  TCLab Temperature Control Lab Firmware
  Jeffrey Kantor
  February, 2019

  This firmware provides a high level interface to the Temperature Control Lab. The
  firmware scans the serial port for commands. Commands are case-insensitive. Any
  unrecognized command results in sleep model. Each command returns a result string.

  A         software restart. Returns "Start".
  LED float set LED to float for 10 sec. range 0 to 100. Returns actual float
  P1 float  set pwm limit on heater 1, range 0 to 255. Default 200. Returns P1.
  P2 float  set pwm limit on heater 2, range 0 to 255. Default 100. Returns P2.
  Q1 float  set Heater 1, range 0 to 100. Returns value of Q1.
  Q2 float  set Heater 2, range 0 to 100. Returns value of Q2.
  R1        get value of Heater 1, range 0 to 100
  R2        get value of Heater 2, range 0 to 100
  SCAN      get values T1 T2 Q1 Q1 in line delimited values
  T1        get Temperature T1. Returns value of T1 in °C.
  T2        get Temperature T2. Returns value of T2 in °C.
  VER       get firmware version string
  X         stop, enter sleep mode. Returns "Stop"

  Limits on the heater can be configured with the constants below.

  Status is indicated by LED1 on the Temperature Control Lab. Status conditions are:

      LED1        LED1
      Brightness  State
      ----------  -----
      dim         steady     Normal operation, heaters off
      bright      steady     Normal operation, heaters on
      dim         blinking   High temperature alarm on, heaters off
      bright      blinking   High temperature alarm on, heaters on

  The Temperature Control Lab shuts down the heaters if it receives no host commands
  during a timeout period (configure below), receives an "X" command, or receives
  an unrecognized command from the host.

  The constants can be used to configure the firmware.

  Version History
      1.0.1 first version included in the tclab package
      1.1.0 added R1 and R2 commands to read current heater values
            modified heater values to units of percent of full power
            added P1 and P2 commands to set heater power limits
            rewrote readCommand to avoid busy states
            simplified LED status model
      1.2.0 added LED command
      1.2.1 correctly reset heater values on close
            added version history
      1.2.2 shorten version string for better display by TCLab
      1.2.3 move baudrate to from 9600 to 115200
      1.3.0 add SCAN function
            report board type in version string
      1.4.0 changed Q1 and Q2 to float from int
      1.4.1 fix missing Serial.flush() at end of command loop
      1.4.2 fix bug with X command
      1.4.3 required Arduino IDE Version >= 1.0.0
      1.5.0 remove webusb
*/

#include "Arduino.h"
#include "NTC_Thermistor.h"
#include <PID_v1.h>

#define N_CHANNEL 4

NTC_Thermistor* thermistors[N_CHANNEL];

String boardType = "Arduino Due";

double Kp = 39, Ki = 2, Kd = 150;

double setPoints[N_CHANNEL];
double valInputs[N_CHANNEL];
double valOutputs[N_CHANNEL];

PID* myPIDs [N_CHANNEL];

const int offsets [N_CHANNEL]   = {10.0, 10.0, 10.0, 10.0};
const int enables [N_CHANNEL] = {1, 0, 0, 0};
const int inputPins [N_CHANNEL]   = {A0, A1, A2, A3};
const int outputPins [N_CHANNEL]   = { 13, 7, 12, 11 }; //TODO 14 added

#define REFERENCE_RESISTANCE 8000
#define NOMINAL_RESISTANCE     100000
#define NOMINAL_TEMPERATURE    25
#define B_VALUE                3950

// Enable debugging output
const bool DEBUG = true;

// constants
const String vers = "1.4.3";   // version of this firmware
const long baud = 9600;      // serial baud rate
const char sp = ' ';           // command separator
const char nl = '\n';          // command terminator

// global variables
char Buffer[64];               // buffer for parsing serial input
int buffer_index = 0;          // index for Buffer
String cmd;                    // command
float val;                     // command value

boolean newData = false;       // boolean flag indicating new command


void readCommand() {
  while (Serial && (Serial.available() > 0) && (newData == false)) {
    int byte = Serial.read();
    if ((byte != '\r') && (byte != nl) && (buffer_index < 64)) {
      Buffer[buffer_index] = byte;
      buffer_index++;
    }
    else {
      newData = true;
    }
  }
}

// for debugging with the serial monitor in Arduino IDE
void echoCommand() {
  if (newData) {
    Serial.write("Received Command: ");
    Serial.write(Buffer, buffer_index);
    Serial.write(nl);
    Serial.flush();
  }
}

// return thermister temperature in °C
inline float readTemperature(int pin) {
  if ((pin >= 0) && (pin < 5)) {
    return thermistors[pin]->readCelsius();
  }
  return 0;
}

void parseCommand(void) {
  if (newData) {
    String read_ = String(Buffer);

    // separate command from associated data
    int idx = read_.indexOf(sp);
    cmd = read_.substring(0, idx);
    cmd.trim();
    cmd.toUpperCase();

    // extract data. toFloat() returns 0 on error
    String data = read_.substring(idx + 1);
    data.trim();
    val = data.toFloat();

    // reset parameter for next command
    memset(Buffer, 0, sizeof(Buffer));
    buffer_index = 0;
    newData = false;
  }
}

void sendResponse(String msg) {
  Serial.println(msg);
}

void dispatchCommand(void) {
  if (cmd == "A") {
    for (int i = 0; i < N_CHANNEL; i++) {
      setHeater(i, 0);
    }
    sendResponse("Start");
  }
  else if (cmd.startsWith("Q")) {
    int index = cmd[1] - '0';
    setHeater(index, val);
    sendResponse(String(setPoints[index]));
  }
  else if (cmd.startsWith("T")) {
    int index = cmd[1] - '0';
    sendResponse(String(valInputs[index]));
  }
  else if (cmd == "VER") {
    sendResponse("TCLab Firmware " + vers + " " + boardType);
  }
  else if (cmd == "X") {
    for (int i = 0; i < N_CHANNEL; i++) {
      setHeater(i, 0);
    }
    sendResponse("Stop");
  }
  else if (cmd.length() > 0) {
    for (int i = 0; i < N_CHANNEL; i++) {
      setHeater(i, 0);
    }
    sendResponse(cmd);
  }
  Serial.flush();
  cmd = "";
}



void setHeater(int index, float qval) {
  if ((index >= 0) && (index < N_CHANNEL)) {
    setPoints[index] = qval;
  }
}


void updatePID() {
  for (int i = 0; i < N_CHANNEL; i++) {
    valInputs[i] = readTemperature(i);
    //Serial.print(i);
    Serial.print("\t");
    Serial.print(valInputs[i]);
    myPIDs[i]->Compute();
    Serial.print("\t");
    Serial.print(valOutputs[i]);
    if (enables[i]) {
      analogWrite(outputPins[i], valOutputs[i]);
      Serial.print("!");
    }
    Serial.print("->");
    Serial.print(setPoints[i]);
  }
  Serial.println();
}


// arduino startup
void setup() {
  //analogReference(EXTERNAL);
  while (!Serial) {
    ; // wait for serial port to connect.
   }
  Serial.begin(baud);
  Serial.flush();
  Serial.println("start");

  for (int i = 0; i < N_CHANNEL; i++) {
    setHeater(i, 0);

    Serial.println(i);
    thermistors[i] = new NTC_Thermistor(
      inputPins[i],
      REFERENCE_RESISTANCE,
      offsets[i],
      NOMINAL_RESISTANCE,
      NOMINAL_TEMPERATURE,
      B_VALUE
    );
    //thermistors[i]=tmp_thermistor;

    myPIDs[i] = new PID(&valInputs[i], &valOutputs[i], &setPoints[i], Kp, Ki, Kd, DIRECT);
    myPIDs[i]->SetMode(AUTOMATIC);

  }
  Serial.println("end setup");
  setHeater(0, 100);
}

// arduino main event loop
void loop() {
  // Serial.println("loop");
  readCommand();
  if (DEBUG) echoCommand();
  parseCommand();
  dispatchCommand();
  updatePID();
}
