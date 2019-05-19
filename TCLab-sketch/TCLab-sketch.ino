#include "Arduino.h"
#include "NTC_Thermistor.h"
#include "PID_v1.h"

#define N_CHANNEL 4

NTC_Thermistor* thermistors[N_CHANNEL];

String boardType = "Arduino Due";

double Kp = 39, Ki = 2, Kd = 150;

double setPoints[N_CHANNEL];
double valInputs[N_CHANNEL];
double valOutputs[N_CHANNEL];

PID* myPIDs [N_CHANNEL];

double offsets [N_CHANNEL]   = {10.0, 10.0, 10.0, 10.0};
int enables [N_CHANNEL] = {0, 0, 0, 0};
const int inputPins [N_CHANNEL]   = {A0, A1, A2, A3};
const int outputPins [N_CHANNEL]   = { 13, 7, 12, 11 }; //TODO 14 added

#define REFERENCE_RESISTANCE 8000
#define NOMINAL_RESISTANCE     100000
#define NOMINAL_TEMPERATURE    25
#define B_VALUE                3950

// Enable debugging output
const bool DEBUG = false;

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
unsigned long last_request;

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
    last_request = millis();
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
      disable(i);
    }
    sendResponse("Start");
  }
  else if (cmd.startsWith("Q")) {
    int index = cmd[1] - '0';
    setHeater(index, val);
    sendResponse(String(setPoints[index]));
  }
  else if (cmd.startsWith("R")) {
    int index = cmd[1] - '0';
    sendResponse(String(setPoints[index]));
  }
  else if (cmd.startsWith("T")) {
    int index = cmd[1] - '0';
    sendResponse(String(valInputs[index]));
  }
    else if (cmd.startsWith("E")) {
    int index = cmd[1] - '0';
    enable(index);
    sendResponse(String(enables[index]));
  }
    else if (cmd.startsWith("D")) {
    int index = cmd[1] - '0';
    disable(index);
    sendResponse(String(enables[index]));
  }
  else if (cmd == "VER") {
    sendResponse("TCLab Firmware " + vers + " " + boardType);
  }
  else if (cmd == "X") {
    for (int i = 0; i < N_CHANNEL; i++) {
      disable(i);
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

void enable(int index) {
  if ((index >= 0) && (index < N_CHANNEL)) {
    enables[index]=true;
  }
}
void disable(int index) {
  if ((index >= 0) && (index < N_CHANNEL)) {
    enables[index]=false;
  }
}



void updatePID() {
  for (int i = 0; i < N_CHANNEL; i++) {
    valInputs[i] = readTemperature(i);
    myPIDs[i]->Compute();
    if (enables[i]) {
      analogWrite(outputPins[i], valOutputs[i]);
    }
  }
}


// arduino startup
void setup() {
  //analogReference(EXTERNAL);
  while (!Serial) {
    ; // wait for serial port to connect.
   }
  Serial.begin(baud);
  Serial.flush();
  //Serial.println("start");

  for (int i = 0; i < N_CHANNEL; i++) {
    setHeater(i, 0);

    //Serial.println(i);
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
  //Serial.println("end setup");
  //setHeater(0, 100);
}

// arduino main event loop
void loop() {
  // Serial.println("loop");
  readCommand();
  if (DEBUG) echoCommand();
  parseCommand();
  dispatchCommand();
  updatePID();
  if(( millis()-last_request )>10000){
    for (int i = 0; i < N_CHANNEL; i++) {
      disable(i);
    }
  }
}
