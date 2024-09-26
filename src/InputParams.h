#ifndef INPUTPARAMS_H
#define INPUTPARAMS_H

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"

class InputParams
{
public:
  InputParams(int numOfSensors) : numberOfSensors(numOfSensors), protocol(""), espnow(""), address(""), sensor(""), sensor1(""), sensor2(""), period(""), mac(""), protocolMode(0), espnowMode(0), modbusAddress(0), sensorType(0), sleepPeriod(0), newMacAddress() {};

  String protocol;
  String espnow;
  String address;

  String sensor;
  String sensor1;
  String sensor2;

  String period;
  String mac;
  String addition;

  int protocolMode;
  int espnowMode;
  int modbusAddress;
  int sensorType;
  int sleepPeriod;
  uint8_t newMacAddress[6];
  int additionValue;

  int numberOfSensors;

  const char *PARAM_INPUT_1 = "protocol";
  const char *PARAM_INPUT_2 = "espnow";
  const char *PARAM_INPUT_3 = "address";

  const char *PARAM_INPUT_4 = "sensor1";
  const char *PARAM_INPUT_5 = "sensor2";

  const char *PARAM_INPUT_6 = "period";
  const char *PARAM_INPUT_7 = "mac";
  const char *PARAM_INPUT_8 = "addition";

  const char *protocolPath = "/protocol.txt";
  const char *espnowPath = "/espnow.txt";
  const char *addressPath = "/address.txt";
  const char *sensorPath = "/sensor.txt";
  const char *periodPath = "/period.txt";
  const char *macPath = "/mac.txt";
  const char *additionPath = "/addition.txt";

  void setParam(AsyncWebParameter *p);
  bool checkSettingValues();
  void showSettingValues();
  void writeToFile();
  void readFromFile();
  bool isWiredCommunicationMode();
  bool isWirelessCommunicationMode();
  bool isRs485Mode();
  bool isSensingMode();
  void parseMacString();

  int getSensorType();
};

void initSPIFFS();
void writeFile(fs::FS &fs, const char *path, const char *message);
String readFile(fs::FS &fs, const char *path);

#endif
