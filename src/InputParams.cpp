#include "InputParams.h"

void InputParams::setParam(AsyncWebParameter *p)
{
  if (p->name() == PARAM_INPUT_1) // protocol
  {
    protocol = p->value().c_str();
  }
  else if (p->name() == PARAM_INPUT_2) // espnow
  {
    espnow = p->value().c_str();
  }
  else if (p->name() == PARAM_INPUT_3) // address
  {
    address = p->value().c_str();
  }
  else if (p->name() == PARAM_INPUT_4) // sensor1;
  {
    sensor1 = p->value().c_str();
  }
  else if (p->name() == PARAM_INPUT_5) // sensor2;
  {
    sensor2 = p->value().c_str();
  }
  else if (p->name() == PARAM_INPUT_6) // period
  {
    period = p->value().c_str();
  }
  else if (p->name() == PARAM_INPUT_7) // mac
  {
    mac = p->value().c_str();
  }
  else if (p->name() == PARAM_INPUT_8) // addition
  {
    addition = p->value().c_str();
  }
}

bool InputParams::checkSettingValues()
{
  bool isValid = true;

  // 입력값 하나라도 비어있으면 false
  bool hasEssentialParams = ((protocol != "" && address != "" && sensor != "") ||                              // 유선 표준화 모드
                             (protocol != "" && address != "" && sensor != "" && espnow != "" && mac != "") || // 무선 표준화 모드
                             (protocol != "" && sensor != "" && period != "" && espnow != "" && mac != ""));   // 무선 센서 모드

  protocolMode = protocol.toInt();  // 1. 프로토콜 (유/무선)
  espnowMode = espnow.toInt();      // 2. 무선 통신 모드 (표준화-Slave/센서보드-Master)
  modbusAddress = address.toInt();  // 3. 표준화 보드용 modbus 주소
  sensorType = sensor.toInt();      // 4. 부착 센서 선택
  sleepPeriod = period.toInt();     // 5. 측정 주기
  parseMacString();                 // 6. MAC 주소 (ESP-NOW용 임의설정)
  additionValue = addition.toInt(); // 7. 무선 통신 센서 추가 (표준화:센서 = 1:2 연결)

  Serial.print("Protocol : ");
  Serial.println(protocolMode);
  Serial.print("ESP-Now Mode : ");
  Serial.println(espnowMode);
  Serial.print("Modbus Id : ");
  Serial.println(modbusAddress);
  Serial.print("Sensor Type : ");
  Serial.println(sensorType);
  Serial.print("Sensing period : ");
  Serial.println(sleepPeriod);
  Serial.print("Broadcast Mac Address : ");
  Serial.println(mac);
  Serial.print("Addition : ");
  Serial.println(additionValue);

  Serial.print("hasEssentialParams : ");
  Serial.println(hasEssentialParams ? "true" : "false");

  if (!hasEssentialParams) // 사전 입력값이 불완전할 시 false
  {
    Serial.println("Start AP mode for board setting.");
    Serial.println("Connect the WiFi called DAON-WIFI-MANAGER and connect to the following IP.");
    return false;
  }

  // Wired Mode
  if (isWiredCommunicationMode())
  {
    if (sensorType < 1 || sensorType > numberOfSensors)
    {
      Serial.println("Invalid sensor type value: " + String(sensorType));
      isValid = false;
    }

    if (modbusAddress < 1 || modbusAddress > 127)
    {
      Serial.println("Invalid modbus address value: " + String(modbusAddress));
      isValid = false;
    }
  }
  else if (isWirelessCommunicationMode())
  {
    // Wireless Mode + Master
    if (isRs485Mode()) // (==1) 무선+표준화 보드이면
    {
      if (modbusAddress < 1 || modbusAddress > 127) // modbus 주소 범위 오류
      {
        Serial.println("Invalid modbus address value: " + String(modbusAddress));
        isValid = false;
      }
    }
    // Wireless Mode + Slave
    else if (isSensingMode()) // (==2) 무선+센서 보드이면
    {
      // nothing
    }
    else
    {
      Serial.println("Invalid rs485/sensing mode value: " + String(espnowMode));
      isValid = false;
    }
  }
  // 유선(1)도 무선(2)도 아니면 오류
  else
  {
    Serial.println("Invalid Protocol value: " + String(protocolMode));
    isValid = false;
  }

  return isValid;
}

// MAC 주소를 문자열 형식에서 바이트 배열로 변환하는 함수
void InputParams::parseMacString()
{
  char *ptr; // strtoul 함수는 변환되지 않은 문자열의 포인터를 이 변수에 저장, 여기서는 사용되지 않지만 함수 호출에 필요
  for (int i = 0; i < 6; i++)
  {
    // mac 문자열의 부분 문자열 추출 str to ul(unsigned long)
    newMacAddress[i] = strtoul(mac.substring(i * 3, i * 3 + 2).c_str(), &ptr, 16); // substring(a, b): 인덱스 a부터 b-1까지 추출
  }
}

// 시리얼 모니터에 flash 내부 데이터 표시
void InputParams::showSettingValues()
{
  Serial.print("Protocol : ");
  Serial.println(protocolMode);
  Serial.print("ESP-Now Mode : ");
  Serial.println(espnowMode);
  Serial.print("Modbus Id : ");
  Serial.println(modbusAddress);
  Serial.print("Sensor Type : ");
  Serial.println(sensorType);
  Serial.print("Sensing period : ");
  Serial.println(sleepPeriod);
  Serial.print("Broadcast Mac Address : ");
  for (int i = 0; i < 6; i++) // 0~5 6개
  {
    if (mac[i] < 0x10) // 한자릿수 보간
    {
      Serial.print("0");
    }
    Serial.print(newMacAddress[i], HEX);
    if (i < 5) // 0~4 5개 콜론 (:)
    {
      Serial.print(":");
    }
  }
  Serial.println();
  Serial.print("Additional Device Flag : ");
  Serial.println(additionValue);
}

// WIFI Mananger에서 입력되어 플래시에 저장된 파일의 값을 변수에 할당하는 함수 묶음
void InputParams::readFromFile()
{
  protocol = readFile(SPIFFS, protocolPath);
  espnow = readFile(SPIFFS, espnowPath);
  address = readFile(SPIFFS, addressPath);
  sensor = readFile(SPIFFS, sensorPath);
  period = readFile(SPIFFS, periodPath);
  mac = readFile(SPIFFS, macPath);
  addition = readFile(SPIFFS, additionPath);
}

// WIFI Mananger에서 입력한 값을 플래시에 파일로 쓰는 함수 묶음
void InputParams::writeToFile()
{
  writeFile(SPIFFS, protocolPath, protocol.c_str());
  writeFile(SPIFFS, espnowPath, espnow.c_str());
  writeFile(SPIFFS, addressPath, address.c_str());
  writeFile(SPIFFS, sensorPath, (sensor1 + sensor2).c_str());
  writeFile(SPIFFS, periodPath, period.c_str());
  writeFile(SPIFFS, macPath, mac.c_str());
  writeFile(SPIFFS, additionPath, addition.c_str());
}

bool InputParams::isWiredCommunicationMode()
{
  return (protocolMode == 1); // 유선통신
}

bool InputParams::isWirelessCommunicationMode()
{
  return (protocolMode == 2); // 무선통신: espnow 사용
}

bool InputParams::isRs485Mode()
{
  return (espnowMode == 1); // 표준화 통신보드
}

bool InputParams::isSensingMode()
{
  return (espnowMode == 2); // 센서보드
}

void initSPIFFS()
{
  if (!SPIFFS.begin(true))
  {
    Serial.println("An error has occurred while mounting SPIFFS");
  }
  Serial.println("SPIFFS mounted successfully\r\n");
}

// Flash에 파일 쓰기 함수
void writeFile(fs::FS &fs, const char *path, const char *message)
{
  Serial.printf("Writing file: %s\r\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file)
  {
    Serial.println("- Failed to open file for Writing");
    return;
  }
  if (file.print(message))
  {
    Serial.println("- Write Success");
  }
  else
  {
    Serial.println("- Write Failed");
  }
}

// Flash에서 파일 읽기 함수
String readFile(fs::FS &fs, const char *path)
{
  File file = fs.open(path);
  if (!file || file.isDirectory())
  {
    return String();
  }

  String fileContent;
  while (file.available())
  {
    fileContent = file.readStringUntil('\n');
    break;
  }
  return fileContent;
}
