#include <Arduino.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <esp32-sdi12.h>
#include "CRC.h"
#include "InputParams.h"

#define VERSION "0.3.2" // UI/UX 개선: 센서 입력부 2개 및 JS기능 추가, 아두이노용 코드 추가

#define ADDR_RK52002 0XFE

/* ----- Sensors ----- */

enum SensorId // enum은 값을 명시하지 않은 경우, 이전 항목의 값보다 1씩 증가하는 값을 자동으로 할당
{
  idTeros21SoilWaterPotential = 1,
  idWoosungRainDetector, // 2
  idHaimilRainDetector,  // 3
  // Complex Sensor (Only Wireless)
  idRk52002, // 4
  idRk52002Ec = 41,
  idRk52002Humi, // 42
  idRk52002Temp, // 43
  // Additional Sensor (Only Wireless, Not standard)
  idAddTeros21SoilWaterPotential = 5,
  idAddRk52002 = 6,
  idAddRk52002Ec = 61,
  idAddRk52002Humi = 62,
  idAddRk52002Temp = 63
};

// Number of sensor types
const int numberOfSensors = 6;
unsigned long heartbeatTimes[numberOfSensors];      // 센서별 timeout 계산에 필요한 millis() 값을 저장하는 배열
const unsigned long heartbeatTimeout = 1800 * 1000; // 1800초, 30분
void checkHeartbeatTime(int id);                    // 사용 안하는듯?
void resetSensorDataRegisters(int sensorType);      // 센서 타입(코드번호)으로 구분하여 레지스터 주소 설정

// Function to register in the registerMap.; 센서 연결 시 레지스터맵에 기록하는 함수
void attachTeros21SoilWaterPotential();
void attachWoosungRainDetector();
void attachHaimilRainDetector();
void attachRk52002();

// Function that reads sensor information and stores it in a registerMap and espnowData.; 센서값 획득 함수
void getTeros21SoilWaterPotential();
void getWoosungRainDetector(); // 단종
void getHaimilRainDetector();  // 단종
void getRk52002();

// Array of function pointers; 함수포인터로 함수선택
void (*attachSensors[numberOfSensors])() = {attachTeros21SoilWaterPotential, attachWoosungRainDetector, attachHaimilRainDetector, attachRk52002};
void (*getSensors[numberOfSensors])() = {getTeros21SoilWaterPotential, getWoosungRainDetector, getHaimilRainDetector, getRk52002};

// Function to set sensor pins.
void initSensorPins();

// Function to obtain the starting address of the register map that stores sensor data.
int getDataStoringIndex(int sensorType);

// SDI-12
const int sdi12DataPin = 5;
const int deviceAddr = 0;
ESP32_SDI12 sdi12(sdi12DataPin);
float sdi12Values[10];
uint8_t numberOfReturnedValues;

// EL817
const int el817Pin = 4;
float rainValue;

// Battery Analog Pin
const int batteryCheckPin = 32; // ADC1핀(32~35) 사용해야; ADC2는 WIFI 점유

/* ----- AP Mode ----- */

AsyncWebServer server(80);

InputParams params(numberOfSensors);

int loopFlag = 0;

enum LoopMode
{
  apMode = 1,
  wiredMode,
  wirelessRs485Mode,
  wirelessSensingMode
};

/* ----- ESP-NOW ----- */

typedef struct structMessage // structMessage 자료형
{
  int id;               // esp 센서보드의 id
  float sensorData;     // 센서 값
  uint16_t sensorState; // 센서 상태
  uint16_t battery;     // 배터리 전압값
} structMessage;

structMessage espnowData;
esp_now_peer_info_t peerInfo;

/* ----- KS3267 ----- */

enum ERROR_CODE
{
  illegalFunction = 1,
  illegalDataAddress,
  illegalDataValue,
  slaveDeviceFailure
};

const int ledPin = 2;
const int rs485TxPin = 17;
const int rs485RxPin = 16;
const int rs485TxEnPin = 12;
const int registerMapSize = 392;
const int mbFrameMaxSize = 256;
const int readHoldingRegisters = 0x03;

uint16_t registerMap[registerMapSize + 1]; // Index 0 is not used.

/* ----- Declare Setup Functions ----- */

void setupApMode(); // wifi manager 설정

void setupWiredMode();           // 유선 / 표준화 모드 (RS485 통신보드; Slave)
void setupWirelessRs485Mode();   // 무선 / 표준화 모드 (RS485 통신보드; Slave, ESP-NOW Slave)
void setupWirelessSensingMode(); // 무선 / 센서 모드 (센서 정보 측정; Master, ESP-NOW Master)

void initRs485();                               // RS485 통신 설정
void initWiredRegisterMap(int sensorTypeIndex); // 유선 모드에서 노드정보와 센서정보를 설정하는 함수
void initWirelessRegisterMap();                 // 무선 모드에서 노드정보와 센서정보를 설정하는 함수

void setNodeInfo(); // 노드 정보 설정
void setNodeAttachSensorInfo(int index);

/* ----- Declare Loop Functions ----- */

void loopWiredMode();
void loopWirelessRs485Mode();
void loopWirelessSensingMode();

/* ----- Declare Others ----- */

void blinkLed();
int charToInt(char c);

uint16_t getBatteryPercentage(int sensorType); // SZH-SSBH-043 모듈 사용, 전압값 계산
bool sendsOthersData = false;                  // hasCrcError() 함수와 연동하여 온도/전압 및 다른 센서값 전송 여부 결정

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len);

/* ----- Declare Modbus Functions (modbus.cpp) ----- */

void addCrc(byte buf[], int bufLen);               // CRC 계산해 마지막 2바이트에 붙이는 함수
uint16_t combineByte(byte highData, byte lowData); // 상위 8비트와 하위 8비트 결합
void receiveModbusFrame(byte *frame, int *frameLen);
bool hasRequestError(byte *mbRequest, int mbRequestLen, int slaveAddress);
bool hasCrcError(byte frame[], int frameLen);
void sendErrorResponse(int errorCode, byte slaveAddress, byte functionCode);
void sendResponse(byte *response, int responseLen);
void saveFloatToRegisterMap(float value, int startIndex);
void modbusDebugPrint(byte *buf, int bufLen);

/* ----- Declare Sensor Functions (sensor.cpp) ----- */

void checkSensorAwakeTime(int id); // onDataRecv에서 센서타입(id)을 받아 millis()로 타임아웃 계산

/* ----- Main ----- */

void setup()
{
  Serial.begin(115200);
  Serial.println("Sensor Board ver" + String(VERSION));
  Serial.println("Board Mac Address: " + WiFi.macAddress());

  initSPIFFS();
  params.readFromFile(); // 사전 입력값을 불러옴

  if (params.checkSettingValues()) // 사전 입력값 검증
  {
    params.showSettingValues();

    if (params.isWiredCommunicationMode())
    {
      setupWiredMode();
      loopFlag = wiredMode;
    }
    else if (params.isWirelessCommunicationMode() && params.isRs485Mode())
    {
      setupWirelessRs485Mode();
      loopFlag = wirelessRs485Mode;
    }
    else if (params.isWirelessCommunicationMode() && params.isSensingMode())
    {
      setupWirelessSensingMode();
      loopFlag = wirelessSensingMode;
    }
  }
  else
  {
    setupApMode(); // wifi manager 설정
    loopFlag = apMode;
  }
}

void loop()
{
  switch (loopFlag)
  {
  case apMode:
    return;
  case wiredMode:
    loopWiredMode();
    break;
  case wirelessRs485Mode:
    loopWirelessRs485Mode();
    break;
  case wirelessSensingMode:
    loopWirelessSensingMode();
    break;
  default:
    break;
  }
}

/* ----- Setup Functions ----- */

void setupApMode()
{
  Serial.println("Setting AP (Access Point)");
  WiFi.softAP("DAON-WIFI-MANAGER", NULL);
  // WiFi.softAP("uxF-StandardBoard-test", NULL);
  // WiFi.softAP("uxF-SensorBoard02-test", NULL);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // Web Server Root URL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/wifimanager.html", "text/html"); });

  server.serveStatic("/", SPIFFS, "/");

  server.on("/", HTTP_POST, [](AsyncWebServerRequest *request)
            {
    int paramsCnt = request->params();
    for (int i = 0; i < paramsCnt; i++)
    {
      AsyncWebParameter* p = request->getParam(i);
      if (p->isPost())
      {
        params.setParam(p);
      }
    }
    params.writeToFile();
    request->send(200, "text/plain", "Done. ESP will restart");
    delay(3000);
    ESP.restart(); });
  server.begin();
}

// 유선 표준화 모드
void setupWiredMode()
{
  Serial.println("Start wired Mode");

  initRs485();

  initSensorPins();

  initWiredRegisterMap(params.sensorType);
}

// 무선 표준화 모드; MAC 주소 임의설정; ESP-NOW 사용 (Slave)
void setupWirelessRs485Mode()
{
  Serial.println("Start wireless Rs485 Mode");

  initRs485();

  WiFi.mode(WIFI_STA);

  esp_wifi_set_mac(WIFI_IF_STA, &params.newMacAddress[0]); // wifi manager 사전입력 mac 주소로 변경
  Serial.print("[NEW] Broadcast MAC Address of RS485 Board :  ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(onDataRecv);

  initWirelessRegisterMap();
}

// 무선 센서 모드; 임의설정된 MAC 주소로 peer 등록; ESP-NOW 사용 (Master)
void setupWirelessSensingMode()
{
  Serial.println("Start wireless Sensing Mode");

  initRs485();
  initSensorPins();

  pinMode(32, INPUT); // 배터리 전압 확인 용도?

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(onDataSent);

  memcpy(peerInfo.peer_addr, params.newMacAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
}

// RS485 통신 설정
void initRs485()
{
  Serial2.begin(9600, SERIAL_8N1, rs485RxPin, rs485TxPin);
  pinMode(ledPin, OUTPUT);

  // GPIO12: RS485_DE
  pinMode(rs485TxEnPin, OUTPUT);
  digitalWrite(rs485TxEnPin, LOW);
}

// 유선 모드에서 노드정보와 센서정보를 설정하는 함수
void initWiredRegisterMap(int sensorTypeIndex)
{
  setNodeInfo();
  setNodeAttachSensorInfo(sensorTypeIndex);
}

// 무선 모드에서 노드정보와 센서정보를 설정하는 함수
void initWirelessRegisterMap()
{
  setNodeInfo();
  for (int i = 0; i < params.sensor.length(); i++)
  {
    setNodeAttachSensorInfo(charToInt(params.sensor[i]));
  }
}

// 노드 정보를 registerMap배열에 설정
void setNodeInfo()
{
  const uint16_t notUsed = 0;
  uint16_t nodeInfo[] = {notUsed, 0, 0, 1, 0, 10, 30}; // 노드 정보; 7개원소; 무슨 의미일까?
  memcpy(registerMap, nodeInfo, sizeof(nodeInfo));     // 노드 정보를 레지스터맵 [0]~[6] 까지 복사
}

// 센서 정보를 노드(레지스터 맵)에 설정
void setNodeAttachSensorInfo(int index)
{
  if (index < 1 || index > numberOfSensors) // 센서 타입 인덱스 유효성 검사 (wifimanager 센서번호)
  {
    Serial.println("Unsupported sensor type.");
    return;
  }
  Serial.println("Sensor type " + String(index) + " is attached");
  (*attachSensors[index - 1])(); // 함수 포인터: 인덱스에 해당하는 함수를 호출해 센서 타입 초기화 -> 레지스터맵에 센서 등록
}

/* ----- Loop Functions ----- */

// Modbus 통신을 처리하는 루프; modbus 요청수신 - 유효성검사 - 레지스터읽고 응답
void loopWiredMode()
{
  // Modbus 요청 수신 및 처리
  byte mbRequest[mbFrameMaxSize] = {
      // modbus 요청을 저장할 배열 0 초기화
      0,
  };
  int mbRequestLen = 0;                         // modbus 요청 길이
  receiveModbusFrame(mbRequest, &mbRequestLen); // modbus 요청 수신하여 배열에 저장

  // 요청 유효성 검사 및 디버깅 출력
  if (mbRequestLen > 0) // 요청이 있는 경우
  {
    Serial.print("Modbus request: ");
    modbusDebugPrint(mbRequest, mbRequestLen); // 요청 수신되면 요청 내용을 디버깅 출력

    if (hasRequestError(mbRequest, mbRequestLen, params.modbusAddress)) // 요청 유효성 검사
    {
      Serial.println("Modbus request error.");
      return;
    }

    uint16_t functionCode = mbRequest[1];     // modbus function code 추출
    if (functionCode == readHoldingRegisters) // func code: 0x03
    {
      uint16_t startAddress = combineByte(mbRequest[2], mbRequest[3]);      // 시작 주소 추출
      uint16_t numberOfRegisters = combineByte(mbRequest[4], mbRequest[5]); // 레지스터 수 추출

      // Request error check; 요청 오류 검사
      if (!(numberOfRegisters >= 0x0001 && numberOfRegisters <= 0x007D))
      {
        sendErrorResponse(illegalDataValue, params.modbusAddress, functionCode); // 잘못된 데이터 '값' 오류 응답을 전송
        return;
      }
      if (!(startAddress >= 0 && startAddress <= 598 && startAddress + numberOfRegisters <= 599))
      {
        sendErrorResponse(illegalDataAddress, params.modbusAddress, functionCode); // 잘못된 데이터 '주소' 오류 응답을 전송
        return;
      }

      // Sensing Functions; 센서 데이터 수집
      (*getSensors[params.sensorType - 1])(); // 배열의 함수 포인터를 사용해 현재 센서 타입에 맞는 데이터를 수집하는 함수 호출

      // Modbus 응답 생성 및 전송
      //  Send Response; 응답 전송 준비
      int mbResponseLen = numberOfRegisters * 2 + 5; // 응답 길이 계산
      byte mbResponse[mbResponseLen];
      mbResponse[0] = params.modbusAddress;  // 응답의 첫 바이트: Modbus 주소
      mbResponse[1] = functionCode;          // 응답의 두 번째 바이트: Function Code
      mbResponse[2] = numberOfRegisters * 2; // 바이트 수

      // 레지스터 데이터 채우기
      for (int i = 3; i < mbResponseLen - 2; i += 2)
      {
        mbResponse[i] = (registerMap[startAddress + i - (i + 3) / 2]) >> 8;
        mbResponse[i + 1] = (registerMap[startAddress + i - (i + 3) / 2]) & 0xFF;
      }
      addCrc(mbResponse, mbResponseLen); // CRC 추가

      Serial.print("Modbus response: ");
      modbusDebugPrint(mbResponse, mbResponseLen); // 응답 디버그 출력

      sendResponse(mbResponse, mbResponseLen); // 응답 전송
      Serial.println("");
      blinkLed(); // LED 깜박임
    }
    else
    {
      sendErrorResponse(illegalFunction, params.modbusAddress, functionCode); // 잘못된 기능 코드 오류 응답 전송
    }
  }
}

// 무선 표준화 모드에서 실행되는 loop()
void loopWirelessRs485Mode()
{
  for (int i = 0; i < numberOfSensors; i++)
  {
    if ((heartbeatTimes[i] != 0))
    {
      if (millis() - heartbeatTimes[i] > heartbeatTimeout)
      {
        Serial.println("Error : Sensor Type " + String(i + 1) + " is not response.");
        delay(10);
        resetSensorDataRegisters(i + 1);
        heartbeatTimes[i] = millis(); // 시간 초기화
      }
    }
  }

  byte mbRequest[mbFrameMaxSize] = {
      0,
  };
  int mbRequestLen = 0;
  receiveModbusFrame(mbRequest, &mbRequestLen);

  if (mbRequestLen > 0)
  {
    Serial.print("Modbus request: ");
    modbusDebugPrint(mbRequest, mbRequestLen);

    if (hasRequestError(mbRequest, mbRequestLen, params.modbusAddress))
    {
      Serial.println("Modbus request error.");
      return;
    }

    uint16_t functionCode = mbRequest[1];
    if (functionCode == readHoldingRegisters)
    {
      uint16_t startAddress = combineByte(mbRequest[2], mbRequest[3]);
      uint16_t numberOfRegisters = combineByte(mbRequest[4], mbRequest[5]);

      // Request error check.
      if (!(numberOfRegisters >= 0x0001 && numberOfRegisters <= 0x007D))
      {
        sendErrorResponse(illegalDataValue, params.modbusAddress, functionCode);
        return;
      }
      if (!(startAddress >= 0 && startAddress <= 598 && startAddress + numberOfRegisters <= 599))
      {
        sendErrorResponse(illegalDataAddress, params.modbusAddress, functionCode);
        return;
      }

      // Send Response
      int mbResponseLen = numberOfRegisters * 2 + 5;
      byte mbResponse[mbResponseLen];
      mbResponse[0] = params.modbusAddress;
      mbResponse[1] = functionCode;
      mbResponse[2] = numberOfRegisters * 2;

      for (int i = 3; i < mbResponseLen - 2; i += 2)
      {
        mbResponse[i] = (registerMap[startAddress + i - (i + 3) / 2]) >> 8;
        mbResponse[i + 1] = (registerMap[startAddress + i - (i + 3) / 2]) & 0xFF;
      }
      addCrc(mbResponse, mbResponseLen);

      Serial.print("Modbus response: ");
      modbusDebugPrint(mbResponse, mbResponseLen);

      sendResponse(mbResponse, mbResponseLen);
      blinkLed();
    }
    else
    {
      sendErrorResponse(illegalFunction, params.modbusAddress, functionCode);
    }
  }
}

void loopWirelessSensingMode()
{
  for (int i = 0; i < params.sensor.length(); i++)
  {
    int index = charToInt(params.sensor[i]);
    Serial.print("Sensor index: ");
    Serial.println(index);
    if (index < 1 || index > numberOfSensors)
    {
      Serial.println("Unsupported sensor type.");
    }
    else
    {
      // Sensing Functions; 센서값 획득
      (*getSensors[index - 1])(); // 연동1. EC센서의 경우 EC, Humi값은 이미 보냄

      // Check Wireless Sensor Board Battery.
      uint16_t batteryCharge = getBatteryPercentage(index);
      Serial.printf("battery Percentage: %u%%\n", batteryCharge);
      espnowData.battery = batteryCharge;

      if (sendsOthersData) // 연동2. EC, Humi 값을 못보낸 상황: getRk52002()의 hasCrcError() 발생 시 온도값 역시 전송 차단()
      {
        // Send message via ESP-NOW; 이 부분은 온도/배터리값 합쳐서 보내는듯? + 온도 뿐만아니라 수분장력 등 다른 센서값도 사용하는 영역
        esp_err_t result = esp_now_send(params.newMacAddress, (uint8_t *)&espnowData, sizeof(espnowData));

        if (result != ESP_OK)
        {
          Serial.println("Error: Send esp-now RK520-02: Temp/BattV data.");
          Serial.print("Code: ");
          Serial.println(result);
        }
      }
      else
      {
        Serial.println("Not send ESP-NOW data.");
      }
    }
  }

  int timeDelay = params.sleepPeriod * 1000;
  delay(timeDelay);
  Serial.println("\r\n");
}

/* ----- Others ----- */

uint16_t getBatteryPercentage(int sensorType)
{
  float analogVal = analogRead(batteryCheckPin);    // 0~4095 12비트 해상도
  float vout = analogVal * 3.3 / 4095;              // 아날로그 값을 전압으로 변환 esp32(0~4095) -> (0~3.3v)
  float vin = vout / (7500.0 / (30000.0 + 7500.0)); // SZH-SSBH-043 모듈 저항의 전압 분배에 의한 5배 역산

  // Serial.print("Input Analog Value: ");
  // Serial.println(analogVal);
  // Serial.print("Analog to Voltage (max 3.3v): ");
  // Serial.println(vout);
  // Serial.print("Battery Voltage: ");
  // Serial.println(vin);

  const float dischargeVolt = 7.6;    // 방전 기준 전압
  const float fullchargeVolt = 12.29; // 완충 기준 전압

  if (vin <= dischargeVolt)
  {
    return 0;
  }
  else if (vin >= fullchargeVolt)
  {
    return 100;
  }
  else
  {
    return (uint16_t)(((vin - dischargeVolt) / (fullchargeVolt - dischargeVolt)) * 100.0);
  }
}

void blinkLed()
{
  digitalWrite(ledPin, HIGH);
  delay(100);
  digitalWrite(ledPin, LOW);
}

int charToInt(char c)
{
  return (c >= '0' && c <= '9') ? (c - '0') : 0;
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success." : "Delivery Fail...");
}

void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len)
{
  // 송신자 정보 출력
  // MAC Addr
  char macStr[18]; // 2자리씩 6개 바이트 + 5개 구분자 + '\0'
  Serial.print("Packet received from: ");
  // snprintf() 함수로 포맷 지정 문자열을 사용해 mac 주소 형식의 문자열로 변환, 'macStr'에 저장
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
  memcpy(&espnowData, incomingData, sizeof(espnowData)); // incomingData 배열에 수신된 데이터를 espnowData 구조체로 복사
  Serial.printf("Sensor Type: %u \r\n", espnowData.id);  // %u 부호 없는 10진 정수

  checkSensorAwakeTime(espnowData.id); // 센서 타임아웃 갱신

  // 센서명+센서값 출력
  String sensorName;
  switch (espnowData.id)
  {
  case 1:
    sensorName = "Teros-21 Water Potential";
    break;
  case 41:
    sensorName = "RK520-02 EC";
    break;
  case 42:
    sensorName = "RK520-02 HUMI";
    break;
  case 43:
    sensorName = "RK520-02 TEMP";
    break;
  default:
    break;
  }
  Serial.print(sensorName);
  Serial.printf(": %.2f \r\n", espnowData.sensorData); // %u 부호 없는 10진 정수

  // save sensor data
  int regMapIndex = getDataStoringIndex(espnowData.id); // sensorType Case문
  saveFloatToRegisterMap(espnowData.sensorData, regMapIndex);

  // save battery
  int sensorId = espnowData.id > 10 ? espnowData.id / 10 : espnowData.id; // 1,2,3,4, 41, 42, 43... 을 앞의 한 자릿수로 통일(내림)

  // Serial.println("Debug Point 00");
  // Serial.print("getSensorType(): ");
  // Serial.println(params.getSensorType());
  // Serial.print("sensorId: ");
  // Serial.println(sensorId);
  // Serial.print("String(params.getSensorType())[0]: ");
  // Serial.println(String(params.getSensorType())[0]);
  // Serial.print("String(params.getSensorType())[1]: ");
  // Serial.println(String(params.getSensorType())[1]);

  // Serial.println((sensorId == String(params.getSensorType())[0] - '0' && (sensorId < 4 || espnowData.id == 43)) ? "true" : "false");
  // Serial.println((sensorId == String(params.getSensorType())[1] - '0' && (sensorId < 4 || espnowData.id == 43)) ? "true" : "false");
  // Serial.println((sensorId == String(params.getSensorType())[0] - '0') ? "true" : "false");
  // Serial.println((sensorId < 4 || espnowData.id == 43) ? "true" : "false");
  // Serial.println(params.getSensorType() > 10 ? "true" : "false");

  if (params.getSensorType() > 10) // 2개의 다른 센서가 하나의 표준화 보드와 매칭일 때
  {
    if (sensorId == String(params.getSensorType())[0] - '0' && (sensorId < 4 || espnowData.id == 43))
    { // sensor 01
      registerMap[293] = espnowData.battery;
      Serial.print("[293]Battery Percentage: ");
      Serial.printf("%u%% \r\n", espnowData.battery);
    }
    if (sensorId == String(params.getSensorType())[1] - '0' && (sensorId < 4 || espnowData.id == 43))
    { // sensor 02
      registerMap[393] = espnowData.battery;
      Serial.print("[393]Battery Percentage: ");
      Serial.printf("%u%% \r\n", espnowData.battery);
    }
  }
  else if (sensorId <= 4) // 1개만 연결했거나 2개의 같은 센서가 하나의 표준화 보드와 매칭일 때
  {
    registerMap[293] = espnowData.battery;
    Serial.print("[293]Battery Percentage: ");
    Serial.printf("%u%% \r\n", espnowData.battery);
  }
  else if (sensorId == 5 || sensorId == 6)
  {
    registerMap[393] = espnowData.battery;
    Serial.print("[393]Battery Percentage: ");
    Serial.printf("%u%% \r\n", espnowData.battery);
  }
  else
  {
    Serial.println("ESP-NOW sensor ID error.");
  }

  Serial.println(); // 수신 로그 구분 개행
}

// modbus.cpp ************************************************************************************************************************************
void addCrc(byte buf[], int bufLen)
{
  uint16_t crc = calcCRC16(buf, bufLen - 2, 0x8005, 0xFFFF, 0, true, true); // CRC16 값 계산, true, true는 영향 없나?
  byte crcHigh = crc >> 8;                                                  // 계산된 16비트 CRC 값의 상위 8비트
  byte crcLow = crc & 0xFF;                                                 // 계산된 16비트 CRC 값의 하위 8비트
  *(buf + (bufLen - 2)) = crcLow;                                           // 버퍼의 마지막에서 두 번째 위치에 하위 8비트 저장
  *(buf + (bufLen - 1)) = crcHigh;                                          // 버퍼의 마지막 위치에 상위 8비트 저장
}

uint16_t combineByte(byte highData, byte lowData)
{
  return ((uint16_t)highData << 8) | lowData;
}

// Serial2 포트에서 Modbus 프레임을 수신하여 주어진 프레임 배열에 저장하고, 프레임의 길이를 갱신합니다.
// 각 바이트를 읽은 후에는 짧은 지연을 추가하여 다음 바이트가 수신될 때까지 기다립니다.
void receiveModbusFrame(byte *frame, int *frameLen)
{
  while (Serial2.available())
  {                                    // Serial2 포트에 수신된 데이터가 있는지 확인
    frame[*frameLen] = Serial2.read(); // 데이터를 읽어와 프레임 배열에 저장
    (*frameLen)++;                     // 프레임 길이를 1 증가
    delayMicroseconds(1250);           // 1/9600*8*1.5 sec, 다음 바이트를 읽기 전에 짧은 지연(1.25밀리초)
                                       // 9600 baud의 통신 속도에서는 1 바이트(8비트)를 전송하는 데 약 1.04밀리초가 필요하며,
                                       // 여기에 안전 마진을 고려하여 1.5배를 곱한 값입니다.
                                       // 이는 연속된 바이트 사이에 적절한 간격을 유지하여 데이터 수신이 안정적으로 이루어지도록 합니다.
  }
}

bool hasRequestError(byte *mbRequest, int mbRequestLen, int slaveAddress)
{
  return mbRequest[0] != slaveAddress || mbRequestLen < 2 || hasCrcError(mbRequest, mbRequestLen);
}

// 주어진 프레임의 CRC를 검증하여 오류가 있는지 확인하는 함수 (Cyclic Redundancy Check)
bool hasCrcError(byte frame[], int frameLen)
{
  if (frameLen < 2) // 프레임 길이가 2보다 작은 경우 유효한 CRC를 포함할 수 없음으로 오류
  {
    Serial.println("Debug Point 01: frameLen < 2");
    Serial.printf("frameLen: %d\n", frameLen);
    return true;
  }

  byte dataField[frameLen - 2]; // 프레임에서 마지막 두 바이트(CRC)를 제외한 데이터를 복사
  for (int i = 0; i < frameLen - 2; i++)
  {
    dataField[i] = frame[i];
  }
  uint16_t newCrc = calcCRC16(dataField, frameLen - 2, 0x8005, 0xFFFF, 0, true, true);
  byte newCrcHighByte = newCrc >> 8;  // CRC 상위 바이트
  byte newCrcLowByte = newCrc & 0xFF; // CRC 하위 바이트

  Serial.printf("(High)frameCRC[frameLen-1]: %x\n", frame[frameLen - 1]);
  Serial.printf("(Low)frameCRC[frameLen-2]: %x\n", frame[frameLen - 2]);

  Serial.print("newCRC: ");
  Serial.printf("%x\n", newCrc);

  // 프레임의 CRC와 계산된 CRC를 비교
  return frame[frameLen - 1] != newCrcHighByte ||
         frame[frameLen - 2] != newCrcLowByte;
}

// 특정 오류 코드를 포함한 Modbus 예외 응답을 생성하는 함수
void sendErrorResponse(int errorCode, byte slaveAddress, byte functionCode)
{
  int exceptionResponseBufLen = 5;                    // 예외 응답 버퍼의 길이 설정 (5 Bytes)
  byte exceptionResponseBuf[exceptionResponseBufLen]; // 예외 응답 버퍼 선언

  exceptionResponseBuf[0] = slaveAddress;                // 슬레이브 주소 설정
  exceptionResponseBuf[1] = functionCode + 0x80;         // 기능 코드에 0x80을 더하여 예외 응답 코드로 설정
  exceptionResponseBuf[2] = errorCode;                   // 오류 코드 설정
  addCrc(exceptionResponseBuf, exceptionResponseBufLen); // CRC를 예외 응답 버퍼에 추가 (CRC: 데이터 무결성 확인용)

  sendResponse(exceptionResponseBuf, exceptionResponseBufLen); // 오류 응답 전송
}

// 주어진 응답 데이터를 RS485 통신으로 전송하는 함수
void sendResponse(byte *response, int responseLen)
{
  digitalWrite(rs485TxEnPin, HIGH);     // RS485 송신 모드로 전환
  Serial2.write(response, responseLen); // 응답 데이터 전송; response 배열의 데이터를 responseLen 길이만큼 전송
  Serial2.flush();                      // 모든 데이터 전송 완료 대기; flush() 함수는 송신 버퍼가 비워질 때까지 블록됩니다.
  digitalWrite(rs485TxEnPin, LOW);      // RS485 수신 모드로 전환
}

void saveFloatToRegisterMap(float value, int startIndex)
{
  const int minFieldIndex = 203;
  // const int maxFieldIndex = 292;
  const int maxFieldIndex = 392;
  const int sensorState = 0;

  if (startIndex < minFieldIndex || startIndex > maxFieldIndex)
  {
    Serial.println("Invalid registerMap Index");
    return;
  }

  uint32_t bits = *(reinterpret_cast<uint32_t *>(&value));
  uint16_t bottom16bits = bits & 0xFFFF;
  uint16_t top16bits = bits >> 16;

  registerMap[startIndex] = bottom16bits;
  registerMap[startIndex + 1] = top16bits;
  registerMap[startIndex + 2] = sensorState;
}

void modbusDebugPrint(byte *buf, int bufLen)
{
  for (int i = 0; i < bufLen; i++)
  {
    if (buf[i] < 0x10)
    {
      Serial.print("0");
    }
    Serial.print(buf[i], HEX);
    Serial.print(" ");
  }
  Serial.println("");
}

// sensor.cpp ************************************************************************************************************************************
// Sensor Function Implementation File

// Function to set sensor pins.
void initSensorPins()
{
  sdi12.begin();            // 수분장력 센서용
  pinMode(el817Pin, INPUT); // 감우센서용
}

// Function to obtain the starting address of the register map that stores sensor data.
int getDataStoringIndex(int sensorType)
{
  switch (sensorType)
  {
  // WaterPotential
  case 1:
    return 251;
  // Rain
  case 2:
  case 3:
    return 218;
  // Wireless only Composite sensor 4 (401~403)
  case 41: // RK520-02 EC
    return 242;
  case 42: // RK520-02 HUMI
    return 248;
  case 43: // RK520-02 TEMP
    return 257;
  // Additional Sensor
  case 5: // 추가된 수분장력센서
    return 251 + 100;
  case 61: // 추가된 RK520-02 EC
    return 242 + 100;
  case 62: // 추가된 RK520-02 HUMI
    return 248 + 100;
  case 63: // 추가된 RK520-02 TEMP
    return 257 + 100;
  default:
    Serial.println("Attempting to find the address of the register map with an invalid sensor type.");
    return 0;
  }
}

void checkSensorAwakeTime(int id)
{
  if (id == idRk52002Ec || id == idRk52002Humi || id == idRk52002Temp) // 41,42,43 -> 4(Rk52002)
  {
    id = idRk52002;
  }
  else if (id == idAddRk52002Ec || id == idAddRk52002Humi || id == idAddRk52002Temp)
  {
    id = idAddRk52002;
  }

  heartbeatTimes[id - 1] = millis(); // id 인덱스 배열에 시간저장
}

// 센서 타입(코드번호)으로 구분하여 레지스터 주소 설정
void resetSensorDataRegisters(int sensorType)
{
  // Complex Sensor Rk520-02
  if (sensorType == idRk52002)
  {
    const int ecAddress = 242;
    const int humiAddress = 248;
    const int tempAddress = 257;

    // ec 레지스터 주소
    registerMap[ecAddress] = 0;     // 242 : 0
    registerMap[ecAddress + 1] = 0; // 243 : 0
    registerMap[ecAddress + 2] = 1; // 244 : 1

    // 지습 레지스터 주소
    registerMap[humiAddress] = 0;     // 248 : 0
    registerMap[humiAddress + 1] = 0; // 249 : 0
    registerMap[humiAddress + 2] = 1; // 250 : 1

    // 지온 레지스터 주소
    registerMap[tempAddress] = 0;     // 257 : 0
    registerMap[tempAddress + 1] = 0; // 258 : 0
    registerMap[tempAddress + 2] = 1; // 259 : 1
  }
  else if (sensorType == idAddRk52002)
  {
    const int ecAddress = 242 + 100;
    const int humiAddress = 248 + 100;
    const int tempAddress = 257 + 100;

    registerMap[ecAddress] = 0;
    registerMap[ecAddress + 1] = 0;
    registerMap[ecAddress + 2] = 1;

    registerMap[humiAddress] = 0;
    registerMap[humiAddress + 1] = 0;
    registerMap[humiAddress + 2] = 1;

    registerMap[tempAddress] = 0;
    registerMap[tempAddress + 1] = 0;
    registerMap[tempAddress + 2] = 1;
  }
  // Other Sensors
  else
  {
    int registerAddress = getDataStoringIndex(sensorType);
    registerMap[registerAddress] = 0;
    registerMap[registerAddress + 1] = 0;
    registerMap[registerAddress + 2] = 1;
  }
}

void attachTeros21SoilWaterPotential()
{
  registerMap[117] = 15;
}

void attachWoosungRainDetector()
{
  registerMap[106] = 4;
}

void attachHaimilRainDetector()
{
  registerMap[106] = 4;
}

void attachRk52002()
{
  registerMap[114] = 12;
  registerMap[116] = 14;
  registerMap[119] = 17;
}

void getTeros21SoilWaterPotential()
{
  ESP32_SDI12::Status res = sdi12.measure(deviceAddr, sdi12Values, sizeof(sdi12Values), &numberOfReturnedValues);

  sendsOthersData = true;

  if (res != ESP32_SDI12::SDI12_OK)
  {
    Serial.printf("ESP32_SDI12 Error: %d\n", res);
    sendsOthersData = false; // 통신에 오류있으면 보내지 않음
  }
  float sensorValue = sdi12Values[0];
  Serial.print("Soil Water Potential: ");
  Serial.println(sensorValue);

  Serial.println("Debug Point 02 ************");

  Serial.print("SDI12 Arrays: ");
  for (int i = 0; i < sizeof(sdi12Values) / sizeof(sdi12Values[0]); i++)
  {
    Serial.printf("%.2f ", sdi12Values[i]);
  }
  Serial.println();

  const int soilWaterPotentialIndex = 251;
  saveFloatToRegisterMap(sdi12Values[0], soilWaterPotentialIndex);

  espnowData.id = idTeros21SoilWaterPotential;
  espnowData.sensorData = sdi12Values[0];
  espnowData.sensorState = 0;

  if (params.additionValue)
  {
    espnowData.id = idAddTeros21SoilWaterPotential;
  }
}

void getWoosungRainDetector()
{
  int digitalValue = digitalRead(el817Pin);
  if (digitalValue == LOW)
  {
    Serial.println("Rain Detected");
    rainValue = 1.0;
  }
  else
  {
    Serial.println("Rain Undetected");
    rainValue = 0.0;
  }

  const int rainDetectorIndex = 218;
  saveFloatToRegisterMap(rainValue, rainDetectorIndex);

  espnowData.id = idWoosungRainDetector;
  espnowData.sensorData = rainValue;
  espnowData.sensorState = 0;
}

void getHaimilRainDetector()
{
  int digitalValue = digitalRead(el817Pin);
  if (digitalValue == LOW)
  {
    Serial.println("Rain Detected");
    rainValue = 1.0;
  }
  else
  {
    Serial.println("Rain Undetected");
    rainValue = 0.0;
  }

  const int rainDetectorIndex = 218;
  saveFloatToRegisterMap(rainValue, rainDetectorIndex);

  espnowData.id = idHaimilRainDetector;
  espnowData.sensorData = rainValue;
  espnowData.sensorState = 0;
}

void getRk52002()
{
  byte request[] = {ADDR_RK52002, 0x03, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00}; // RK520-02 8bytes Request
  int reqLen = sizeof(request) / sizeof(request[0]);

  addCrc(request, reqLen); // CRC 붙이기

  digitalWrite(rs485TxEnPin, HIGH); // HIGH: 송신 모드 활성화
  Serial2.write(request, reqLen);   // request 배열의 데이터를 Serial2로 전송
  Serial2.flush();                  // 송신 버퍼에 남아있는 데이터가 모두 전송될 때까지 대기
  digitalWrite(rs485TxEnPin, LOW);  // 모든 데이터가 전송된 이후 송신 모드 비활성화 (LOW)

  delay(100);

  // modbus 응답 처리 ****************************************************************
  byte response[11] = {0}; // 배열 0으로 초기화 선언
  int responseLen = 0;

  bool validStart = false; // 유효한 modbus 응답이 왔는지 flag

  // response 배열을 채우는 while문
  while (responseLen < sizeof(response) / sizeof(response[0])) // 길이가 11보다 작으면 수행
  {
    if (!Serial2.available())
    {
      break; // 데이터가 더 이상 없으면 종료
    }

    byte incomingByte = Serial2.read();                   // 한 바이트 읽고
    if (incomingByte == ADDR_RK52002 && responseLen == 0) // 유효 시작바이트가 RK52002 주소
    {
      // 정상적인 Modbus 신호 시작 (0xFE)
      validStart = true;
      Serial.print("Rk520-02 response: ");
    }

    if (validStart)
    {
      // Serial 모니터 출력부
      if (incomingByte < 0x10) // 16진수 출력 포맷 자릿수
      {
        Serial.print("0");
      }
      Serial.print(incomingByte, HEX); // 16진수 출력
      Serial.print(" ");               // 각 바이트 사이의 공백

      response[responseLen++] = incomingByte; // 대입연산 수행 후 증가

      if (responseLen == sizeof(response) / sizeof(response[0]))
      {
        // 11바이트가 모두 수신되면 종료
        break;
      }
    } // if (validStart)
  } // while

  Serial.println();

  Serial.print("Response length: ");
  Serial.println(responseLen);

  if (hasCrcError(response, responseLen))
  {
    sendsOthersData = false; // 나머지 data도 보내지 않음
    Serial.println("Sensor Response CRC error");
  }
  else
  {
    sendsOthersData = true;

    byte tempHigh = response[3];
    byte tempLow = response[4];
    byte humiHigh = response[5];
    byte humiLow = response[6];
    byte ecHigh = response[7];
    byte ecLow = response[8];

    float temp = 0.0;
    float humi = 0.0;
    float ec = 0.0;

    int tempData = tempHigh * 256 + tempLow;
    if (tempData < 0x8000)
    {
      temp = tempData / 10.0;
    }
    else
    {
      temp = (tempData - 0xFFFF - 0x01) / 10.0;
    }
    humi = (humiHigh * 256 + humiLow) / 10.0;
    ec = (ecHigh * 256 + ecLow) / 1000.0;

    // Send message via ESP-NOW
    espnowData.sensorState = 0;
    esp_err_t result;

    espnowData.id = idRk52002Ec;
    espnowData.sensorData = ec;
    if (params.additionValue)
    {
      espnowData.id = idAddRk52002Ec;
    }
    result = esp_now_send(params.newMacAddress, (uint8_t *)&espnowData, sizeof(espnowData)); // EC 전송
    if (result != ESP_OK)
    {
      Serial.println("Error : send esp-now RK520-02: EC data.");
      Serial.print("Code: ");
      Serial.println(result);
    }
    delay(1);

    espnowData.id = idRk52002Humi;
    espnowData.sensorData = humi;
    if (params.additionValue)
    {
      espnowData.id = idAddRk52002Humi;
    }
    result = esp_now_send(params.newMacAddress, (uint8_t *)&espnowData, sizeof(espnowData)); // Humi 전송
    if (result != ESP_OK)
    {
      Serial.println("Error : send esp-now RK520-02: Humi data.");
      Serial.print("Code: ");
      Serial.println(result);
    }
    delay(1);

    espnowData.id = idRk52002Temp;
    espnowData.sensorData = temp;
    if (params.additionValue)
    {
      espnowData.id = idAddRk52002Temp;
    }
    // 왜 여기 밑이 없지? -> 배터리 전압값 포함해서 온도랑 같이 보내기 때문인듯
  }
}
