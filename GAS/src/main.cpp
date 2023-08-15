#include <Wire.h>
#include <Arduino.h>
#include "driver/twai.h"
#include <PID_v1.h>

#define CAN_RX_PIN 44
#define CAN_TX_PIN 43

#define NODE_TYPE 0x11
#define FIRMWARE_VERSION "1.0"

int major, minor;

unsigned long last_adc_reading_time[6] = {0};

// interval mezi čteními vzorků z ADC
const unsigned long ADC_READ_INTERVAL = 20; // 20 ms

double input1, output1, setpoint1, lastSetpoint1;
double input2, output2, setpoint2, lastSetpoint2;
double input3, output3, setpoint3, lastSetpoint3;

double Kp = 1, Ki = 3, Kd = 0;
const int pidSampleTime = 50;
const int outputMin = 300;
PID pidVent1(&input1, &output1, &setpoint1, Kp, Ki, Kd, P_ON_E, DIRECT);
PID pidVent2(&input2, &output2, &setpoint2, Kp, Ki, Kd, P_ON_E, DIRECT);
PID pidVent3(&input3, &output3, &setpoint3, Kp, Ki, Kd, P_ON_E, DIRECT);

const int ADC_AVERAGE_MEASURE = 4;

uint16_t pidForVent1 = 0;
uint16_t pidForVent2 = 0;
uint16_t pidForVent3 = 0;

enum GPIOToSend
{
  PR_GAS = 10,  // RELE 1
  CUT_OXI = 11, // RELE 2
  PR_OXI = 17,  // RELE 3
  EXHAUST = 18, // RELE 4
  HP_OXI = 16,  // RELE 5

  PWM_VENT_1 = 12,
  PWM_VENT_2 = 13,
  PWM_VENT_3 = 14,
};

const int PWM_FREQUENCY_PWM1 = 500; // Frekvence PWM   ************************DOPLNIT************************
const int PWM_FREQUENCY_PWM2 = 500; // Frekvence PWM   ************************DOPLNIT************************
const int PWM_FREQUENCY_PWM3 = 500; // Frekvence PWM   ************************DOPLNIT************************
const int PWM_RESOLUTION = 12;      // Rozlišení PWM (0 -  4095)
const int PWM_MAX = 4095;           // Max hodnota PWM

const int VENT_1_CHANNEL = 0;
const int VENT_2_CHANNEL = 1;
const int VENT_3_CHANNEL = 2;

const int VENT_1_TIMER = 0;
const int VENT_2_TIMER = 1;
const int VENT_3_TIMER = 2;

uint16_t ADC_FROM_CAN_1 = 0;
uint16_t ADC_FROM_CAN_2 = 0;
uint16_t ADC_FROM_CAN_3 = 0;
uint16_t ADC_FROM_CAN_4 = 0;
uint16_t ADC_FROM_CAN_5 = 0;
uint16_t ADC_FROM_CAN_6 = 0;

uint16_t ADC_FROM_ADC_1 = 0;
uint16_t ADC_FROM_ADC_2 = 0;
uint16_t ADC_FROM_ADC_3 = 0;
uint16_t ADC_FROM_ADC_4 = 0;
uint16_t ADC_FROM_ADC_5 = 0;
uint16_t ADC_FROM_ADC_6 = 0;

enum GPIOToRead
{
  ADC_1 = 4,
  ADC_2 = 5,
  ADC_3 = 6,
  ADC_4 = 7,
  ADC_5 = 8,
  ADC_6 = 9,
};

enum MessageIDToRecieve
{
  PWM_TARGET_ID = 0x287,
  ADC_VALUE_ID = 0x288,
  COMMAND_ID = 0x500,
};

enum MessageIDToSend
{
  SEND_OUTPUT_ADC_ID = 0x2E0,
  SEND_INPUT_ADC_ID = 0x2E1,
  SEDN_COMMAND_RESPOND_ID = 0x400,
  SEDN_PID_COMPUTED_ID = 0x600,
};

struct Message
{
  uint32_t id;

  void *data;
  uint8_t dataSize;
  uint32_t period;
  uint32_t lastSentTime;
};

struct MessageCommon
{
  uint8_t byte0;
  uint8_t byte1;
  uint8_t byte2;
  uint8_t byte3;
  uint8_t byte4;
  uint8_t byte5;
  uint8_t byte6;
  uint8_t byte7;
};

int readGPIO(int gpioPin)
{
  pinMode(gpioPin, INPUT);
  return digitalRead(gpioPin);
}

void setGPIO(int GPIO, bool state)
{
  digitalWrite(GPIO, state ? HIGH : LOW);
}

void splitVersion(const char *version, int &major, int &minor)
{
  if (sscanf(version, "%d.%d", &major, &minor) != 2)
  {
    Serial.println("Chyba: Neplatný formát verze.");
  }
}

MessageCommon *createMessageWithAllOnes()
{
  static MessageCommon *message = nullptr;
  if (message == nullptr)
  {
    message = new MessageCommon;
  }

  message->byte0 = 255;
  message->byte1 = 255;
  message->byte2 = 255;
  message->byte3 = 255;
  message->byte4 = 255;
  message->byte5 = 255;
  message->byte6 = 255;
  message->byte7 = 255;

  return message;
}

MessageCommon *createMessageWithAllZero()
{
  static MessageCommon *message = nullptr;
  if (message == nullptr)
  {
    message = new MessageCommon;
  }

  message->byte0 = 0;
  message->byte1 = 0;
  message->byte2 = 0;
  message->byte3 = 0;
  message->byte4 = 0;
  message->byte5 = 0;
  message->byte6 = 0;
  message->byte7 = 0;

  return message;
}

int read_adc_value(int pin)
{
  // aktuální čas v milisekundách
  unsigned long current_time = millis();

  // čtení hodnoty z ADC pinu
  int adc_value = 0;

  if (current_time - last_adc_reading_time[pin - 4] >= ADC_READ_INTERVAL)
  {
    adc_value = analogRead(pin);
    last_adc_reading_time[pin - 4] = current_time;
  }

  return adc_value;
}

MessageCommon *createMessage_SendInputADC()
{
  int adc_reading1 = ADC_FROM_ADC_1;
  int adc_reading3 = ADC_FROM_ADC_3;
  int adc_reading5 = ADC_FROM_ADC_5;

  static MessageCommon *message = nullptr;
  if (message == nullptr)
  {
    message = new MessageCommon;
  }
  message->byte0 = (adc_reading1 >> 8);   // horních 8 bitů z value1
  message->byte1 = (adc_reading1 & 0xFF); // spodních 8 bitů z value1
  message->byte2 = (adc_reading3 >> 8);   // horních 8 bitů z value2
  message->byte3 = (adc_reading3 & 0xFF); // spodních 8 bitů z value2
  message->byte4 = (adc_reading5 >> 8);   // horních 8 bitů z value3
  message->byte5 = (adc_reading5 & 0xFF); // spodních 8 bitů z value3
  message->byte6 = 0;
  message->byte7 = 0;
  return message;
}

MessageCommon *createMessage_SendOutPutADC()
{
  int adc_reading2 = ADC_FROM_ADC_2;
  int adc_reading4 = ADC_FROM_ADC_4;
  int adc_reading6 = ADC_FROM_ADC_6;

  static MessageCommon *message = nullptr;
  if (message == nullptr)
  {
    message = new MessageCommon;
  }
  message->byte0 = (adc_reading2 >> 8);   // horních 8 bitů z value1
  message->byte1 = (adc_reading2 & 0xFF); // spodních 8 bitů z value1
  message->byte2 = (adc_reading4 >> 8);   // horních 8 bitů z value2
  message->byte3 = (adc_reading4 & 0xFF); // spodních 8 bitů z value2
  message->byte4 = (adc_reading6 >> 8);   // horních 8 bitů z value3
  message->byte5 = (adc_reading6 & 0xFF); // spodních 8 bitů z value3
  message->byte6 = 0;
  message->byte7 = 0;
  return message;
}

MessageCommon *createMessage_SendComputePid()
{

  int compute_pid_vent1 = pidForVent1;
  int compute_pid_vent2 = pidForVent2;
  int compute_pid_vent3 = pidForVent3;
  static MessageCommon *message = nullptr;
  if (message == nullptr)
  {
    message = new MessageCommon;
  }
  message->byte0 = (compute_pid_vent1 >> 8);   // horních 8 bitů z value1
  message->byte1 = (compute_pid_vent1 & 0xFF); // spodních 8 bitů z value1
  message->byte2 = (compute_pid_vent2 >> 8);   // horních 8 bitů z value2
  message->byte3 = (compute_pid_vent2 & 0xFF); // spodních 8 bitů z value2
  message->byte4 = (compute_pid_vent3 >> 8);   // horních 8 bitů z value3
  message->byte5 = (compute_pid_vent3 & 0xFF); // spodních 8 bitů z value3
  message->byte6 = 0;
  message->byte7 = 0;
  return message;
}

void sendCANMessage(uint32_t messageID, void *data, uint8_t dataSize)
{
  // Připravení zprávy
  twai_message_t message;
  message.identifier = messageID;
  message.extd = 0;
  message.rtr = 0;
  message.data_length_code = dataSize;
  memcpy(message.data, data, message.data_length_code);
  Serial.print(" Send, data: ");
  Serial.println();
  Serial.print(" delka, data: ");
  Serial.println(message.data_length_code);
  Serial.println();
  Serial.print("Data bytes: ");
  for (int i = 0; i < message.data_length_code; ++i)
  {
    Serial.print(message.data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  twai_transmit(&message, pdMS_TO_TICKS(100));
}

MessageCommon *createMessage_COMMAND_RESPOND()
{
  static MessageCommon *message = nullptr;
  if (message == nullptr)
  {
    message = new MessageCommon;
  }
  message->byte0 = 0;
  message->byte1 = 0;
  message->byte2 = 0;
  message->byte3 = major;
  message->byte4 = minor;
  message->byte5 = 0;
  message->byte6 = 0;
  message->byte7 = 0;
  return message;
}

void handleCommand(int command)
{
  if (command == 0x01)
  {
    Serial.println("Get version");
    Serial.println();
    splitVersion(FIRMWARE_VERSION, major, minor);
  }
}

uint16_t *extractValuesFromCANMessage(uint8_t canData[8])
{

  static uint16_t values[3] = {0};

  for (int i = 0; i < 3; i++)
  {
    values[i] = (uint16_t)canData[i * 2] << 8 | canData[i * 2 + 1];
    Serial.println("Hodnota extrahovana z prichozi zpravy: " + values[i]);
    Serial.println();
  }

  return values;
}

uint16_t convertTo12Bit(uint16_t value)
{
  /*return value >> 4;*/ // Posun bitů doprava o 3 pozice
  return value;          // Posun bitů doprava o 3 pozice
}

void setVents(int toSetpint1, int toSetpint2, int toSetpint3)
{
  int adc_reading2 = ADC_FROM_CAN_2;
  int adc_reading4 = ADC_FROM_CAN_4;
  int adc_reading6 = ADC_FROM_CAN_6;

  setpoint1 = toSetpint1;
  setpoint2 = toSetpint2;
  setpoint3 = toSetpint3;

  input1 = adc_reading2;
  input2 = adc_reading4;
  input3 = adc_reading6;

  if (setpoint1 == 0)
  {
    pidVent1.SetMode(MANUAL);
    output1 = 0;
  }
  else
  {
    if (lastSetpoint1 == 0)
    {
      output1 = outputMin;
    }
    pidVent1.SetMode(AUTOMATIC);
  }

  if (setpoint2 == 0)
  {
    pidVent2.SetMode(MANUAL);
    output2 = 0;
  }
  else
  {
    if (lastSetpoint2 == 0)
    {
      output2 = outputMin;
    }
    pidVent2.SetMode(AUTOMATIC);
  }

  if (setpoint3 == 0)
  {
    pidVent3.SetMode(MANUAL);
    output3 = 0;
  }
  else
  {
    if (lastSetpoint3 == 0)
    {
      output3 = outputMin;
    }
    pidVent3.SetMode(AUTOMATIC);
  }

  // Provedení PID výpočtu pro každý ventil
  pidVent1.Compute();
  pidVent2.Compute();
  pidVent3.Compute();

  pidForVent1 = output1;
  pidForVent2 = output2;
  pidForVent3 = output3;

  // Nastavení PWM výstupů pro každý ventil invertovaně
  Serial.println("-----------PWM1------------");
  Serial.println((int)output1);
  Serial.println();
  Serial.println("-----------PWM2------------");
  Serial.println((int)output2);
  Serial.println();
  Serial.println("-----------PWM3------------");
  Serial.println((int)output3);
  Serial.println();
  ledcWrite(VENT_1_CHANNEL, setpoint1 == 0 ? 0 : (int)output1);
  ledcWrite(VENT_2_CHANNEL, setpoint2 == 0 ? 0 : (int)output2);
  ledcWrite(VENT_3_CHANNEL, setpoint3 == 0 ? 0 : (int)output3);

  lastSetpoint1 = setpoint1;
  lastSetpoint2 = setpoint2;
  lastSetpoint3 = setpoint3;
}

void handleCANMessage(int id, twai_message_t message)
{
  Serial.print("Message with ID 0x");
  Serial.print(id, HEX);
  Serial.print(" received, data: ");
  Serial.println();
  uint8_t *data = message.data;
  uint16_t *values = extractValuesFromCANMessage(message.data);
  switch (id)
  {
  case PWM_TARGET_ID:
    Serial.print("-----------RecievePWM_TARGET------------");
    Serial.println();
    Serial.print("Value1");
    Serial.println(values[0]);
    Serial.print("Value2");
    Serial.println(values[1]);
    Serial.print("Value3");
    Serial.println(values[2]);
    Serial.println();
    setVents(convertTo12Bit(values[0]), convertTo12Bit(values[1]), convertTo12Bit(values[2]));
    setGPIO(CUT_OXI, ((data[7] >> 0) & 0x01));
    setGPIO(PR_OXI, ((data[7] >> 1) & 0x01));
    setGPIO(PR_GAS, ((data[7] >> 2) & 0x01));
    setGPIO(HP_OXI, ((data[7] >> 3) & 0x01));
    setGPIO(EXHAUST, ((data[7] >> 4) & 0x01));

    sendCANMessage(SEND_OUTPUT_ADC_ID, createMessage_SendOutPutADC(), 8);
    sendCANMessage(SEND_INPUT_ADC_ID, createMessage_SendInputADC(), 8);
    sendCANMessage(SEDN_PID_COMPUTED_ID, createMessage_SendComputePid(), 8);
    break;

  case ADC_VALUE_ID:
    Serial.print("-----------RecieveADC_VALUE_ID------------");
    Serial.println();
    Serial.print("ADC2");
    Serial.println(values[0]);
    Serial.print("ADC4");
    Serial.println(values[1]);
    Serial.print("ADC6");
    Serial.println(values[2]);
    Serial.println();
    ADC_FROM_CAN_2 = convertTo12Bit(values[0]);
    ADC_FROM_CAN_4 = convertTo12Bit(values[1]);
    ADC_FROM_CAN_6 = convertTo12Bit(values[2]);
    break;

  case COMMAND_ID:
    Serial.print("-----------RecieveCOMMAND_ID------------");
    Serial.println();
    Serial.print("Command");
    Serial.println(data[2]);
    Serial.println();
    handleCommand(data[2]);
    sendCANMessage(SEDN_COMMAND_RESPOND_ID, createMessage_COMMAND_RESPOND(), 8);
    break;
  default:
    Serial.println("Neznama zprava");
    break;
  }
}

void setup()
{
  Serial.begin(115200);

  digitalWrite(PR_GAS, LOW);
  digitalWrite(CUT_OXI, LOW);
  digitalWrite(PR_OXI, LOW);
  digitalWrite(EXHAUST, LOW);
  digitalWrite(HP_OXI, LOW);
  pinMode(PR_GAS, OUTPUT);
  pinMode(CUT_OXI, OUTPUT);
  pinMode(PR_OXI, OUTPUT);
  pinMode(EXHAUST, OUTPUT);
  pinMode(HP_OXI, OUTPUT);

  digitalWrite(PWM_VENT_1, LOW);
  digitalWrite(PWM_VENT_2, LOW);
  digitalWrite(PWM_VENT_3, LOW);
  pinMode(PWM_VENT_1, OUTPUT);
  pinMode(PWM_VENT_2, OUTPUT);
  pinMode(PWM_VENT_3, OUTPUT);

  analogReadResolution(12);
  pinMode(ADC_1, INPUT);
  pinMode(ADC_2, INPUT);
  pinMode(ADC_3, INPUT);
  pinMode(ADC_4, INPUT);
  pinMode(ADC_5, INPUT);
  pinMode(ADC_6, INPUT);

  ledcSetup(VENT_1_CHANNEL, PWM_FREQUENCY_PWM1, PWM_RESOLUTION);
  ledcAttachPin(PWM_VENT_1, VENT_1_CHANNEL);

  ledcSetup(VENT_2_CHANNEL, PWM_FREQUENCY_PWM2, PWM_RESOLUTION);
  ledcAttachPin(PWM_VENT_2, VENT_2_CHANNEL);

  ledcSetup(VENT_3_CHANNEL, PWM_FREQUENCY_PWM3, PWM_RESOLUTION);
  ledcAttachPin(PWM_VENT_3, VENT_3_CHANNEL);

  ledcWrite(VENT_1_CHANNEL, 0);
  ledcWrite(VENT_2_CHANNEL, 0);
  ledcWrite(VENT_3_CHANNEL, 0);

  pidVent1.SetMode(AUTOMATIC);
  pidVent1.SetOutputLimits(0, PWM_MAX);
  pidVent1.SetControllerDirection(DIRECT);
  pidVent1.SetSampleTime(pidSampleTime);

  pidVent2.SetMode(AUTOMATIC);
  pidVent2.SetOutputLimits(0, PWM_MAX);
  pidVent2.SetControllerDirection(DIRECT);
  pidVent2.SetSampleTime(pidSampleTime);

  pidVent3.SetMode(AUTOMATIC);
  pidVent3.SetOutputLimits(0, PWM_MAX);
  pidVent3.SetControllerDirection(DIRECT);
  pidVent3.SetSampleTime(pidSampleTime);

  setpoint1 = 0;
  setpoint2 = 0;
  setpoint3 = 0;

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NO_ACK); // TWAI_MODE_NORMAL, TWAI_MODE_NO_ACK or TWAI_MODE_LISTEN_ONLY
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_driver_install(&g_config, &t_config, &f_config);
  twai_start();
}

void printReceivedMessage(const twai_message_t &message)
{
  Serial.println("-------------------Recieved---------------: ");
  Serial.print("Identifier: ");
  Serial.println(message.identifier, HEX);

  Serial.print("Data length code: ");
  Serial.println(message.data_length_code, DEC);

  Serial.print("Data: ");
  for (uint8_t i = 0; i < message.data_length_code; i++)
  {
    Serial.print(message.data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  Serial.println();
}

void recieveMessage(void)
{
  twai_message_t message;
  const TickType_t timeout_ticks = pdMS_TO_TICKS(10); // Set timeout to 10 milliseconds

  while (twai_receive(&message, timeout_ticks) == ESP_OK)
  {
    printReceivedMessage(message);
    handleCANMessage(message.identifier, message);
  }
}
uint16_t recalculateADCinRange(int adcPin) // prepocet hodnoty z ADC do rozsahu 0 - 4095 na delic 1k2 a 2k2
{
  const int numReadings = ADC_AVERAGE_MEASURE;
  int total = 0; // součet všech čtení

  for (int i = 0; i < numReadings; i++)
  {
    uint16_t reading = analogRead(adcPin);

    if (reading < 375)
    {
      reading = 0;
    }
    else
    {
      reading = map(reading, 375, 3516, 0, 4095); // 375 je 0.5V, 3516 je 4.51V, vse v rozsahu napeti 0 - 0.5V je vyhodnoceno jako 0
    }

    total += reading;

    Serial.print(" reading: ");
    Serial.print(reading);
  }

  Serial.println(" ");

  return total / numReadings; // průměrování čtení
}

void readADC()
{
  ADC_FROM_ADC_1 = recalculateADCinRange(ADC_1);
  ADC_FROM_ADC_2 = recalculateADCinRange(ADC_2);
  ADC_FROM_ADC_3 = recalculateADCinRange(ADC_3);
  ADC_FROM_ADC_4 = recalculateADCinRange(ADC_4);
  ADC_FROM_ADC_5 = recalculateADCinRange(ADC_5);
  ADC_FROM_ADC_6 = recalculateADCinRange(ADC_6);
}

void loop()
{
  readADC();
  recieveMessage();
}