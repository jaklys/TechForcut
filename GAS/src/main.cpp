#include <Wire.h>
#include <Arduino.h>
#include "driver/twai.h"
#include <PID_v1.h>

#define CAN_RX_PIN 44
#define CAN_TX_PIN 43

unsigned long last_adc_reading_time[6] = {0};

// interval mezi čteními vzorků z ADC
const unsigned long ADC_READ_INTERVAL = 10; // 10 ms

double input1, output1, setpoint1;
double input2, output2, setpoint2;
double input3, output3, setpoint3;

double Kp = 2, Ki = 5, Kd = 1;

PID pidVent1(&input1, &output1, &setpoint1, Kp, Ki, Kd, DIRECT);
PID pidVent2(&input2, &output2, &setpoint2, Kp, Ki, Kd, DIRECT);
PID pidVent3(&input3, &output3, &setpoint3, Kp, Ki, Kd, DIRECT);

uint16_t pidForVent1 = 0;
uint16_t pidForVent2 = 0;
uint16_t pidForVent3 = 0;

enum GPIOToSend
{
  SOL1 = 10,
  SOL2 = 11,
  PWM_VENT_1 = 14,
  PWM_VENT_2 = 13,
  PWM_VENT_3 = 12,
};

const int PWM_FREQUENCY = 500; // Frekvence PWM
const int PWM_RESOLUTION = 13; // Rozlišení PWM (0 -  8191)
const int PWM_MAX = 8191;      // Max hodnota PWM

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
  MESSAGE3_ID = 0x500,
};

enum MessageIDToSend
{
  SEND_OUTPUT_ADC_ID = 0x2E0,
  SEND_INPUT_ADC_ID = 0x2E1,
  COMMAND_RESPOND_ID = 0x400,
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

MessageCommon *createMessage_SendOutPutADC()
{
  int adc_reading1 = ADC_FROM_ADC_1;
  int adc_reading2 = ADC_FROM_ADC_2;
  int adc_reading3 = ADC_FROM_ADC_3;

  static MessageCommon *message = nullptr;
  if (message == nullptr)
  {
    message = new MessageCommon;
  }
  message->byte0 = (adc_reading1 >> 8);   // horních 8 bitů z value1
  message->byte1 = (adc_reading1 & 0xFF); // spodních 8 bitů z value1
  message->byte2 = (adc_reading2 >> 8);   // horních 8 bitů z value2
  message->byte3 = (adc_reading2 & 0xFF); // spodních 8 bitů z value2
  message->byte4 = (adc_reading3 >> 8);   // horních 8 bitů z value3
  message->byte5 = (adc_reading3 & 0xFF); // spodních 8 bitů z value3
  message->byte6 = 0;
  message->byte7 = 0;
  return message;
}

MessageCommon *createMessage_SendInputADC()
{
  int adc_reading4 = ADC_FROM_ADC_4;
  int adc_reading5 = ADC_FROM_ADC_5;
  int adc_reading6 = ADC_FROM_ADC_6;

  static MessageCommon *message = nullptr;
  if (message == nullptr)
  {
    message = new MessageCommon;
  }
  message->byte0 = (adc_reading4 >> 8);   // horních 8 bitů z value1
  message->byte1 = (adc_reading4 & 0xFF); // spodních 8 bitů z value1
  message->byte2 = (adc_reading5 >> 8);   // horních 8 bitů z value2
  message->byte3 = (adc_reading5 & 0xFF); // spodních 8 bitů z value2
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
  message->byte3 = 0;
  message->byte4 = 0;
  message->byte5 = 0;
  message->byte6 = 0;
  message->byte7 = 0;
  return message;
}

uint16_t *extractValuesFromCANMessage(const uint8_t canData[8])
{

  static uint16_t values[3] = {0};

  for (int i = 0; i < 3; i++)
  {
    values[i] = (uint16_t)canData[i * 2] << 8 | canData[i * 2 + 1];
  }

  return values;
}

uint16_t convertTo12Bit(uint16_t value)
{
   /*return value >> 4;*/ // Posun bitů doprava o 3 pozice
  return value; // Posun bitů doprava o 3 pozice
}

void setVents(int toSetpint1, int toSetpint2, int toSetpint3)
{

  int adc_reading1 = ADC_FROM_CAN_1;
  int adc_reading2 = ADC_FROM_CAN_2;
  int adc_reading3 = ADC_FROM_CAN_3;

  setpoint1 = toSetpint1;
  setpoint2 = toSetpint2;
  setpoint3 = toSetpint3;

  input1 = adc_reading1;
  input2 = adc_reading2;
  input3 = adc_reading3;

  // Provedení PID výpočtu pro každý ventil
  pidVent1.Compute();
  pidVent2.Compute();
  pidVent3.Compute();

  pidForVent1 = output1;
  pidForVent2 = output2;
  pidForVent3 = output3;

  // Nastavení PWM výstupů pro každý ventil invertovaně
  Serial.println("-----------PWM1------------");
  Serial.println(PWM_MAX - (int)output1);
  Serial.println();
  Serial.println("-----------PWM2------------");
  Serial.println(PWM_MAX - (int)output2);
  Serial.println();
  Serial.println("-----------PWM3------------");
  Serial.println(PWM_MAX - (int)output3);
  Serial.println();
  ledcWrite(VENT_1_CHANNEL, PWM_MAX - (int)output1);
  ledcWrite(VENT_2_CHANNEL, PWM_MAX - (int)output2);
  ledcWrite(VENT_3_CHANNEL, PWM_MAX - (int)output3);
}

void handleCANMessage(int id, twai_message_t message)
{
  Serial.print("Message with ID 0x");
  Serial.print(id, HEX);
  Serial.print(" received, data: ");
  Serial.println();
  uint8_t *data = message.data;

  switch (id)
  {
  case PWM_TARGET_ID:
    static uint16_t *values = extractValuesFromCANMessage(data);
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
    setGPIO(SOL1, (data[0] >> 0) & 0x01); // získat logický stav 2. bitu
    setGPIO(SOL2, (data[0] >> 4) & 0x01); // získat logický stav 3. bitu

    sendCANMessage(SEND_OUTPUT_ADC_ID, createMessage_SendOutPutADC(), 8);
    sendCANMessage(SEND_INPUT_ADC_ID, createMessage_SendInputADC(), 8);
    sendCANMessage(SEDN_PID_COMPUTED_ID, createMessage_SendComputePid(), 8);
    break;

  case ADC_VALUE_ID:

    Serial.print("-----------RecieveADC_VALUE_ID------------");
    Serial.println();
    ADC_FROM_CAN_1 = convertTo12Bit(values[0]);
    ADC_FROM_CAN_2 = convertTo12Bit(values[1]);
    ADC_FROM_CAN_3 = convertTo12Bit(values[2]);
    break;

  case COMMAND_RESPOND_ID:
    break;
  default:
    Serial.println("Neznama zprava");
    break;
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(SOL1, OUTPUT);
  pinMode(SOL2, OUTPUT);
  pinMode(PWM_VENT_1, OUTPUT);
  pinMode(PWM_VENT_2, OUTPUT);
  pinMode(PWM_VENT_3, OUTPUT);

  digitalWrite(SOL1, HIGH);
  digitalWrite(SOL2, HIGH);
  digitalWrite(PWM_VENT_1, HIGH);
  digitalWrite(PWM_VENT_2, HIGH);
  digitalWrite(PWM_VENT_3, HIGH);

  analogReadResolution(12);
  pinMode(ADC_1, INPUT);
  pinMode(ADC_2, INPUT);
  pinMode(ADC_3, INPUT);
  pinMode(ADC_4, INPUT);
  pinMode(ADC_5, INPUT);
  pinMode(ADC_6, INPUT);

  ledcSetup(VENT_1_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(PWM_VENT_1, VENT_1_CHANNEL);

  ledcSetup(VENT_2_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(PWM_VENT_2, VENT_2_CHANNEL);

  ledcSetup(VENT_3_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(PWM_VENT_3, VENT_3_CHANNEL);

  ledcWrite(VENT_1_CHANNEL, PWM_MAX);
  ledcWrite(VENT_2_CHANNEL, PWM_MAX);
  ledcWrite(VENT_3_CHANNEL, PWM_MAX);

  pidVent1.SetMode(AUTOMATIC);
  pidVent1.SetOutputLimits(0, PWM_MAX);

  pidVent2.SetMode(AUTOMATIC);
  pidVent2.SetOutputLimits(0, PWM_MAX);

  pidVent3.SetMode(AUTOMATIC);
  pidVent3.SetOutputLimits(0, PWM_MAX);

  setpoint1 = PWM_MAX;
  setpoint2 = PWM_MAX;
  setpoint3 = PWM_MAX;

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
  twai_clear_receive_queue();
}

void readADC()
{
  ADC_FROM_ADC_1 = analogRead(ADC_1);
  ADC_FROM_ADC_2 = analogRead(ADC_2);
  ADC_FROM_ADC_3 = analogRead(ADC_3);
  ADC_FROM_ADC_4 = analogRead(ADC_4);
  ADC_FROM_ADC_5 = analogRead(ADC_5);
  ADC_FROM_ADC_6 = analogRead(ADC_6);
}

void loop()
{
  readADC();
  recieveMessage();
}