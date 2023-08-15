#include <Wire.h>
#include <Arduino.h>
#include "driver/twai.h"
#include "nvs.h"

#define CAN_RX_PIN 44
#define CAN_TX_PIN 43

#define ADS1110_ADDRESS 0x48 // Předpokládaná adresa ADS1110, upravte podle potřeby
#define SDA_PIN 4
#define SCL_PIN 5

#define RS422_RX_PIN 18
#define RS422_TX_PIN 17

#define NODE_TYPE 0x01
#define FIRMWARE_VERSION "1.0"

#define DEFAULT_BAUDRATE 9600

int32_t RS485_baudrate = 0;
int RS485_baudrate_changed_status = 0;
nvs_handle my_handle;
const char *KEY_STORAGE = "baudrate";

int major, minor;

static uint8_t buffer[8] = {0};
static uint8_t bufferIndex = 0;

HardwareSerial RS422(2);
enum MessageIDToRecieve
{
  PLASMA_REMOTE_POWER_ID = 0x204,
  P_M_P_CC_ID = 0x206,
  SERIAL_TX_ID = 0x208,
  COMMAND_ID = 0x500,
};

enum GPIOToSend
{
  ARC_ON = 8,
  ERROR = 10,
  TOUCH_PLATE = 12,
  READY = 9,
  AUTO_PIERCE_DEFECT = 11,
};

enum GPIOToRead
{
  PLASMA_START = 21,
  MARKING = 14, // 48
  PIERCING = 45,
  CORNER_CURRENT = 38,
  PLASMA_REMOTE_POWER = 48, // 14
};

enum MessageIDToSend
{
  AETRA_ARC_ID = 0x200,
  SERIAL_RX_ID = 0x203,
  SEDN_COMMAND_RESPOND_ID = 0x400,
};

enum MessagePeriod
{
  AETRA_ARC_PERIOD = 20,
  SERIAL_RX_ID_PERIOD = 50,
  COMMAND_RESPOND_ID_PERIOD = 20,
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

void splitVersion(const char *version, int &major, int &minor)
{
  if (sscanf(version, "%d.%d", &major, &minor) != 2)
  {
    Serial.println("Chyba: Neplatný formát verze.");
  }
}

bool saveBaudrate(int32_t baudrate)
{
  // Save the baudrate to NVS
  esp_err_t err = nvs_set_i32(my_handle, KEY_STORAGE, baudrate);

  if (err == ESP_OK)
  {
    // Commit the changes
    err = nvs_commit(my_handle);

    if (err == ESP_OK)
    {
      // Successfully saved and committed
      return true;
    }
  }

  // Failed to save or commit
  return false;
}

int16_t readPlasmaADS()
{
  Wire.requestFrom(ADS1110_ADDRESS, 2);

  if (Wire.available() == 2)
  {
    int16_t value = (Wire.read() << 8) | Wire.read();
    return value;
  }

  return 0; // Vracíme nulu v případě, že se nepodařilo získat data z ADS1110
}

int readGPIO(int gpioPin)
{
  return digitalRead(gpioPin);
}

void setGPIO(int GPIO, bool state)
{
  digitalWrite(GPIO, state ? HIGH : LOW);
}

float convertToVoltage(int16_t adcValue)
{
  // Upravte rozsah napětí a referenci podle vaší konfigurace a zapojení ADS1110
  float voltageReference = 2.048;                  // Např. 2.048V referenční napětí
  float voltageRange = voltageReference / 32768.0; // 16-bit ADC
  return adcValue * voltageRange;
}

MessageCommon *createMessage_AETRA_ARC()
{
  int16_t ads_value = readPlasmaADS();

  float volatege = convertToVoltage(ads_value);
  Serial.println("Voltage:!");
  Serial.println(volatege);
  Serial.println("_");

  Serial.println("adc:");
  Serial.println(ads_value);
  Serial.println("_");

  if (ads_value < 0)
  {
    ads_value = 0;
  }

  static MessageCommon *message = nullptr;
  if (message == nullptr)
  {
    message = new MessageCommon;
  }
  Serial.println("ReadINPUTS:");
  Serial.println(readGPIO(ARC_ON));
  Serial.println(readGPIO(ERROR));
  Serial.println(readGPIO(TOUCH_PLATE));
  Serial.println(readGPIO(READY));
  Serial.println(readGPIO(AUTO_PIERCE_DEFECT));

  message->byte0 = (readGPIO(ARC_ON) << 0) | (readGPIO(ERROR) << 1) | (readGPIO(TOUCH_PLATE) << 2) | (readGPIO(READY) << 3) | (readGPIO(AUTO_PIERCE_DEFECT) << 4);
  message->byte1 = 0;
  message->byte2 = 0;
  message->byte3 = 0;
  message->byte4 = 0;
  message->byte5 = 0;
  message->byte6 = (ads_value >> 8) & 0xFF;
  message->byte7 = ads_value & 0xFF;
  return message;
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

void sendCANMessage(uint32_t messageID, void *data, uint8_t dataSize)
{
  // Připravení zprávy
  twai_message_t message;
  message.identifier = messageID;
  message.extd = 0;
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
  message.extd = 0;
  message.rtr = 0;

  twai_transmit(&message, pdMS_TO_TICKS(10));
}

void sendMessageSerialRX()
{
  static unsigned long last_time = millis();

  while (RS422.available() > 0)
  {
    buffer[bufferIndex++] = RS422.read();

    if (bufferIndex >= 8)
    {
      break;
    }
  }

  unsigned long current_time = millis();
  unsigned long elapsed = current_time - last_time;

  if ((bufferIndex > 0 && elapsed >= 50) || bufferIndex >= 8)
  {
    static MessageCommon *message = nullptr;
    if (message == nullptr)
    {
      message = new MessageCommon;
    }
    message->byte0 = buffer[0];
    message->byte1 = buffer[1];
    message->byte2 = buffer[2];
    message->byte3 = buffer[3];
    message->byte4 = buffer[4];
    message->byte5 = buffer[5];
    message->byte6 = buffer[6];
    message->byte7 = buffer[7];

    // Reset buffer index, timer, and clear the buffer
    bufferIndex = 0;
    last_time = current_time;
    memset(buffer, 0, sizeof(buffer));

    sendCANMessage(SERIAL_RX_ID, message, 8);
  }
}

MessageCommon *createMessage_COMMAND_RESPOND_FIRMWARE_VERSION()
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

MessageCommon *createMessage_COMMAND_RESPOND_SET_RS_SPEED()
{
  static MessageCommon *message = nullptr;
  if (message == nullptr)
  {
    message = new MessageCommon;
  }
  message->byte0 = 0;
  message->byte1 = 0;
  message->byte2 = 0;
  message->byte3 = RS485_baudrate_changed_status;
  message->byte4 = 0;
  message->byte5 = 0;
  message->byte6 = 0;
  message->byte7 = 0;
  return message;
}
void changeBaudrate(int baudrate, MessageCommon *message)
{
  RS485_baudrate_changed_status = saveBaudrate(baudrate);
  Serial.println(baudrate);
  RS422.updateBaudRate(baudrate);
  sendCANMessage(SEDN_COMMAND_RESPOND_ID, message, 8);
}

void handleCommand(uint8_t *data)
{
  int command = data[2];

  if (command == 0x01)
  {
    Serial.println("Get version");
    Serial.println();
    splitVersion(FIRMWARE_VERSION, major, minor);
    sendCANMessage(SEDN_COMMAND_RESPOND_ID, createMessage_COMMAND_RESPOND_FIRMWARE_VERSION(), 8);
  }
  if (command == 0x04)
  {
    int baudrate_code = data[3];
    if (baudrate_code == 0x01)
    {
      Serial.println("Set RS485 baudrate");
      RS485_baudrate = 19200;
      changeBaudrate(RS485_baudrate, createMessage_COMMAND_RESPOND_SET_RS_SPEED());
    }
    if (baudrate_code == 0x02)
    {
      Serial.println("Set RS485 baudrate");
      RS485_baudrate = 115200;
      changeBaudrate(RS485_baudrate, createMessage_COMMAND_RESPOND_SET_RS_SPEED());
    }
  }
}

void handleCANMessage(int id, twai_message_t message)
{
  Serial.print("Message with ID 0x");
  Serial.print(id, HEX);
  Serial.print(" received, data: ");
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

  uint8_t *data = message.data;
  Serial.print(*data);
  switch (id)
  {
  case PLASMA_REMOTE_POWER_ID:
    setGPIO(PLASMA_REMOTE_POWER, (data[7] >> 3) & 0x01);
    break;

  case P_M_P_CC_ID:
    setGPIO(PLASMA_START, (data[0] >> 2) & 0x01);   // získat logický stav 2. bitu
    setGPIO(MARKING, (data[0] >> 3) & 0x01);        // získat logický stav 3. bitu
    setGPIO(PIERCING, (data[0] >> 4) & 0x01);       // získat logický stav 4. bitu
    setGPIO(CORNER_CURRENT, (data[0] >> 5) & 0x01); // získat logický stav 5. bitu
    sendCANMessage(AETRA_ARC_ID, createMessage_AETRA_ARC(), 8);
    break;

  case SERIAL_TX_ID:
    RS422.write(data, 8);
    Serial.println("TX-DATA-=============");
    for (int i = 0; i < 8; ++i)
    {
      Serial.print(data[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    break;
  case COMMAND_ID:
    Serial.print("-----------RecieveCOMMAND_ID------------");
    Serial.println();
    Serial.print("Command");
    Serial.println(data[2]);
    Serial.println();
    handleCommand(data);
    sendCANMessage(SEDN_COMMAND_RESPOND_ID, createMessage_COMMAND_RESPOND_FIRMWARE_VERSION(), 8);
    break;
  default:
    Serial.println("Neznama zprava");
    break;
  }
}

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.print("READY-Lisening");
  Wire.begin(SDA_PIN, SCL_PIN);

  if (nvs_open("storage", NVS_READWRITE, &my_handle) != ESP_OK)
  {
    Serial.println("Error opening NVS");
  }
  esp_err_t err = nvs_get_i32(my_handle, KEY_STORAGE, &RS485_baudrate);
  RS485_baudrate = (err == ESP_OK) ? RS485_baudrate : DEFAULT_BAUDRATE;
  RS422.begin(RS485_baudrate, SERIAL_8N1, RS422_RX_PIN, RS422_TX_PIN);

  pinMode(PLASMA_START, OUTPUT);
  pinMode(MARKING, OUTPUT);
  pinMode(PIERCING, OUTPUT);
  pinMode(CORNER_CURRENT, OUTPUT);
  pinMode(PLASMA_REMOTE_POWER, OUTPUT);

  pinMode(ARC_ON, INPUT);
  pinMode(ERROR, INPUT);
  pinMode(TOUCH_PLATE, INPUT);
  pinMode(READY, INPUT);
  pinMode(AUTO_PIERCE_DEFECT, INPUT);

  // Konfigurace ADS1110 (upravte podle potřeby)
  byte configByte = 0b10001100; // 16-bit, gain 1, continuous conversion
  Wire.beginTransmission(ADS1110_ADDRESS);
  Wire.write(configByte);
  Wire.endTransmission();

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NO_ACK); // TWAI_MODE_NORMAL, TWAI_MODE_NO_ACK or TWAI_MODE_LISTEN_ONLY
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_driver_install(&g_config, &t_config, &f_config);
  twai_start();
}

void printReceivedMessage(const twai_message_t &message)
{
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
    // printReceivedMessage(message);
    handleCANMessage(message.identifier, message);
  }
  twai_clear_receive_queue();
}

void loop()
{
  recieveMessage();
  sendMessageSerialRX();
  recieveMessage();
}