#include <Wire.h>
#include <Arduino.h>
#include <math.h>
#include "driver/twai.h"
#include "driver/mcpwm.h"
#include "nvs.h"

int pwmFreq = 1000;                // Frekvence PWM v Hz
unsigned long lastMessageTime = 0; // Čas poslední zprávy
bool isMoving = false;             // Příznak pohybu motoru
bool wasMoving = false;
int DELAY_MOVE = 150;

unsigned long startTime;
float accelerationTime = 1000; // Doba (ms) pro dosažení maximální rychlosti.
int currentSpeed = 0;          // aktuální rychlost motoru
int accelerationRate = 200;    // rychlost akcelerace
int setDirection = 0;          // nastavený směr

#define CAN_RX_PIN 44
#define CAN_TX_PIN 43

int OXI_HEAD_ID = 0x00; // OXI_HEAD_ID = is loaded from NVS or recieved by command message
#define DEFAULT_ID 0x00
int OXI_HEAD_CAN_ID = 0x600;

#define NODE_TYPE 0x21
#define FIRMWARE_VERSION "1.0"
int major, minor;

int setIDstatus = 0;
nvs_handle my_handle;
const char *KEY_STORAGE = "oxy_head_id";

const int stepPin = 21;   // PUL/STEP pin na ESP32S3
const int dirPin = 48;    // DIR pin na ESP32S3
const int enablePin = 45; // ENABLE pin na ESP32S3

unsigned long previousMillis = 0;
int stepsCompleted = 0;
int setSpeed = 0;
int targetSteps = 0;

int currentDirection = 0;
int DEFAULT_STEP = 10;

bool UP_direction = false;
bool Down_direction = false;

enum OXI_HEAD_IDs
{
  OXI_HEAD_ID_1 = 0x280,
  OXI_HEAD_ID_2 = 0x290,
  OXI_HEAD_ID_3 = 0x2A0,
  OXI_HEAD_ID_4 = 0x2B0,
  OXI_HEAD_ID_5 = 0x2C0,
  OXI_HEAD_ID_6 = 0x2D0,
};

enum GPIOToRead
{
  BOTTOM_LIMIT = 10,
  SIDE_LIMIT = 11,
  UP_LIMIT = 9,
  SERVO_LIMIT = 12,
};

enum GPIOToSend
{
  OUT_ENABLE = 13,
  OUT_IGNITION = 14,
};

enum MessageIDToRecieve
{
  OXI_TECHNOLOGY_ID = 0x286,
  COMMAND_ID = 0x500,
};

enum MessageIDToSend
{
  SEDN_COMMAND_RESPOND_ID = 0x400,
  UNKNOUWN_ID = 0x600,
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

bool saveHeadId(int32_t headId)
{
  // Save the headId to NVS
  esp_err_t err = nvs_set_i32(my_handle, KEY_STORAGE, headId);

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

void setCanID(int32_t headId)
{
  if (headId == 0x00)
  {
    OXI_HEAD_CAN_ID = UNKNOUWN_ID;
  }
  else if (headId == 0x01)
  {
    OXI_HEAD_CAN_ID = OXI_HEAD_ID_1;
  }
  else if (headId == 0x02)
  {
    OXI_HEAD_CAN_ID = OXI_HEAD_ID_2;
  }
  else if (headId == 0x03)
  {
    OXI_HEAD_CAN_ID = OXI_HEAD_ID_3;
  }
  else if (headId == 0x04)
  {
    OXI_HEAD_CAN_ID = OXI_HEAD_ID_4;
  }
  else if (headId == 0x05)
  {
    OXI_HEAD_CAN_ID = OXI_HEAD_ID_5;
  }
  else if (headId == 0x06)
  {
    OXI_HEAD_CAN_ID = OXI_HEAD_ID_6;
  }
  else
  {
    OXI_HEAD_CAN_ID = UNKNOUWN_ID;
  }
}

MessageCommon *createMessage_COMMAND_RESPOND_SET_ID()
{
  static MessageCommon *message = nullptr;
  if (message == nullptr)
  {
    message = new MessageCommon;
  }
  message->byte0 = 0;
  message->byte1 = 0;
  message->byte2 = 0;
  message->byte3 = setIDstatus;
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

void changeHeadID(int32_t headId, MessageCommon *message)
{
  setIDstatus = saveHeadId(headId);
  if (setIDstatus)
  {
    Serial.println("************SET node ID*************************");
    Serial.println();
    Serial.println(headId);
    Serial.println();
    Serial.println("***************Successful**********************");
    OXI_HEAD_ID = headId;
    setCanID(OXI_HEAD_ID);
  }
  else
  {
    Serial.println("************SET node ID*************************");
    Serial.println();
    Serial.println("Error");
    Serial.println();
    Serial.println("**********************************************");
  }
  sendCANMessage(SEDN_COMMAND_RESPOND_ID, createMessage_COMMAND_RESPOND_SET_ID(), 8);
}

void resetHeadID(MessageCommon *message)
{
  setIDstatus = saveHeadId(DEFAULT_ID);
  if (setIDstatus)
  {
    Serial.println("************RESET node ID*************************");
    Serial.println();
    Serial.println(DEFAULT_ID);
    Serial.println();
    Serial.println("***************Successful**********************");
    OXI_HEAD_ID = DEFAULT_ID;
    setCanID(OXI_HEAD_ID);
  }
  else
  {
    Serial.println("************RESET node ID*************************");
    Serial.println();
    Serial.println("Error");
    Serial.println();
    Serial.println("**********************************************");
  }
  sendCANMessage(SEDN_COMMAND_RESPOND_ID, createMessage_COMMAND_RESPOND_SET_ID(), 8);
}

void stepperMovementMove(int direction, int speed)
{
  //
  digitalWrite(dirPin, direction);                         // Nastavení směru
  mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0, speed); // Nastavení duty cycle pro kanál A v procentech
  if (!wasMoving)
  {
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50.0);
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
    wasMoving = 1;
  }
  // Zapnutí PWM
}

void stepperMovementStop()
{
  mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
  digitalWrite(enablePin, HIGH); // Deaktivace pinu Enable (pokud je aktivace LOW, změňte na HIGH)
  wasMoving = 0;
}

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

MessageCommon *createMessage_OXI_HEAD()
{
  static MessageCommon *message = nullptr;
  if (message == nullptr)
  {
    message = new MessageCommon;
  }
  Serial.println("-------------Read Inputs------------:");
  Serial.println(readGPIO(BOTTOM_LIMIT));
  Serial.println(readGPIO(SIDE_LIMIT));
  Serial.println(readGPIO(UP_LIMIT));
  Serial.println(readGPIO(SERVO_LIMIT));

  message->byte0 = (readGPIO(UP_LIMIT) << 0) | (readGPIO(SIDE_LIMIT) << 1) | (readGPIO(UP_LIMIT) << 2) | (readGPIO(SERVO_LIMIT) << 3);
  message->byte1 = 0;
  message->byte2 = 0;
  message->byte3 = 0;
  message->byte4 = 0;
  message->byte5 = 0;
  message->byte6 = 0;
  message->byte7 = 0;
  return message;
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

bool convertDirection(bool UP_direction, bool Down_direction)
{
  if (UP_direction && !Down_direction)
  {
    return 0; // Směr nahoru
  }
  else if (!UP_direction && Down_direction)
  {
    return 1; // Směr dolů
  }
  else
  {
    return 0; // Směr nahoru
  }
}

void splitVersion(const char *version, int &major, int &minor)
{
  if (sscanf(version, "%d.%d", &major, &minor) != 2)
  {
    Serial.println("Chyba: Neplatný formát verze.");
  }
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
  if (command == 0x02)
  {
    changeHeadID(data[3], createMessage_COMMAND_RESPOND_SET_ID());
  }
  if (command == 0x03)
  {
    Serial.println("************RESET node ID**********************");
    Serial.println();
  }
}

void handleCANMessage(int id, twai_message_t message, int nodeID = OXI_HEAD_ID)
{
  uint8_t *data = message.data;

  switch (id)
  {
  case OXI_TECHNOLOGY_ID:
    setGPIO(OUT_IGNITION, (data[0] >> nodeID) & 0x01);
    setGPIO(OUT_ENABLE, (data[1] >> nodeID) & 0x01);
    UP_direction = (data[3] >> nodeID) & 0x01;
    Down_direction = (data[4] >> nodeID) & 0x01;
    if ((UP_direction && !Down_direction) || (!UP_direction && Down_direction)) // Pokud je směr nahoru nebo dolů
    {
      Serial.println("************************Smer detekovan**********************");
      Serial.println((UP_direction && !Down_direction) || (!UP_direction && Down_direction));
      Serial.println("**********************************************");

      setDirection = convertDirection(UP_direction, Down_direction);

      setSpeed = map(data[5], 0, 255, 200, 8000);

      lastMessageTime = millis(); // Aktualizujeme čas poslední zprávy
      digitalWrite(enablePin, LOW);
      isMoving = true; // Nastavíme příznak pohybu na true
    }
    else
    {
      isMoving = false; // Nastavíme příznak pohybu na false
    }
    sendCANMessage(OXI_HEAD_CAN_ID, createMessage_OXI_HEAD(), 8);
    break;

  case COMMAND_ID:
    Serial.print("-----------RecieveCOMMAND_ID------------");
    Serial.println();
    Serial.print("Command");
    Serial.println(data[2]);
    Serial.println();
    handleCommand(data);
    break;
  default:
    break;
  }
}

void setup()
{
  Serial.begin(115200);

  if (nvs_open("storage", NVS_READWRITE, &my_handle) != ESP_OK) // Open NVS
  {
    Serial.println("Error opening NVS");
  }
  esp_err_t err = nvs_get_i32(my_handle, KEY_STORAGE, &OXI_HEAD_ID); // Get the headId from NVS (if it exists) and store it in the variable
  OXI_HEAD_ID = (err == ESP_OK) ? OXI_HEAD_ID : DEFAULT_ID;          // If there is no headId in NVS, use the default one
  setCanID(OXI_HEAD_ID);                                             // Set the CAN ID based on the headId

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enablePin, OUTPUT);

  pinMode(BOTTOM_LIMIT, INPUT);
  pinMode(SIDE_LIMIT, INPUT);
  pinMode(UP_LIMIT, INPUT);
  pinMode(SERVO_LIMIT, INPUT);

  pinMode(OUT_ENABLE, OUTPUT);
  pinMode(OUT_IGNITION, OUTPUT);

  // Deaktivovat pin Enable (pokud je aktivace LOW, změňte na HIGH)
  digitalWrite(enablePin, HIGH);

  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, stepPin);

  mcpwm_config_t pwm_config;
  pwm_config.frequency = pwmFreq; // Frekvence v Hz
  pwm_config.cmpr_a = 50.0;       // Duty cycle pro kanál A v procentech
  pwm_config.cmpr_b = 0.0;        // Duty cycle pro kanál B v procentech
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

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

/*
void stepperMotorNonBlockingControl()
{ Serial.println("*******************Enter Stepper********************");
  Serial.println(isMoving);
  Serial.println(isMoving && (millis() - lastMessageTime < DELAY_MOVE));
  if (isMoving && (millis() - lastMessageTime < DELAY_MOVE))
  { Serial.println("*******************Moving True Time true********************");
   if (currentSpeed < setSpeed)
    {                                   // pokud je aktuální rychlost menší než cílová
      currentSpeed += accelerationRate; // zvyšujeme rychlost
      if (currentSpeed > setSpeed)
        currentSpeed = setSpeed; // omezení maximální rychlosti
    }
    else if (currentSpeed > setSpeed)
    {                                   // pokud je aktuální rychlost větší než cílová
      currentSpeed -= accelerationRate; // zpomalujeme
      if (currentSpeed < setSpeed)
        currentSpeed = setSpeed; // omezení minimální rychlosti
    }
    else if (currentSpeed == setSpeed) // pokud je aktuální rychlost rovna cílové
    { Serial.println("*********************Reached Speed********************");
      currentSpeed = setSpeed;

    }

    stepperMovementMove(setDirection, currentSpeed); // Provedeme pohyb motoru
  }
  else
  {
    isMoving = false; // Nastavíme příznak pohybu na false
    stepperMovementStop();
    currentSpeed = 0;
  }
}
*/
void stepperMotorNonBlockingControl()
{
  Serial.println("*******************Enter Stepper********************");
  Serial.println(isMoving);
  Serial.println(isMoving && (millis() - lastMessageTime < DELAY_MOVE));

  if (isMoving && (millis() - lastMessageTime < DELAY_MOVE))
  {
    Serial.println("*******************Moving True Time true********************");

    float timeElapsed = millis() - startTime;
    float accelerationFactor = sin((M_PI / 2) * (timeElapsed / accelerationTime)); // Sinusová funkce

    if (timeElapsed >= accelerationTime)
    {
      accelerationFactor = 1; // Omezení na 1, aby jsme nepřekročili cílovou rychlost
    }

    currentSpeed = accelerationFactor * setSpeed;

    if (currentSpeed == setSpeed)
    {
      Serial.println("*********************Reached Speed********************");
    }

    stepperMovementMove(setDirection, currentSpeed); // Provedeme pohyb motoru
  }
  else
  {
    isMoving = false;
    startTime = millis(); // Resetujte čas, když začne nový pohyb
    stepperMovementStop();
    currentSpeed = 0;
  }
}

void loop()
{
  recieveMessage();
  stepperMotorNonBlockingControl();
}