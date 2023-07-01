#include <Wire.h>
#include <Arduino.h>
#include "driver/twai.h"

#include "driver/mcpwm.h"

#define PWM_FREQUENCY 11000 // Frekvence PWM v Hz

#define CAN_RX_PIN 44
#define CAN_TX_PIN 43

const int stepPin = 21;   // PUL/STEP pin na ESP32S3
const int dirPin = 48;    // DIR pin na ESP32S3
const int enablePin = 45; // ENABLE pin na ESP32S3

const int minStepInterval = 2;  // Minimální prodleva mezi kroky v ms (rychlost)
const int maxStepInterval = 5;  // Maximální prodleva mezi kroky v ms
const int accelerationRate = 1; // Rychlost zrychlení/zpomalení v ms

unsigned long previousMillis = 0;
int stepsCompleted = 0;
int stepInterval = maxStepInterval;
int targetSteps = 0;
bool direction = HIGH;

bool setDirection = false;
int DEFAULT_STEP = 10;

bool UP_direction = false;
bool Down_direction = false;

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
  MESSAGE3_ID = 0x500,
};

enum MessageIDToSend
{
  OXI_HEAD_1_ID = 0x280,
  // OXI_HEAD_2_ID = 0x290,
  COMMAND_RESPOND_ID = 0x400,
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

void startStepperMovement(bool dir, int numSteps)
{
  stepsCompleted = 0;
  direction = dir;
  targetSteps = numSteps;
}

/*void moveStepperNonBlocking()
{
  unsigned long currentMillis = milis();

  if (currentMillis - previousMillis >= stepInterval)
  {
    previousMillis = currentMillis;

    // Aktivovat pin Enable (pokud je aktivace LOW, změňte na LOW)
    digitalWrite(enablePin, LOW);

    if (stepsCompleted < targetSteps)
    {
      digitalWrite(dirPin, direction);
      digitalWrite(stepPin, !digitalRead(stepPin));

      if (digitalRead(stepPin) == LOW)
      {
        stepsCompleted++;
      }
    }
  }
}*/

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
    Serial.println("Směr je neplatný");
    return 0; // Návratová hodnota pro neplatný vstup nebo chybu
  }
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
  case OXI_TECHNOLOGY_ID:
    setGPIO(OUT_IGNITION, (data[0] >> 0) & 0x01);
    setGPIO(OUT_ENABLE, (data[1] >> 0) & 0x01);
    UP_direction = (data[3] >> 0) & 0x01;
    Down_direction = (data[4] >> 0) & 0x01;
    setDirection = convertDirection(UP_direction, Down_direction);
    stepInterval = map(data[5], 0, 255, 1000, 1);
    startStepperMovement(setDirection, DEFAULT_STEP);
    sendCANMessage(OXI_HEAD_1_ID, createMessageWithAllOnes(), 8);
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
  digitalWrite(enablePin, LOW);

   mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, stepPin);
  
    mcpwm_config_t pwm_config;
    pwm_config.frequency = PWM_FREQUENCY; // Frekvence v Hz
    pwm_config.cmpr_a = 50.0; // Duty cycle pro kanál A v procentech
    pwm_config.cmpr_b = 0.0; // Duty cycle pro kanál B (nevyužíváme)
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
void test()
{ 
  UP_direction = 0x00;
  Down_direction = 0x01;
  setDirection = convertDirection(UP_direction, Down_direction);
  /*stepInterval = map(255, 0, 255, 5000, 1);*/
  stepInterval = 1;
  startStepperMovement(setDirection, 50);
}

void loop()
{ 
   mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50.0); // Nastavíme duty cycle na 75%
   digitalWrite(dirPin, 1);
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0); // Zapnutí PWM
    delay(20);
    mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0); // Vypnutí PWM
    delay(20);
  recieveMessage();
  //moveStepperNonBlocking();
}