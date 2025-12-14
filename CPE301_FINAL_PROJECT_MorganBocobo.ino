//Morgan Bocobo
//CPE 301 Final Project

#include <LiquidCrystal.h>
#include <DHT.h>
#include <Stepper.h>

#define LCD_RS 12
#define LCD_E 11
#define LCD_D4 5
#define LCD_D5 4
#define LCD_D6 3
#define LCD_D7 2
#define DHT_PIN 7
#define DHT_TYPE DHT11
#define WATER_CHANNEL 0
#define LED_YELLOW 37
#define LED_GREEN 36
#define LED_BLUE 35
#define LED_RED 34
#define BTN_START 18
#define BTN_STOP 48
#define BTN_RESET 47
#define BTN_VENT_LEFT 46
#define BTN_VENT_RIGHT 45
#define FAN_PIN 10
#define STEPPER_IN1 22
#define STEPPER_IN2 23
#define STEPPER_IN3 24
#define STEPPER_IN4 25
#define RDA 0x80
#define TBE 0x20
#define TEMP_THRESHOLD 75.0
#define WATER_THRESHOLD 100
#define STEPS_PER_REV 2048
#define VENT_STEP_SIZE 512

volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int *myUBRR0 = (unsigned int *)0x00C4;
volatile unsigned char *myUDR0 = (unsigned char *)0x00C6;
volatile unsigned char *my_ADMUX = (unsigned char *)0x7C;
volatile unsigned char *my_ADCSRB = (unsigned char *)0x7B;
volatile unsigned char *my_ADCSRA = (unsigned char *)0x7A;
volatile unsigned int *my_ADC_DATA = (unsigned int *)0x78;
volatile unsigned char *port_c = (unsigned char *)0x28;
volatile unsigned char *ddr_c = (unsigned char *)0x27;
volatile unsigned char *pin_c = (unsigned char *)0x26;
volatile unsigned char *port_l = (unsigned char *)0x10B;
volatile unsigned char *ddr_l = (unsigned char *)0x10A;
volatile unsigned char *pin_l = (unsigned char *)0x109;
volatile unsigned char *port_d = (unsigned char *)0x2B;
volatile unsigned char *ddr_d = (unsigned char *)0x2A;
volatile unsigned char *pin_d = (unsigned char *)0x29;
volatile unsigned char *port_b = (unsigned char *)0x25;
volatile unsigned char *ddr_b = (unsigned char *)0x24;

enum SystemState { DISABLED, IDLE, RUNNING, ERROR_STATE };

SystemState currentState = DISABLED;
SystemState previousState = DISABLED;

LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
DHT dht(DHT_PIN, DHT_TYPE);
Stepper stepper(STEPS_PER_REV, STEPPER_IN1, STEPPER_IN3, STEPPER_IN2, STEPPER_IN4);

unsigned long systemStartTime = 0, lastTempCheck = 0, lastWaterCheck = 0;
unsigned long lastLCDUpdate = 0, lastSerialReport = 0;
float temperature = 72.0, humidity = 50.0;
unsigned int waterLevel = 0;
int ventPosition = 0;
volatile bool startButtonPressed = false;
bool systemInitialized = false;

void U0init(int U0baud) {
  unsigned int tbaud = (16000000 / 16 / U0baud - 1);
  *myUCSR0A = 0x20;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
  *myUBRR0 = tbaud;
}

void U0putchar(unsigned char U0pdata) {
  while ((*myUCSR0A & TBE) == 0);
  *myUDR0 = U0pdata;
}

void U0print(const char* str) {
  int i = 0;
  while (str[i] != '\0') U0putchar(str[i++]);
}

void U0println(const char* str) {
  U0print(str);
  U0putchar('\r');
  U0putchar('\n');
}

void printNumber(int num) {
  if (num < 0) { U0putchar('-'); num = -num; }
  if (num == 0) { U0putchar('0'); return; }
  char buffer[10];
  int i = 0;
  while (num > 0) { buffer[i++] = (num % 10) + '0'; num /= 10; }
  for (int j = i - 1; j >= 0; j--) U0putchar(buffer[j]);
}

void printFloat(float num, int decimals) {
  int intPart = (int)num;
  printNumber(intPart);
  if (decimals > 0) {
    U0putchar('.');
    float fracPart = num - intPart;
    for (int i = 0; i < decimals; i++) {
      fracPart *= 10;
      int digit = (int)fracPart;
      U0putchar(digit + '0');
      fracPart -= digit;
    }
  }
}

void printTimestamp() {
  unsigned long totalSeconds = (millis() - systemStartTime) / 1000;
  unsigned int hours = totalSeconds / 3600;
  unsigned int minutes = (totalSeconds % 3600) / 60;
  unsigned int seconds = totalSeconds % 60;
  U0print("[");
  if (hours < 10) U0putchar('0');
  printNumber(hours);
  U0putchar(':');
  if (minutes < 10) U0putchar('0');
  printNumber(minutes);
  U0putchar(':');
  if (seconds < 10) U0putchar('0');
  printNumber(seconds);
  U0print("] ");
}

void adc_init() {
  *my_ADCSRA |= 0x87;
  *my_ADCSRA &= ~0x28;
  *my_ADCSRB &= 0xF7;
  *my_ADCSRB &= 0xF8;
  *my_ADMUX &= ~0x80;
  *my_ADMUX |= 0x40;
  *my_ADMUX &= 0xDF;
  *my_ADMUX &= 0xE0;
}

unsigned int adc_read(unsigned char ch) {
  *my_ADMUX &= 0xE0;
  *my_ADCSRB &= ~0x08;
  if (ch > 7) { ch -= 8; *my_ADCSRB |= 0x08; }
  *my_ADMUX |= ch;
  *my_ADCSRA |= 0x40;
  while ((*my_ADCSRA & 0x40) != 0);
  return *my_ADC_DATA;
}

void gpio_init() {
  *ddr_c |= 0x0F;
  *port_c &= ~0x0F;
  *ddr_l &= ~0x1E;
  *port_l |= 0x1E;
  *ddr_d &= ~0x08;
  *port_d |= 0x08;
  *ddr_b |= 0x10;
  *port_b &= ~0x10;
}

void setLED(unsigned char pin, bool state) {
  unsigned char bit;
  if (pin == 37) bit = 0;
  else if (pin == 36) bit = 1;
  else if (pin == 35) bit = 2;
  else if (pin == 34) bit = 3;
  else return;
  if (state) *port_c |= (1 << bit);
  else *port_c &= ~(1 << bit);
}

bool readButton(unsigned char pin) {
  unsigned char bit;
  if (pin == 48) bit = 1;
  else if (pin == 47) bit = 2;
  else if (pin == 46) bit = 3;
  else if (pin == 45) bit = 4;
  else return false;
  return ((*pin_l & (1 << bit)) == 0);
}

void setFan(bool state) {
  if (state) analogWrite(FAN_PIN, 255);
  else analogWrite(FAN_PIN, 0);
}

void startISR() {
  static unsigned long lastInterrupt = 0;
  if (millis() - lastInterrupt > 200) {
    startButtonPressed = true;
    lastInterrupt = millis();
  }
}

const char* stateToString(SystemState s) {
  if (s == DISABLED) return "DISABLED";
  else if (s == IDLE) return "IDLE";
  else if (s == RUNNING) return "RUNNING";
  else return "ERROR";
}

void changeState(SystemState newState) {
  if (newState == currentState) return;
  previousState = currentState;
  currentState = newState;
  setLED(LED_YELLOW, false);
  setLED(LED_GREEN, false);
  setLED(LED_BLUE, false);
  setLED(LED_RED, false);
  if (currentState == DISABLED) setLED(LED_YELLOW, true);
  else if (currentState == IDLE) setLED(LED_GREEN, true);
  else if (currentState == RUNNING) setLED(LED_BLUE, true);
  else if (currentState == ERROR_STATE) setLED(LED_RED, true);
  if (currentState == RUNNING) {
    setFan(true);
    printTimestamp();
    U0println("Fan motor: ON");
  } else {
    setFan(false);
    if (previousState == RUNNING) {
      printTimestamp();
      U0println("Fan motor: OFF");
    }
  }
  if (systemInitialized) {
    printTimestamp();
    U0print("State changed: ");
    U0print(stateToString(previousState));
    U0print(" -> ");
    U0println(stateToString(currentState));
  }
  lcd.clear();
  lastTempCheck = 0;
  lastWaterCheck = 0;
  lastSerialReport = 0;
}

void rotateVent(int steps) {
  stepper.step(steps);
  ventPosition += steps;
  if (ventPosition > STEPS_PER_REV) ventPosition = STEPS_PER_REV;
  if (ventPosition < -STEPS_PER_REV) ventPosition = -STEPS_PER_REV;
}

void updateDisplay() {
  lcd.setCursor(0, 0);
  if (currentState == DISABLED) {
    lcd.print("DISABLED        ");
    lcd.setCursor(0, 1);
    lcd.print("Press START     ");
  } else if (currentState == IDLE) {
    lcd.print("IDLE ");
    lcd.print((int)temperature);
    lcd.print("F ");
    lcd.print((int)humidity);
    lcd.print("%  ");
    lcd.setCursor(0, 1);
    lcd.print("Water: ");
    if (waterLevel >= WATER_THRESHOLD) {
      lcd.print("OK (");
      lcd.print(waterLevel);
      lcd.print(") ");
    } else lcd.print("LOW     ");
  } else if (currentState == RUNNING) {
    lcd.print("COOLING ");
    lcd.print((int)temperature);
    lcd.print("F  ");
    lcd.setCursor(0, 1);
    lcd.print("Hum:");
    lcd.print((int)humidity);
    lcd.print("% W:");
    lcd.print(waterLevel);
    lcd.print("  ");
  } else if (currentState == ERROR_STATE) {
    lcd.print("ERROR: NO WATER ");
    lcd.setCursor(0, 1);
    lcd.print("Refill & RESET  ");
  }
}

void handleButtons() {
  if (startButtonPressed) {
    startButtonPressed = false;
    if (currentState == DISABLED) {
      changeState(IDLE);
      printTimestamp();
      U0println("START button pressed - System enabled");
    }
  }
  static bool stopLast = false, resetLast = false, ventLLast = false, ventRLast = false;
  bool stopPressed = readButton(BTN_STOP);
  bool resetPressed = readButton(BTN_RESET);
  bool ventLPressed = readButton(BTN_VENT_LEFT);
  bool ventRPressed = readButton(BTN_VENT_RIGHT);
  if (stopPressed && !stopLast && currentState != DISABLED) {
    changeState(DISABLED);
    printTimestamp();
    U0println("STOP button pressed - System disabled");
    unsigned long d = millis(); while (millis() - d < 200);
  }
  if (resetPressed && !resetLast && currentState == ERROR_STATE) {
    changeState(IDLE);
    printTimestamp();
    U0println("RESET button pressed - Error cleared");
    unsigned long d = millis(); while (millis() - d < 200);
  }
  if (currentState != DISABLED) {
    if (ventLPressed && !ventLLast) {
      rotateVent(-VENT_STEP_SIZE);
      printTimestamp();
      U0println("Vent rotated LEFT");
      unsigned long d = millis(); while (millis() - d < 200);
    }
    if (ventRPressed && !ventRLast) {
      rotateVent(VENT_STEP_SIZE);
      printTimestamp();
      U0println("Vent rotated RIGHT");
      unsigned long d = millis(); while (millis() - d < 200);
    }
  }
  stopLast = stopPressed;
  resetLast = resetPressed;
  ventLLast = ventLPressed;
  ventRLast = ventRPressed;
}

void stateMachine() {
  if (currentState == IDLE) {
    if (waterLevel < WATER_THRESHOLD) changeState(ERROR_STATE);
    else if (temperature > TEMP_THRESHOLD) changeState(RUNNING);
  } else if (currentState == RUNNING) {
    if (waterLevel < WATER_THRESHOLD) changeState(ERROR_STATE);
    else if (temperature <= TEMP_THRESHOLD) changeState(IDLE);
  } else if (currentState == ERROR_STATE && waterLevel >= WATER_THRESHOLD) {
    changeState(IDLE);
    printTimestamp();
    U0println("Water refilled - Auto-recovery to IDLE");
  }
}

void setup() {
  U0init(9600);
  unsigned long w = millis(); while (millis() - w < 100);
  adc_init();
  gpio_init();
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Swamp Cooler");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  dht.begin();
  stepper.setSpeed(10);
  attachInterrupt(digitalPinToInterrupt(BTN_START), startISR, FALLING);
  changeState(DISABLED);
  w = millis(); while (millis() - w < 2000);
  lcd.clear();
  systemStartTime = millis();
  float tempC = dht.readTemperature();
  float hum = dht.readHumidity();
  if (!isnan(tempC) && !isnan(hum)) {
    temperature = (tempC * 9.0 / 5.0) + 32.0;
    humidity = hum;
  }
  waterLevel = adc_read(WATER_CHANNEL);
  systemInitialized = true;
}

void loop() {
  unsigned long currentTime = millis();
  if (currentState != DISABLED) {
    if (currentTime - lastTempCheck >= 1000 || lastTempCheck == 0) {
      float tempC = dht.readTemperature();
      float hum = dht.readHumidity();
      if (!isnan(tempC) && !isnan(hum)) {
        temperature = (tempC * 9.0 / 5.0) + 32.0;
        humidity = hum;
      }
      lastTempCheck = currentTime;
    }
    if (currentTime - lastWaterCheck >= 1000 || lastWaterCheck == 0) {
      waterLevel = adc_read(WATER_CHANNEL);
      lastWaterCheck = currentTime;
    }
    if (currentTime - lastSerialReport >= 60000 || lastSerialReport == 0) {
      printTimestamp();
      U0print("Temp: ");
      printFloat(temperature, 1);
      U0print("F | Humidity: ");
      printFloat(humidity, 1);
      U0print("% | Water Level: ");
      printNumber(waterLevel);
      U0println("");
      lastSerialReport = currentTime;
    }
  }
  if (currentTime - lastLCDUpdate >= 1000) {
    updateDisplay();
    lastLCDUpdate = currentTime;
  }
  handleButtons();
  stateMachine();
}
