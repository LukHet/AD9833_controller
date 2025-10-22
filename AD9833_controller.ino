#include <SPI.h>
#include <LiquidCrystal.h>

// Definicje bitów kontrolnych
#define CTRL_B28        (1 << 13)
#define CTRL_RESET      (1 << 8)
#define CTRL_FREQ_REGISTER (1 << 14)
#define CTRL_OPBITEN    (1 << 5)
#define CTRL_MODE       (1 << 1)
#define FSYNC           10
#define FREQ_FACTOR     268435456.0
#define OSC_FREQ        25000000.0
#define SINE            0x2000
#define SQUARE          0x2028
#define TRIANGLE        0x2002
#define PHASE_ZERO      0xC000
#define POT_INIT        0x11
#define MIN_VOLTAGE     0
#define MAX_VOLTAGE     16
#define ENCODER_PIN_1   1
#define ENCODER_PIN_2   2
#define RS              8
#define EN              9
#define D4              4
#define D5              5
#define D6              6
#define D7              7 
#define SWITCH_PIN      11
#define EN_POT          3

volatile int encoderDirection = 0;
int currentMode = 0;
float currentVoltage = 0;
uint16_t gen_freq = 1000;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50; 
int lastButtonState = HIGH;
int buttonState;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

//przesylam dane do potencjometru cyfrowego
void sendDigitalPotentiometerData(float voltage) {
  if (voltage > MAX_VOLTAGE) voltage = MAX_VOLTAGE;
  if (voltage < MIN_VOLTAGE) voltage = MIN_VOLTAGE;
  byte potValue = (voltage / MAX_VOLTAGE) * 255;
  SPI.setDataMode(SPI_MODE0);
  digitalWrite(EN_POT, LOW);
  SPI.transfer(POT_INIT);
  SPI.transfer(potValue);
  digitalWrite(EN_POT, HIGH);
  SPI.setDataMode(SPI_MODE2);
}

//ustawiam napiece
void setVoltage (float val) {
  sendDigitalPotentiometerData(val);  
  currentVoltage = val;
}

//przesylanie danych - ustawiam FSYNC na 0 i przesylam dane w dwoch bajtach - najpierw najstarszy potem najmlodszy
void sendData(uint16_t data) {
  digitalWrite(FSYNC, LOW);
  SPI.transfer(highByte(data));
  SPI.transfer(lowByte(data));
  digitalWrite(FSYNC, HIGH);
}

//resetowanie ukladu - ustawiam bit RESET na 1
void resetIc() {
  sendData(CTRL_RESET);
  sendData(CTRL_B28);
  delay(10);
}

//ustiawianie czestotliwosci - tworze slowo z czestotliwoscia przeliczona wg wzoru i dziele na mlodszy i starszy bit
void setFrequency(unsigned long freq) {
  SPI.setDataMode(SPI_MODE2);
  uint16_t mask = 0x3FFF; //maska dla slowa 
  uint8_t bitsToShift = 14; //przesuwam o 14 bitow bo dane o czestotliwosci sa w 14 najmlodszych bitach
  unsigned long FreqWord = (unsigned long)((freq * FREQ_FACTOR) / OSC_FREQ);

  uint16_t MSB = (FreqWord >> bitsToShift) & mask;
  uint16_t LSB = FreqWord & mask;

  LSB |= 0x4000;
  MSB |= 0x4000;

  //sendData(CTRL_B28);  // reset wyłączony, B28 włączony
  sendData(LSB);
  sendData(MSB);
  //sendData(PHASE_ZERO);  // faza zero
}

//ustawia ksztalt przebiegu na przebieg sinusoidalny
void setModeToSine() {
  sendData(SINE);
}

//ustawia ksztalt przebiegu na przebieg prostokatny
void setModeToSquare() {
  sendData(SQUARE);
}

//ustawia ksztalt przebiegu na przebieg trojkatny
void setModeToTriangle() {
  sendData(TRIANGLE);
}

void encoderISR() {
  bool A = digitalRead(ENCODER_PIN_1);
  bool B = digitalRead(ENCODER_PIN_2);

  if (A == B) {
    encoderDirection = 1;
  } else {
    encoderDirection = -1;
  }
}

void handleModeChange() {
  if(currentMode == 0) {
    currentMode++;
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print("TRIANGLE");
    setModeToTriangle();
  } else if(currentMode == 1) {
    currentMode++;
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print("SQUARE");
    setModeToSquare();
  } else {
    currentMode = 0;
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print("SINE");
    setModeToSine();
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(ENCODER_PIN_1, INPUT);
  pinMode(ENCODER_PIN_2, INPUT);
  pinMode(SWITCH_PIN, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_1), encoderISR, FALLING);
  pinMode(FSYNC, OUTPUT);
  digitalWrite(FSYNC, HIGH);
  pinMode(EN_POT, OUTPUT);
  digitalWrite(EN_POT, HIGH);
  SPI.begin();
  SPI.setDataMode(SPI_MODE2);
  resetIc();
  setFrequency(gen_freq);
  setModeToSine();
  setVoltage(10);
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print(gen_freq);
  lcd.print(" Hz");
  lcd.setCursor(0, 1);
  lcd.print("SINE");
  lcd.setCursor(8, 0);
  lcd.print(currentVoltage);
  lcd.print(" V");
}

void loop() {
    int switchPinVal = digitalRead(SWITCH_PIN);
    
    if (encoderDirection != 0) {
    if(encoderDirection > 0) {
      gen_freq = gen_freq + 100;
    } else if(gen_freq - 100 > 0) {
      gen_freq = gen_freq - 100;
    }
    
    setFrequency(gen_freq);
    lcd.setCursor(0, 0);
    lcd.print("                ");
    lcd.setCursor(0, 0);
    lcd.print(gen_freq);
    lcd.print(" Hz");

    if(encoderDirection > 0 && currentVoltage + 0.1 < MAX_VOLTAGE) {
        currentVoltage = currentVoltage + 0.1;
      } else if(encoderDirection < 0 && currentVoltage - 0.1 > MIN_VOLTAGE)  {
        currentVoltage = currentVoltage - 0.1;
      }
    
    lcd.setCursor(8, 0);
    lcd.print(currentVoltage);
    lcd.print(" V");
    setVoltage(currentVoltage);
    
    encoderDirection = 0;
  }

  if (switchPinVal != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (switchPinVal != buttonState) {
      buttonState = switchPinVal;
      if (buttonState == LOW) {
        handleModeChange();
      }
    }
  }
    lastButtonState = switchPinVal;
}
