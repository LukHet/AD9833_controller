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
#define ARROW_CODE      62

volatile int encoderDirection = 0;
int currentMode = 0;
float currentVoltage = 0;
uint16_t gen_freq = 1000;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50; 
int lastButtonState = HIGH;
int buttonState;
int currentWaveMode = 0;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

//przesylam dane do potencjometru cyfrowego
void sendDigitalPotentiometerData(float voltage) {
  voltage = constrain(voltage, MIN_VOLTAGE, MAX_VOLTAGE);
  float gain = 1.0;
  if (currentWaveMode == 1) {
    gain = 1.0 / 7.0;
  }
  float effectiveVoltage = voltage * gain;
  float potFloat = (effectiveVoltage / MAX_VOLTAGE) * 255.0;
  byte potValue = constrain((int)(potFloat + 0.5), 0, 255);

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
  Serial.print("handling mode change: ");
  Serial.println(currentMode);

  if(currentMode == 0) {
    currentMode = 1;
  } else if (currentMode == 1) {
    currentMode = 2;
  } else {
    currentMode = 0;
  }

  if(currentMode == 0) {
    lcd.setCursor(0, 0);
    lcd.print((char) ARROW_CODE);
    lcd.setCursor(8, 0);
    lcd.print(" ");
    lcd.setCursor(0, 1);
    lcd.print(" ");
  } else if (currentMode == 1) {
    lcd.setCursor(9, 0);
    lcd.print((char) ARROW_CODE);
    lcd.setCursor(0, 0);
    lcd.print(" ");
    lcd.setCursor(0, 1);
    lcd.print(" ");
  } else if (currentMode == 2) {
    lcd.setCursor(0, 1);
    lcd.print((char) ARROW_CODE);
    lcd.setCursor(9, 0);
    lcd.print(" ");
    lcd.setCursor(0, 0);
    lcd.print(" ");
  }
}

void handleWaveformChange() {

    if(currentWaveMode > 2) {
      currentWaveMode = 0;
    } else if (currentWaveMode < 0) {
      currentWaveMode = 2;
    }
    if(currentWaveMode == 0) {
      lcd.setCursor(1, 1);
      lcd.print("                ");
      lcd.setCursor(1, 1);
      lcd.print("TRIANGLE");
      setModeToTriangle();
      setVoltage(currentVoltage);
    } else if(currentWaveMode == 1) {
      lcd.setCursor(1, 1);
      lcd.print("                ");
      lcd.setCursor(1, 1);
      lcd.print("SQUARE");
      setModeToSquare();
      setVoltage(currentVoltage);
    } else if(currentWaveMode == 2) {
      lcd.setCursor(1, 1);
      lcd.print("                ");
      lcd.setCursor(1, 1);
      lcd.print("SINE");
      setModeToSine();
      setVoltage(currentVoltage);
  } else {
    currentWaveMode = 0;
  }
}

void handleEncoderClick(int pin) {
  
  if (pin != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (pin != buttonState) {
      buttonState = pin;
      if (buttonState == LOW) {
        Serial.write("encoder has been clicked");
        handleModeChange();
      }
    }
  }
    lastButtonState = pin;
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
  lcd.print((char) ARROW_CODE);
  lcd.setCursor(1, 0);
  lcd.print(gen_freq);
  lcd.print(" Hz");
  lcd.setCursor(1, 1);
  lcd.print("SINE");
  lcd.setCursor(10, 0);
  lcd.print(currentVoltage, 1);
  lcd.print(" V");
}

void loop() {
    int switchPinVal = digitalRead(SWITCH_PIN);

    
    if (encoderDirection != 0) {
       if (currentMode == 0) {
        if(encoderDirection > 0) {
          gen_freq = gen_freq + 100;
        } else if(gen_freq - 100 > 0) {
          gen_freq = gen_freq - 100;
        }
        
        setFrequency(gen_freq);
        lcd.setCursor(1, 0);
        lcd.print("       ");
        lcd.setCursor(1, 0);
        lcd.print(gen_freq);
        lcd.print(" Hz");
       } else if (currentMode == 1) {

          if(encoderDirection > 0 && currentVoltage + 0.1 < MAX_VOLTAGE) {
              currentVoltage = currentVoltage + 0.1;
            } else if(encoderDirection < 0 && currentVoltage - 0.1 > MIN_VOLTAGE)  {
              currentVoltage = currentVoltage - 0.1;
            }
          
          lcd.setCursor(10, 0);
          lcd.print(currentVoltage, 1);
          lcd.print(" V");

          if(currentVoltage < 10.0) {
            lcd.setCursor(15, 0);
            lcd.print(" ");
          }
          setVoltage(currentVoltage);
          
     } else if (currentMode == 2) {
      
          if(encoderDirection > 0) {
             currentWaveMode++;
          } else if(encoderDirection < 0) {
             currentWaveMode--;
          }
          handleWaveformChange();
     }
    
    encoderDirection = 0;
  }

  handleEncoderClick(switchPinVal);
}
