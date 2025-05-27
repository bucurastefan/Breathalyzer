#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Define pin numbers
#define pinLedRosu 13    // PB5
#define pinLedVerde 12   // PB4
#define pinBuzzer A0     // PC0
#define pinSenzorAlcool A1  // PC1
#define encoderA 8       // PB0
#define encoderB 9       // PB1
#define encoderBtn 10    // PB2

// Encoder variables
int prag1 = 500;
int prag2 = 750;
int valoareAlcool = 0;
int menuIndex = 0;
const char* menuItems[] = {"Start Test", "Set Prag 1", "Set Prag 2"};
const int menuLength = sizeof(menuItems) / sizeof(menuItems[0]);
unsigned long lastEncoderTime = 0;
unsigned long encoderDelay = 150;
bool lastButtonState = 1;
bool buttonPressed = false;
unsigned long lastButtonTime = 0;
unsigned long buttonDelay = 250;

// Custom timer variables (to avoid conflicts with Arduino core)
volatile unsigned long custom_timer_millis = 0;
volatile unsigned char custom_timer_fract = 0;

void setup() {
  // Initialize GPIO pins using direct register manipulation
  initGPIO();
  
  // Initialize custom timer
  initCustomTimer();
  
  // Initialize ADC
  initADC();
  
  // Initialize UART for debugging (optional)
  initUART();
  
  // Enable global interrupts
  sei();
  
  // Initialize display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  startupScreen();
}

void initGPIO() {
  // Set pin directions using DDR registers
  DDRB |= (1 << DDB5);  // pin 13 (LED Red) as output
  DDRB |= (1 << DDB4);  // pin 12 (LED Green) as output
  DDRC |= (1 << DDC0);  // A0 (Buzzer) as output
  DDRC &= ~(1 << DDC1); // A1 (Alcohol sensor) as input
  DDRB &= ~(1 << DDB0); // pin 8 (encoder A) as input
  DDRB &= ~(1 << DDB1); // pin 9 (encoder B) as input
  DDRB &= ~(1 << DDB2); // pin 10 (encoder button) as input

  // Enable pull-up resistors for encoder pins
  PORTB |= (1 << PORTB0); // pull-up for pin 8
  PORTB |= (1 << PORTB1); // pull-up for pin 9
  PORTB |= (1 << PORTB2); // pull-up for pin 10
  
  // Initialize outputs to LOW
  PORTB &= ~(1 << PORTB5); // LED Red OFF
  PORTB &= ~(1 << PORTB4); // LED Green OFF
  PORTC &= ~(1 << PORTC0); // Buzzer OFF
}

void initCustomTimer() {
  // Configure Timer1 instead of Timer0 to avoid conflicts
  // Set prescaler to 64: CS12=0, CS11=1, CS10=1
  TCCR1B |= (1 << CS11) | (1 << CS10);
  TCCR1B &= ~(1 << CS12);
  
  // Set Timer1 to CTC mode with OCR1A as top
  TCCR1B |= (1 << WGM12);
  
  // Set compare value for 1ms interrupt (16MHz/64/250 = 1000Hz = 1ms)
  OCR1A = 249;
  
  // Enable Timer1 compare interrupt
  TIMSK1 |= (1 << OCIE1A);
}

void initADC() {
  // Set ADC reference voltage to AVCC
  ADMUX |= (1 << REFS0);
  ADMUX &= ~(1 << REFS1);
  
  // Set ADC prescaler to 128 for 16MHz/128 = 125kHz
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  
  // Enable ADC
  ADCSRA |= (1 << ADEN);
}

void initUART() {
  // Set baud rate to 9600 (for 16MHz: UBRR = 103)
  UBRR0H = 0;
  UBRR0L = 103;
  
  // Enable transmitter
  UCSR0B |= (1 << TXEN0);
  
  // Set frame format: 8 data bits, 1 stop bit
  UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
}

// Timer1 compare interrupt for custom millis functionality
ISR(TIMER1_COMPA_vect) {
  custom_timer_millis++;
}

unsigned long customMillis() {
  unsigned long m;
  uint8_t oldSREG = SREG;
  
  cli(); // Disable interrupts
  m = custom_timer_millis;
  SREG = oldSREG; // Restore interrupt state
  
  return m;
}

void customDelay(unsigned long ms) {
  unsigned long start = customMillis();
  while (customMillis() - start < ms) {
    // Wait
  }
}

int analogReadDirect(uint8_t pin) {
  // Select ADC channel
  ADMUX = (ADMUX & 0xF0) | ((pin - A0) & 0x0F);
  
  // Start conversion
  ADCSRA |= (1 << ADSC);
  
  // Wait for conversion to complete
  while (ADCSRA & (1 << ADSC));
  
  // Return ADC result
  return ADC;
}

bool digitalReadDirect(uint8_t pin) {
  switch(pin) {
    case 8:  return (PINB & (1 << PINB0)) != 0;
    case 9:  return (PINB & (1 << PINB1)) != 0;
    case 10: return (PINB & (1 << PINB2)) != 0;
    default: return false;
  }
}

void digitalWriteDirect(uint8_t pin, bool value) {
  switch(pin) {
    case 13: // LED Red
      if (value) PORTB |= (1 << PORTB5);
      else PORTB &= ~(1 << PORTB5);
      break;
    case 12: // LED Green
      if (value) PORTB |= (1 << PORTB4);
      else PORTB &= ~(1 << PORTB4);
      break;
    case A0: // Buzzer
      if (value) PORTC |= (1 << PORTC0);
      else PORTC &= ~(1 << PORTC0);
      break;
  }
}

void generateTone(uint16_t frequency, uint16_t duration) {
  // Simple tone generation using delay
  unsigned long cycles = (unsigned long)frequency * duration / 1000;
  unsigned long halfPeriod = 500000UL / frequency; // microseconds
  
  for (unsigned long i = 0; i < cycles; i++) {
    digitalWriteDirect(pinBuzzer, HIGH);
    delayMicroseconds(halfPeriod);
    digitalWriteDirect(pinBuzzer, LOW);
    delayMicroseconds(halfPeriod);
  }
}

void bip() {
  generateTone(1000, 100); // 1kHz tone for 100ms
}

void loop() {
  citireEncoder();
  afiseazaMeniu();

  bool buttonState = !digitalReadDirect(encoderBtn);
  if (lastButtonState == 1 && buttonState && (customMillis() - lastButtonTime > buttonDelay)) {
    buttonPressed = true;
    lastButtonTime = customMillis();
    executaOptiune(menuIndex);
  }
  lastButtonState = buttonState;
}

void citireEncoder() {
  static int lastA = 1;
  int currentA = digitalReadDirect(encoderA);
  int currentB = digitalReadDirect(encoderB);

  if (currentA != lastA && currentA == 0) {
    if (currentB) {
      menuIndex = (menuIndex + 1) % menuLength;
    } else {
      menuIndex = (menuIndex - 1 + menuLength) % menuLength;
    }
    customDelay(10);
  }
  lastA = currentA;
}

void afiseazaMeniu() {
  display.clearDisplay();
  display.setTextSize(1);

  for (int i = 0; i < menuLength; i++) {
    int y = i * 16;
    if (i == menuIndex) {
      display.fillRect(0, y, SCREEN_WIDTH, 16, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    } else {
      display.setTextColor(SSD1306_WHITE);
    }
    display.setCursor(8, y + 4);
    display.println(menuItems[i]);
  }
  display.display();
}

void executaOptiune(int opt) {
  switch (opt) {
    case 0: startTest(); break;
    case 1: modificaPrag(&prag1, "Setare Prag 1"); break;
    case 2: modificaPrag(&prag2, "Setare Prag 2"); break;
  }
}

void startTest() {
  display.clearDisplay();
  display.setCursor(10, 20);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.println("Sufla timp de 3 sec");
  display.display();
  customDelay(3000);

  valoareAlcool = analogReadDirect(pinSenzorAlcool);
  float mgL = (float)(valoareAlcool - 450) * 1.6 / (950 - 450);
  if (mgL < 0.0) mgL = 0.0;
  if (mgL > 1.6) mgL = 1.6;

  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);

  if (valoareAlcool >= prag2) {
    afiseazaDosarPenal();
  } else if (valoareAlcool >= prag1) {
    digitalWriteDirect(pinLedRosu, 1);
    digitalWriteDirect(pinLedVerde, 0);
    bip();
    display.setTextColor(SSD1306_WHITE);
    display.println("SUSPENDARE");
  } else {
    digitalWriteDirect(pinLedVerde, 1);
    digitalWriteDirect(pinLedRosu, 0);
    display.setTextColor(SSD1306_WHITE);
    display.println("OK");
  }

  display.setTextSize(1);
  display.setCursor(0, 40);
  display.print("Valoare: ");
  display.print(mgL, 2);
  display.println(" mg/L");
  display.display();
  customDelay(3000);
  digitalWriteDirect(pinLedRosu, 0);
  digitalWriteDirect(pinLedVerde, 0);
}

void afiseazaDosarPenal() {
  for (int i = 0; i < 3; i++) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    display.setCursor(10, 10);
    display.println("DOSAR");
    display.setCursor(10, 30);
    display.println("PENAL");
    display.display();
    bip();
    customDelay(250);
  }
}

void modificaPrag(int* prag, const char* label) {
  bool inSubmeniu = true;
  int localIndex = *prag;

  while (inSubmeniu) {
    citireEncoderSubmeniu(localIndex);
    valoareAlcool = analogReadDirect(pinSenzorAlcool);

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(label);
    display.setCursor(0, 20);
    display.print("Valoare: ");
    display.println(localIndex);
    display.setCursor(0, 40);
    display.print("Senzor: ");
    display.println(valoareAlcool);
    display.display();

    bool btnState = !digitalReadDirect(encoderBtn);
    if (btnState && !buttonPressed && customMillis() - lastButtonTime > buttonDelay) {
      buttonPressed = true;
      lastButtonTime = customMillis();
      *prag = localIndex;
      inSubmeniu = false;
    }
    if (!btnState) {
      buttonPressed = false;
    }
  }
}

void citireEncoderSubmeniu(int &val) {
  static int lastA = 1;
  int currentA = digitalReadDirect(encoderA);
  int currentB = digitalReadDirect(encoderB);

  if (currentA != lastA && currentA == 0) {
    if (currentB) {
      val -= 10;
    } else {
      val += 10;
    }
    if (val < 0) val = 0;
    if (val > 1023) val = 1023;
    customDelay(10);
  }
  lastA = currentA;
}

void startupScreen() {
  bip();
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(15, 10);
  display.println("ETILOTEST");
  display.setTextSize(1);
  display.setCursor(25, 35);
  display.println("Pornire sistem");
  display.display();
  customDelay(1500);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println("Incalzire senzor...");
  display.display();

  for (int i = 0; i <= 100; i += 10) {
    display.fillRect(10, 30, i, 10, SSD1306_WHITE);
    display.display();
    customDelay(300);
  }

  display.setCursor(0, 50);
  display.println("Senzor gata!");
  display.display();
  customDelay(1000);
}
