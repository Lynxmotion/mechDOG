#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

LiquidCrystal_I2C lcd(0x3F, 16, 2); // address = 0x3F, 16 chars, 2 lines

RF24 radio(12, 14); // CE, CSN
const uint64_t dataPipe = 0x646F67676FLL;

struct DATA_STRUCTURE
{
  int joystickB; // button
  int joystickX;
  int joystickY;
  int joystickZ;
  int button1;
  int button2;
  int button3;
  int button4;
  int toggle1;
  int toggle2;
  int toggle3;
};

DATA_STRUCTURE data;

void setup()
{
  Serial.begin(9600);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(3, 0);
  lcd.print("Mech-Dickel");
  lcd.setCursor(4, 1);
  lcd.print("Robotics");

  pinMode(10, OUTPUT);       // GND for radio module
  digitalWrite(10, LOW);

  pinMode(32, INPUT_PULLUP); // joystick button
  pinMode(33, OUTPUT);       // GND for joystick button
  digitalWrite(33, LOW);

  pinMode(28, INPUT_PULLUP); // button 1
  pinMode(29, INPUT_PULLUP); // button 2
  pinMode(30, INPUT_PULLUP); // button 3
  pinMode(31, INPUT_PULLUP); // button 4
  pinMode(26, OUTPUT);       // GND for button pad
  digitalWrite(26, LOW);

  pinMode(2, INPUT_PULLUP);  // toggle 1
  pinMode(3, OUTPUT);        // GND for toggle 1
  digitalWrite(3, LOW);

  pinMode(6, INPUT_PULLUP);  // toggle 2
  pinMode(7, OUTPUT);        // GND for toggle 2
  digitalWrite(7, LOW);

  pinMode(8, INPUT_PULLUP);  // toggle 3
  pinMode(9, OUTPUT);        // GND for toggle 3
  digitalWrite(9, LOW);

  pinMode(A2, OUTPUT);       // 5V and
  digitalWrite(A2, HIGH);
  pinMode(A4, OUTPUT);       // GND for joystick X axis
  digitalWrite(A4, LOW);

  pinMode(A3, OUTPUT);       // 5V and
  digitalWrite(A3, HIGH);
  pinMode(A5, OUTPUT);       // GND for joystick Y
  digitalWrite(A5, LOW);

  pinMode(A8, OUTPUT);       // 5V and
  digitalWrite(A8, HIGH);
  pinMode(A10, OUTPUT);      // GND for joystick Z
  digitalWrite(A10, LOW);

  radio.begin();
  radio.openWritingPipe(dataPipe);
}

void loop()
{
  data.joystickB = digitalRead(32);

  data.joystickX = analogRead(A0);
  data.joystickX = map(data.joystickX, 0, 1023, 1023, 0);
  data.joystickY = analogRead(A1);
  data.joystickZ = analogRead(A6);

  data.button1 = digitalRead(28);
  data.button2 = digitalRead(29);
  data.button3 = digitalRead(30);
  data.button4 = digitalRead(31);

  data.toggle1 = digitalRead(2);
  data.toggle2 = digitalRead(6);
  data.toggle3 = digitalRead(8);

  radio.write(&data, sizeof(DATA_STRUCTURE));

/*
  Serial.println("\r\n----------------------");

  Serial.print("joystick button = "); Serial.println(data.joystickB);
  Serial.print("joystick X axis = "); Serial.println(data.joystickX);
  Serial.print("joystick Y axis = "); Serial.println(data.joystickY);
  Serial.print("joystick Z axis = "); Serial.println(data.joystickZ);
  Serial.print("button 1        = "); Serial.println(data.button1);
  Serial.print("button 2        = "); Serial.println(data.button2);
  Serial.print("button 3        = "); Serial.println(data.button3);
  Serial.print("button 4        = "); Serial.println(data.button4);
  Serial.print("toggle 1        = "); Serial.println(data.toggle1);
  Serial.print("toggle 2        = "); Serial.println(data.toggle2);
  Serial.print("toggle 3        = "); Serial.println(data.toggle3);
  delay(1000);
*/
}
