// set up for neopixel
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel neopixel = Adafruit_NeoPixel(1, 12, NEO_RGB);

// set up for the sensor
#include <Adafruit_LSM6DSOX.h>

// set up for the bluetooth function
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "Adafruit_BLEMIDI.h"
#if SOFTWARE_SERIAL_AVAILABLE
#include <SoftwareSerial.h>
#endif

#include "BluefruitConfig.h"

#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.7.0"

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

Adafruit_BLEMIDI midi(ble);

bool isConnected = false;
int current_note = 60;

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// callback
void connected(void)
{
  isConnected = true;

//  Serial.println(F(" CONNECTED!"));
  delay(10);

}

void disconnected(void)
{
//  Serial.println("disconnected");
  isConnected = false;
}

void BleMidiRX(uint16_t timestamp, uint8_t status, uint8_t byte1, uint8_t byte2)
{
//  Serial.print("[MIDI ");
//  Serial.print(timestamp);
//  Serial.print(" ] ");
//
//  Serial.print(status, HEX); Serial.print(" ");
//  Serial.print(byte1 , HEX); Serial.print(" ");
//  Serial.print(byte2 , HEX); Serial.print(" ");
//
//  Serial.println();
}

// __     __         _       _     _
// \ \   / /_ _ _ __(_) __ _| |__ | | ___  ___
//  \ \ / / _` | '__| |/ _` | '_ \| |/ _ \/ __|
//   \ V / (_| | |  | | (_| | |_) | |  __/\__ \
//    \_/ \__,_|_|  |_|\__,_|_.__/|_|\___||___/
int ledArray[4] = {5, 6, 9, 10};
int totalLeds = 4;

int potControl1 = 0;
int mappedPotControl1 = 0;
int mappedMidiControl1 = 0;

int potControl2 = 0;
int mappedPotControl2 = 0;
int mappedMidiControl2 = 0;

int potControl3 = 0;
int mappedPotControl3 = 0;
int mappedMidiControl3 = 0;

bool onOffButtonState = LOW;
bool onOffButtonLastState = LOW;
int onOffButton = 13;
bool onOffButtonOn = false;

bool modeSwitchState = LOW;
bool modeSwitchLastState = LOW;
int modeSwitch = 1;
bool modeSwitchOn = false;

int tempoVal = 0;

int startingNote = 64;

int majorScale[43] = {-36, -34, -32, -31, -29, -27, -25, -24, -22, -20, -19, -17, -15, -13, -12, -10, -8, -7, -5, -3, -1, 0, 2, 4, 5, 7, 9, 11, 12, 14, 16, 17, 19, 21, 23, 24, 26, 28, 29, 31, 33, 35, 36};
int minorScale[43] = {-36, -34, -33, -31, -29, -28, -26, -24, -22, -21, -19, -17, -16, -14, -12, -10, -9, -7, -5, -4, -2, 0, 2, 3, 5, 7, 8, 10, 12, 14, 15, 17, 19, 20, 22, 24, 26, 27, 29, 31, 32, 34, 36};
int majorPentatonicScale[31] = {-36, -34, -32, -29, -27, -24, -22, -20, -17, -15, -12, -10, -8, -5, -3, 0, 2, 4, 7, 9, 12, 14, 16, 19, 21, 24, 26, 28, 31, 33, 36};
int minorPentatonicScale[31] = {-36, -34, -33, -29, -28, -24, -22, -21, -17, -16, -12, -10, -9, -5, -4, 0, 2, 3, 7, 8, 12, 14, 15, 19, 20, 24, 26, 27, 31, 32, 36};

int numSteps = 5;
int currentStep = 0;
unsigned long lastStepTime = 0;

int nextChannelButton = 20;
bool nextChannelButtonState = LOW;
bool nextChannelButtonLastState = LOW;
bool nextChannelButtonOn = false;

int channel = 0;


// set up for the sensor, just copy whatever is in the example code that make it work
// some of the codes aren't useful to your purpose, just test them and choose whether keep them or delete them
Adafruit_LSM6DSOX sox;
void setup(void) {
  Serial.begin(9600);
  pinMode(onOffButton, INPUT);
  for (int i = 0; i < totalLeds; i++) {
    pinMode(ledArray[i], OUTPUT);
  }
//  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
//    Serial.println(F("SSD1306 allocation failed"));
//    for (;;); // Don't proceed, loop forever
//  }

// code to make neopixel work
  neopixel.begin();
  neopixel.clear();
  neopixel.show();

  while (!Serial)
    delay(10);

  if (!sox.begin_I2C()) {
    while (1) {
      delay(10);
    }
  }

  // sox.setAccelRange(LSM6DS_ACCEL_RANGE_8_G);
//  Serial.print(F("Accelerometer range set to: "));
  switch (sox.getAccelRange()) {
    case LSM6DS_ACCEL_RANGE_2_G:
      Serial.println(F("+-2G"));
      break;
    case LSM6DS_ACCEL_RANGE_4_G:
      Serial.println(F("+-4G"));
      break;
    case LSM6DS_ACCEL_RANGE_8_G:
      Serial.println(F("+-8G"));
      break;
    case LSM6DS_ACCEL_RANGE_16_G:
      Serial.println(F("+-16G"));
      break;
  }

  // sox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS );
//  Serial.print(F("Gyro range set to: "));
  switch (sox.getGyroRange()) {
    case LSM6DS_GYRO_RANGE_125_DPS:
      Serial.println(F("125 degrees/s"));
      break;
    case LSM6DS_GYRO_RANGE_250_DPS:
      Serial.println(F("250 degrees/s"));
      break;
    case LSM6DS_GYRO_RANGE_500_DPS:
      Serial.println(F("500 degrees/s"));
      break;
    case LSM6DS_GYRO_RANGE_1000_DPS:
      Serial.println(F("1000 degrees/s"));
      break;
    case LSM6DS_GYRO_RANGE_2000_DPS:
      Serial.println(F("2000 degrees/s"));
      break;
    case ISM330DHCX_GYRO_RANGE_4000_DPS:
      break; // unsupported range for the DSOX
  }

  // sox.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
//  Serial.print(F("Accelerometer data rate set to: "));
  switch (sox.getAccelDataRate()) {
    case LSM6DS_RATE_SHUTDOWN:
      Serial.println(F("0 Hz"));
      break;
    case LSM6DS_RATE_12_5_HZ:
      Serial.println(F("12.5 Hz"));
      break;
    case LSM6DS_RATE_26_HZ:
      Serial.println(F("26 Hz"));
      break;
    case LSM6DS_RATE_52_HZ:
      Serial.println(F("52 Hz"));
      break;
    case LSM6DS_RATE_104_HZ:
      Serial.println(F("104 Hz"));
      break;
    case LSM6DS_RATE_208_HZ:
      Serial.println(F("208 Hz"));
      break;
    case LSM6DS_RATE_416_HZ:
      Serial.println(F("416 Hz"));
      break;
    case LSM6DS_RATE_833_HZ:
      Serial.println(F("833 Hz"));
      break;
    case LSM6DS_RATE_1_66K_HZ:
      Serial.println(F("1.66 KHz"));
      break;
    case LSM6DS_RATE_3_33K_HZ:
      Serial.println(F("3.33 KHz"));
      break;
    case LSM6DS_RATE_6_66K_HZ:
      Serial.println(F("6.66 KHz"));
      break;
  }

  // sox.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
//  Serial.print(F("Gyro data rate set to: "));
  switch (sox.getGyroDataRate()) {
    case LSM6DS_RATE_SHUTDOWN:
      Serial.println(F("0 Hz"));
      break;
    case LSM6DS_RATE_12_5_HZ:
      Serial.println(F("12.5 Hz"));
      break;
    case LSM6DS_RATE_26_HZ:
      Serial.println(F("26 Hz"));
      break;
    case LSM6DS_RATE_52_HZ:
      Serial.println(F("52 Hz"));
      break;
    case LSM6DS_RATE_104_HZ:
      Serial.println(F("104 Hz"));
      break;
    case LSM6DS_RATE_208_HZ:
      Serial.println(F("208 Hz"));
      break;
    case LSM6DS_RATE_416_HZ:
      Serial.println(F("416 Hz"));
      break;
    case LSM6DS_RATE_833_HZ:
      Serial.println(F("833 Hz"));
      break;
    case LSM6DS_RATE_1_66K_HZ:
      Serial.println(F("1.66 KHz"));
      break;
    case LSM6DS_RATE_3_33K_HZ:
      Serial.println(F("3.33 KHz"));
      break;
    case LSM6DS_RATE_6_66K_HZ:
      Serial.println(F("6.66 KHz"));
      break;
  }
  while (!Serial);  // required for Flora & Micro
  delay(10);

//  Serial.begin(115200);
//  Serial.println(F("Adafruit Bluefruit MIDI Example"));
//  Serial.println(F("---------------------------------------"));

  /* Initialise the module */
//  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
//  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
//    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }

  //ble.sendCommandCheckOK(F("AT+uartflow=off"));
  ble.echo(false);

//  Serial.println(F("Requesting Bluefruit info:"));
  /* Print Bluefruit information */
  ble.info();

  /* Set BLE callbacks */
  ble.setConnectCallback(connected);
  ble.setDisconnectCallback(disconnected);

  // Set MIDI RX callback
  midi.setRxCallback(BleMidiRX);

  Serial.println(F("Enable MIDI: "));
  if ( ! midi.begin(true) )
  {
    error(F("Could not enable MIDI"));
  }

  ble.verbose(false);
  Serial.print(F("Waiting for a connection..."));
}

void loop(void) {
  checkOnOffButton();
  checkModeSwitch();
  tempoVal = analogRead(A1);
  nextChannelButtonCheck();

  //  _     ____  __  __  __   ____  ____   _____  __
  // | |   / ___||  \/  |/ /_ |  _ \/ ___| / _ \ \/ /
  // | |   \___ \| |\/| | '_ \| | | \___ \| | | \  /
  // | |___ ___) | |  | | (_) | |_| |___) | |_| /  \ 
  // |_____|____/|_|  |_|\___/|____/|____/ \___/_/\_\    

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  sox.getEvent(&accel, &gyro, &temp);
  delay(10);

  //               _               _   _
  //     /\       | |             | | | |
  //    /  \   ___| |_ _   _  __ _| | | |     ___   ___  _ __
  //   / /\ \ / __| __| | | |/ _` | | | |    / _ \ / _ \| '_ \ 
  //  / ____ \ (__| |_| |_| | (_| | | | |___| (_) | (_) | |_) |
  // /_/    \_\___|\__|\__,_|\__,_|_| |______\___/ \___/| .__/
  //                                                    | |
  //                                                    |_|

  if (onOffButtonOn == true) {                   //power on
    neopixel.setPixelColor(0, 0, 255, 0);
    neopixel.show();
    if (modeSwitchOn == true) {                  //keybaord mode
      digitalWrite(ledArray[2], LOW);
      digitalWrite(ledArray[3], HIGH);
//      Serial.println(F("keybaord mode"));
//      textKeyMode();
      if (currentStep == 0) {
        ble.update(500);
        if (! isConnected)
          return;
        Serial.println(F("chromatic scale"));
//        textChromaticScale();
        potSetUpForKeyboard(int(100 * accel.acceleration.x), int(100 * accel.acceleration.y), int(tempoVal));
        playKey(mappedPotControl1, mappedPotControl2);
      } else if (currentStep == 1) {
        Serial.println(F("major scale"));
//        textMajorScale();
        setUpForScale(int(100 * accel.acceleration.x), int(100 * accel.acceleration.y), int(tempoVal));
        playMajorScale(mappedPotControl1, mappedPotControl2);
      } else if (currentStep == 2) {
        Serial.println(F("minor scale"));
//        textMinorScale();
        setUpForScale(int(100 * accel.acceleration.x), int(100 * accel.acceleration.y), int(tempoVal));
        playMinorScale(mappedPotControl1, mappedPotControl2);
      } else if (currentStep == 3) {
        Serial.println(F("pentatonic major scale"));
//        textMajorPentaScale();
        setUpForPentaScale(int(100 * accel.acceleration.x), int(100 * accel.acceleration.y), int(tempoVal));
        playPentaMajorScale(mappedPotControl1, mappedPotControl2);
      } else if (currentStep == 4) {
        Serial.println(F("pentatonic minor scale"));
//        textMinorPentaScale();
        setUpForPentaScale(int(100 * accel.acceleration.x), int(100 * accel.acceleration.y), int(tempoVal));
        playPentaMinorScale(mappedPotControl1, mappedPotControl2);
      }
    } else if (modeSwitchOn == false) {          //midi control mode
//      Serial.println(F("midi control mode"));
      digitalWrite(ledArray[3], LOW);
      digitalWrite(ledArray[2], HIGH);
//      textMidiMode();
      potSetUpForControl(int(100 * accel.acceleration.x), int(100 * accel.acceleration.y), int(tempoVal));
    }
  } else {                                       //power off
    digitalWrite(ledArray[2], LOW);
    neopixel.setPixelColor(0, 0, 0, 0);
    neopixel.show();
    onOffButtonOn = false;
    modeSwitchOn = false;
  }
}

//  _  __          _                         _   __  __           _
// | |/ /___ _   _| |__   ___   __ _ _ __ __| | |  \/  | ___   __| | ___
// | ' // _ \ | | | '_ \ / _ \ / _` | '__/ _` | | |\/| |/ _ \ / _` |/ _ \
// | . \  __/ |_| | |_) | (_) | (_| | | | (_| | | |  | | (_) | (_| |  __/
// |_|\_\___|\__, |_.__/ \___/ \__,_|_|  \__,_| |_|  |_|\___/ \__,_|\___| (chromatic)
//           |___/
void potSetUpForKeyboard(int x, int y, int z) {
  // control of keyboard note range
  potControl1 = x;
  mappedPotControl1 = map(potControl1, -1000, 1000, 21, 108);

  // control of note velocity
  potControl2 = y;
  mappedPotControl2 = map(potControl2, -1000, 1000, 0, 127);

  // control of the delay
  potControl3 = z;
  mappedPotControl3 = map(potControl3, 0, 1023, 50, 500);
}


//  _  __          _                         _   __  __           _
// | |/ /___ _   _| |__   ___   __ _ _ __ __| | |  \/  | ___   __| | ___
// | ' // _ \ | | | '_ \ / _ \ / _` | '__/ _` | | |\/| |/ _ \ / _` |/ _ \
// | . \  __/ |_| | |_) | (_) | (_| | | | (_| | | |  | | (_) | (_| |  __/
// |_|\_\___|\__, |_.__/ \___/ \__,_|_|  \__,_| |_|  |_|\___/ \__,_|\___| (major and minor)
//           |___/
void setUpForScale(int x, int y, int z) {
  // control of keyboard note range
  potControl1 = x;
  mappedPotControl1 = map(potControl1, -1000, 1000, 0, 43);

  // control of note velocity
  potControl2 = y;
  mappedPotControl2 = map(potControl2, -1000, 1000, 0, 127);

  // control of the delay
  potControl3 = z;
  mappedPotControl3 = map(potControl3, 0, 1023, 50, 500);
}

//  _  __          _                         _   __  __           _
// | |/ /___ _   _| |__   ___   __ _ _ __ __| | |  \/  | ___   __| | ___
// | ' // _ \ | | | '_ \ / _ \ / _` | '__/ _` | | |\/| |/ _ \ / _` |/ _ \
// | . \  __/ |_| | |_) | (_) | (_| | | | (_| | | |  | | (_) | (_| |  __/
// |_|\_\___|\__, |_.__/ \___/ \__,_|_|  \__,_| |_|  |_|\___/ \__,_|\___| (Pentatonic)
//           |___/
void setUpForPentaScale(int x, int y, int z) {
  // control of keyboard note range
  potControl1 = x;
  mappedPotControl1 = map(potControl1, -1000, 1000, 0, 31);

  // control of note velocity
  potControl2 = y;
  mappedPotControl2 = map(potControl2, -1000, 1000, 0, 127);

  // control of the delay
  potControl3 = z;
  mappedPotControl3 = map(potControl3, 0, 1023, 50, 500);
}

//  ____  _               _  __
// |  _ \| | __ _ _   _  | |/ /___ _   _
// | |_) | |/ _` | | | | | ' // _ \ | | |
// |  __/| | (_| | |_| | | . \  __/ |_| |
// |_|   |_|\__,_|\__, | |_|\_\___|\__, |
//                |___/            |___/
void playKey(int x, int y) {
  digitalWrite(ledArray[0], HIGH);
  midi.send(0b10010001, x, y);
  delay(mappedPotControl3);
  digitalWrite(ledArray[0], LOW);
  midi.send(0b10010001, x, 0);
}

//  ____  _               _  __
// |  _ \| | __ _ _   _  | |/ /___ _   _
// | |_) | |/ _` | | | | | ' // _ \ | | |
// |  __/| | (_| | |_| | | . \  __/ |_| |
// |_|   |_|\__,_|\__, | |_|\_\___|\__, |
//                |___/            |___/  (major)
void playMajorScale(int x, int y) {
  digitalWrite(ledArray[0], HIGH);
  midi.send(0b10010001, startingNote + majorScale[x], y);
  delay(mappedPotControl3);
  digitalWrite(ledArray[0], LOW);
  midi.send(0b10010001, startingNote + majorScale[x], 0);
}

//  ____  _               _  __
// |  _ \| | __ _ _   _  | |/ /___ _   _
// | |_) | |/ _` | | | | | ' // _ \ | | |
// |  __/| | (_| | |_| | | . \  __/ |_| |
// |_|   |_|\__,_|\__, | |_|\_\___|\__, |
//                |___/            |___/  (minor)
void playMinorScale(int x, int y) {
  digitalWrite(ledArray[0], HIGH);
  midi.send(0b10010001, startingNote + minorScale[x], y);
  delay(mappedPotControl3);
  digitalWrite(ledArray[0], LOW);
  midi.send(0b10010001, startingNote + minorScale[x], 0);
}

//  ____  _               _  __
// |  _ \| | __ _ _   _  | |/ /___ _   _
// | |_) | |/ _` | | | | | ' // _ \ | | |
// |  __/| | (_| | |_| | | . \  __/ |_| |
// |_|   |_|\__,_|\__, | |_|\_\___|\__, |
//                |___/            |___/  (pentatonic major)
void playPentaMajorScale(int x, int y) {
  digitalWrite(ledArray[0], HIGH);
  midi.send(0b10010001, startingNote + majorPentatonicScale[x], y);
  delay(mappedPotControl3);
  digitalWrite(ledArray[0], LOW);
  midi.send(0b10010001, startingNote + majorPentatonicScale[x], 0);
}

//  ____  _               _  __
// |  _ \| | __ _ _   _  | |/ /___ _   _
// | |_) | |/ _` | | | | | ' // _ \ | | |
// |  __/| | (_| | |_| | | . \  __/ |_| |
// |_|   |_|\__,_|\__, | |_|\_\___|\__, |
//                |___/            |___/  (pentatonic minor)
void playPentaMinorScale(int x, int y) {
  digitalWrite(ledArray[0], HIGH);
  midi.send(0b10010001, startingNote + minorPentatonicScale[x], y);
  delay(mappedPotControl3);
  digitalWrite(ledArray[0], LOW);
  midi.send(0b10010001, startingNote + minorPentatonicScale[x], 0);
}

//  __  __ _     _ _    ____            _             _   __  __           _
// |  \/  (_) __| (_)  / ___|___  _ __ | |_ _ __ ___ | | |  \/  | ___   __| | ___
// | |\/| | |/ _` | | | |   / _ \| '_ \| __| '__/ _ \| | | |\/| |/ _ \ / _` |/ _ \
// | |  | | | (_| | | | |__| (_) | | | | |_| | | (_) | | | |  | | (_) | (_| |  __/
// |_|  |_|_|\__,_|_|  \____\___/|_| |_|\__|_|  \___/|_| |_|  |_|\___/ \__,_|\___|
void potSetUpForControl(int x, int y, int z) {
  // midi control 1
  potControl1 = x;
  mappedMidiControl1 = map(potControl1, 1000, -1000, 0, 127);
  midi.send(0b10110001, 0, mappedMidiControl1);
  //  midi(channel, 0xB, 1, mappedMidiControl1);

  // midi control 2
  potControl2 = y;
  mappedMidiControl2 = map(potControl2, -1000, 1000, 0, 127);
  midi.send(0b10110001, 1, mappedMidiControl2);

  // midi control 3
  potControl3 = z;
  mappedMidiControl3 = map(potControl3, 0, 1023, 0, 127);
  //usbMIDI.sendControlChange(2, mappedMidiControl3, 1);
}

//
//   ___           __   ___   __  __   ____        _   _
//  / _ \ _ __    / /  / _ \ / _|/ _| | __ ) _   _| |_| |_ ___  _ __
// | | | | '_ \  / /  | | | | |_| |_  |  _ \| | | | __| __/ _ \| '_ \ 
// | |_| | | | |/ /   | |_| |  _|  _| | |_) | |_| | |_| || (_) | | | |
//  \___/|_| |_/_/     \___/|_| |_|   |____/ \__,_|\__|\__\___/|_| |_|
void checkOnOffButton() {
  onOffButtonLastState = onOffButtonState;
  onOffButtonState = digitalRead(onOffButton);
  if (onOffButtonLastState == LOW and onOffButtonState == HIGH) {
    onOffButtonOn = !onOffButtonOn;
    delay(5);
  } else if (onOffButtonLastState == HIGH and onOffButtonState == LOW) {
    delay(5);
  }
}

//  __  __           _        ____          _ _       _
// |  \/  | ___   __| | ___  / ___|_      _(_) |_ ___| |__
// | |\/| |/ _ \ / _` |/ _ \ \___ \ \ /\ / / | __/ __| '_ \ 
// | |  | | (_) | (_| |  __/  ___) \ V  V /| | || (__| | | |
// |_|  |_|\___/ \__,_|\___| |____/ \_/\_/ |_|\__\___|_| |_|
void checkModeSwitch() {
  modeSwitchLastState = modeSwitchState;
  modeSwitchState = digitalRead(modeSwitch);
  if (modeSwitchLastState == LOW and modeSwitchState == HIGH) {
    modeSwitchOn = !modeSwitchOn;
    delay(5);
  } else if (modeSwitchLastState == HIGH and modeSwitchState == LOW) {
    delay(5);
  }
}

//  _   _           _      _____ _                            _   ____        _   _
// | \ | |         | |    / ____| |                          | | |  _ \      | | | |
// |  \| | _____  _| |_  | |    | |__   __ _ _ __  _ __   ___| | | |_) |_   _| |_| |_ ___  _ __
// | . ` |/ _ \ \/ / __| | |    | '_ \ / _` | '_ \| '_ \ / _ \ | |  _ <| | | | __| __/ _ \| '_ \ 
// | |\  |  __/>  <| |_  | |____| | | | (_| | | | | | | |  __/ | | |_) | |_| | |_| || (_) | | | |
// |_| \_|\___/_/\_\\__|  \_____|_| |_|\__,_|_| |_|_| |_|\___|_| |____/ \__,_|\__|\__\___/|_| |_|
void nextChannelButtonCheck() {
  nextChannelButtonLastState = nextChannelButtonState;
  nextChannelButtonState = digitalRead(nextChannelButton);
  if (nextChannelButtonLastState == LOW and nextChannelButtonState == HIGH) {
    nextStep();
    delay(5);
  } else if (nextChannelButtonLastState == HIGH and nextChannelButtonState == LOW) {
    delay(5);
  }
}

//                  _    _____ _
//                 | |  / ____| |
//  _ __   _____  _| |_| (___ | |_ ___ _ __
// | '_ \ / _ \ \/ / __|\___ \| __/ _ \ '_ \ 
// | | | |  __/>  <| |_ ____) | ||  __/ |_) |
// |_| |_|\___/_/\_\\__|_____/ \__\___| .__/
//                                    | |
//                                    |_|
void nextStep() {
  currentStep = currentStep + 1;
  if (currentStep >= numSteps) {
    currentStep = 0;
  }
}
