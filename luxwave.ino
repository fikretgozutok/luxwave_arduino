#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>
#include <ArduinoJson.h>

#define RX 0
#define TX 1
#define PIXEL_PIN 7 //Defines WS2812B Strip Led Control Pin
#define NUMPIXELS 11 //Defines Number of Pixels

struct RGBData {
  String mode = "wpe";
  int red = 255;
  int green = 255;
  int blue = 255;
  int delayMS = 50;
  uint8_t brightness = 255;
};

void saveToEEPROM(int, const RGBData&);
void readFromEEPROM(int, RGBData&);
void clearEEPROM();

uint32_t Wheel(byte WheelPos);

void colorWipe(uint32_t color, int wait);
void breathe(uint32_t color, int wait);
void rainbow(int wait);
void rgb(int wait);
void theaterChase(uint32_t color, int wait);
void theaterChaseRGB(int wait);
void theaterChase2W(uint32_t color, int wait);
void theaterChaseRGB2W(int wait);

RGBData data;

SoftwareSerial btSerial(RX, TX);
Adafruit_NeoPixel strip(NUMPIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

uint8_t brightness = 0;
bool wayFlag = true;

uint8_t wheelPose = 0;

uint8_t pos = 0;

void setup() {
  Serial.begin(9600);
  btSerial.begin(9600);
  strip.begin();
  strip.show();

  
  readFromEEPROM(0, data);
  brightness = data.brightness;

  if (data.mode == "wpe") {
    colorWipe(strip.Color(data.red, data.green, data.blue), data.delayMS);
  } else if (data.mode == "brt") {
    breathe(strip.Color(data.red, data.green, data.blue), data.delayMS);
  } else if (data.mode == "rbw") {
    rainbow(data.delayMS);
  } else if (data.mode == "rgb") {
    rgb(data.delayMS);
  } else if (data.mode == "tc") {
    theaterChase(strip.Color(data.red, data.green, data.blue), data.delayMS);
  }else if (data.mode == "tcr") {
    theaterChaseRGB(data.delayMS);
  }else if (data.mode == "tc2") {
    theaterChase2W(strip.Color(data.red, data.green, data.blue), data.delayMS);
  }else if (data.mode == "tcr2") {
    theaterChaseRGB2W(data.delayMS);
  }
}

void loop() {
  if (btSerial.available()) {
    String jsonData = btSerial.readStringUntil('\n');

    while (btSerial.available()) {
      btSerial.read();
    }

    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, jsonData);

    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());

      jsonData = btSerial.readStringUntil('\n');
      error = deserializeJson(doc, jsonData);

      if (error) {
        Serial.print(F("deserializeJson() failed again: "));
        Serial.println(error.f_str());
        return;
      }
    }

    data.mode = doc["mode"].as<String>();
    data.red = doc["red"].as<int>();
    data.green = doc["green"].as<int>();
    data.blue = doc["blue"].as<int>();
    data.delayMS = doc["delayMS"].as<int>();
    data.brightness = doc["brightness"].as<uint8_t>();

    brightness = data.brightness;

    pos = 0;
    wayFlag = true;
    wheelPose = 0;

    saveToEEPROM(0, data);

    while (btSerial.available()) {
      btSerial.read();
    }
  }

  if (data.mode == "wpe") {
    colorWipe(strip.Color(data.red, data.green, data.blue), data.delayMS);
  } else if (data.mode == "brt") {
    breathe(strip.Color(data.red, data.green, data.blue), data.delayMS);
  } else if (data.mode == "rbw") {
    rainbow(data.delayMS);
  } else if (data.mode == "rgb") {
    rgb(data.delayMS);
  } else if (data.mode == "tc") {
    theaterChase(strip.Color(data.red, data.green, data.blue), data.delayMS);
  }else if (data.mode == "tcr") {
    theaterChaseRGB(data.delayMS);
  }else if (data.mode == "tc2") {
    theaterChase2W(strip.Color(data.red, data.green, data.blue), data.delayMS);
  }else if (data.mode == "tcr2") {
    theaterChaseRGB2W(data.delayMS);
  }
  
}

void saveToEEPROM(int address, const RGBData& data) {
  int modeLength = data.mode.length();
  EEPROM.put(address, modeLength);
  address += sizeof(modeLength);
  for (int i = 0; i < modeLength; i++) {
    EEPROM.write(address + i, data.mode[i]);
  }
  address += modeLength;

  EEPROM.put(address, data.red);
  address += sizeof(data.red);
  EEPROM.put(address, data.green);
  address += sizeof(data.green);
  EEPROM.put(address, data.blue);
  address += sizeof(data.blue);
  EEPROM.put(address, data.delayMS);
  address += sizeof(data.delayMS);
  EEPROM.put(address, data.brightness);
}

void readFromEEPROM(int address, RGBData& data) {
  int modeLength;
  EEPROM.get(address, modeLength);
  address += sizeof(modeLength);
  char modeBuffer[modeLength + 1];
  for (int i = 0; i < modeLength; i++) {
    modeBuffer[i] = EEPROM.read(address + i);
  }
  modeBuffer[modeLength] = '\0';
  data.mode = String(modeBuffer);
  address += modeLength;

  EEPROM.get(address, data.red);
  address += sizeof(data.red);
  EEPROM.get(address, data.green);
  address += sizeof(data.green);
  EEPROM.get(address, data.blue);
  address += sizeof(data.blue);
  EEPROM.get(address, data.delayMS);
  address += sizeof(data.delayMS);
  EEPROM.get(address, data.brightness);
}

void clearEEPROM() {
  for (int i = 0; i < EEPROM.length(); i++) {
    EEPROM.write(i, 0);
  }
}

uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

void colorWipe(uint32_t color, int wait) {
  if (pos <= strip.numPixels()) {
    strip.setPixelColor(strip.numPixels() - pos, color); 
    strip.setBrightness(brightness);        
    strip.show();                          
    delay(wait);
    pos++;
  }
}

void breathe(uint32_t color, int wait) {
  for (int i = 0; i < NUMPIXELS; i++) {
    strip.setPixelColor(i, color);
  }
  
  strip.setBrightness(brightness);
  if (wayFlag == true) {
    brightness++;
    if (brightness == 255) {
      wayFlag = false;
    }
  } else {
    brightness--;
    if (brightness == 0) {
      wayFlag = true;
    }
  }
  strip.show();
  delay(wait);
}

void rainbow(int wait) {
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, Wheel((i + wheelPose) & 255));
  }

  strip.setBrightness(brightness);
  strip.show();
  wheelPose++;
  delay(20);
}

void rgb(int wait) {
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + wheelPose) & 255));
  }

  strip.setBrightness(brightness);
  strip.show();
  delay(wait);
  wheelPose++;
}

void theaterChase(uint32_t color, int wait) {
  strip.setPixelColor(strip.numPixels() - pos, color);
  strip.setBrightness(brightness);
  strip.show();

  delay(wait);

  strip.setPixelColor(strip.numPixels() - pos, strip.Color(0, 0, 0));
  pos++;
  if (pos > NUMPIXELS) {
    pos = 0;
  }
}

void theaterChaseRGB(int wait){
  theaterChase(Wheel((((strip.numPixels() - pos) * 256 / strip.numPixels()) + wheelPose) & 255), wait);
  wheelPose++;
}


void theaterChase2W(uint32_t color, int wait) {
  strip.setPixelColor(strip.numPixels() - pos, color);
  strip.setBrightness(brightness);
  strip.show();

  delay(wait);

  strip.setPixelColor(strip.numPixels() - pos, strip.Color(0, 0, 0));
  if (wayFlag == true) {
    pos++;
    if (pos == strip.numPixels()) {
      wayFlag = false;
    }
  } else {
    pos--;
    if (pos == 0) {
      wayFlag = true;
    }
  }
}

void theaterChaseRGB2W(int wait){
  theaterChase2W(Wheel((((strip.numPixels() - pos) * 256 / strip.numPixels()) + wheelPose) & 255), wait);
  wheelPose++;
}
