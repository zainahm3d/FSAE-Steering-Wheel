#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <FlexCAN.h>

// Button Inputs
int downShiftPin = 14;
int upShiftPin = 15;
int DRSPin = 16;

// Necessary CAN frames
CAN_message_t inMsg;
CAN_message_t downShiftMsg;
CAN_message_t upShiftMsg;
CAN_message_t DRSPressedMsg;
CAN_message_t DRSReleasedMsg;

CAN_message_t CANFrames[5] = {inMsg, downShiftMsg, upShiftMsg, DRSPressedMsg,
                              DRSReleasedMsg};

// interrupt set these to true and then command is handled in the main loop
bool shouldUpShift = false;
bool shouldDownShift = false;
bool DRSPressed = false;
bool DRSReleased = false;

void setup() {
  Serial.begin(9600);
  Serial.println("Online");

  // Pull up all input pins
  pinMode(downShiftPin, INPUT_PULLUP);
  pinMode(upShiftPin, INPUT_PULLUP);
  pinMode(DRSPin, INPUT_PULLUP);

  Can0.begin(250000);

  // Allow Extended CAN id's through
  CAN_filter_t allPassFilter;
  allPassFilter.ext = 1;
  for (uint8_t filterNum = 1; filterNum < 16; filterNum++) {
    Can0.setFilter(allPassFilter, filterNum);
  }

  // Set all CAN frames as extended
  for (int i = 0; i < 5; i++) {
    CANFrames[i].ext = true;
    CANFrames[i].len = 8;

    // Set all frame ID's to 0 except for the inMsg
    if (i != 0) {
      CANFrames[i].id = 0;
    }
  }

  //----- FRAME DEFINITIONS -----
  upShiftMsg.buf[0] = 10;

  downShiftMsg.buf[0] = 11;
}

void loop() {}