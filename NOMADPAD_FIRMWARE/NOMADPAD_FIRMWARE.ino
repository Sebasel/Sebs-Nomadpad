// Copyright (c) 2017 Electronic Theatre Controls, Inc., http://www.etcconnect.com
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.


/*******************************************************************************
 *
 *   Electronic Theatre Controls
 *
 *   lighthack - USB Test Sketch
 *
 *   (c) 2017 by ETC
 *
 *
 *   This code provides a minimal OSC/USB implementation designed for
 *   troubleshooting issues with the other DIY modules. It does not interface
 *   with any hardware; it simply attempts to connect with Eos. Upon doing so,
 *   it begins periodically sending /eos/ping commands which can then be viewed
 *   in the Eos application to verify that the OSC pipe is working.
 *
 *******************************************************************************
 *
 *  Revision History
 *
 *  yyyy-mm-dd   Vxx        By_Who                 Comment
 *
 *  2017-10-20   V1.0.0.1   Sam Kearney            Original creation
 *
 ******************************************************************************/

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <OSCBoards.h>
#include <OSCBundle.h>
#include <OSCData.h>
#include <OSCMatch.h>
#include <OSCMessage.h>
#include <OSCTiming.h>
#include <RotaryEncoder.h>
#include <Arduino.h>
#ifdef BOARD_HAS_USB_SERIAL
#include <SLIPEncodedUSBSerial.h>
SLIPEncodedUSBSerial SLIPSerial(thisBoardsSerialUSB);
#else
#include <SLIPEncodedSerial.h>
SLIPEncodedSerial SLIPSerial(Serial);
#endif
#include <string.h>

#include <Keypad.h>

 /*******************************************************************************
  * Macros and Constants
  ******************************************************************************/
const String HANDSHAKE_QUERY = "ETCOSC?";
const String HANDSHAKE_REPLY = "OK";

#define SHIFT_BTN           10
#define PIN_IN1             A1
#define PIN_IN2             A2

#define PIN_IN3             A3
#define PIN_IN4             A4

#define PIN_IN5             A5
#define PIN_IN6             A6

#define PIN_IN7             A7
#define PIN_IN8             A8

#define SUBSCRIBE           ((int32_t)1)
#define UNSUBSCRIBE         ((int32_t)0)

#define EDGE_DOWN           ((int32_t)1)
#define EDGE_UP             ((int32_t)0)

#define FORWARD             0
#define REVERSE             1

#define ENC1_DIR            FORWARD
#define ENC2_DIR            FORWARD
#define ENC3_DIR            FORWARD
#define ENC4_DIR            FORWARD

#define ENC1_SCALE          1
#define ENC2_SCALE          1
#define ENC3_SCALE          1
#define ENC4_SCALE          1

/*******************************************************************************
 * Local Types
 ******************************************************************************/
enum WHEEL_MODE { COARSE, FINE };
enum WHEEL_NUM {ENC1, ENC2, ENC3, ENC4};

struct Encoder
{
  uint8_t pinA;
  uint8_t pinB;
  int pinAPrevious;
  int pinBPrevious;
  float pos;
  uint8_t direction;
};

struct Encoder enc1;
struct Encoder enc2;
struct Encoder enc3;
struct Encoder enc4;
/*******************************************************************************
 * Global Variables
 ******************************************************************************/
const byte ROWS = 8; // ten rows
const byte COLS = 13; // ten columns

// The value 0 is used for NO_KEY so we have to pick 100 values greater than 0.
// This should work for values up to 255 but since 'char' is a signed value we should stay below 128.
char keys[ROWS*COLS] = 
{1,2,3,4,5,6,7,8,
9,10,11,12,13,14,15,16,
17,18,19,20,21,22,23,24,
25,26,27,28,29,30,31,32,
33,34,35,36,37,38,39,40,
41,42,43,44,45,46,47,48,
49,50,51,52,53,54,55,56,
57,58,59,60,61,62,63,64,
65,66,67,68,69,71,71,72,
73,74,75,76,77,78,79,80,
81,82,83,84,85,86,87,88,
89,90,91,92,93,94,95,96,
97,98,99,100,101,102,103,104
};

byte rowPins[ROWS] = {53, 51, 49, 47, 45, 43, 41, 39}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {52, 50, 48, 46, 44, 42, 40, 38, 36, 34, 32, 30, 28}; //connect to the column pinouts of the keypad
const int rowCount = sizeof(rowPins)/sizeof(rowPins[0]);
const int colCount = sizeof(colPins)/sizeof(colPins[0]);

/*******************************************************************************
 * Local Functions
 ******************************************************************************/
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS ); 
String msg;

void issueEosSubscribes()
{
  // Add a filter so we don't get spammed with unwanted OSC messages from Eos
  OSCMessage filter("/eos/filter/add");
  filter.add("/eos/out/param/*");
  filter.add("/eos/out/ping");
  SLIPSerial.beginPacket();
  filter.send(SLIPSerial);
  SLIPSerial.endPacket();
}
/*******************************************************************************
 * Given an unknown OSC message we check to see if it's a handshake message.
 * The handshake message must be replied to for Eos to recognize this device
 * as an OSC device.
 *
 * Parameters:
 *  msg - The OSC message of unknown importance
 *
 * Return Value: void
 *
 ******************************************************************************/
void parseOSCMessage(String& msg)
{
    // check to see if this is the handshake string
    if (msg.indexOf(HANDSHAKE_QUERY) != -1)
    {
        // handshake string found!
        SLIPSerial.beginPacket();
        SLIPSerial.write((const uint8_t*)HANDSHAKE_REPLY.c_str(), (size_t)HANDSHAKE_REPLY.length());
        SLIPSerial.endPacket();
    }
}

// function for sending specific keypresses to EOS
void OSCKey(String KEY){
    String keyPress("/eos/key");
    keyPress.concat(KEY);

    OSCMessage keyMsg(keyPress.c_str());
    
    SLIPSerial.beginPacket();
    keyMsg.send(SLIPSerial);
    SLIPSerial.endPacket();
}

/*******************************************************************************
   Initializes a given encoder struct to the requested parameters.

   Parameters:
    encoder - Pointer to the encoder we will be initializing
    pinA - Where the A pin is connected to the Arduino
    pinB - Where the B pin is connected to the Arduino
    direction - Determines if clockwise or counterclockwise is "forward"

   Return Value: void

 ******************************************************************************/
void initEncoder(struct Encoder* encoder, uint8_t pinA, uint8_t pinB, uint8_t direction)
{
  encoder->pinA = pinA;
  encoder->pinB = pinB;
  encoder->pos = 0;
  encoder->direction = direction;

  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);

  encoder->pinAPrevious = digitalRead(pinA);
  encoder->pinBPrevious = digitalRead(pinB);
}

/*******************************************************************************
   Checks if the encoder has moved by comparing the previous state of the pins
   with the current state. If they are different, we know there is movement.
   In the event of movement we update the current state of our pins.

   Parameters:
    encoder - Pointer to the encoder we will be checking for motion

   Return Value:
    encoderMotion - Returns the 0 if the encoder has not moved
                                1 for forward motion
                               -1 for reverse motion

 ******************************************************************************/
int8_t updateEncoder(struct Encoder* encoder)
{
  int8_t encoderMotion = 0;
  int pinACurrent = digitalRead(encoder->pinA);
  int pinBCurrent = digitalRead(encoder->pinB);

  // has the encoder moved at all?
  if (encoder->pinAPrevious != pinACurrent)
  {
    // Since it has moved, we must determine if the encoder has moved forwards or backwards
    encoderMotion = (encoder->pinAPrevious == encoder->pinBPrevious) ? -1 : 1;

    // If we are in reverse mode, flip the direction of the encoder motion
    if (encoder->direction == REVERSE)
      encoderMotion = -encoderMotion;
  }
  encoder->pinAPrevious = pinACurrent;
  encoder->pinBPrevious = pinBCurrent;

  return encoderMotion;
}

/*******************************************************************************
   Sends a message to Eos informing them of a wheel movement.

   Parameters:
    type - the type of wheel that's moving (i.e. pan or tilt)
    ticks - the direction and intensity of the movement

   Return Value: void

 ******************************************************************************/

void sendOscMessage(const String &address, float value)
{
  OSCMessage msg(address.c_str());
  msg.add(value);
  SLIPSerial.beginPacket();
  msg.send(SLIPSerial);
  SLIPSerial.endPacket();
  //Serial.println("SEND SLIPSERIAL");
}

void sendEosWheelMove(WHEEL_NUM type, float ticks)
{
    Serial.println(type);
  String wheelMsg("/eos/active/wheel");


  if (digitalRead(SHIFT_BTN) == LOW)
    wheelMsg.concat("/fine");
  else
    wheelMsg.concat("/coarse");

  switch(type){
    case 0:
    wheelMsg.concat("/1");
    break;
    case ENC2:
    wheelMsg.concat("/2");
    break;
    case ENC3:
    wheelMsg.concat("/3");
    break;
    case ENC4:
    wheelMsg.concat("/4");
    break;

  }
  sendOscMessage(wheelMsg, ticks);
}
/*******************************************************************************
 * Here we prepare to communicate OSC with Eos by setting up SLIPSerial. Once
 * we are done with setup() we pass control over to loop() and never call
 * setup() again.
 *
 * Parameters: none
 *
 * Return Value: void
 *
 ******************************************************************************/
void setup()
{
    for(int x=0; x<rowCount; x++) {
		pinMode(rowPins[x], INPUT);
	}
    
    for(int x=0; x<colCount; x++) {
		pinMode(colPins[x], INPUT_PULLUP);
	}

    SLIPSerial.begin(115200);
    // This is a hack around an Arduino bug. It was taken from the OSC library
    // examples
#ifdef BOARD_HAS_USB_SERIAL
    while (!SerialUSB);
#else
    while (!Serial);
#endif

    // this is necessary for reconnecting a device because it needs some time
    // for the serial port to open, but meanwhile the handshake message was
    // sent from Eos
    SLIPSerial.beginPacket();
    SLIPSerial.write((const uint8_t*)HANDSHAKE_REPLY.c_str(), (size_t)HANDSHAKE_REPLY.length());
    SLIPSerial.endPacket();


    // If it's an Eos, request updates on some things
    //issueEosSubscribes();

    initEncoder(&enc1, PIN_IN1, PIN_IN2, ENC1_DIR);
    initEncoder(&enc2, PIN_IN3, PIN_IN4, ENC2_DIR);
    initEncoder(&enc3, PIN_IN5, PIN_IN6, ENC3_DIR);
    initEncoder(&enc4, PIN_IN7, PIN_IN8, ENC4_DIR);
}

/*******************************************************************************
 * Main loop: manage OSC I/O. Send a ping command to Eos every second.
 *
 * Parameters: none
 *
 * Return Value: void
 *
 ******************************************************************************/
void loop()
{
    static String curMsg;
    static unsigned long lastTimeSent;
    static int32_t pingNum;
    unsigned long curTime;
    int size;
    char key = keypad.getKey();

    // Check to see if any OSC commands have come from Eos that we need to respond to.
    size = SLIPSerial.available();

    int32_t enc1Motion = updateEncoder(&enc1);
    int32_t enc2Motion = updateEncoder(&enc2);
    int32_t enc3Motion = updateEncoder(&enc3);
    int32_t enc4Motion = updateEncoder(&enc4);

    enc1Motion *= ENC1_SCALE;
    enc2Motion *= ENC2_SCALE;
    enc3Motion *= ENC3_SCALE;
    enc4Motion *= ENC4_SCALE;
    
    if (enc1Motion != 0)
    {
        sendEosWheelMove(ENC1, enc1Motion);
        //Serial.println("wheel1MOVE");
    }
    if (enc2Motion != 0)
    {
        sendEosWheelMove(ENC2, enc2Motion);
    }
    
    if (enc3Motion != 0)
    {
        sendEosWheelMove(ENC3, enc3Motion);
    }
    
    if (enc4Motion != 0)
    {
        sendEosWheelMove(ENC4, enc4Motion);
    }
    

    if (size > 0)
    {
        // Fill the msg with all of the available bytes
        while (size--)
            curMsg += (char)(SLIPSerial.read());
    }
    if (SLIPSerial.endofPacket())
    {
        parseOSCMessage(curMsg);
        curMsg = String();
    }

        if (key != NO_KEY) {
        switch (key){
            
            default:
            Serial.println((int)key);
            break;
            
// ROW 1
            case 1:
            OSCKey("/DISPLAYS");
            break;

            case 2:
            OSCKey("/MACRO");
            break;

            case 3:
            OSCKey("/DELETE");
            break;

            case 4:
            OSCKey("/PART");
            break;

            case 5:
            OSCKey("/INT_PALETTE");
            break;

            case 6:
            OSCKey("/COLOR_PALETTE");
            break;

            case 7:
            OSCKey("/PRESET");
            break;

            case 8:
            OSCKey("/SHIFT");
            break;

// ROW 2
            case 9:
            OSCKey("/ADDRESS");
            break;

            case 10:
            OSCKey("/HELP");
            break;

            case 11:
            OSCKey("/COLOR_PATH");
            break;

            case 12:
            OSCKey("/CUE");
            break;

            case 13:
            OSCKey("/FOCUS_PALETTE");
            break;

            case 14:
            OSCKey("/BEAM_PALETTE");
            break;

            case 15:
            OSCKey("/SUB");
            break;

            case 16:
            OSCKey("/DELAY");
            break;

// ROW 3
            case 17:
            OSCKey("/GIO_ENCODER_DISPLAY");
            break;

            case 18:
            OSCKey("/LEARN");
            break;

            case 19:
            OSCKey("/EFFECT");
            break;

            case 20:
            OSCKey("/RECORD");
            break;

            case 21:
            OSCKey("/RECORD_ONLY");
            break;

            case 22:
            OSCKey("/UPDATE");
            break;

            case 23:
            OSCKey("/GROUP");
            break;

            case 24:
            OSCKey("/TIME");
            break;
// ROW 4
            case 25:
            OSCKey("/INTENSITY");
            break;

            case 26:
            OSCKey("/QUERY");
            break;

            case 27:
            OSCKey("/GO_TO_CUE");
            break;

            case 28:
            OSCKey("/+");
            break;

            case 29:
            OSCKey("/7");
            break;

            case 30:
            OSCKey("/4");
            break;

            case 31:
            OSCKey("/1");
            break;

            case 32:
            OSCKey("/CLEAR");
            break;
// ROW 5
            case 33:
            OSCKey("/FOCUS");
            break;

            case 34:
            OSCKey("/COPY_TO");
            break;

            case 35:
            OSCKey("/BLOCK");
            break;

            case 36:
            OSCKey("/THRU");
            break;

            case 37:
            OSCKey("/8");
            break;

            case 38:
            OSCKey("/5");
            break;

            case 39:
            OSCKey("/2");
            break;

            case 40:
            OSCKey("/0");
            break;
// ROW 6
            case 41:
            OSCKey("/COLOR");
            break;

            case 42:
            OSCKey("/RECALL_FROM");
            break;

            case 43:
            OSCKey("/ASSERT");
            break;

            case 44:
            OSCKey("/-");
            break;

            case 45:
            OSCKey("/9");
            break;

            case 46:
            OSCKey("/6");
            break;

            case 47:
            OSCKey("/3");
            break;

            case 48:
            OSCKey("/.");
            break;
// ROW 7

            case 49:
            OSCKey("/SHUTTER");
            break;

            case 50:
            OSCKey("/LABEL");
            break;

            case 51:
            OSCKey("/UNDO");
            break;

            case 52:
            OSCKey("/\\");
            break;

            case 53:
            OSCKey("/REM_DIM");
            break;

            case 54:
            OSCKey("/OUT");
            break;

            case 55:
            OSCKey("/FULL");
            break;

            case 56:
            OSCKey("/AT");
            break;
// ROW 8
            case 57:
            OSCKey("/OPEN_ML_CONTROLS");
            break;

            case 58:
            OSCKey("/HIGH");
            break;

            case 59:
            OSCKey("/MARK");
            break;

            case 60:
            OSCKey("/+%");
            break;

            case 61:
            OSCKey("/-%");
            break;

            case 62:
            OSCKey("/LEVEL");
            break;

            case 63:
            OSCKey("/ENTER");
            break;

            case 64:
            OSCKey("/FORM");
            break;
// ROW 9
            case 65:
            OSCKey("/ABOUT");
            break;

            case 66:
            OSCKey("/FAN");
            break;

            case 67:
            OSCKey("/SNEAK");
            break;

            case 68:
            OSCKey("/HOME");
            break;

            case 69:
            OSCKey("/TRACE");
            break;

            case 70:
            OSCKey("/CUEONLYTRACK");
            break;

            case 71:
            OSCKey("/MORE_SOFTKEYS");
            break;

            case 72:
            OSCKey("/PARK");
            break;
// ROW 10
            case 73:
            OSCKey("/CAPTURE");
            break;

            case 74:
            OSCKey("/SELECT_LAST");
            break;

            case 75:
            OSCKey("/SELECT_MANUAL");
            break;

            case 76:
            OSCKey("/SELECT_ACTIVE");
            break;

            case 77:
            OSCKey("/LAST");
            break;

            case 78:
            OSCKey("/NEXT");
            break;

            case 79:
            OSCKey("/LIVE");
            break;

            case 80:
            OSCKey("/ESCAPE");
            break;
// ROW 11
            case 81:
            OSCKey("/ARROW_LEFT");
            break;

            case 82:
            OSCKey("/BLIND");
            break;

            case 83:
            OSCKey("/ARROW_UP");
            break;

            case 84:
            OSCKey("/ARROW_DOWN");
            break;

            case 85:
            OSCKey("/SCROLL_MODE");
            break;

            case 86:
            OSCKey("/SELECT");
            break;

            case 87:
            OSCKey("/ARROW_RIGHT");
            break;

            case 88:
            OSCKey("/SHIFT");
            break;
// ROW 12
            //case 89:
            //OSCKey("/DISPLAYS");
            //break;

            case 90:
            OSCKey("/MACRO");
            break;

            case 91:
            OSCKey("/DELETE");
            break;

            case 92:
            OSCKey("/PART");
            break;

            case 93:
            OSCKey("/INT_PALETTE");
            break;

            case 94:
            OSCKey("/COLOR_PALETTE");
            break;

            case 95:
            OSCKey("/PRESET");
            break;

            case 96:
            OSCKey("/SHIFT");
            break;
// ROW 13
            case 97:
            OSCKey("/DISPLAYS");
            break;

            case 98:
            OSCKey("/MACRO");
            break;

            case 99:
            OSCKey("/DELETE");
            break;

            case 100:
            OSCKey("/PART");
            break;

            case 101:
            OSCKey("/INT_PALETTE");
            break;

            case 102:
            OSCKey("/COLOR_PALETTE");
            break;

            case 103:
            OSCKey("/PRESET");
            break;

            case 104:
            OSCKey("/SHIFT");
            break;
        }
        
    }
      
}





